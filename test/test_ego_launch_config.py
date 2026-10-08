#!/usr/bin/env python3
"""Static regression tests for safety-sensitive EGO launch + canonical YAML wiring.

implementation_plan_26082916 §5/§9：业务默认唯一来源为 canonical YAML
（planner_defaults/safety_defaults/resource_limits/ego_swarm_defaults）；launch 只保留
identity/path/明确 deployment override，且不再声明未实现的 neighbor_stale_policy/
neighbor_missing_policy 假配置。
"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "uav_executor_ego.launch"
EXECUTOR_DEFAULTS = Path(__file__).resolve().parents[1] / "config" / "executor_defaults.yaml"
EGO_DEFAULTS = Path(__file__).resolve().parents[1] / "config" / "ego_swarm_defaults.yaml"

# planner canonical 文件在 ego_planner_driver 包（同一工作区 sibling）
_PKG = Path(__file__).resolve().parents[2]
PLANNER_DEFAULTS = _PKG / "ego_planner_driver" / "config" / "planner_defaults.yaml"
SAFETY_DEFAULTS = _PKG / "ego_planner_driver" / "config" / "safety_defaults.yaml"
RESOURCE_LIMITS = _PKG / "ego_planner_driver" / "config" / "resource_limits.yaml"


def _load_yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _param_if(node, name):
    """返回 node 下名为 name 的 param 的 if 属性（无则返回 None）。"""
    for item in node.findall("param"):
        if item.attrib.get("name") == name:
            return item.attrib.get("if")
    return None


class EgoLaunchConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(str(LAUNCH)).getroot()
        cls.planner = next(node for node in cls.root.findall("node")
                           if node.attrib.get("name") == "ego_planner_driver")
        cls.planner_params = {item.attrib["name"]: item.attrib.get("value")
                              for item in cls.planner.findall("param")}
        cls.executor = next(node for node in cls.root.findall("node")
                            if node.attrib.get("type") == "uav_executor_node.py")
        cls.executor_params = {item.attrib["name"]: item.attrib.get("value")
                               for item in cls.executor.findall("param")}
        cls.args = {item.attrib["name"]: item.attrib.get("default")
                    for item in cls.root.findall("arg")}
        cls.pd = _load_yaml(PLANNER_DEFAULTS)
        cls.sd = _load_yaml(SAFETY_DEFAULTS)
        cls.rl = _load_yaml(RESOURCE_LIMITS)
        cls.eg = _load_yaml(EGO_DEFAULTS)["ego_swarm"]

    def test_rebound_is_opt_in_and_forwarded_to_planner(self):
        # 26083018 §3：enable_rebound 业务默认来自 planner_defaults.yaml（false），launch arg
        # 默认空，param 仅在命令行显式传入时条件写入，不保存第二套默认。
        self.assertEqual(self.args.get("enable_rebound"), "")
        self.assertEqual(self.planner_params.get("enable_rebound"),
                         "$(arg enable_rebound)")
        self.assertIn("arg('enable_rebound')", _param_if(self.planner, "enable_rebound"))
        self.assertIs(self.pd["enable_rebound"], False)

    def test_neighbor_intents_reach_executor_private_params(self):
        self.assertEqual(self.args.get("neighbor_intents"), "")
        self.assertEqual(self.executor_params.get("ego_swarm/neighbor_intents"),
                         "$(arg neighbor_intents)")

    def test_predictive_supervisor_is_active_and_logs_in_workspace(self):
        # 26083018 §3：safety_supervisor_mode 业务默认来自 safety_defaults.yaml（active），
        # launch arg 默认空，param 仅在命令行显式传入时条件写入。
        self.assertEqual(self.args.get("safety_supervisor_mode"), "")
        self.assertTrue(self.args.get("diagnostic_log_dir", "").endswith(
            "/runtime_logs/ego_planner"))
        self.assertEqual(self.planner_params.get("safety_supervisor_mode"),
                         "$(arg safety_supervisor_mode)")
        self.assertIn("arg('safety_supervisor_mode')",
                      _param_if(self.planner, "safety_supervisor_mode"))
        self.assertEqual(self.sd["safety_supervisor_mode"], "active")

    def test_no_fake_stale_policy_params_in_launch(self):
        # 26082916 §4：neighbor_stale_policy/neighbor_missing_policy 为从未被源码
        # 读取的假配置，已从 launch 删除；neighbor_intent_stale_s 接入实际 stale 判断。
        self.assertNotIn("neighbor_stale_policy", self.planner_params)
        self.assertNotIn("neighbor_missing_policy", self.planner_params)
        self.assertNotIn("neighbor_stale_policy", self.pd)
        self.assertNotIn("neighbor_missing_policy", self.pd)
        self.assertEqual(self.sd["neighbor_intent_stale_s"], 30.0)

    def test_planner_business_params_come_from_canonical_yaml(self):
        # launch 不再重复业务默认（仅 identity/path/明确 override 内联）。
        allowed_planner_params = {
            "uav_id", "exec_target", "frame_id", "setpoint_out_topic",
            "safety_supervisor_mode", "diagnostic_log_dir",
            "diagnostic_log_queue_size", "enable_rebound",
        }
        self.assertTrue(allowed_planner_params.issuperset(self.planner_params.keys()))
        # 冻结契约/阈值来自 canonical YAML：
        self.assertEqual(self.pd["enable_yield_candidates"], True)
        self.assertEqual(self.pd["yield_clearance_factor"], 1.2)
        self.assertEqual(self.pd["yield_lateral_max_m"], 2.2)
        self.assertEqual(self.pd["yield_max_velocity_mps"], 2.5)
        self.assertEqual(self.pd["reach_thresh_m"], 0.5)
        self.assertEqual(self.pd["max_advance_dist_m"], 3.5)
        self.assertEqual(self.pd["arrival_reach_thresh_m"], 0.5)
        self.assertEqual(self.pd["setpoint_rate_hz"], 30.0)
        self.assertEqual(self.sd["collision_check_rate_hz"], 10.0)
        self.assertEqual(self.sd["protected_distance_m"], 1.0)
        self.assertEqual(self.rl["max_arc_samples"], 4096)
        self.assertEqual(self.rl["max_parameterization_points"], 256)
        self.assertEqual(self.rl["max_trajectory_samples"], 4096)
        self.assertEqual(self.rl["max_neighbor_intent_samples"], 256)
        self.assertEqual(self.rl["segment_direction_epsilon_m"], 0.05)

    def test_launch_planner_has_canonical_rosparam_loads(self):
        loads = [item.attrib.get("file", "") for item in self.planner.findall("rosparam")
                 if item.attrib.get("command") == "load"]
        self.assertTrue(any("planner_defaults.yaml" in f for f in loads))
        self.assertTrue(any("safety_defaults.yaml" in f for f in loads))
        self.assertTrue(any("resource_limits.yaml" in f for f in loads))

    def test_relay_contract_params(self):
        relay = next(node for node in self.root.findall("node")
                     if node.attrib.get("name") == "setpoint_relay")
        relay_params = {item.attrib["name"]: item.attrib.get("value")
                        for item in relay.findall("param")}
        self.assertEqual(relay_params.get("rate_hz"), "30.0")
        self.assertEqual(relay_params.get("candidate_timeout_s"), "0.2")
        self.assertEqual(relay_params.get("output_topic"),
                         "/mavros/setpoint_raw/local")

    def test_executor_defaults_use_single_3d_center_distance(self):
        self.assertEqual(self.eg.get("min_center_distance_m"), 1.0)
        self.assertNotIn("min_horizontal_distance_m", self.eg)
        self.assertNotIn("min_vertical_distance_m", self.eg)

    def test_waypoint_densify_spacing_authoritative_3m_in_yaml_only(self):
        # authoritative default 3.0 唯一来源为 ego_swarm_defaults.yaml；launch 不再重复。
        self.assertEqual(self.eg.get("waypoint_densify_spacing"), 3.0)
        self.assertNotIn("ego_swarm/waypoint_densify_spacing", self.executor_params)

    def test_launch_executor_has_canonical_rosparam_loads(self):
        loads = [item.attrib.get("file", "") for item in self.executor.findall("rosparam")
                 if item.attrib.get("command") == "load"]
        self.assertTrue(any("executor_defaults.yaml" in f for f in loads))
        self.assertTrue(any("ego_swarm_defaults.yaml" in f for f in loads))

    def test_state_transition_timeout_decoupled_from_execution_timeout(self):
        # 26083018 §3：uav_task_timeout_s/ego_hold_timeout_s 业务默认来自 YAML
        # （ego_swarm.task_timeout_s=200.0 / executor_defaults.ego_hold_timeout_s=2.0），
        # launch arg 默认空，param 仅在命令行显式传入时条件写入。
        self.assertEqual(self.args.get("uav_task_timeout_s"), "")
        self.assertEqual(self.args.get("ego_hold_timeout_s"), "")
        self.assertEqual(self.executor_params.get("ego_swarm/task_timeout_s"),
                         "$(arg uav_task_timeout_s)")
        self.assertEqual(self.executor_params.get("ego_hold_timeout_s"),
                         "$(arg ego_hold_timeout_s)")
        self.assertIn("arg('uav_task_timeout_s')",
                      _param_if(self.executor, "ego_swarm/task_timeout_s"))
        self.assertIn("arg('ego_hold_timeout_s')",
                      _param_if(self.executor, "ego_hold_timeout_s"))
        self.assertEqual(self.eg["task_timeout_s"], 200.0)
        self.assertEqual(_load_yaml(EXECUTOR_DEFAULTS)["ego_hold_timeout_s"], 2.0)

    def test_executor_defaults_no_ego_or_mavros_sections(self):
        # 26082916 §5：executor_defaults.yaml 只保留通用 executor 参数。
        data = _load_yaml(EXECUTOR_DEFAULTS)
        self.assertNotIn("ego_swarm", data)
        self.assertNotIn("mavros_position", data)


class MockExecutorLaunchConfigTest(unittest.TestCase):
    """uav_executor_mock.launch：mock 默认来自 executor_defaults.yaml（mock.* 段），
    launch arg 默认空，param 仅在命令行显式传入时条件写入（implementation_plan_26083018 §3）。"""

    @classmethod
    def setUpClass(cls):
        cls.launch = Path(__file__).resolve().parents[1] / "launch" / "uav_executor_mock.launch"
        cls.root = ET.parse(str(cls.launch)).getroot()
        cls.node = next(node for node in cls.root.findall("node")
                        if node.attrib.get("type") == "uav_executor_node.py")
        cls.params = {item.attrib["name"]: item.attrib.get("value")
                      for item in cls.node.findall("param")}
        cls.args = {item.attrib["name"]: item.attrib.get("default")
                    for item in cls.root.findall("arg")}
        cls.ed = _load_yaml(EXECUTOR_DEFAULTS)

    def test_mock_business_defaults_live_in_yaml_not_launch(self):
        self.assertEqual(self.args.get("result_delay_s"), "")
        self.assertEqual(self.args.get("final_success"), "")
        self.assertEqual(self.args.get("error_code"), "")
        self.assertEqual(self.args.get("message"), "")
        self.assertEqual(self.args.get("hold_success"), "")
        self.assertEqual(self.ed["mock"]["result_delay_s"], 0.5)
        self.assertIs(self.ed["mock"]["final_success"], True)
        self.assertEqual(self.ed["mock"]["message"], "mock UAV task finished")
        self.assertIs(self.ed["mock"]["hold_success"], True)

    def test_mock_params_written_only_when_arg_present(self):
        for name in ("mock/result_delay_s", "mock/final_success", "mock/error_code",
                     "mock/message", "mock/hold_success"):
            self.assertEqual(self.params.get(name), f"$(arg {name[5:]})", name)
            self.assertIn(f"arg('{name[5:]}')", _param_if(self.node, name), name)


class MavrosExecutorLaunchConfigTest(unittest.TestCase):
    """uav_executor_mavros.launch：legacy 默认来自 mavros_position_legacy.yaml，
    固定同名业务 param 删除；namespace/frame_id/require_* 仅命令行显式传入时条件写入。"""

    @classmethod
    def setUpClass(cls):
        cls.launch = Path(__file__).resolve().parents[1] / "launch" / "uav_executor_mavros.launch"
        cls.root = ET.parse(str(cls.launch)).getroot()
        cls.node = next(node for node in cls.root.findall("node")
                        if node.attrib.get("type") == "uav_executor_node.py")
        cls.params = {item.attrib["name"]: item.attrib.get("value")
                      for item in cls.node.findall("param")}
        cls.args = {item.attrib["name"]: item.attrib.get("default")
                    for item in cls.root.findall("arg")}
        cls.ml = _load_yaml(Path(__file__).resolve().parents[1] / "config"
                            / "mavros_position_legacy.yaml")["mavros_position"]

    def test_legacy_business_defaults_live_in_yaml_not_launch(self):
        self.assertEqual(self.args.get("mavros_namespace"), "")
        self.assertEqual(self.args.get("frame_id"), "")
        self.assertEqual(self.args.get("require_armed"), "")
        self.assertEqual(self.args.get("require_offboard"), "")
        self.assertEqual(self.args.get("auto_arm"), "")
        self.assertEqual(self.args.get("auto_offboard"), "")
        self.assertEqual(self.ml["namespace"], "/mavros")
        self.assertEqual(self.ml["frame_id"], "map")
        for key in ("require_armed", "require_offboard", "auto_arm", "auto_offboard"):
            self.assertIs(self.ml[key], False, key)

    def test_legacy_params_written_only_when_arg_present(self):
        # namespace/frame_id/require_* 为部署/实验入口，仅显式传入时条件写入。
        arg_by_param = {
            "mavros_position/namespace": "mavros_namespace",
            "mavros_position/frame_id": "frame_id",
            "mavros_position/require_armed": "require_armed",
            "mavros_position/require_offboard": "require_offboard",
            "mavros_position/auto_arm": "auto_arm",
            "mavros_position/auto_offboard": "auto_offboard",
        }
        for name, arg in arg_by_param.items():
            self.assertEqual(self.params.get(name), f"$(arg {arg})", name)
            self.assertIn(f"arg('{arg}')", _param_if(self.node, name), name)


class OnboardNoPropLaunchConfigTest(unittest.TestCase):
    """uav_offboard_ego_real.launch：无桨联调开关 `noprop` 只做"置空转发"，默认不动实飞行为。

    不带 noprop 时 `mavros_state_topic` 必须还是 /mavros/state —— prepare() 的
    connected/armed/OFFBOARD 门照旧（ego_swarm.py:710-721）；只有 noprop:=true 才置空，
    供 `no_prop_link_test.md` 的地面 dry-run 使用。
    """

    @classmethod
    def setUpClass(cls):
        launch = Path(__file__).resolve().parents[1] / "launch" / "uav_offboard_ego_real.launch"
        cls.root = ET.parse(str(launch)).getroot()
        cls.args = {item.attrib["name"]: item.attrib.get("default")
                    for item in cls.root.findall("arg")}
        cls.include_args = {
            item.attrib["name"]: item.attrib.get("value")
            for include in cls.root.findall("include")
            if "uav_executor_ego.launch" in include.attrib.get("file", "")
            for item in include.findall("arg")
        }

    def test_noprop_is_opt_in_and_defaults_off(self):
        self.assertEqual(self.args.get("noprop"), "false")
        self.assertEqual(self.args.get("mavros_state_topic"), "/mavros/state")

    def test_state_topic_blanked_only_through_noprop_guard(self):
        value = self.include_args.get("mavros_state_topic")
        self.assertIsNotNone(value, "executor include 必须转发 mavros_state_topic")
        self.assertIn("arg('noprop')", value)
        self.assertIn("arg('mavros_state_topic')", value)


if __name__ == "__main__":
    unittest.main()
