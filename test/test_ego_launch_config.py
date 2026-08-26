#!/usr/bin/env python3
"""Static regression tests for safety-sensitive EGO launch arguments."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "uav_executor_ego.launch"


class EgoLaunchConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(str(LAUNCH)).getroot()

    def test_rebound_is_opt_in_and_forwarded_to_planner(self):
        args = {item.attrib["name"]: item.attrib.get("default")
                for item in self.root.findall("arg")}
        self.assertEqual(args.get("enable_rebound"), "false")
        planner = next(node for node in self.root.findall("node")
                       if node.attrib.get("name") == "ego_planner_driver")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in planner.findall("param")}
        self.assertEqual(params.get("enable_rebound"), "$(arg enable_rebound)")

    def test_neighbor_intents_reach_executor_private_params(self):
        args = {item.attrib["name"]: item.attrib.get("default")
                for item in self.root.findall("arg")}
        self.assertEqual(args.get("neighbor_intents"), "")
        executor = next(node for node in self.root.findall("node")
                        if node.attrib.get("type") == "uav_executor_node.py")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in executor.findall("param")}
        self.assertEqual(params.get("ego_swarm/neighbor_intents"),
                         "$(arg neighbor_intents)")

    def test_predictive_supervisor_is_shadow_and_logs_in_workspace(self):
        args = {item.attrib["name"]: item.attrib.get("default")
                for item in self.root.findall("arg")}
        self.assertEqual(args.get("safety_supervisor_mode"), "shadow")
        self.assertTrue(args.get("diagnostic_log_dir", "").endswith(
            "/runtime_logs/ego_planner"))
        planner = next(node for node in self.root.findall("node")
                       if node.attrib.get("name") == "ego_planner_driver")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in planner.findall("param")}
        self.assertEqual(params.get("safety_supervisor_mode"),
                         "$(arg safety_supervisor_mode)")
        self.assertEqual(params.get("neighbor_stale_policy"), "diagnose_only")
        self.assertEqual(params.get("neighbor_missing_policy"),
                         "continue_after_barrier")
        # L3 精简删除 negotiation 参数；26082602 §10 恢复 enable_yield_candidates
        # （SLOW/LEFT/RIGHT 让路候选 + right-of-way）；26082621 迭代新增独立速度上限。
        self.assertNotIn("intent_negotiation_wait_s", params)
        self.assertEqual(params.get("enable_yield_candidates"), "true")
        self.assertEqual(params.get("yield_clearance_factor"), "1.2")
        self.assertEqual(params.get("yield_lateral_max_m"), "2.2")
        self.assertEqual(params.get("yield_max_velocity_mps"), "2.5")
        self.assertEqual(params.get("collision_check_rate_hz"), "10.0")
        self.assertEqual(params.get("enable_rebound"), "$(arg enable_rebound)")

    def test_waypoint_densify_spacing_is_3m(self):
        # implementation_plan_26082621：分层关键点按 3m 间距密化后发布 /waypoints，
        # C++ plan-exec 用"相邻下一个 keypoint"推进（无 horizon 窗口滚动）。
        executor = next(node for node in self.root.findall("node")
                        if node.attrib.get("type") == "uav_executor_node.py")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in executor.findall("param")}
        self.assertEqual(params.get("ego_swarm/waypoint_densify_spacing"), "3.0")
        # plan-exec A/B/C 分支阈值参数已下发 C++ 节点（launch 静态断言）。
        planner = next(node for node in self.root.findall("node")
                       if node.attrib.get("name") == "ego_planner_driver")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in planner.findall("param")}
        self.assertEqual(params.get("reach_thresh_m"), "0.5")
        self.assertEqual(params.get("max_advance_dist_m"), "3.5")
        self.assertEqual(params.get("arrival_reach_thresh_m"), "0.2")
        # horizon 窗口滚动已删除：launch 不应再下发 planning_horizon/position_tolerance_m。
        self.assertNotIn("planning_horizon", params)
        self.assertNotIn("position_tolerance_m", params)

    def test_state_transition_timeout_decoupled_from_execution_timeout(self):
        args = {item.attrib["name"]: item.attrib.get("default")
                for item in self.root.findall("arg")}
        self.assertEqual(args.get("uav_task_timeout_s"), "200.0")
        self.assertEqual(args.get("ego_hold_timeout_s"), "2.0")
        executor = next(node for node in self.root.findall("node")
                        if node.attrib.get("type") == "uav_executor_node.py")
        params = {item.attrib["name"]: item.attrib.get("value")
                  for item in executor.findall("param")}
        # 任务执行超时（DRIVER_TIMEOUT 兜底）与 ego HOLD/收口超时（HOLD 确认）
        # 各自独立上层输入，互不引用。
        self.assertEqual(params.get("ego_swarm/task_timeout_s"),
                         "$(arg uav_task_timeout_s)")
        self.assertEqual(params.get("ego_hold_timeout_s"),
                         "$(arg ego_hold_timeout_s)")


if __name__ == "__main__":
    unittest.main()