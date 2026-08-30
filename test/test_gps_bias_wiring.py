#!/usr/bin/env python3
"""Static checks for gps_bias launch wiring, canonical YAML defaults and permissions."""
import os
import stat
import unittest
import xml.etree.ElementTree as ET

import yaml

PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NODE_PATH = os.path.join(PACKAGE_DIR, "scripts", "gps_bias_node.py")
LAUNCH_PATH = os.path.join(PACKAGE_DIR, "launch", "uav_offboard_ego.launch")
DEFAULTS_PATH = os.path.join(PACKAGE_DIR, "config", "gps_bias_defaults.yaml")


class GpsBiasWiringTest(unittest.TestCase):
    def test_node_script_has_git_executable_bits(self):
        mode = stat.S_IMODE(os.stat(NODE_PATH).st_mode)
        self.assertNotEqual(mode & 0o111, 0, "gps_bias_node.py must be executable")

    def test_offboard_launch_loads_canonical_gps_bias_yaml(self):
        # implementation_plan_26082916 §6：参数唯一来源 gps_bias_defaults.yaml，
        # launch 不保存第二套默认（不再内联 ~window_s 等 param）。
        root = ET.parse(LAUNCH_PATH).getroot()
        nodes = [node for node in root.findall(".//node")
                 if node.get("pkg") == "swarm_uav_executor"
                 and node.get("type") == "gps_bias_node.py"]
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.get("name"), "gps_bias_node")
        loads = [item.get("file", "") for item in node.findall("rosparam")
                 if item.get("command") == "load"]
        self.assertTrue(any("gps_bias_defaults.yaml" in f for f in loads),
                        "launch must load gps_bias_defaults.yaml")
        # 不再内联业务参数默认。
        params = {param.get("name"): param.get("value")
                  for param in node.findall("param")}
        self.assertEqual(params, {})

    def test_canonical_yaml_values(self):
        with open(DEFAULTS_PATH, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        self.assertEqual(data["window_s"], 10.0)
        self.assertEqual(data["publish_rate_hz"], 1.0)
        self.assertEqual(data["lockout_s"], 3.0)


if __name__ == "__main__":
    unittest.main()
