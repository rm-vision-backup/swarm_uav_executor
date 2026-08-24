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


if __name__ == "__main__":
    unittest.main()