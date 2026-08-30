#!/usr/bin/env python3
"""Lightweight rostest: no TimeReference means the node stays alive and stays silent."""
import unittest

import rosnode
import rospy
import rostest
from std_msgs.msg import Float64


class GpsBiasNoReferenceRosTest(unittest.TestCase):
    def test_alive_without_time_reference_and_no_bias_message(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if "/gps_bias_node" in rosnode.get_node_names():
                break
            rospy.sleep(0.05)
        self.assertIn("/gps_bias_node", rosnode.get_node_names())

        with self.assertRaises(rospy.ROSException):
            rospy.wait_for_message("/gps_bias", Float64, timeout=2.0)

        # Confirm it did not merely die while the no-message assertion waited.
        self.assertIn("/gps_bias_node", rosnode.get_node_names())


if __name__ == "__main__":
    rospy.init_node("test_gps_bias_no_reference_ros")
    rostest.rosrun("swarm_uav_executor", "gps_bias_no_reference",
                   GpsBiasNoReferenceRosTest)
