#!/usr/bin/env python3
"""BiasEstimator unit tests; these do not require a running ROS master."""
import os
import sys
import unittest

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from gps_bias_node import BiasEstimator  # noqa: E402


class BiasEstimatorTest(unittest.TestCase):
    def test_own_peer_conversion_with_different_receive_times(self):
        own = BiasEstimator(window_s=10.0, lockout_s=3.0)
        peer = BiasEstimator(window_s=10.0, lockout_s=3.0)

        # 同一个 GPS 基准：本机 ROS=GPS+2，邻机 ROS=GPS+3；接收时刻彼此独立。
        own.observe(received_at_s=100.0, ros_stamp_s=1002.0, reference_s=1000.0)
        peer.observe(received_at_s=101.25, ros_stamp_s=1003.0, reference_s=1000.0)
        own_bias = own.value_at(102.0)
        peer_bias = peer.value_at(102.0)

        self.assertAlmostEqual(own_bias, 2.0)
        self.assertAlmostEqual(peer_bias, 3.0)
        peer_stamp = 1013.0  # GPS event 1010 represented on peer's ROS clock.
        self.assertAlmostEqual(peer_stamp - peer_bias + own_bias, 1012.0)

    def test_window_keeps_left_boundary_then_expires_it(self):
        estimator = BiasEstimator(window_s=10.0, lockout_s=20.0)
        estimator.observe(0.0, 101.0, 100.0)       # bias 1
        estimator.observe(10.0, 103.0, 100.0)      # bias 3; t=0 is retained
        self.assertAlmostEqual(estimator.value_at(10.0), 2.0)

        estimator.observe(10.000001, 105.0, 100.0)  # t=0 is now outside
        self.assertAlmostEqual(estimator.value_at(10.000001), 4.0)

    def test_receive_jitter_controls_window_not_message_stamps(self):
        estimator = BiasEstimator(window_s=2.0, lockout_s=3.0)
        estimator.observe(20.0, 1002.1, 1000.0)
        estimator.observe(20.15, 2001.9, 2000.0)
        estimator.observe(21.9, 3002.0, 3000.0)
        self.assertAlmostEqual(estimator.value_at(21.9), 2.0)

        estimator.observe(22.01, 4002.2, 4000.0)
        self.assertAlmostEqual(estimator.value_at(22.01), (1.9 + 2.0 + 2.2) / 3.0)

    def test_lockout_is_strictly_greater_than_boundary(self):
        estimator = BiasEstimator(window_s=10.0, lockout_s=3.0)
        self.assertIsNone(estimator.value_at(0.0))
        estimator.observe(5.0, 12.0, 10.0)
        self.assertAlmostEqual(estimator.value_at(8.0), 2.0)
        self.assertIsNone(estimator.value_at(8.000001))


if __name__ == "__main__":
    unittest.main()
