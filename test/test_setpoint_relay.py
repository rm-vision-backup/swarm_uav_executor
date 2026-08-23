#!/usr/bin/env python3
"""setpoint_relay 仲裁/缓存类纯单元测试（无 ROS Master 依赖）。

覆盖：选源、generation 隔离、候选 freshness、finite/frame/mask 校验、
HOLD 锁存与 pose stale 回退、每 tick 恰发一帧、候选恢复退出 HOLD。
"""
import math
import unittest

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

from swarm_uav_executor.setpoint_relay import (SetpointRelay, MASK_EGO_TRAJECTORY,
                                               MASK_HOLD, MASK_FOLLOWER)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def advance(self, dt):
        self.t += dt

    def __call__(self):
        return self.t


def make_pose(x, y, z, yaw=0.0):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    return pose


def make_target(x=1.0, y=2.0, z=3.0, vx=0.5, vy=0.0, vz=0.0,
                ax=0.0, ay=0.0, az=0.0, yaw=0.0, mask=MASK_EGO_TRAJECTORY,
                frame=None):
    msg = PositionTarget()
    msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED if frame is None else frame
    msg.type_mask = mask
    msg.position.x, msg.position.y, msg.position.z = x, y, z
    msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, vz
    msg.acceleration_or_force.x = ax
    msg.acceleration_or_force.y = ay
    msg.acceleration_or_force.z = az
    msg.yaw = yaw
    return msg


def make_follower(x, y, z, vx, vy, vz, yaw=0.0):
    return make_target(x, y, z, vx, vy, vz, yaw=yaw, mask=MASK_FOLLOWER)


class SetpointRelayTest(unittest.TestCase):
    def _relay(self, clock, candidate_timeout_s=0.2, pose_timeout_s=1.0):
        relay = SetpointRelay(candidate_timeout_s, pose_timeout_s,
                              monotonic_clock=clock)
        relay.on_pose(make_pose(0.0, 0.0, 5.0), now=clock())
        return relay

    def test_default_selects_ego(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(), now=clock())
        out = relay.tick(now=clock(), stamp=1)
        self.assertIsNotNone(out)
        self.assertEqual(out.position.x, 1.0)
        self.assertEqual(out.type_mask, MASK_EGO_TRAJECTORY)
        self.assertEqual(out.header.stamp, 1)

    def test_mode_true_selects_follower(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        relay.on_candidate("follower", make_follower(10.0, 0.0, 12.0, 1.0, 0.0, 0.0),
                           now=clock())
        relay.on_mode(True)
        out = relay.tick(now=clock(), stamp=2)
        self.assertIsNotNone(out)
        # 切换后 follower 缓存被作废 -> HOLD，直到新的 follower 候选。
        self.assertEqual(out.type_mask, MASK_HOLD)
        relay.on_candidate("follower", make_follower(11.0, 0.0, 12.0, 1.0, 0.0, 0.0),
                           now=clock())
        out = relay.tick(now=clock(), stamp=3)
        self.assertEqual(out.type_mask, MASK_FOLLOWER)
        self.assertAlmostEqual(out.position.x, 11.0)

    def test_switch_back_to_ego_requires_fresh_candidate(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        out = relay.tick(now=clock(), stamp=1)
        self.assertEqual(out.position.x, 1.0)

        # 切到 follower，收到 follower 候选，再切回 EGO。
        relay.on_mode(True)
        relay.on_candidate("follower", make_follower(10.0, 0.0, 12.0, 1.0, 0.0, 0.0),
                           now=clock())
        out = relay.tick(now=clock(), stamp=2)
        self.assertEqual(out.type_mask, MASK_FOLLOWER)

        relay.on_mode(False)
        # 旧 EGO 缓存（切换前）不得穿透。
        out = relay.tick(now=clock(), stamp=3)
        self.assertEqual(out.type_mask, MASK_HOLD)
        # 收到切换后的新 EGO 候选后恢复。
        relay.on_candidate("ego", make_target(x=5.0), now=clock())
        out = relay.tick(now=clock(), stamp=4)
        self.assertEqual(out.type_mask, MASK_EGO_TRAJECTORY)
        self.assertAlmostEqual(out.position.x, 5.0)

    def test_candidate_timeout_enters_hold(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        out = relay.tick(now=clock(), stamp=1)
        self.assertEqual(out.type_mask, MASK_EGO_TRAJECTORY)
        clock.advance(0.5)  # > candidate_timeout_s=0.2
        out = relay.tick(now=clock(), stamp=2)
        self.assertEqual(out.type_mask, MASK_HOLD)
        self.assertAlmostEqual(out.position.x, 0.0)  # pose 锁存

    def test_nan_candidate_rejected(self):
        clock = FakeClock()
        relay = self._relay(clock)
        bad = make_target()
        bad.position.y = float("nan")
        self.assertFalse(relay.on_candidate("ego", bad, now=clock()))
        # 未接受 -> tick 进入 HOLD。
        out = relay.tick(now=clock(), stamp=1)
        self.assertEqual(out.type_mask, MASK_HOLD)

    def test_wrong_frame_rejected(self):
        clock = FakeClock()
        relay = self._relay(clock)
        bad = make_target(frame=PositionTarget.FRAME_BODY_NED)
        self.assertFalse(relay.on_candidate("ego", bad, now=clock()))

    def test_wrong_mask_rejected(self):
        clock = FakeClock()
        relay = self._relay(clock)
        # follower 用 EGO mask 或 EGO 用 follower mask 均拒绝。
        self.assertFalse(relay.on_candidate(
            "follower", make_target(), now=clock()))

    def test_unselected_source_cannot_take_over(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        # follower 持续更新但未选中 -> 不接管。
        relay.on_candidate("follower", make_follower(10.0, 0.0, 12.0, 1.0, 0.0, 0.0),
                           now=clock())
        out = relay.tick(now=clock(), stamp=1)
        self.assertEqual(out.type_mask, MASK_EGO_TRAJECTORY)
        self.assertAlmostEqual(out.position.x, 1.0)

    def test_hold_latches_pose_not_drifting(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        out = relay.tick(now=clock(), stamp=1)
        self.assertEqual(out.type_mask, MASK_EGO_TRAJECTORY)

        # 候选过期进入 HOLD：进入时刻锁存 fresh pose (3,3,9)。
        clock.advance(0.5)
        relay.on_pose(make_pose(3.0, 3.0, 9.0), now=clock())
        out1 = relay.tick(now=clock(), stamp=2)
        self.assertEqual(out1.type_mask, MASK_HOLD)
        self.assertAlmostEqual(out1.position.x, 3.0)
        self.assertAlmostEqual(out1.position.z, 9.0)
        # pose 随后漂移到 (4,4,10)，HOLD 仍锁存 (3,3,9)，不跟随噪声。
        clock.advance(0.1)
        relay.on_pose(make_pose(4.0, 4.0, 10.0), now=clock())
        out2 = relay.tick(now=clock(), stamp=3)
        self.assertEqual(out2.type_mask, MASK_HOLD)
        self.assertAlmostEqual(out2.position.x, 3.0)
        self.assertAlmostEqual(out2.position.z, 9.0)

    def test_pose_stale_reuses_last_hold_not_origin(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        clock.advance(0.5)
        out = relay.tick(now=clock(), stamp=1)
        self.assertEqual(out.type_mask, MASK_HOLD)
        self.assertAlmostEqual(out.position.x, 0.0)
        # pose 变 stale 后仍复用上个 HOLD，不生成 (0,0,0)。
        clock.advance(2.0)  # > pose_timeout_s
        relay.on_pose(make_pose(100.0, 100.0, 100.0), now=clock() - 2.0)
        out = relay.tick(now=clock(), stamp=2)
        self.assertEqual(out.type_mask, MASK_HOLD)
        self.assertAlmostEqual(out.position.x, 0.0)
        self.assertAlmostEqual(out.position.z, 5.0)

    def test_no_pose_no_output(self):
        clock = FakeClock()
        relay = SetpointRelay(0.2, 1.0, monotonic_clock=clock)
        self.assertIsNone(relay.tick(now=clock(), stamp=1))

    def test_recovery_exits_hold(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        clock.advance(0.5)
        out = relay.tick(now=clock(), stamp=1)
        self.assertEqual(out.type_mask, MASK_HOLD)
        # 有效候选恢复 -> 退出内部 HOLD。
        relay.on_candidate("ego", make_target(x=7.0), now=clock())
        out = relay.tick(now=clock(), stamp=2)
        self.assertEqual(out.type_mask, MASK_EGO_TRAJECTORY)
        self.assertAlmostEqual(out.position.x, 7.0)

    def test_tick_updates_stamp_and_outputs_one_frame(self):
        clock = FakeClock()
        relay = self._relay(clock)
        relay.on_candidate("ego", make_target(x=1.0), now=clock())
        for i in range(5):
            clock.advance(1.0 / 30.0)
            out = relay.tick(now=clock(), stamp=i)
            self.assertIsNotNone(out)
            self.assertIsInstance(out, PositionTarget)
            self.assertEqual(out.header.stamp, i)


if __name__ == "__main__":
    unittest.main()
