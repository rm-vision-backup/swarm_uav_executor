#!/usr/bin/env python3
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped
from swarm_uav_executor.drivers.ego_swarm import (EgoSwarmDriver,
                                                  _default_neighbor_odom_topics)
from swarm_uav_executor.models import HoldGoal, MotionGoal, MotionResult
from mavros_msgs.msg import State as MavrosState


class FakeTime:
    @staticmethod
    def now():
        return 0


class FakePub:
    def __init__(self): self.msgs = []
    def publish(self, msg): self.msgs.append(msg)


class FakeSub:
    def __init__(self, topic, msg_type, cb, queue_size): self.cb = cb
    def shutdown(self): pass


class FakeRos:
    def __init__(self, ready_param=False):
        self.sleep_calls = 0
        self._ready = ready_param

    def Publisher(self, _t, _m, queue_size=1): return FakePub()
    def Subscriber(self, _t, _m, cb, queue_size=1): return FakeSub(_t, _m, cb, queue_size)
    def get_param(self, name, default=None):
        if name.endswith("/uav_id"):
            return "A01" if self._ready else None
        return default
    def sleep(self, _s):
        self.sleep_calls += 1

    class Time:
        now = FakeTime.now


def push_later(driver, state, delay_s):
    def runner():
        time.sleep(delay_s)
        class Msg:
            def __init__(self, data): self.data = data
        driver._on_state(Msg(state))
    threading.Thread(target=runner, daemon=True).start()


class EgoSwarmDriverTest(unittest.TestCase):
    def _driver(self, **kw):
        ros = FakeRos(ready_param=kw.pop("ready", True))
        kw.setdefault("ros", ros)
        kw.setdefault("state_timeout_s", 1.0)
        return EgoSwarmDriver(**kw), ros

    def _emit_pose(self, driver, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        driver._on_pose(pose)

    def _emit_leader_odom(self, driver, x, y, z):
        from nav_msgs.msg import Odometry
        odom = Odometry()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        driver._on_leader_odom(odom)

    def _dispatch(self, driver, goal, state, delay_s=0.05):
        self._emit_pose(driver, 0.0, 0.0, 15.0)
        result_holder = {}
        def runner():
            result_holder["result"] = driver.start_move_to(
                goal, threading.Event(), time.monotonic() + 2.0)
        thread = threading.Thread(target=runner, daemon=True)
        thread.start()
        push_later(driver, state, delay_s)
        thread.join(1.5)
        return result_holder.get("result")

    # ---- legacy pipeline semantics (MOVE_TO through ego planning) ----

    def test_holds_when_cancelled(self):
        driver, _ = self._driver()
        event = threading.Event(); event.set()
        self._emit_pose(driver, 0.0, 0.0, 15.0)
        result = driver.start_move_to(MotionGoal(1, 0, 15, 0), event, time.monotonic() + 1.0)
        self.assertEqual(result.error_code, "COMMAND_HELD")

    def test_completed_returns_success(self):
        driver, _ = self._driver()
        goal = MotionGoal(1, 0, 15, 0)
        result = self._dispatch(driver, goal, "COMPLETED")
        self.assertTrue(result is not None and result.success)

    def test_horizontal_plan_publishes_frozen_heading(self):
        driver, _ = self._driver()
        goal = MotionGoal(1, 0, 15, 1.25)
        result = self._dispatch(driver, goal, "COMPLETED")
        self.assertTrue(result.success)
        self.assertEqual(len(driver._goal_yaw_pub.msgs), 1)
        self.assertAlmostEqual(driver._goal_yaw_pub.msgs[0].data, 1.25)

    def test_default_neighbor_topics_exclude_own_aircraft(self):
        topics = _default_neighbor_odom_topics("UAV7").split(',')
        self.assertEqual(len(topics), 14)
        self.assertNotIn("/UAV7/mavros/local_position/odom", topics)
        self.assertIn("/UAV15/mavros/local_position/odom", topics)

    def test_plan_failed_returns_error(self):
        driver, _ = self._driver()
        goal = MotionGoal(1, 0, 15, 0)
        result = self._dispatch(driver, goal, "EGO_PLAN_FAILED")
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "EGO_PLAN_FAILED")

    def test_hold_confirms_state(self):
        driver, _ = self._driver()
        def runner():
            time.sleep(0.05)
            driver._hold_pub.publish(None)
            time.sleep(0.05)
            class Msg:
                def __init__(self, data): self.data = data
            driver._on_state(Msg("HOLD"))
        threading.Thread(target=runner, daemon=True).start()
        result = driver.hold(HoldGoal("stop"), time.monotonic() + 1.0)
        self.assertTrue(result.success)

    def test_health_false_when_node_missing(self):
        driver, _ = self._driver(ready=False)
        health = driver.health()
        self.assertFalse(health.ready)
        self.assertEqual(health.error_code, "DRIVER_NOT_READY")

    # ---- HOVER: route the current command through Ego ----

    def test_hover_uses_ego_goal(self):
        driver, _ = self._driver()
        self._emit_pose(driver, 1.0, 2.0, 10.0)
        goal = MotionGoal(999, 999, 10, 0, command="HOVER", layer_z=10.0)
        driver._plan_horizontal = lambda actual_goal, cancel, deadline: (
            self.assertEqual((actual_goal.x, actual_goal.y, actual_goal.z), (999, 999, 10)) or
            MotionResult(True, "", "ok"))
        result = driver.start_move_to(goal, threading.Event(), time.monotonic() + 1.0)
        self.assertTrue(result.success)
        self.assertEqual(len(driver._goal_pub.msgs), 1)

    # ---- FOLLOW_ROUTE follower: PI tracking of leader + offset ----

    def test_follower_target_includes_offset(self):
        driver, _ = self._driver(follower_p_gain=1.0, follower_i_gain=0.0,
                                 follower_limit_xy=20.0, follower_limit_z=1.0,
                                 formation_offsets={"A02": (1.0, 2.0, 0.0)})
        self._emit_pose(driver, 0.0, 0.0, 12.0)
        self._emit_leader_odom(driver, 10.0, 0.0, 12.0)
        goal = MotionGoal(0, 0, 12, 0, command="FOLLOW_ROUTE", leader_id="A02",
                          formation_follow=True, layer_z=12.0)
        event = threading.Event()
        result_holder = {}
        def runner():
            result_holder["result"] = driver.start_move_to(goal, event, time.monotonic() + 2.0)
        thread = threading.Thread(target=runner, daemon=True)
        thread.start()
        time.sleep(0.2)
        event.set()
        thread.join(1.5)
        self.assertEqual(result_holder["result"].error_code, "COMMAND_HELD")
        msgs = driver._setpoint_pub.msgs
        self.assertGreaterEqual(len(msgs), 1)
        # target = leader(10,0,12) + offset(1,2,0) = (11,2,12); P=1 -> vel=(11,2,0),
        # setpoint = own + vel*0.1 = (1.1, 0.2, 12.0)
        self.assertAlmostEqual(msgs[0].pose.position.x, 1.1, delta=0.02)
        self.assertAlmostEqual(msgs[0].pose.position.y, 0.2, delta=0.02)

    def test_follower_pi_clamped(self):
        driver, _ = self._driver(follower_p_gain=1.0, follower_i_gain=0.0,
                                 follower_limit_xy=0.5, follower_limit_z=1.0)
        self._emit_pose(driver, 0.0, 0.0, 12.0)
        self._emit_leader_odom(driver, 10.0, 0.0, 12.0)
        goal = MotionGoal(0, 0, 12, 0, command="FOLLOW_ROUTE", leader_id="A02",
                          formation_follow=True, layer_z=12.0)
        event = threading.Event()
        result_holder = {}
        def runner():
            result_holder["result"] = driver.start_move_to(goal, event, time.monotonic() + 2.0)
        thread = threading.Thread(target=runner, daemon=True)
        thread.start()
        time.sleep(0.2)
        event.set()
        thread.join(1.5)
        self.assertEqual(result_holder["result"].error_code, "COMMAND_HELD")
        msgs = driver._setpoint_pub.msgs
        self.assertGreaterEqual(len(msgs), 1)
        # clamp_xy=0.5 -> setpoint.x step = 0.5*0.1 = 0.05 (unclamped would be 1.0)
        self.assertLessEqual(msgs[0].pose.position.x, 0.5)
        self.assertAlmostEqual(msgs[0].pose.position.x, 0.05, delta=0.02)

    def test_follower_leader_lost(self):
        driver, _ = self._driver()
        self._emit_pose(driver, 0.0, 0.0, 12.0)
        goal = MotionGoal(0, 0, 12, 0, command="FOLLOW_ROUTE", leader_id="A02",
                          formation_follow=True, layer_z=12.0)
        result = driver.start_move_to(goal, threading.Event(), time.monotonic() + 1.0)
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "LEADER_LOST")

    def test_follower_emits_completed_when_formed(self):
        driver, _ = self._driver(steady_s=0.05)
        self._emit_pose(driver, 0.0, 0.0, 12.0)
        self._emit_leader_odom(driver, 0.0, 0.0, 12.0)
        goal = MotionGoal(0, 0, 12, 0, command="FOLLOW_ROUTE", leader_id="A02",
                          formation_follow=True, layer_z=12.0)
        result = driver.start_move_to(goal, threading.Event(), time.monotonic() + 2.0)
        self.assertTrue(result.success)
        self.assertEqual(driver._last_cmd_reply, "COMPLETED")

    def test_intermediate_waypoints_are_duplicated_for_ego(self):
        driver, _ = self._driver()
        goal = MotionGoal(3, 0, 12, 0, command="FOLLOW_ROUTE",
                          waypoints=((1, 0, 5, 0), (2, 0, 10, 0), (3, 0, 12, 0)))
        driver._publish_waypoints(goal)
        points = driver._waypoints_pub.msgs[0].polygon.points
        self.assertEqual([(p.x, p.y, p.z) for p in points],
                         [(1, 0, 5), (1, 0, 5), (2, 0, 10), (2, 0, 10), (3, 0, 12)])


if __name__ == "__main__":
    unittest.main()
