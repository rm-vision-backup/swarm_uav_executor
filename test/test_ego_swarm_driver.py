#!/usr/bin/env python3
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped
from swarm_uav_executor.drivers.ego_swarm import EgoSwarmDriver
from swarm_uav_executor.models import HoldGoal, MotionGoal
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

    def _dispatch(self, driver, goal, state, delay_s=0.05):
        result_holder = {}
        def runner():
            result_holder["result"] = driver.start_move_to(
                goal, threading.Event(), time.monotonic() + 2.0)
        thread = threading.Thread(target=runner, daemon=True)
        thread.start()
        push_later(driver, state, delay_s)
        thread.join(1.5)
        return result_holder.get("result")

    def test_holds_when_cancelled(self):
        driver, _ = self._driver()
        event = threading.Event(); event.set()
        result = driver.start_move_to(MotionGoal(1, 0, 15, 0), event, time.monotonic() + 1.0)
        self.assertEqual(result.error_code, "COMMAND_HELD")

    def test_completed_returns_success(self):
        driver, _ = self._driver()
        goal = MotionGoal(1, 0, 15, 0)
        result = self._dispatch(driver, goal, "COMPLETED")
        self.assertTrue(result is not None and result.success)

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

    def test_end_lease_gate_requires_fresh_pose(self):
        driver, _ = self._driver()
        ok, msg = driver.can_end_safety_lease(0.1)
        self.assertFalse(ok)
        self.assertIn("stale", msg)

    def test_end_lease_gate_blocks_while_executing(self):
        driver, _ = self._driver()
        class Msg:
            def __init__(self, data): self.data = data
        driver._on_state(Msg("EXECUTING"))
        ok, _ = driver.can_end_safety_lease(0.1)
        self.assertFalse(ok)

    # ---- MAVROS-state-aware END gate (P1 handoff item 4) ----
    def _driver_with_mavros(self, **kw):
        driver, ros = self._driver(**kw)
        # Emulate the optional mavros_state_topic wiring.
        class Msg: pass
        state = MavrosState()
        state.connected = True
        state.armed = False
        state.mode = ""
        # FakeSub captures the callback via kw['ros'] subscriber creation;
        # re-emulate the State callback directly.
        msg = type("MavrosStateMsg", (), {"state": state})()
        self._emit_mavros(driver, state)
        return driver

    @staticmethod
    def _emit_mavros(driver, state):
        now = time.monotonic()
        driver._on_mavros_state(state)
        # freezegun-style: advance monotonic clock to observe stable window
        if hasattr(driver, "_disarmed_since_mono_s"):
            driver._disarmed_since_mono_s = now

    @staticmethod
    def _emit_fresh_pose(driver):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.z = 15.0
        pose.pose.orientation.w = 1.0
        driver._on_pose(pose)

    def _fresh_pose_emit(self, driver):
        self._emit_fresh_pose(driver)

    def test_end_lease_gate_mavros_requires_connected(self):
        driver, _ = self._driver()
        driver._mavros_state_topic = "/mavros/state"
        state = MavrosState(); state.connected = False; state.armed = False
        self._emit_fresh_pose(driver)
        self._emit_mavros(driver, state)
        ok, msg = driver.can_end_safety_lease(0.1)
        self.assertFalse(ok)
        self.assertIn("disconnected", msg)

    def test_end_lease_gate_mavros_blocks_while_armed(self):
        driver, _ = self._driver()
        driver._mavros_state_topic = "/mavros/state"
        state = MavrosState(); state.connected = True; state.armed = True
        self._emit_fresh_pose(driver)
        self._emit_mavros(driver, state)
        ok, msg = driver.can_end_safety_lease(0.1)
        self.assertFalse(ok)
        self.assertIn("armed", msg)

    def test_end_lease_gate_mavros_requires_stable_disarm(self):
        driver, _ = self._driver()
        driver._mavros_state_topic = "/mavros/state"
        state = MavrosState(); state.connected = True; state.armed = False
        now = time.monotonic()
        self._emit_fresh_pose(driver)
        driver._on_mavros_state(state)
        # disarm just happened -> not yet stable for disarmed_stable_s=5.0
        ok, msg = driver.can_end_safety_lease(5.0)
        self.assertFalse(ok)
        self.assertIn("stable", msg)

    def test_end_lease_gate_mavros_passes_when_stable_disarmed(self):
        driver, _ = self._driver()
        driver._mavros_state_topic = "/mavros/state"
        state = MavrosState(); state.connected = True; state.armed = False
        self._emit_fresh_pose(driver)
        driver._on_mavros_state(state)
        # Emulate stable disarmed: backdate the disarm timestamp
        driver._disarmed_since_mono_s = time.monotonic() - 10.0
        ok, msg = driver.can_end_safety_lease(3.0)
        self.assertTrue(ok)
        self.assertIn("stably disarmed", msg)


if __name__ == "__main__":
    unittest.main()
