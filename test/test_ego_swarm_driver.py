#!/usr/bin/env python3
import threading
import time
import unittest

from swarm_uav_executor.drivers.ego_swarm import EgoSwarmDriver
from swarm_uav_executor.models import HoldGoal, MotionGoal


class FakeTime:
    @staticmethod
    def now():
        return 0


class FakePub:
    def __init__(self): self.msgs = []
    def publish(self, msg): self.msgs.append(msg)


class FakeSub:
    def __init__(self, topic, msg_type, cb, queue_size): self.cb = cb


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


if __name__ == "__main__":
    unittest.main()
