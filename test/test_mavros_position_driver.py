#!/usr/bin/env python3
import threading
import unittest

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from tf.transformations import quaternion_from_euler

from swarm_uav_executor.drivers.mavros_position import MavrosPositionDriver
from swarm_uav_executor.models import HoldGoal, MotionGoal


class FakeTimeValue:
    def __init__(self, value): self.value = value
    def to_sec(self): return self.value


class FakeHandle:
    def __init__(self): self.stopped = False
    def shutdown(self): self.stopped = True
    def unregister(self): self.stopped = True


class FakePublisher(FakeHandle):
    def __init__(self): super().__init__(); self.messages = []
    def publish(self, message): self.messages.append(message)


class FakeRos:
    class Time: pass
    def __init__(self):
        self.now = 0.0; self.publishers = []; self.subscribers = []; self.timers = []
        self.Time.now = lambda: FakeTimeValue(self.now)
    def Duration(self, value): return value
    def Publisher(self, name, _type, queue_size=10):
        publisher = FakePublisher(); publisher.name = name; self.publishers.append(publisher); return publisher
    def Subscriber(self, name, _type, callback, queue_size=1):
        handle = FakeHandle(); handle.name = name; handle.callback = callback; self.subscribers.append(handle); return handle
    def Timer(self, duration, callback):
        handle = FakeHandle(); handle.duration = duration; handle.callback = callback; self.timers.append(handle); return handle
    def Rate(self, hz):
        ros = self
        class Rate:
            def sleep(self): ros.now += 1.0 / hz
        return Rate()
    def is_shutdown(self): return False


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    message = PoseStamped(); message.header.frame_id = "map"
    message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
    q = quaternion_from_euler(0.0, 0.0, yaw)
    message.pose.orientation.x, message.pose.orientation.y = q[0], q[1]
    message.pose.orientation.z, message.pose.orientation.w = q[2], q[3]
    return message


class MavrosPositionDriverTest(unittest.TestCase):
    def setUp(self):
        self.ros = FakeRos()
        self.driver = MavrosPositionDriver(
            namespace="/UAV1/mavros", pose_timeout_s=1.0, state_timeout_s=1.0,
            settle_duration_s=0.2, ros=self.ros, monotonic_clock=lambda: self.ros.now,
        )
        self.driver._pose_callback(pose())
        self.driver._state_callback(State(connected=True, armed=False, mode="MANUAL"))

    def test_uses_only_configured_local_namespace_and_publishes_target(self):
        self.assertEqual(self.ros.publishers[0].name, "/UAV1/mavros/setpoint_position/local")
        self.driver._set_target(pose(1.0, 2.0, 3.0, 0.4))
        self.driver._publish_setpoint()
        self.assertEqual(len(self.ros.publishers[0].messages), 1)
        self.assertEqual(self.ros.publishers[0].messages[0].header.frame_id, "map")

    def test_health_rejects_disconnect_and_stale_pose(self):
        self.driver._state_callback(State(connected=False))
        self.assertEqual(self.driver.health().error_code, "MAVROS_DISCONNECTED")
        self.driver._state_callback(State(connected=True)); self.ros.now = 1.1
        self.assertEqual(self.driver.health().error_code, "POSE_STALE")

    def test_stable_window_and_wrapped_yaw_error(self):
        self.driver._pose_callback(pose(1.0, 2.0, 3.0, -3.13))
        target = pose(1.0, 2.0, 3.0, 3.13)
        result = self.driver._wait_until_stable(target, threading.Event(), 2.0)
        self.assertTrue(result.success)
        self.assertGreaterEqual(self.ros.now, 0.2)

    def test_cancel_stops_move_wait(self):
        event = threading.Event(); event.set()
        result = self.driver.start_move_to(MotionGoal(5.0, 0.0, 0.0, 0.0), event, 2.0)
        self.assertEqual(result.error_code, "COMMAND_HELD")

    def test_hold_captures_current_pose_and_never_arms_or_changes_mode(self):
        current = pose(4.0, 5.0, 6.0, 0.7); self.driver._pose_callback(current)
        result = self.driver.hold(HoldGoal("test"), 2.0)
        self.assertTrue(result.success)
        self.driver._publish_setpoint()
        published = self.ros.publishers[0].messages[-1]
        self.assertEqual((published.pose.position.x, published.pose.position.y, published.pose.position.z), (4.0, 5.0, 6.0))
        self.assertEqual(len(self.ros.publishers), 1)

    def test_forbids_automatic_arm_and_offboard(self):
        with self.assertRaises(ValueError):
            MavrosPositionDriver(auto_arm=True, ros=FakeRos())
        with self.assertRaises(ValueError):
            MavrosPositionDriver(auto_offboard=True, ros=FakeRos())

    def test_end_requires_stably_disarmed_state(self):
        safe, _ = self.driver.can_end_safety_lease(3.0); self.assertFalse(safe)
        self.ros.now = 3.0
        self.driver._state_callback(State(connected=True, armed=False))
        safe, _ = self.driver.can_end_safety_lease(3.0); self.assertTrue(safe)
        self.driver._state_callback(State(connected=True, armed=True))
        safe, _ = self.driver.can_end_safety_lease(3.0); self.assertFalse(safe)

    def test_end_rejects_stale_or_disconnected_state(self):
        self.ros.now = 3.0
        safe, message = self.driver.can_end_safety_lease(3.0)
        self.assertFalse(safe); self.assertIn("stale", message)
        self.driver._state_callback(State(connected=False, armed=False))
        safe, message = self.driver.can_end_safety_lease(3.0)
        self.assertFalse(safe); self.assertIn("disconnected", message)

    def test_exposes_ros_clock_and_shutdown_stops_all_handles(self):
        self.ros.now = 4.2
        self.assertEqual(self.driver.clock(), 4.2)
        self.driver._set_target(pose(1.0, 0.0, 0.0))
        self.driver.shutdown()
        self.driver._publish_setpoint()
        self.assertEqual(len(self.ros.publishers[0].messages), 0)
        self.assertTrue(self.ros.publishers[0].stopped)
        self.assertTrue(all(handle.stopped for handle in self.ros.subscribers + self.ros.timers))


if __name__ == "__main__": unittest.main()
