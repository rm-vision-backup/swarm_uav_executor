"""Direct local MAVROS position-setpoint driver.

The driver never arms the vehicle or changes flight mode.  Those operations
remain an explicit external safety procedure.
"""
from __future__ import annotations

import copy
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from .base import MotionDriver
from ..models import DriverHealth, MotionResult


def _normalise_namespace(value):
    namespace = "/" + str(value or "").strip("/")
    if namespace == "/":
        raise ValueError("mavros namespace must be explicit")
    return namespace


class MavrosPositionDriver(MotionDriver):
    """Continuously publishes one local vehicle's position target."""

    def __init__(self, namespace="/mavros", frame_id="map", setpoint_rate_hz=30.0,
                 pose_timeout_s=0.5, state_timeout_s=1.0,
                 position_tolerance_m=0.20, yaw_tolerance_rad=0.15,
                 settle_duration_s=1.0, require_connected=True,
                 require_armed=False, require_offboard=False,
                  auto_arm=False, auto_offboard=False, ros=rospy,
                  pose_type=PoseStamped, state_type=State,
                  monotonic_clock=time.monotonic):
        if auto_arm or auto_offboard:
            raise ValueError("automatic arm/offboard is forbidden by the safety baseline")
        if not frame_id or frame_id.startswith("/"):
            raise ValueError("frame_id must be a non-empty TF frame without leading slash")
        if setpoint_rate_hz < 2.0:
            raise ValueError("setpoint_rate_hz must be at least 2 Hz")
        if min(pose_timeout_s, state_timeout_s, position_tolerance_m,
               yaw_tolerance_rad, settle_duration_s) <= 0.0:
            raise ValueError("MAVROS timing and tolerance values must be positive")

        self._ros = ros
        self.namespace = _normalise_namespace(namespace)
        self.frame_id = frame_id
        self.setpoint_rate_hz = float(setpoint_rate_hz)
        self.pose_timeout_s = float(pose_timeout_s)
        self.state_timeout_s = float(state_timeout_s)
        self.position_tolerance_m = float(position_tolerance_m)
        self.yaw_tolerance_rad = float(yaw_tolerance_rad)
        self.settle_duration_s = float(settle_duration_s)
        self.require_connected = bool(require_connected)
        self.require_armed = bool(require_armed)
        self.require_offboard = bool(require_offboard)
        self._pose_type = pose_type
        self._lock = threading.RLock()
        self._pose = None
        self._pose_received_s = None
        self._state = None
        self._state_received_s = None
        self._state_received_monotonic_s = None
        self._target = None
        self._shutdown = False
        self._monotonic_clock = monotonic_clock
        self._disarmed_since_monotonic_s = None
        self.clock = self._now_s

        self._publisher = ros.Publisher(
            self.namespace + "/setpoint_position/local", pose_type, queue_size=10
        )
        self._pose_subscriber = ros.Subscriber(
            self.namespace + "/local_position/pose", pose_type,
            self._pose_callback, queue_size=1
        )
        self._state_subscriber = ros.Subscriber(
            self.namespace + "/state", state_type,
            self._state_callback, queue_size=1
        )
        self._timer = ros.Timer(
            ros.Duration(1.0 / self.setpoint_rate_hz), self._publish_setpoint
        )

    @classmethod
    def from_ros_params(cls):
        prefix = "~mavros_position/"
        auto_arm = rospy.get_param(prefix + "auto_arm", False)
        auto_offboard = rospy.get_param(prefix + "auto_offboard", False)
        return cls(
            namespace=rospy.get_param(prefix + "namespace"),
            frame_id=rospy.get_param(prefix + "frame_id", "map"),
            setpoint_rate_hz=rospy.get_param(prefix + "setpoint_rate_hz", 30.0),
            pose_timeout_s=rospy.get_param(prefix + "pose_timeout_s", 0.5),
            state_timeout_s=rospy.get_param(prefix + "state_timeout_s", 1.0),
            position_tolerance_m=rospy.get_param(prefix + "position_tolerance_m", 0.2),
            yaw_tolerance_rad=rospy.get_param(prefix + "yaw_tolerance_rad", 0.15),
            settle_duration_s=rospy.get_param(prefix + "settle_duration_s", 1.0),
            require_connected=rospy.get_param(prefix + "require_connected", True),
            require_armed=rospy.get_param(prefix + "require_armed", False),
            require_offboard=rospy.get_param(prefix + "require_offboard", False),
            auto_arm=auto_arm,
            auto_offboard=auto_offboard,
        )

    def _now_s(self):
        return self._ros.Time.now().to_sec()

    def _pose_callback(self, message):
        with self._lock:
            self._pose = copy.deepcopy(message)
            self._pose_received_s = self._now_s()

    def _state_callback(self, message):
        with self._lock:
            self._state = copy.deepcopy(message)
            self._state_received_s = self._now_s()
            self._state_received_monotonic_s = self._monotonic_clock()
            if message.armed:
                self._disarmed_since_monotonic_s = None
            elif self._disarmed_since_monotonic_s is None:
                self._disarmed_since_monotonic_s = self._monotonic_clock()

    def _set_target(self, target):
        if target.header.frame_id != self.frame_id:
            raise ValueError("target frame does not match configured local frame")
        with self._lock:
            self._target = copy.deepcopy(target)

    def _publish_setpoint(self, _event=None):
        with self._lock:
            if self._shutdown or self._target is None:
                return
            message = copy.deepcopy(self._target)
        message.header.stamp = self._ros.Time.now()
        self._publisher.publish(message)

    def health(self):
        now = self._now_s()
        with self._lock:
            pose_age = None if self._pose_received_s is None else now - self._pose_received_s
            state_age = None if self._state_received_s is None else now - self._state_received_s
            state = copy.deepcopy(self._state)
            stopped = self._shutdown
        if stopped:
            return DriverHealth(False, "DRIVER_SHUTDOWN", "MAVROS driver is shut down")
        if pose_age is None or pose_age > self.pose_timeout_s:
            return DriverHealth(False, "POSE_STALE", "local pose is unavailable or stale")
        if state_age is None or state_age > self.state_timeout_s:
            return DriverHealth(False, "MAVROS_STATE_STALE", "MAVROS state is unavailable or stale")
        if self.require_connected and not state.connected:
            return DriverHealth(False, "MAVROS_DISCONNECTED", "MAVROS is disconnected")
        if self.require_armed and not state.armed:
            return DriverHealth(False, "VEHICLE_NOT_ARMED", "vehicle is not armed")
        if self.require_offboard and str(state.mode).upper() != "OFFBOARD":
            return DriverHealth(False, "OFFBOARD_NOT_ACTIVE", "OFFBOARD mode is not active")
        return DriverHealth(True, "", "MAVROS position driver ready")

    def _target_from_goal(self, goal):
        target = self._pose_type()
        target.header.frame_id = self.frame_id
        target.pose.position.x = goal.x
        target.pose.position.y = goal.y
        target.pose.position.z = goal.z
        quaternion = quaternion_from_euler(0.0, 0.0, goal.yaw)
        target.pose.orientation.x, target.pose.orientation.y = quaternion[0], quaternion[1]
        target.pose.orientation.z, target.pose.orientation.w = quaternion[2], quaternion[3]
        return target

    @staticmethod
    def _yaw(pose):
        q = pose.pose.orientation
        return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

    @classmethod
    def _target_error(cls, current, target):
        dx = current.pose.position.x - target.pose.position.x
        dy = current.pose.position.y - target.pose.position.y
        dz = current.pose.position.z - target.pose.position.z
        yaw_error = cls._yaw(current) - cls._yaw(target)
        yaw_error = abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error)))
        return math.sqrt(dx * dx + dy * dy + dz * dz), yaw_error

    def _wait_until_stable(self, target, cancel_event, deadline):
        stable_since = None
        rate = self._ros.Rate(max(10.0, min(self.setpoint_rate_hz, 50.0)))
        while not self._ros.is_shutdown():
            now = self._now_s()
            if cancel_event is not None and cancel_event.is_set():
                return MotionResult(False, "COMMAND_HELD", "movement cancelled by HOLD")
            if now >= deadline:
                return MotionResult(False, "LOCAL_TIMEOUT", "movement deadline exceeded")
            health = self.health()
            if not health.ready:
                return MotionResult(False, health.error_code, health.message)
            with self._lock:
                current = copy.deepcopy(self._pose)
            position_error, yaw_error = self._target_error(current, target)
            if (position_error <= self.position_tolerance_m and
                    yaw_error <= self.yaw_tolerance_rad):
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= self.settle_duration_s:
                    return MotionResult(True, "", "target reached and stable")
            else:
                stable_since = None
            rate.sleep()
        return MotionResult(False, "ROS_SHUTDOWN", "ROS shutdown while moving")

    def start_move_to(self, goal, cancel_event, deadline):
        health = self.health()
        if not health.ready:
            return MotionResult(False, health.error_code, health.message)
        target = self._target_from_goal(goal)
        self._set_target(target)
        return self._wait_until_stable(target, cancel_event, deadline)

    def _capture_hold_pose(self):
        health = self.health()
        if not health.ready:
            raise RuntimeError("%s: %s" % (health.error_code, health.message))
        with self._lock:
            target = copy.deepcopy(self._pose)
        target.header.frame_id = self.frame_id
        target.header.stamp = self._ros.Time.now()
        return target

    def hold(self, _goal, deadline):
        try:
            target = self._capture_hold_pose()
        except RuntimeError as error:
            return MotionResult(False, "HOLD_FAILED", str(error))
        self._set_target(target)
        # Capturing the current healthy pose is the HOLD acceptance criterion;
        # continuous publication remains active after this method returns.
        if self._now_s() >= deadline:
            return MotionResult(False, "HOLD_FAILED", "HOLD deadline exceeded")
        return MotionResult(True, "", "current local pose captured for HOLD")

    def shutdown(self):
        with self._lock:
            self._shutdown = True
            self._target = None
        for handle in (self._timer, self._pose_subscriber, self._state_subscriber,
                       self._publisher):
            try:
                handle.shutdown() if hasattr(handle, "shutdown") else handle.unregister()
            except Exception:
                pass
