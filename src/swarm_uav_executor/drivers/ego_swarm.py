"""Ego-swarm driver bridging the ego_planner_driver C++ node.

The C++ node publishes a latched exec_state topic that this driver uses as
the single source of truth for the motion lifecycle:
  IDLE -> EXECUTING -> COMPLETED / EGO_PLAN_FAILED / EGO_EXEC_TIMEOUT / HOLD

The driver itself never arms the vehicle or switches flight mode; the
underlying node only publishes position setpoints (PX4 OFFBOARD compatible).
"""
from __future__ import annotations

import copy
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseStamped
from std_msgs.msg import Empty, String

from .base import MotionDriver
from ..models import DriverHealth, HoldGoal, MotionResult

_STATE_EXECUTING = "EXECUTING"
_STATE_COMPLETED = "COMPLETED"
_STATE_HOLD = "HOLD"
_STATE_POSE_STALE = "POSE_STALE"
_STATE_PLAN_FAILED = "EGO_PLAN_FAILED"
_STATE_TIMEOUT = "EGO_EXEC_TIMEOUT"
_TERMINAL_OK = frozenset((_STATE_COMPLETED,))
_TERMINAL_BAD = frozenset((_STATE_POSE_STALE, _STATE_PLAN_FAILED, _STATE_TIMEOUT))
_MONITOR_HZ = 20.0


def _split_topics(raw):
    if not raw:
        return []
    return [x.strip() for x in raw.split(',') if x.strip()]


class EgoSwarmDriver(MotionDriver):
    """MotionDriver over the ego_planner_driver binary via its topics."""

    def __init__(self, namespace="/UAV1", state_timeout_s=5.0,
                 ros=rospy, monotonic_clock=time.monotonic,
                 pos_tolerance_m=0.2, steady_s=1.0,
                 pose_timeout_s=1.0, neighbor_intents='',
                 intent_type=None, mavros_state_topic=""):
        self.namespace = "/" + str(namespace).strip("/")
        self.state_timeout_s = float(state_timeout_s)
        self.pos_tolerance_m = float(pos_tolerance_m)
        self.steady_s = float(steady_s)
        self.pose_timeout_s = float(pose_timeout_s)
        self._ros = ros
        self._monotonic_clock = monotonic_clock
        self.clock = monotonic_clock          # executor deadlines must share this clock
        self._shutdown = False
        self._lock = threading.RLock()
        self._last_cmd_reply = None
        self._last_pose = None
        self._last_pose_mono_s = None
        self._run_start_mono_s = 0.0
        self._node_ready = False
        self._mavros_state_topic = mavros_state_topic or ""
        self._last_mavros_state = None
        self._last_mavros_state_mono_s = None
        self._disarmed_since_mono_s = None
        if self._mavros_state_topic:
            from mavros_msgs.msg import State
            self._mavros_state_sub = ros.Subscriber(
                self._mavros_state_topic, State, self._on_mavros_state, queue_size=1
            )

        self._state_sub = ros.Subscriber(
            self.namespace + "/exec_state", String, self._on_state, queue_size=1
        )
        self._pose_sub = ros.Subscriber(
            self.namespace + "/local_pose", PoseStamped, self._on_pose, queue_size=1
        )
        self._goal_pub = ros.Publisher(
            self.namespace + "/goal", PointStamped, queue_size=1
        )
        self._waypoints_pub = ros.Publisher(
            self.namespace + "/waypoints", PolygonStamped, queue_size=1
        )
        self._hold_pub = ros.Publisher(
            self.namespace + "/hold", Empty, queue_size=1
        )
        from swarm_uav_interfaces.msg import UavTrajectoryIntent
        self._intent_type = intent_type or UavTrajectoryIntent
        self._neighbor_intent_pub = ros.Publisher(
            self.namespace + "/neighbor_intent", self._intent_type, queue_size=10)
        self._neighbor_subs = []
        for topic in _split_topics(neighbor_intents):
            self._neighbor_subs.append(ros.Subscriber(
                topic, self._intent_type, self._on_neighbor_intent, queue_size=10))
        self._ros.sleep(0.2)  # let advertisers connect before first command
        self._node_ready = self._ros.get_param(
            self.namespace + "/uav_id", None
        ) is not None

    def _on_state(self, msg):
        with self._lock:
            self._last_cmd_reply = msg.data
            self._node_ready = True

    def _on_neighbor_intent(self, msg):
        with self._lock:
            self._neighbor_intent_pub.publish(msg)

    def _on_pose(self, msg):
        with self._lock:
            self._last_pose = msg.pose
            self._last_pose_mono_s = self._monotonic_clock()

    def _on_mavros_state(self, msg):
        with self._lock:
            self._last_mavros_state = copy.deepcopy(msg)
            self._last_mavros_state_mono_s = self._monotonic_clock()
            if msg.armed:
                self._disarmed_since_mono_s = None
            elif self._disarmed_since_mono_s is None:
                self._disarmed_since_mono_s = self._monotonic_clock()

    def _publish_goal(self, goal):
        msg = PointStamped()
        msg.header.stamp = self._ros.Time.now()
        msg.point.x = goal.x
        msg.point.y = goal.y
        msg.point.z = goal.z
        self._goal_pub.publish(msg)

    def _publish_waypoints(self, goal):
        msg = PolygonStamped()
        msg.header.stamp = self._ros.Time.now()
        for wp in goal.waypoints:
            p = msg.polygon.points.add()
            p.x, p.y, p.z = wp[0], wp[1], wp[2]
        self._waypoints_pub.publish(msg)

    def _issue_hold(self):
        with self._lock:
            self._last_cmd_reply = None
        self._hold_pub.publish(Empty())

    def _wait_for_terminal(self, cancel_event, deadline, base_state=None):
        while not self._shutdown:
            if cancel_event is not None and cancel_event.is_set():
                return MotionResult(False, "COMMAND_HELD", "motion cancelled by HOLD")
            if self._monotonic_clock() >= float(deadline):
                return MotionResult(False, "LOCAL_TIMEOUT", "motion deadline exceeded")
            if base_state is not None and self._monotonic_clock() >= base_state + self.state_timeout_s:
                return MotionResult(False, "DRIVER_TIMEOUT", "no terminal exec_state within timeout")
            with self._lock:
                state = self._last_cmd_reply
            if state is None:
                time.sleep(1.0 / _MONITOR_HZ)
                continue
            if state in _TERMINAL_OK:
                return MotionResult(True, "", "egoswarm trajectory completed")
            if state in _TERMINAL_BAD:
                return MotionResult(False, state, "egoswarm planning/execution failure")
            if state == _STATE_HOLD:
                return MotionResult(False, "COMMAND_HELD", "egoswarm entered HOLD")
            time.sleep(1.0 / _MONITOR_HZ)
        return MotionResult(False, "SHUTTING_DOWN", "driver shutting down")

    def start_move_to(self, goal, cancel_event, deadline):
        with self._lock:
            self._last_cmd_reply = None
            self._run_start_mono_s = self._monotonic_clock()
        if goal.command == "FOLLOW_ROUTE":
            # P1 only supports the leader role: follower formation follow is
            # rejected at validation with NOT_IMPLEMENTED, so a FOLLOW_ROUTE
            # goal that reaches the driver always carries the route waypoints.
            if goal.waypoints:
                self._publish_waypoints(goal)
            else:
                self._publish_goal(goal)
        else:
            self._publish_goal(goal)
        base_state = self._run_start_mono_s
        return self._wait_for_terminal(cancel_event, deadline, base_state)

    def hold(self, goal: HoldGoal, deadline):
        self._issue_hold()
        # Synchronous confirm: wait until the node reports HOLD or a timeout.
        end = self._monotonic_clock() + self.state_timeout_s
        while self._monotonic_clock() < end and not self._shutdown:
            with self._lock:
                state = self._last_cmd_reply
            if state == _STATE_HOLD:
                return MotionResult(True, "", "egoswarm HOLD confirmed")
            time.sleep(1.0 / _MONITOR_HZ)
        return MotionResult(False, "HOLD_TIMEOUT", "egoswarm did not confirm HOLD")

    def health(self):
        with self._lock:
            ready = self._node_ready and not self._shutdown
            code = "" if ready else ("DRIVER_NOT_READY" if not self._node_ready else "SHUTTING_DOWN")
            message = "" if ready else "ego_planner_driver node not ready or shut down"
        return DriverHealth(ready, code, message)

    def can_end_safety_lease(self, disarmed_stable_s):
        # Base gate: fresh local pose and no active trajectory.
        with self._lock:
            pose_fresh = (self._last_pose_mono_s is not None and
                          self._monotonic_clock() - self._last_pose_mono_s <= self.pose_timeout_s)
            running = self._last_cmd_reply == _STATE_EXECUTING
            state = copy.deepcopy(self._last_mavros_state) if self._mavros_state_topic else None
            state_received = self._last_mavros_state_mono_s
            disarmed_since = self._disarmed_since_mono_s
        if not pose_fresh:
            return False, "local pose is stale for lease END gate"
        if running:
            return False, "trajectory still executing"
        # When a MAVROS state topic is configured, the END gate also requires
        # MAVROS to be connected, state fresh and the vehicle stably disarmed
        # (mirrors MavrosPositionDriver). Without it (pure ego smoke) the base
        # gate remains sufficient.
        if self._mavros_state_topic:
            if state is None:
                return False, "MAVROS state has not been received"
            now = self._monotonic_clock()
            if state_received is None or now - state_received > self.pose_timeout_s:
                return False, "MAVROS state is stale"
            if not state.connected:
                return False, "MAVROS is disconnected"
            if state.armed or disarmed_since is None:
                return False, "vehicle is armed"
            stable_for = now - disarmed_since
            if stable_for < float(disarmed_stable_s):
                return False, "vehicle disarm state is not yet stable"
            return True, "ego pose fresh, MAVROS connected and vehicle stably disarmed"
        return True, "ego pose fresh and trajectory not executing"

    def shutdown(self):
        self._shutdown = True

    @classmethod
    def from_ros_params(cls):
        return cls(
            namespace=rospy.get_param("~ego_swarm/namespace", "/UAV1"),
            state_timeout_s=rospy.get_param("~ego_swarm/state_timeout_s", 5.0),
            pos_tolerance_m=rospy.get_param("~ego_swarm/position_tolerance_m", 0.2),
            steady_s=rospy.get_param("~ego_swarm/arrival_stable_s", 1.0),
            pose_timeout_s=rospy.get_param("~ego_swarm/pose_timeout_s", 1.0),
            neighbor_intents=rospy.get_param("~ego_swarm/neighbor_intents", ""),
            mavros_state_topic=rospy.get_param("~ego_swarm/mavros_state_topic", ""),
        )
