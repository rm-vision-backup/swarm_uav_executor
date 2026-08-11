"""Ego-swarm driver bridging the ego_planner_driver C++ node.

The C++ node publishes a latched exec_state topic that this driver uses as
the single source of truth for the motion lifecycle:
  IDLE -> EXECUTING -> COMPLETED / EGO_PLAN_FAILED / EGO_EXEC_TIMEOUT / HOLD

The driver itself never arms the vehicle or switches flight mode; the
underlying node only publishes position setpoints (PX4 OFFBOARD compatible).

Command semantics (four full action commands):
- MOVE_TO / FAULT_EXIT / FOLLOW_ROUTE(leader): ego real-time planning. A pure
  vertical transition to the command's height layer runs first when the local
  altitude differs from the target layer, then the horizontal goal/waypoints.
- HOVER: freeze the current local pose at command receipt and republish that
  frozen pose on the setpoint topic; no ego planning, no horizontal motion.
- FOLLOW_ROUTE(follower): track leader odom + formation offset with a
  position-loop PI controller publishing setpoints; no ego planning.
"""
from __future__ import annotations

import copy
import math
import re
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseStamped
from std_msgs.msg import Empty, String

from .base import MotionDriver
from ..models import DriverHealth, HoldGoal, MotionGoal, MotionResult

_STATE_EXECUTING = "EXECUTING"
_STATE_COMPLETED = "COMPLETED"
_STATE_HOLD = "HOLD"
_STATE_POSE_STALE = "POSE_STALE"
_STATE_PLAN_FAILED = "EGO_PLAN_FAILED"
_STATE_TIMEOUT = "EGO_EXEC_TIMEOUT"
_TERMINAL_OK = frozenset((_STATE_COMPLETED,))
_TERMINAL_BAD = frozenset((_STATE_POSE_STALE, _STATE_PLAN_FAILED, _STATE_TIMEOUT))
_MONITOR_HZ = 20.0

_LAYER_MOVE_TO = 15.0
_LAYER_FOLLOW_ROUTE = 12.0
_LAYER_FAULT_EXIT = 8.0
_LAYER_TOL = 0.5          # close to the target layer: skip the vertical transition
_HOVER_SETPOINT_HZ = 30.0
_FOLLOW_POSE_TIMEOUT_S = 1.0   # leader odom timeout -> LEADER_LOST
_FOLLOW_LOOP_HZ = 10.0         # follower PI control update rate


def _split_topics(raw):
    if not raw:
        return []
    return [x.strip() for x in raw.split(',') if x.strip()]


class EgoSwarmDriver(MotionDriver):
    """MotionDriver over the ego_planner_driver binary via its topics."""

    def __init__(self, namespace="", state_timeout_s=5.0,
                 ros=rospy, monotonic_clock=time.monotonic,
                 pos_tolerance_m=0.2, steady_s=1.0,
                 pose_timeout_s=1.0, neighbor_intents='',
                 intent_type=None, mavros_state_topic="",
                 follower_p_gain=1.0, follower_i_gain=0.1,
                 follower_limit_xy=2.0, follower_limit_z=1.0,
                 layer_move_to=15.0, layer_follow_route=12.0, layer_fault_exit=8.0,
                 formation_offsets=None, leader_odom_topic_prefix=""):
        # Onboard premise: this node normally runs without a namespace prefix
        # (like MAVROS /mavros/*), so an empty namespace publishes to plain
        # /setpoint /exec_state etc. A non-empty namespace (e.g. "UAV1") is
        # preserved for backward-compatible single-master multi-UAV setups.
        raw = str(namespace or "").strip("/")
        self.namespace = ("/" + raw) if raw else ""
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
        # 30 Hz position setpoint output used by HOVER and FOLLOW_ROUTE follower.
        self._setpoint_pub = ros.Publisher(
            self.namespace + "/setpoint", PoseStamped, queue_size=1
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
        # Identity parameter is placed at the global scope by the onboard
        # launch (no namespace), so check "/uav_id" when namespace is empty.
        self._node_ready = self._ros.get_param(
            (self.namespace + "/uav_id") if self.namespace else "/uav_id", None
        ) is not None

        # Follower formation-follow state.
        self._follower_p_gain = float(follower_p_gain)
        self._follower_i_gain = float(follower_i_gain)
        self._follower_limit_xy = float(follower_limit_xy)
        self._follower_limit_z = float(follower_limit_z)
        self._layer_move_to = float(layer_move_to)
        self._layer_follow_route = float(layer_follow_route)
        self._layer_fault_exit = float(layer_fault_exit)
        self._formation_offsets = dict(formation_offsets or {})
        self._leader_odom_topic_prefix = str(leader_odom_topic_prefix or "").strip("/")
        self._leader_odom_sub = None
        self._leader_odom_topic = ""
        self._last_leader_pose = None
        self._last_leader_odom_mono_s = None

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

    def _on_leader_odom(self, msg):
        with self._lock:
            self._last_leader_pose = msg.pose.pose
            self._last_leader_odom_mono_s = self._monotonic_clock()

    def _leader_odom_topic_for(self, leader_id):
        if self._leader_odom_topic_prefix:
            return "/%s/mavros/local_position/odom" % self._leader_odom_topic_prefix
        digits = re.sub(r"\D", "", str(leader_id or ""))
        leader_ns = "UAV" + (str(int(digits)) if digits else "0")
        return "/%s/mavros/local_position/odom" % leader_ns

    def _ensure_leader_odom_sub(self, leader_id):
        topic = self._leader_odom_topic_for(leader_id)
        if self._leader_odom_sub is not None and self._leader_odom_topic == topic:
            return
        from nav_msgs.msg import Odometry
        self._leader_odom_sub = self._ros.Subscriber(
            topic, Odometry, self._on_leader_odom, queue_size=1)
        self._leader_odom_topic = topic

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

    def _publish_setpoint(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self._ros.Time.now()
        msg.pose = copy.deepcopy(pose)
        self._setpoint_pub.publish(msg)

    def _issue_hold(self):
        with self._lock:
            self._last_cmd_reply = None
        self._hold_pub.publish(Empty())

    def _emit_completed(self):
        with self._lock:
            self._last_cmd_reply = _STATE_COMPLETED

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

    def _layer_for_goal(self, goal):
        if goal.command == "FAULT_EXIT":
            return self._layer_fault_exit
        if goal.command == "FOLLOW_ROUTE":
            return self._layer_follow_route
        return self._layer_move_to

    def _plan_vertical_transition(self, layer_z, cancel, deadline):
        """Pure vertical transition to the command layer when far from it."""
        with self._lock:
            pose = self._last_pose
        if pose is None:
            return MotionResult(False, "POSE_STALE", "no local pose for vertical transition")
        cur_z = pose.position.z
        cur_x = pose.position.x
        cur_y = pose.position.y
        if abs(cur_z - layer_z) <= _LAYER_TOL:
            return MotionResult(True, "", "already at target layer")
        vertical_goal = MotionGoal(cur_x, cur_y, layer_z, 0.0, command="MOVE_TO")
        return self._plan_horizontal(vertical_goal, cancel, deadline)

    def _plan_horizontal(self, goal, cancel, deadline):
        with self._lock:
            self._last_cmd_reply = None
            self._run_start_mono_s = self._monotonic_clock()
        if goal.waypoints:
            self._publish_waypoints(goal)
        else:
            self._publish_goal(goal)
        return self._wait_for_terminal(cancel, deadline, self._run_start_mono_s)

    def _hover_loop(self, goal, cancel, deadline):
        """Freeze the current local pose at HOVER receipt and republish it."""
        with self._lock:
            pose = self._last_pose
        if pose is None:
            return MotionResult(False, "POSE_STALE", "no local pose to freeze for HOVER")
        captured = copy.deepcopy(pose)
        arrived_since = None
        rate = 1.0 / _HOVER_SETPOINT_HZ
        while not self._shutdown:
            if cancel is not None and cancel.is_set():
                return MotionResult(False, "COMMAND_HELD", "motion cancelled by HOLD")
            if self._monotonic_clock() >= float(deadline):
                return MotionResult(False, "LOCAL_TIMEOUT", "motion deadline exceeded")
            self._publish_setpoint(captured)
            now = self._monotonic_clock()
            with self._lock:
                own = self._last_pose
            if own is not None and self._near(own, captured):
                if arrived_since is None:
                    arrived_since = now
                elif now - arrived_since >= self.steady_s:
                    self._emit_completed()
                    return MotionResult(True, "", "hover frozen setpoint reached")
            else:
                arrived_since = None
            time.sleep(rate)
        return MotionResult(False, "SHUTTING_DOWN", "driver shutting down")

    def _follower_loop(self, goal, cancel, deadline):
        """Track leader odom + formation offset with position-loop PI control."""
        leader_id = goal.leader_id
        offset = self._formation_offsets.get(leader_id)
        if offset is None:
            offset = goal.formation_offset
        self._ensure_leader_odom_sub(leader_id)
        integral = [0.0, 0.0, 0.0]
        loop_dt = 1.0 / _FOLLOW_LOOP_HZ
        publish_dt = 1.0 / _HOVER_SETPOINT_HZ
        last_calc = None
        last_setpoint = None
        arrived_since = None
        while not self._shutdown:
            if cancel is not None and cancel.is_set():
                return MotionResult(False, "COMMAND_HELD", "motion cancelled by HOLD")
            if self._monotonic_clock() >= float(deadline):
                return MotionResult(False, "LOCAL_TIMEOUT", "motion deadline exceeded")
            now = self._monotonic_clock()
            with self._lock:
                leader_pose = self._last_leader_pose
                leader_ts = self._last_leader_odom_mono_s
                own_pose = self._last_pose
            if leader_pose is None or leader_ts is None or now - leader_ts > _FOLLOW_POSE_TIMEOUT_S:
                return MotionResult(False, "LEADER_LOST", "leader odom timed out")
            if own_pose is None:
                return MotionResult(False, "POSE_STALE", "no local pose for follower loop")
            target = (leader_pose.position.x + offset[0],
                      leader_pose.position.y + offset[1],
                      leader_pose.position.z + offset[2])
            if last_calc is None or now - last_calc >= loop_dt:
                error = (target[0] - own_pose.position.x,
                         target[1] - own_pose.position.y,
                         target[2] - own_pose.position.z)
                integral = [integral[i] + error[i] * loop_dt for i in range(3)]
                vel = [self._follower_p_gain * error[i] + self._follower_i_gain * integral[i]
                       for i in range(3)]
                # Clamp the horizontal compensation and the vertical component.
                k_xy = math.hypot(vel[0], vel[1])
                if k_xy > self._follower_limit_xy:
                    scale = self._follower_limit_xy / k_xy
                    vel[0] *= scale
                    vel[1] *= scale
                if abs(vel[2]) > self._follower_limit_z:
                    vel[2] = math.copysign(self._follower_limit_z, vel[2])
                setpoint = PoseStamped()
                setpoint.header.stamp = self._ros.Time.now()
                setpoint.pose.position.x = own_pose.position.x + vel[0] * loop_dt
                setpoint.pose.position.y = own_pose.position.y + vel[1] * loop_dt
                setpoint.pose.position.z = own_pose.position.z + vel[2] * loop_dt
                setpoint.pose.orientation = own_pose.orientation
                last_setpoint = setpoint
                last_calc = now
            if last_setpoint is not None:
                self._setpoint_pub.publish(last_setpoint)
            target_pose = type("PoseLike", (), {"position": type(
                "PosLike", (), {"x": target[0], "y": target[1], "z": target[2]})()})()
            if self._near(own_pose, target_pose):
                if arrived_since is None:
                    arrived_since = now
                elif now - arrived_since >= self.steady_s:
                    self._emit_completed()
                    return MotionResult(True, "", "follower formation reached")
            else:
                arrived_since = None
            time.sleep(publish_dt)
        return MotionResult(False, "SHUTTING_DOWN", "driver shutting down")

    @staticmethod
    def _near(own, target):
        dx = own.position.x - target.position.x
        dy = own.position.y - target.position.y
        dz = own.position.z - target.position.z
        return math.hypot(math.hypot(dx, dy), dz) <= 0.5

    def start_move_to(self, goal, cancel_event, deadline):
        if goal.command == "HOVER":
            return self._hover_loop(goal, cancel_event, deadline)
        if goal.command == "FOLLOW_ROUTE" and goal.formation_follow:
            return self._follower_loop(goal, cancel_event, deadline)
        if goal.command in ("MOVE_TO", "FAULT_EXIT", "FOLLOW_ROUTE"):
            # Command height layers: vertical-first, then the horizontal plan.
            layer_z = self._layer_for_goal(goal)
            result = self._plan_vertical_transition(layer_z, cancel_event, deadline)
            if not result.success:
                return result
            return self._plan_horizontal(goal, cancel_event, deadline)
        return self._plan_horizontal(goal, cancel_event, deadline)

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
            namespace=rospy.get_param("~ego_swarm/namespace", ""),
            state_timeout_s=rospy.get_param("~ego_swarm/state_timeout_s", 5.0),
            pos_tolerance_m=rospy.get_param("~ego_swarm/position_tolerance_m", 0.2),
            steady_s=rospy.get_param("~ego_swarm/arrival_stable_s", 1.0),
            pose_timeout_s=rospy.get_param("~ego_swarm/pose_timeout_s", 1.0),
            neighbor_intents=rospy.get_param("~ego_swarm/neighbor_intents", ""),
            mavros_state_topic=rospy.get_param("~ego_swarm/mavros_state_topic", ""),
            follower_p_gain=rospy.get_param("~ego_swarm/follower_p_gain", 1.0),
            follower_i_gain=rospy.get_param("~ego_swarm/follower_i_gain", 0.1),
            follower_limit_xy=rospy.get_param("~ego_swarm/follower_limit_xy", 2.0),
            follower_limit_z=rospy.get_param("~ego_swarm/follower_limit_z", 1.0),
            layer_move_to=rospy.get_param("~ego_swarm/layer_move_to", 15.0),
            layer_follow_route=rospy.get_param("~ego_swarm/layer_follow_route", 12.0),
            layer_fault_exit=rospy.get_param("~ego_swarm/layer_fault_exit", 8.0),
            formation_offsets=rospy.get_param("~ego_swarm/formation_offsets", {}),
            leader_odom_topic_prefix=rospy.get_param("~ego_swarm/leader_odom_topic_prefix", ""),
        )