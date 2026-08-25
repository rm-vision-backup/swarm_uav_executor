"""Ego-swarm driver bridging the ego_planner_driver C++ node.

The C++ node publishes a latched exec_state topic that this driver uses as
the single source of truth for the motion lifecycle:
  IDLE -> EXECUTING -> COMPLETED / EGO_PLAN_FAILED / EGO_EXEC_TIMEOUT / HOLD

  The driver owns arm/OFFBOARD preparation for motion commands.  The C++ node
  generates all trajectory setpoints (PX4 OFFBOARD compatible), except for the
  FOLLOW_ROUTE follower's PI position loop.

- MOVE_TO / FAULT_EXIT / FOLLOW_ROUTE(leader) / HOVER: ego real-time planning.
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
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool, Empty, Float64, String

from .base import MotionDriver
from ..models import DriverHealth, HoldGoal, MotionGoal, MotionResult

_STATE_EXECUTING = "EXECUTING"
_STATE_COMPLETED = "COMPLETED"
_STATE_HOLD = "HOLD"
_STATE_POSE_STALE = "POSE_STALE"
_STATE_PLAN_FAILED = "EGO_PLAN_FAILED"
_STATE_TIMEOUT = "EGO_EXEC_TIMEOUT"
_STATE_EMERGENCY_BRAKE = "EMERGENCY_BRAKE"
_STATE_BRAKE_HOLD = "BRAKE_HOLD"
_TERMINAL_OK = frozenset((_STATE_COMPLETED,))
_TERMINAL_BAD = frozenset((_STATE_POSE_STALE, _STATE_PLAN_FAILED, _STATE_TIMEOUT,
                           _STATE_BRAKE_HOLD))
_MONITOR_HZ = 20.0

_SETPOINT_HZ = 30.0
_FOLLOW_POSE_TIMEOUT_S = 1.0   # leader odom timeout -> LEADER_LOST
_FOLLOW_LOOP_HZ = 10.0         # follower PI control update rate

# PositionTarget 契约（与 setpoint_relay 校验一致）：
#   follower PI: position + velocity + yaw, ignore acceleration + yaw rate = 2496
#   HOLD       : position + yaw, ignore velocity + acceleration + yaw rate = 2552
_FOLLOWER_MASK = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                  PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
_HOLD_MASK = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
              PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
              PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
              PositionTarget.IGNORE_YAW_RATE)


def _split_topics(raw):
    if not raw:
        return []
    return [x.strip() for x in raw.split(',') if x.strip()]


def _default_neighbor_odom_topics(exec_target):
    own = str(exec_target or "").strip("/")
    return ",".join("/UAV%d/mavros/local_position/odom" % idx
                    for idx in range(1, 16) if "UAV%d" % idx != own)


class EgoSwarmDriver(MotionDriver):
    """MotionDriver over the ego_planner_driver binary via its topics."""

    def __init__(self, namespace="", state_timeout_s=200.0,
                 ros=rospy, monotonic_clock=time.monotonic,
                 pos_tolerance_m=0.2, steady_s=1.0,
                 pose_timeout_s=1.0, neighbor_intents='',
                 intent_type=None, mavros_state_topic="",
                 follower_p_gain=1.0, follower_i_gain=0.1,
                 follower_limit_xy=2.0, follower_limit_z=1.0,
                 formation_offsets=None, leader_odom_topic_prefix="",
                 arm_service="/mavros/cmd/arming", mode_service="/mavros/set_mode",
                 origin_confirmed_topic="/gp_origin_confirmed",
                 neighbor_odom_topics='', min_horizontal_distance_m=1.0,
                  min_vertical_distance_m=2.0, configure_px4_params=False,
                  px4_param_set_timeout_s=5.0,
                  px4_params=None,
                  layer_move_to=15.0, layer_follow_route=12.0,
                  layer_fault_exit=8.0, layer_tolerance_m=0.5,
                  waypoint_densify_spacing=2.0,
                  follower_setpoint_topic="/setpoint/follower"):
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
        self._origin_confirmed = False
        self._disarmed_since_mono_s = None
        self._configure_px4_params = bool(configure_px4_params)
        self._px4_param_set_timeout_s = float(px4_param_set_timeout_s)
        self._px4_params = dict(px4_params or {
            "COM_RCL_EXCEPT": 4,
            "NAV_RCL_ACT": 0,
        })
        if self._mavros_state_topic:
            from mavros_msgs.msg import State
            self._mavros_state_sub = ros.Subscriber(
                self._mavros_state_topic, State, self._on_mavros_state, queue_size=1
            )
        self._origin_confirmed_sub = ros.Subscriber(
            origin_confirmed_topic, Bool, self._on_origin_confirmed, queue_size=1)

        self._state_sub = ros.Subscriber(
            self.namespace + "/exec_state", String, self._on_state, queue_size=1
        )
        self._pose_sub = ros.Subscriber(
            self.namespace + "/local_pose", PoseStamped, self._on_pose, queue_size=1
        )
        self._goal_pub = ros.Publisher(
            self.namespace + "/goal", PointStamped, queue_size=1
        )
        self._goal_yaw_pub = ros.Publisher(
            self.namespace + "/goal_yaw", Float64, queue_size=1
        )
        self._waypoints_pub = ros.Publisher(
            self.namespace + "/waypoints", PolygonStamped, queue_size=1
        )
        self._hold_pub = ros.Publisher(
            self.namespace + "/hold", Empty, queue_size=1
        )
        # Follower PI candidate topic: PositionTarget (mask 2496). EGO trajectory
        # publishes /setpoint/ego; setpoint_relay arbitrates both sources and is
        # the sole owner of /mavros/setpoint_raw/local.
        self._follower_setpoint_topic = str(follower_setpoint_topic)
        self._setpoint_pub = ros.Publisher(
            self._follower_setpoint_topic, PositionTarget, queue_size=1
        )
        self._direct_control_pub = ros.Publisher(
            self.namespace + "/direct_control_active", Bool, queue_size=1)
        self._direct_control_pub.publish(Bool(False))
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
        self._formation_offsets = dict(formation_offsets or {})
        self._leader_odom_topic_prefix = str(leader_odom_topic_prefix or "").strip("/")
        self._leader_odom_sub = None
        self._leader_odom_topic = ""
        self._last_leader_pose = None
        self._last_leader_odom_mono_s = None
        self._neighbor_poses = {}
        self._neighbor_odom_subs = []
        from nav_msgs.msg import Odometry
        for topic in _split_topics(neighbor_odom_topics):
            self._neighbor_odom_subs.append(ros.Subscriber(
                topic, Odometry, self._on_neighbor_odom, callback_args=topic, queue_size=1))
        self._min_horizontal_distance_m = float(min_horizontal_distance_m)
        self._min_vertical_distance_m = float(min_vertical_distance_m)
        self._layer_move_to = float(layer_move_to)
        self._layer_follow_route = float(layer_follow_route)
        self._layer_fault_exit = float(layer_fault_exit)
        self._layer_tolerance_m = float(layer_tolerance_m)
        self._waypoint_densify_spacing = float(waypoint_densify_spacing)
        self._arm_service = arm_service
        self._mode_service = mode_service

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

    def _on_origin_confirmed(self, msg):
        with self._lock:
            self._origin_confirmed = bool(msg.data)

    def _on_leader_odom(self, msg):
        with self._lock:
            self._last_leader_pose = msg.pose.pose
            self._last_leader_odom_mono_s = self._monotonic_clock()

    def _on_neighbor_odom(self, msg, topic):
        with self._lock:
            self._neighbor_poses[topic] = (copy.deepcopy(msg.pose.pose), self._monotonic_clock())

    def _distance_safe(self, own_pose):
        now = self._monotonic_clock()
        with self._lock:
            neighbors = tuple(self._neighbor_poses.values())
        for pose, stamp in neighbors:
            if now - stamp > self.pose_timeout_s:
                continue
            horizontal = math.hypot(own_pose.position.x - pose.position.x,
                                    own_pose.position.y - pose.position.y)
            vertical = abs(own_pose.position.z - pose.position.z)
            if horizontal < self._min_horizontal_distance_m and vertical < self._min_vertical_distance_m:
                return False
        return True

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
        # Keep each intermediate waypoint for two B-spline control samples.
        # The terminal waypoint remains single so Ego still converges to the
        # requested endpoint instead of dwelling there as an intermediate knot.
        waypoints = []
        for index, wp in enumerate(goal.waypoints):
            waypoints.append(wp)
            if index < len(goal.waypoints) - 1:
                waypoints.append(wp)
        for wp in waypoints:
            p = Point32()
            p.x, p.y, p.z = wp[0], wp[1], wp[2]
            msg.polygon.points.append(p)
        self._waypoints_pub.publish(msg)

    def _publish_waypoints_from_list(self, points):
        """Publish an already-densified ENU waypoint list to the C++ node."""
        msg = PolygonStamped()
        msg.header.stamp = self._ros.Time.now()
        for point in points:
            p = Point32()
            p.x, p.y, p.z = point[0], point[1], point[2]
            msg.polygon.points.append(p)
        self._waypoints_pub.publish(msg)

    def _last_position_enu(self):
        with self._lock:
            if self._last_pose is None:
                return None
            pos = self._last_pose.position
            return (pos.x, pos.y, pos.z)

    def _build_layered_keypoints(self, goal, start):
        """Build command-specific layered keypoints (vertical/horizontal/vertical).

        `start` (current ENU pose, read once before dispatch while HOLD) is
        included as the first keypoint so the vertical climb/drop segments get
        intermediate points after densification — without it the B-spline
        smooths the whole climb into a slanted segment.

        Returns [] when the goal is already reachable on the target layer or
        the command has no layering semantics (HOVER).
        """
        if start is None:
            return []
        sx, sy, sz = start
        gx, gy, gz = goal.x, goal.y, goal.z
        if goal.command == "MOVE_TO":
            layer = self._layer_move_to
        elif goal.command == "FAULT_EXIT":
            layer = self._layer_fault_exit
        elif goal.command == "FOLLOW_ROUTE":
            layer = self._layer_follow_route
        else:
            return []
        points = [start]
        if abs(sz - layer) > self._layer_tolerance_m:
            points.append((sx, sy, layer))
        if goal.command == "FOLLOW_ROUTE" and goal.waypoints:
            for waypoint in goal.waypoints:
                points.append((float(waypoint[0]), float(waypoint[1]), layer))
            if abs(gz - layer) > self._layer_tolerance_m:
                points.append((gx, gy, gz))
        else:
            points.append((gx, gy, layer))
            if abs(gz - layer) > self._layer_tolerance_m:
                points.append((gx, gy, gz))
        return points

    def _densify_waypoints(self, points, max_spacing):
        """Uniformly interpolate so every adjacent pair is at most max_spacing
        apart.  Ported from verification/plot_ego_bspline_waypoints.py."""
        if max_spacing <= 0.0 or len(points) < 2:
            return [tuple(point) for point in points]
        dense = [tuple(points[0])]
        for left, right in zip(points, points[1:]):
            lx, ly, lz = left
            rx, ry, rz = right
            distance = math.sqrt((rx - lx) ** 2 + (ry - ly) ** 2 + (rz - lz) ** 2)
            segments = max(1, int(math.ceil(distance / max_spacing)))
            for index in range(1, segments + 1):
                t = index / segments
                dense.append((lx + (rx - lx) * t,
                              ly + (ry - ly) * t,
                              lz + (rz - lz) * t))
        return dense

    def _publish_setpoint(self, pose):
        """Publish a frozen position/yaw HOLD candidate (PositionTarget mask 2552)."""
        msg = PositionTarget()
        msg.header.stamp = self._ros.Time.now()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = _HOLD_MASK
        msg.position = copy.deepcopy(pose.position)
        msg.yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z +
                   pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
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
            # 执行层运行时距离门禁（min-snap 重构）：EGO 巡航期间邻机过近
            # （水平 < min_horizontal_distance_m 且垂直 < min_vertical_distance_m）
            # -> 本机 HOLD 暂停，任务以 FAILED/MIN_DISTANCE_BREACH 收口（避碰
            # 失败兜底，不自动恢复）。复用 _distance_safe（follower 同款逻辑）。
            with self._lock:
                own_pose = self._last_pose
            if own_pose is not None and not self._distance_safe(own_pose):
                self._issue_hold()
                return MotionResult(False, "MIN_DISTANCE_BREACH",
                                    "neighbor distance breached during ego cruise")
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

    def _plan_horizontal(self, goal, cancel, deadline):
        with self._lock:
            self._last_cmd_reply = None
            self._run_start_mono_s = self._monotonic_clock()
        self._goal_yaw_pub.publish(Float64(goal.yaw))
        # 方案 B 重构（min-snap）：非 follower 自主动作先构造含动作分层的关键点序列
        # 并按最大 waypoint_densify_spacing（2m）均匀密化，作为 waypoints 发布给 C++；
        # 2m 密化点只用于 C++ 窗口推进（advanceConsumed）/末点参考，不进 B-spline——
        # B-spline 初始 point_set 由 C++ 侧 min-snap 单段多项式按 0.4m 弧长等距采样生成。
        # HOVER 保持原有冻结语义。
        if goal.command in ("MOVE_TO", "FAULT_EXIT") or (
                goal.command == "FOLLOW_ROUTE" and not goal.formation_follow):
            start = self._last_position_enu()
            keypoints = self._build_layered_keypoints(goal, start)
            if keypoints:
                dense = self._densify_waypoints(
                    keypoints, self._waypoint_densify_spacing)
                self._publish_waypoints_from_list(dense)
            else:
                self._publish_goal(goal)
        elif goal.waypoints:
            self._publish_waypoints(goal)
        else:
            self._publish_goal(goal)
        return self._wait_for_terminal(cancel, deadline, self._run_start_mono_s)

    def _follower_loop(self, goal, cancel, deadline):
        """Track leader odom + formation offset with position-loop PI control."""
        leader_id = goal.leader_id
        # 编队信息由 GCS_A 随任务下发（goal.formation_offset）；机载不再存编队
        # 配置，仅当 GCS_A 未下发（全零占位）时回退本机配置作兼容。
        offset = goal.formation_offset
        if offset is None or all(abs(v) < 1e-9 for v in offset):
            offset = self._formation_offsets.get(leader_id)
        if offset is None:
            offset = (0.0, 0.0, 0.0)
        self._ensure_leader_odom_sub(leader_id)
        integral = [0.0, 0.0, 0.0]
        loop_dt = 1.0 / _FOLLOW_LOOP_HZ
        publish_dt = 1.0 / _SETPOINT_HZ
        last_calc = None
        last_setpoint = None
        arrived_since = None
        self._direct_control_pub.publish(Bool(True))
        try:
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
                if not self._distance_safe(own_pose):
                    return MotionResult(False, "MIN_DISTANCE_BREACH",
                                        "neighbor distance breached during follower PI control")
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
                    k_xy = math.hypot(vel[0], vel[1])
                    if k_xy > self._follower_limit_xy:
                        scale = self._follower_limit_xy / k_xy
                        vel[0] *= scale
                        vel[1] *= scale
                    if abs(vel[2]) > self._follower_limit_z:
                        vel[2] = math.copysign(self._follower_limit_z, vel[2])
                    # PositionTarget 候选：position=leader+offset（权威目标），
                    # velocity=限幅后的 PI 输出（显式速度前馈），yaw=冻结参考航向。
                    setpoint = PositionTarget()
                    setpoint.header.stamp = self._ros.Time.now()
                    setpoint.header.frame_id = "map"
                    setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    setpoint.type_mask = _FOLLOWER_MASK
                    setpoint.position.x, setpoint.position.y, setpoint.position.z = target
                    setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z = \
                        vel[0], vel[1], vel[2]
                    setpoint.yaw = goal.yaw
                    last_setpoint = setpoint
                    last_calc = now
                if last_setpoint is not None:
                    self._setpoint_pub.publish(last_setpoint)
                target_pose = type("PoseLike", (), {"position": type(
                    "PosLike", (), {"x": target[0], "y": target[1], "z": target[2]})()})()
                leader_final = type("PoseLike", (), {"position": type(
                    "PosLike", (), {"x": goal.x, "y": goal.y, "z": goal.z})()})()
                if self._near(own_pose, target_pose) and self._near(leader_pose, leader_final):
                    if arrived_since is None:
                        arrived_since = now
                    elif now - arrived_since >= self.steady_s:
                        self._emit_completed()
                        return MotionResult(True, "", "follower formation reached")
                else:
                    arrived_since = None
                time.sleep(publish_dt)
        finally:
            self._direct_control_pub.publish(Bool(False))
        return MotionResult(False, "SHUTTING_DOWN", "driver shutting down")

    def _near(self, own, target):
        dx = own.position.x - target.position.x
        dy = own.position.y - target.position.y
        dz = own.position.z - target.position.z
        return math.hypot(math.hypot(dx, dy), dz) <= self.pos_tolerance_m

    def start_move_to(self, goal, cancel_event, deadline):
        if goal.command == "HOVER":
            return self._plan_horizontal(goal, cancel_event, deadline)
        if goal.command == "FOLLOW_ROUTE" and goal.formation_follow:
            return self._follower_loop(goal, cancel_event, deadline)
        return self._plan_horizontal(goal, cancel_event, deadline)

    def hold(self, goal: HoldGoal, deadline):
        # Already holding (HOLD or BRAKE_HOLD after emergency braking): confirm
        # immediately without clearing _last_cmd_reply or republishing, so a
        # safety HOLD cannot be blocked by the confirm loop itself.
        with self._lock:
            state = self._last_cmd_reply
        if state in (_STATE_HOLD, _STATE_BRAKE_HOLD):
            return MotionResult(True, "", "egoswarm already holding")
        self._issue_hold()
        # Bounded confirm: wait at most min(caller deadline, state_timeout_s).
        # Ignoring the caller deadline used to block for state_timeout_s (200s)
        # when the node never confirmed, stalling the whole task entry.
        now = self._monotonic_clock()
        timeout_s = float(self.state_timeout_s)
        if deadline is not None:
            timeout_s = min(timeout_s, max(0.0, float(deadline) - now))
        end = now + timeout_s
        while self._monotonic_clock() < end and not self._shutdown:
            with self._lock:
                state = self._last_cmd_reply
            if state in (_STATE_HOLD, _STATE_BRAKE_HOLD):
                return MotionResult(True, "", "egoswarm HOLD confirmed")
            time.sleep(1.0 / _MONITOR_HZ)
        return MotionResult(False, "HOLD_TIMEOUT", "egoswarm did not confirm HOLD")

    def health(self):
        with self._lock:
            ready = self._node_ready and not self._shutdown
            code = "" if ready else ("DRIVER_NOT_READY" if not self._node_ready else "SHUTTING_DOWN")
            message = "" if ready else "ego_planner_driver node not ready or shut down"
        return DriverHealth(ready, code, message)

    def prepare(self, goal):
        base = self.health()
        if not base.ready:
            return base
        with self._lock:
            pose_fresh = (self._last_pose_mono_s is not None and
                          self._monotonic_clock() - self._last_pose_mono_s <= self.pose_timeout_s)
            state = copy.deepcopy(self._last_mavros_state)
            state_fresh = (self._last_mavros_state_mono_s is not None and
                           self._monotonic_clock() - self._last_mavros_state_mono_s <= self.pose_timeout_s)
            origin_confirmed = self._origin_confirmed
        if not pose_fresh:
            return DriverHealth(False, "POSE_STALE", "local pose is not fresh")
        if self._mavros_state_topic:
            if state is None or not state_fresh or not state.connected:
                return DriverHealth(False, "MAVROS_NOT_READY", "MAVROS state is stale or disconnected")
            if not state.armed:
                return DriverHealth(False, "VEHICLE_NOT_ARMED",
                                    "%s requires an already armed vehicle" % goal.command)
            if state.mode != "OFFBOARD":
                return DriverHealth(False, "VEHICLE_NOT_OFFBOARD",
                                    "%s requires an already OFFBOARD vehicle" % goal.command)
            if goal.command == "MOVE_TO" and not origin_confirmed:
                return DriverHealth(False, "ORIGIN_NOT_CONFIRMED", "global origin has not been confirmed")
        return DriverHealth(True, "", "arm-before checks passed")

    def shutdown(self):
        self._shutdown = True

    @classmethod
    def from_ros_params(cls):
        exec_target = rospy.get_param("/exec_target", "")
        neighbor_odom_topics = rospy.get_param(
            "~ego_swarm/neighbor_odom_topics", "")
        if not neighbor_odom_topics:
            neighbor_odom_topics = _default_neighbor_odom_topics(exec_target)
        return cls(
            namespace=rospy.get_param("~ego_swarm/namespace", ""),
            state_timeout_s=rospy.get_param("~ego_swarm/state_timeout_s", 200.0),
            pos_tolerance_m=rospy.get_param("~ego_swarm/position_tolerance_m", 0.2),
            steady_s=rospy.get_param("~ego_swarm/arrival_stable_s", 1.0),
            pose_timeout_s=rospy.get_param("~ego_swarm/pose_timeout_s", 1.0),
            neighbor_intents=rospy.get_param("~ego_swarm/neighbor_intents", ""),
            mavros_state_topic=rospy.get_param("~ego_swarm/mavros_state_topic", ""),
            follower_p_gain=rospy.get_param("~ego_swarm/follower_p_gain", 1.0),
            follower_i_gain=rospy.get_param("~ego_swarm/follower_i_gain", 0.1),
            follower_limit_xy=rospy.get_param("~ego_swarm/follower_limit_xy", 2.0),
            follower_limit_z=rospy.get_param("~ego_swarm/follower_limit_z", 1.0),
            formation_offsets=rospy.get_param("~ego_swarm/formation_offsets", {}),
            leader_odom_topic_prefix=rospy.get_param("~ego_swarm/leader_odom_topic_prefix", ""),
            layer_move_to=rospy.get_param("~ego_swarm/layer_move_to", 15.0),
            layer_follow_route=rospy.get_param("~ego_swarm/layer_follow_route", 12.0),
            layer_fault_exit=rospy.get_param("~ego_swarm/layer_fault_exit", 8.0),
            layer_tolerance_m=rospy.get_param("~ego_swarm/layer_tolerance_m", 0.5),
            waypoint_densify_spacing=rospy.get_param(
                "~ego_swarm/waypoint_densify_spacing", 2.0),
            follower_setpoint_topic=rospy.get_param(
                "~ego_swarm/follower_setpoint_topic", "/setpoint/follower"),
            arm_service=rospy.get_param("~ego_swarm/arm_service", "/mavros/cmd/arming"),
            mode_service=rospy.get_param("~ego_swarm/mode_service", "/mavros/set_mode"),
            origin_confirmed_topic=rospy.get_param("~ego_swarm/origin_confirmed_topic", "/gp_origin_confirmed"),
            neighbor_odom_topics=neighbor_odom_topics,
            min_horizontal_distance_m=rospy.get_param("~ego_swarm/min_horizontal_distance_m", 1.0),
            min_vertical_distance_m=rospy.get_param("~ego_swarm/min_vertical_distance_m", 2.0),
            configure_px4_params=rospy.get_param(
                "~ego_swarm/configure_px4_params", False),
            px4_param_set_timeout_s=rospy.get_param(
                "~ego_swarm/px4_param_set_timeout_s", 5.0),
            px4_params={
                "COM_RCL_EXCEPT": rospy.get_param(
                    "~ego_swarm/com_rcl_except", 4),
                "NAV_RCL_ACT": rospy.get_param(
                    "~ego_swarm/nav_rcl_act", 0),
            },
        )
