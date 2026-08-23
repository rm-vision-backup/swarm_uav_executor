"""setpoint_relay 纯仲裁/缓存类（无 ROS 依赖，可单测）。

两个业务发布者（EGO 轨迹、follower PI）只发布 PositionTarget 候选，本类：
- 按 `direct_control_active`（false 选 EGO，true 选 follower）选择源；
- 模式切换递增 generation 并作废新选中源旧缓存，只接受切换后收到的消息；
- 校验 coordinate_frame、type_mask 与启用字段 finite；
- 选中源过期/无效时输出固定 position/yaw HOLD（锁存一次 fresh local pose），
  不自动回退未选中源，不生成 (0,0,0) 原点目标。
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

# Contract masks (mavros_msgs constants):
#   EGO trajectory : position + velocity + acceleration + yaw, ignore yaw rate = 2048
#   EGO HOLD       : position + yaw, ignore velocity + acceleration + yaw rate = 2552
#   follower PI    : position + velocity + yaw, ignore acceleration + yaw rate = 2496
#   relay HOLD     : same as EGO HOLD (2552)
MASK_EGO_TRAJECTORY = PositionTarget.IGNORE_YAW_RATE
MASK_HOLD = (
    PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
    PositionTarget.IGNORE_YAW_RATE)
MASK_FOLLOWER = (
    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
    PositionTarget.IGNORE_YAW_RATE)

# Per-source accepted masks: EGO may publish both trajectory and its own HOLD
# (TAKEOFF/steady/COMPLETED), follower only the PI velocity candidate.
_ALLOWED_MASKS = {
    "ego": (MASK_EGO_TRAJECTORY, MASK_HOLD),
    "follower": (MASK_FOLLOWER,),
}

FRAME_LOCAL_NED = PositionTarget.FRAME_LOCAL_NED


def _fields_finite(msg: PositionTarget, mask: int) -> bool:
    """Finite check restricted to the fields enabled by the mask.

    Ignored fields are excluded from the check (the publisher still zeroes
    them for auditability).  Position and yaw are enabled in all contracts.
    """
    for value in (msg.position.x, msg.position.y, msg.position.z):
        if not math.isfinite(value):
            return False
    if not (mask & PositionTarget.IGNORE_VX) and not math.isfinite(msg.velocity.x):
        return False
    if not (mask & PositionTarget.IGNORE_VY) and not math.isfinite(msg.velocity.y):
        return False
    if not (mask & PositionTarget.IGNORE_VZ) and not math.isfinite(msg.velocity.z):
        return False
    if not (mask & PositionTarget.IGNORE_AFX) and not math.isfinite(msg.acceleration_or_force.x):
        return False
    if not (mask & PositionTarget.IGNORE_AFY) and not math.isfinite(msg.acceleration_or_force.y):
        return False
    if not (mask & PositionTarget.IGNORE_AFZ) and not math.isfinite(msg.acceleration_or_force.z):
        return False
    if not (mask & PositionTarget.IGNORE_YAW) and not math.isfinite(msg.yaw):
        return False
    return True


class SetpointRelay:
    """Arbitrates EGO/follower candidates and produces the sole raw-local output.

    Callbacks only cache messages with monotonic receive time; the 30 Hz ROS
    timer calls `tick()` which performs selection, freshness/finite checks and
    builds the output (or None when no safe target exists yet).
    """

    def __init__(self, candidate_timeout_s: float = 0.2,
                 local_pose_timeout_s: float = 1.0,
                 monotonic_clock=time.monotonic):
        self._clock = monotonic_clock
        self._candidate_timeout_s = float(candidate_timeout_s)
        self._pose_timeout_s = float(local_pose_timeout_s)
        self._selected = "ego"
        self._generation = {"ego": 0, "follower": 0}
        # source -> (msg, received_mono, generation)
        self._cache: dict[str, Tuple[PositionTarget, float, int]] = {}
        self._pose: Optional[Tuple[PoseStamped, float]] = None
        self._hold: Optional[Tuple[float, float, float, float]] = None  # active lock
        self._last_hold: Optional[Tuple[float, float, float, float]] = None  # verified lock
        self._error_since_mono = 0.0

    # --- callbacks ---

    def on_mode(self, select_follower: bool) -> None:
        new = "follower" if select_follower else "ego"
        if new != self._selected:
            self._selected = new
            self._generation[new] += 1
            self._cache.pop(new, None)
            # Re-enter HOLD at the current pose until a fresh post-switch
            # candidate arrives; last verified hold stays as a fallback.
            self._hold = None

    def on_candidate(self, source: str, msg: PositionTarget,
                     now: Optional[float] = None) -> bool:
        if now is None:
            now = self._clock()
        if source not in _ALLOWED_MASKS:
            return False
        if msg.coordinate_frame != FRAME_LOCAL_NED:
            return False
        if msg.type_mask not in _ALLOWED_MASKS[source]:
            return False
        if not _fields_finite(msg, msg.type_mask):
            return False
        self._cache[source] = (msg, now, self._generation[source])
        return True

    def on_pose(self, pose: PoseStamped, now: Optional[float] = None) -> None:
        if now is None:
            now = self._clock()
        self._pose = (pose, now)

    # --- tick ---

    def tick(self, now: Optional[float] = None, stamp=None) -> Optional[PositionTarget]:
        """Returns the output frame or None when no safe target exists.

        `stamp` is copied verbatim into the output header (the ROS node passes
        ros::Time::now(); tests pass a scalar).
        """
        if now is None:
            now = self._clock()
        cached = self._cache.get(self._selected)
        if cached is not None and cached[2] >= self._generation[self._selected] \
                and (now - cached[1]) <= self._candidate_timeout_s:
            # Valid fresh candidate from the selected source.
            self._hold = None  # exit internal HOLD
            msg = cached[0]
            out = self._from_candidate(msg)
        else:
            out = self._hold_output(now)
        if out is not None:
            out.header.stamp = stamp if stamp is not None else 0
            out.header.frame_id = "map"
        return out

    def _from_candidate(self, msg: PositionTarget) -> PositionTarget:
        out = PositionTarget()
        out.coordinate_frame = FRAME_LOCAL_NED
        out.type_mask = msg.type_mask
        out.position.x, out.position.y, out.position.z = \
            msg.position.x, msg.position.y, msg.position.z
        out.velocity.x, out.velocity.y, out.velocity.z = \
            msg.velocity.x, msg.velocity.y, msg.velocity.z
        out.acceleration_or_force.x, out.acceleration_or_force.y, out.acceleration_or_force.z = \
            msg.acceleration_or_force.x, msg.acceleration_or_force.y, msg.acceleration_or_force.z
        out.yaw = msg.yaw
        out.yaw_rate = msg.yaw_rate
        return out

    def _hold_output(self, now: float) -> Optional[PositionTarget]:
        if self._hold is None:
            if self._pose is not None and (now - self._pose[1]) <= self._pose_timeout_s:
                self._hold = self._capture(self._pose[0])
                self._last_hold = self._hold
            elif self._last_hold is not None:
                self._hold = self._last_hold
            else:
                if now - self._error_since_mono > 2.0:
                    self._error_since_mono = now
                    import sys
                    sys.stderr.write(
                        "setpoint_relay: no safe HOLD target (no fresh local pose); "
                        "no raw setpoint published\n")
                return None
        out = PositionTarget()
        out.coordinate_frame = FRAME_LOCAL_NED
        out.type_mask = MASK_HOLD
        out.position.x, out.position.y, out.position.z = self._hold[0], self._hold[1], self._hold[2]
        out.velocity.x = out.velocity.y = out.velocity.z = 0.0
        out.acceleration_or_force.x = out.acceleration_or_force.y = out.acceleration_or_force.z = 0.0
        out.yaw = self._hold[3]
        out.yaw_rate = 0.0
        return out

    @staticmethod
    def _capture(pose: PoseStamped) -> Tuple[float, float, float, float]:
        from math import atan2
        yaw = atan2(2.0 * (pose.pose.orientation.w * pose.pose.orientation.z +
                           pose.pose.orientation.x * pose.pose.orientation.y),
                    1.0 - 2.0 * (pose.pose.orientation.y ** 2 + pose.pose.orientation.z ** 2))
        return (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw)

