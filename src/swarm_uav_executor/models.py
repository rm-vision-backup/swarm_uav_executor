"""Internal immutable values for the single-UAV executor."""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Optional, Tuple

PROTOCOL_VERSION = "1.0"
STATE_ACCEPTED = "ACCEPTED"
STATE_COMPLETED = "COMPLETED"
STATE_FAILED = "FAILED"
TERMINAL_STATES = frozenset((STATE_COMPLETED, STATE_FAILED))

LEASE_INACTIVE = "INACTIVE"
LEASE_ACTIVE = "ACTIVE"
LEASE_EXPIRED_HOLD = "EXPIRED_HOLD"
LEASE_END_PENDING = "END_PENDING"


@dataclass(frozen=True)
class ExecutorIdentity:
    uav_id: str
    exec_target: str


@dataclass(frozen=True)
class ExecutorConfig:
    task_service: str
    hold_service: str
    state_topic: str
    task_control_service: str = ""
    require_explicit_start: bool = False
    supported_commands: Tuple[str, ...] = ("MOVE_TO",)
    store_ttl_s: float = 3600.0
    store_max_records: int = 1024
    terminal_republish_count: int = 3
    terminal_republish_interval_s: float = 0.2
    shutdown_hold_timeout_s: float = 2.0
    # Follower PI gains: scale the position error into a setpoint offset.
    follower_p_gain: float = 1.0        # position-error proportional gain
    follower_i_gain: float = 0.1        # position-error integral gain
    follower_limit_xy: float = 2.0      # max horizontal follower setpoint compensation (m/s equivalent)
    follower_limit_z: float = 1.0       # max vertical follower compensation (m/s)


@dataclass(frozen=True)
class SafetyLeaseConfig:
    service_name: str
    watchdog_hz: float = 10.0
    default_ttl_s: float = 5.0
    min_ttl_s: float = 1.0
    max_ttl_s: float = 30.0
    disarmed_stable_s: float = 3.0
    required_for_tasks: bool = True


@dataclass(frozen=True)
class SafetyLeaseRecord:
    mission_id: str
    session_epoch: str
    last_seq: int
    ttl_s: float
    last_renew_monotonic_s: float
    state: str = LEASE_ACTIVE
    expired_at_monotonic_s: Optional[float] = None


@dataclass(frozen=True)
class LeaseExpiryEvent:
    mission_id: str
    session_epoch: str
    expired_at_monotonic_s: float
    hold_success: Optional[bool] = None
    error_code: str = ""
    message: str = ""


@dataclass(frozen=True)
class SafetyHoldResult:
    success: bool
    error_code: str = ""
    message: str = ""


@dataclass(frozen=True, order=True)
class TaskKey:
    mission_id: str
    command_id: str
    uav_id: str


@dataclass(frozen=True)
class MotionGoal:
    x: float
    y: float
    z: float
    yaw: float
    # FOLLOW_ROUTE leader waypoints: (x, y, z, yaw) tuples (P1 leader only).
    waypoints: Tuple[Tuple[float, float, float, float], ...] = ()
    leader_id: str = ""                 # follower=leader A-id; empty for leader/single-UAV
    formation_follow: bool = False      # True=follower formation follow semantics
    command: str = "MOVE_TO"            # MOVE_TO / FOLLOW_ROUTE / FAULT_EXIT / HOVER
    layer_z: float = 15.0               # target execution height layer (15/12/8/frozen); drives vertical-first
    formation_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # follower ENU offset from leader (GCS_A slots)


@dataclass(frozen=True)
class HoldGoal:
    reason: str = ""


@dataclass(frozen=True)
class MotionResult:
    success: bool
    error_code: str = ""
    message: str = ""


@dataclass(frozen=True)
class DriverHealth:
    ready: bool
    error_code: str = ""
    message: str = ""


@dataclass(frozen=True)
class TaskRecord:
    key: TaskKey
    fingerprint: str
    exec_target: str
    command: str
    timeout_s: float
    goal: MotionGoal
    status: str = STATE_ACCEPTED
    status_seq: int = 1
    detail_stage: str = STATE_ACCEPTED
    error_code: str = ""
    message: str = ""
    updated_at: float = 0.0
    started: bool = False
    prepared_at: float = 0.0

    @property
    def terminal(self) -> bool:
        return self.status in TERMINAL_STATES


@dataclass(frozen=True)
class TaskTransition:
    status: str
    detail_stage: str
    error_code: str = ""
    message: str = ""
    updated_at: float = 0.0


@dataclass(frozen=True)
class RegisterResult:
    outcome: str
    record: TaskRecord


REGISTERED = "REGISTERED"
DUPLICATE = "DUPLICATE"
CONFLICT = "CONFLICT"
