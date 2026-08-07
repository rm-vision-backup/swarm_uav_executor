"""Internal immutable values for the single-UAV executor."""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Optional, Tuple

PROTOCOL_VERSION = "1.0"
STATE_ACCEPTED = "ACCEPTED"
STATE_COMPLETED = "COMPLETED"
STATE_FAILED = "FAILED"
TERMINAL_STATES = frozenset((STATE_COMPLETED, STATE_FAILED))


@dataclass(frozen=True)
class ExecutorIdentity:
    uav_id: str
    exec_target: str


@dataclass(frozen=True)
class ExecutorConfig:
    task_service: str
    hold_service: str
    state_topic: str
    supported_commands: Tuple[str, ...] = ("MOVE_TO",)
    store_ttl_s: float = 3600.0
    store_max_records: int = 1024
    terminal_republish_count: int = 3
    terminal_republish_interval_s: float = 0.2
    shutdown_hold_timeout_s: float = 2.0


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
