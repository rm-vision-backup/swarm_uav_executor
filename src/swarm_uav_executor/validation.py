"""Side-effect-free validation of cross-machine requests."""
from __future__ import annotations

import hashlib
import json
import math
from typing import Collection

from .models import ExecutorIdentity, MotionGoal, PROTOCOL_VERSION, TaskKey


class RequestValidationError(ValueError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.message = message


def _require(value: str, field: str) -> None:
    if not str(value or "").strip():
        raise RequestValidationError("INVALID_REQUEST", "%s is required" % field)


def _validate_identity(uav_id: str, exec_target: str, identity: ExecutorIdentity) -> None:
    if uav_id != identity.uav_id or exec_target != identity.exec_target:
        raise RequestValidationError("IDENTITY_MISMATCH", "request is not addressed to this UAV executor")


def validate_task_request(request, identity: ExecutorIdentity, supported_commands: Collection[str]) -> None:
    if request.protocol_version != PROTOCOL_VERSION:
        raise RequestValidationError("UNSUPPORTED_PROTOCOL", "protocol_version must be %s" % PROTOCOL_VERSION)
    for value, field in ((request.mission_id, "mission_id"), (request.command_id, "command_id"),
                         (request.uav_id, "uav_id"), (request.exec_target, "exec_target"),
                         (request.command, "command")):
        _require(value, field)
    if request.group_id != "GroupA":
        raise RequestValidationError("WRONG_GROUP", "only GroupA is supported")
    _validate_identity(request.uav_id, request.exec_target, identity)
    if request.assignment.uav_id != identity.uav_id:
        raise RequestValidationError("IDENTITY_MISMATCH", "assignment.uav_id does not match this UAV")
    if not math.isfinite(float(request.timeout_s)) or request.timeout_s <= 0.0:
        raise RequestValidationError("INVALID_TIMEOUT", "timeout_s must be finite and positive")
    if request.command not in supported_commands:
        raise RequestValidationError("UNSUPPORTED_COMMAND", "unsupported command: %s" % request.command)
    if request.command in ("MOVE_TO", "FAULT_EXIT", "HOVER"):
        validate_move_to_assignment(request.assignment)
    elif request.command == "FOLLOW_ROUTE":
        if request.leader_id and request.leader_id != request.uav_id:
            raise RequestValidationError(
                "NOT_IMPLEMENTED",
                "FOLLOW_ROUTE follower semantics (leader_id differs from uav_id) is P2")
        validate_route_assignment(request.assignment)


def validate_move_to_assignment(assignment) -> MotionGoal:
    if assignment.formation_follow or assignment.target_id or assignment.waypoints:
        raise RequestValidationError("INVALID_ASSIGNMENT", "MOVE_TO accepts only target_pose")
    values = (assignment.target_pose.x, assignment.target_pose.y, assignment.target_pose.z, assignment.target_pose.yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise RequestValidationError("INVALID_ASSIGNMENT", "MOVE_TO target_pose must be finite")
    return MotionGoal(*(float(value) for value in values))


def validate_route_assignment(assignment) -> MotionGoal:
    if assignment.formation_follow:
        raise RequestValidationError("NOT_IMPLEMENTED", "FOLLOW_ROUTE formation follow is P2")
    if not assignment.waypoints:
        raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE leader requires waypoints")
    waypoints = [(float(p.x), float(p.y), float(p.z), float(p.yaw)) for p in assignment.waypoints]
    if not all(math.isfinite(v) for wp in waypoints for v in wp):
        raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE waypoints must be finite")
    last = waypoints[-1]
    return MotionGoal(last[0], last[1], last[2], last[3], waypoints=tuple(waypoints), command="FOLLOW_ROUTE")


def build_goal(request) -> MotionGoal:
    if request.command in ("MOVE_TO", "FAULT_EXIT", "HOVER"):
        goal = validate_move_to_assignment(request.assignment)
        return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, (), request.leader_id,
                          bool(request.assignment.formation_follow), request.command)
    if request.command == "FOLLOW_ROUTE":
        # FOLLOW_ROUTE reaching the driver means the leader role: validation
        # already rejects formation_follow as NOT_IMPLEMENTED, so keep the
        # leader_id for traceability instead of dropping it on the floor.
        goal = validate_route_assignment(request.assignment)
        return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, goal.waypoints,
                          request.leader_id, False, goal.command)
    raise RequestValidationError("UNSUPPORTED_COMMAND", "unsupported command: %s" % request.command)


def validate_hold_request(request, identity: ExecutorIdentity) -> None:
    if request.protocol_version != PROTOCOL_VERSION:
        raise RequestValidationError("UNSUPPORTED_PROTOCOL", "protocol_version must be %s" % PROTOCOL_VERSION)
    for value, field in ((request.mission_id, "mission_id"), (request.command_id, "command_id"),
                         (request.uav_id, "uav_id"), (request.exec_target, "exec_target")):
        _require(value, field)
    _validate_identity(request.uav_id, request.exec_target, identity)


def task_key_from_request(request) -> TaskKey:
    return TaskKey(request.mission_id, request.command_id, request.uav_id)


def request_fingerprint(request) -> str:
    pose = request.assignment.target_pose
    body = {
        "protocol_version": request.protocol_version, "group_id": request.group_id,
        "exec_target": request.exec_target, "command": request.command,
        "timeout_s": float(request.timeout_s), "leader_id": request.leader_id,
        "assignment": {"uav_id": request.assignment.uav_id,
                       "formation_follow": bool(request.assignment.formation_follow),
                       "target_id": request.assignment.target_id,
                       "target_pose": [float(pose.x), float(pose.y), float(pose.z), float(pose.yaw)],
                       "waypoints": [[float(p.x), float(p.y), float(p.z), float(p.yaw)] for p in request.assignment.waypoints]},
    }
    return hashlib.sha256(json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
