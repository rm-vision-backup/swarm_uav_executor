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
    if request.command == "MOVE_TO":
        validate_move_to_assignment(request.assignment)
    elif request.command == "FAULT_EXIT":
        # FAULT_EXIT runs in the 8 m low layer; accepts either a single
        # target_pose or an exit route carried as waypoints.
        validate_fault_exit_assignment(request.assignment)
    elif request.command == "HOVER":
        validate_hover_assignment(request.assignment)
    elif request.command == "FOLLOW_ROUTE":
        # FOLLOW_ROUTE runs in the 12 m mid layer (leader route waypoints).
        for wp in request.assignment.waypoints:
            if abs(float(wp.z) - 12.0) > 0.5:
                raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE waypoint z must be in the 12 m mid layer")
        if request.leader_id and request.leader_id != request.uav_id:
            # Follower semantics: leader_id differs from uav_id -> formation follow.
            _validate_follower_assignment(request.assignment, request.leader_id, identity)
        else:
            validate_route_assignment(request.assignment)


def validate_move_to_assignment(assignment) -> MotionGoal:
    if assignment.formation_follow or assignment.target_id or assignment.waypoints:
        raise RequestValidationError("INVALID_ASSIGNMENT", "MOVE_TO accepts only target_pose")
    return _goal_from_target_pose(assignment.target_pose, "MOVE_TO")


def validate_fault_exit_assignment(assignment) -> MotionGoal:
    """FAULT_EXIT accepts a single target_pose or an exit waypoint route.

    Both forms must stay in the 8 m low layer. When waypoints are given, the
    goal keeps them so the driver can plan the exit route after the vertical
    transition.
    """
    if assignment.waypoints:
        waypoints = [(float(p.x), float(p.y), float(p.z), float(p.yaw)) for p in assignment.waypoints]
        if not all(math.isfinite(v) for wp in waypoints for v in wp):
            raise RequestValidationError("INVALID_ASSIGNMENT", "FAULT_EXIT waypoints must be finite")
        for wp in waypoints:
            if abs(wp[2] - 8.0) > 0.5:
                raise RequestValidationError("INVALID_ASSIGNMENT", "FAULT_EXIT waypoint z must be in the 8 m low layer")
        last = waypoints[-1]
        return MotionGoal(last[0], last[1], last[2], last[3],
                          waypoints=tuple(waypoints), command="FAULT_EXIT", layer_z=8.0)
    if assignment.formation_follow or assignment.target_id:
        raise RequestValidationError("INVALID_ASSIGNMENT", "FAULT_EXIT accepts only target_pose or waypoints")
    goal = _goal_from_target_pose(assignment.target_pose, "FAULT_EXIT")
    if abs(goal.z - 8.0) > 0.5:
        raise RequestValidationError("INVALID_ASSIGNMENT", "FAULT_EXIT target z must be in the 8 m low layer")
    return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, (), goal.leader_id,
                      False, "FAULT_EXIT", 8.0)


def validate_hover_assignment(assignment) -> MotionGoal:
    """HOVER accepts an empty/placeholder assignment.

    The runtime sends HOVER assignments with only a uav_id (target_pose is a
    zero placeholder). The frozen-height semantics is resolved by the driver
    from the current local pose, so only a non-zero height needs the 1..16 m
    sanity bound.
    """
    if assignment.formation_follow or assignment.target_id or assignment.waypoints:
        raise RequestValidationError("INVALID_ASSIGNMENT", "HOVER accepts only target_pose")
    goal = _goal_from_target_pose(assignment.target_pose, "HOVER")
    z = goal.z
    if z != 0.0 and not (1.0 <= z <= 16.0):
        raise RequestValidationError("INVALID_ASSIGNMENT", "HOVER target z must be between 1 and 16 m")
    return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, (), goal.leader_id,
                      False, "HOVER", z)


def _formation_offset_tuple(assignment):
    """Read a formation_offset from the assignment (Pose3DYaw) as a 3-tuple.

    The offset is optional; defaults to (0,0,0) when unset (e.g. leader
    reference path or GCS_A has not injected any formation slot).
    """
    try:
        off = assignment.formation_offset
        value = (off.x, off.y, off.z)
        if not all(math.isfinite(float(v)) for v in value):
            return (0.0, 0.0, 0.0)
        return (float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        return (0.0, 0.0, 0.0)


def _goal_from_target_pose(target_pose, command: str) -> MotionGoal:
    values = (target_pose.x, target_pose.y, target_pose.z, target_pose.yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise RequestValidationError("INVALID_ASSIGNMENT", "%s target_pose must be finite" % command)
    return MotionGoal(*(float(value) for value in values))


def _validate_follower_assignment(assignment, leader_id: str, identity: ExecutorIdentity) -> None:
    """Validate a FOLLOW_ROUTE follower assignment.

    The follower does not need waypoints: it tracks the leader pose plus a
    formation offset resolved on the driver side. Any waypoints carried are
    only a leader reference and must still be finite.
    """
    if not str(leader_id or "").strip():
        raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE follower requires leader_id")
    if str(leader_id) == str(identity.uav_id):
        raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE follower leader_id must differ from uav_id")
    if assignment.waypoints:
        waypoints = [(float(p.x), float(p.y), float(p.z), float(p.yaw)) for p in assignment.waypoints]
        if not all(math.isfinite(v) for wp in waypoints for v in wp):
            raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE waypoints must be finite")


def validate_route_assignment(assignment, leader_id: str = "") -> MotionGoal:
    # Follower semantics: explicit formation_follow flag OR a leader_id that
    # differs from the local uav_id (both paths accepted, waypoints optional).
    follower = bool(assignment.formation_follow) or bool(
        str(leader_id or "").strip() and str(leader_id) != str(assignment.uav_id))
    if follower:
        if not str(leader_id or "").strip() or str(leader_id) == str(assignment.uav_id):
            raise RequestValidationError(
                "INVALID_ASSIGNMENT", "FOLLOW_ROUTE follower requires leader_id different from uav_id")
        # target_pose is only a leader reference for the follower; the driver
        # resolves the actual target from leader odom + formation_offset.
        if assignment.target_pose:
            last = (assignment.target_pose.x, assignment.target_pose.y, assignment.target_pose.z, assignment.target_pose.yaw)
        else:
            last = (0.0, 0.0, 12.0, 0.0)
        formation_offset = _formation_offset_tuple(assignment)
        return MotionGoal(*(float(v) for v in last), waypoints=(), leader_id=leader_id,
                          formation_follow=True, command="FOLLOW_ROUTE", layer_z=12.0,
                          formation_offset=formation_offset)
    if not assignment.waypoints:
        raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE leader requires waypoints")
    waypoints = [(float(p.x), float(p.y), float(p.z), float(p.yaw)) for p in assignment.waypoints]
    if not all(math.isfinite(v) for wp in waypoints for v in wp):
        raise RequestValidationError("INVALID_ASSIGNMENT", "FOLLOW_ROUTE waypoints must be finite")
    last = waypoints[-1]
    return MotionGoal(last[0], last[1], last[2], last[3], waypoints=tuple(waypoints),
                      leader_id=leader_id, command="FOLLOW_ROUTE", layer_z=12.0)


def build_goal(request) -> MotionGoal:
    if request.command == "MOVE_TO":
        goal = validate_move_to_assignment(request.assignment)
        # layer_z drives the vertical-first transition: MOVE_TO@15.
        return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, (), request.leader_id,
                          False, request.command, 15.0)
    if request.command == "FAULT_EXIT":
        goal = validate_fault_exit_assignment(request.assignment)
        return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, goal.waypoints,
                          request.leader_id, False, request.command, 8.0)
    if request.command == "HOVER":
        goal = validate_hover_assignment(request.assignment)
        # HOVER freezes at the current local pose; layer_z carries the
        # validated/placeholder goal height for traceability.
        return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, (), request.leader_id,
                          False, request.command, goal.z)
    if request.command == "FOLLOW_ROUTE":
        goal = validate_route_assignment(request.assignment, request.leader_id)
        follower = bool(request.assignment.formation_follow) or bool(
            request.leader_id and request.leader_id != request.uav_id)
        if follower:
            return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, goal.waypoints,
                              request.leader_id, True, goal.command, 12.0,
                              goal.formation_offset)
        return MotionGoal(goal.x, goal.y, goal.z, goal.yaw, goal.waypoints,
                          request.leader_id, False, goal.command, 12.0)
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
