#!/usr/bin/env python3
import unittest
from swarm_uav_interfaces.srv import UavTaskRequest, UavHoldRequest
from swarm_uav_interfaces.msg import Pose3DYaw, TaskAssignment
from swarm_uav_executor.models import ExecutorIdentity
from swarm_uav_executor.validation import RequestValidationError, build_goal, request_fingerprint, validate_task_request


def request():
    req = UavTaskRequest(protocol_version="1.0", mission_id="m1", group_id="GroupA", command_id="c1",
        uav_id="A01", exec_target="UAV1", command="MOVE_TO", timeout_s=2.0, leader_id="")
    req.assignment = TaskAssignment(uav_id="A01"); req.assignment.target_pose.x = 1.0
    return req

class ValidationTest(unittest.TestCase):
    def test_valid_and_stable_fingerprint(self):
        req = request(); validate_task_request(req, ExecutorIdentity("A01", "UAV1"), ("MOVE_TO",))
        self.assertEqual(request_fingerprint(req), request_fingerprint(req))
    def test_rejects_other_uav(self):
        req = request(); req.uav_id = "A02"
        with self.assertRaises(RequestValidationError) as caught: validate_task_request(req, ExecutorIdentity("A01", "UAV1"), ("MOVE_TO",))
        self.assertEqual(caught.exception.error_code, "IDENTITY_MISMATCH")
    def test_rejects_route_fields(self):
        req = request(); req.assignment.target_id = "A02"
        with self.assertRaises(RequestValidationError): validate_task_request(req, ExecutorIdentity("A01", "UAV1"), ("MOVE_TO",))
class RouteValidationTest(unittest.TestCase):
    def _route(self, follower=False):
        req = UavTaskRequest(protocol_version="1.0", mission_id="m1", group_id="GroupA",
            command_id="c2", uav_id="A01", exec_target="UAV1", command="FOLLOW_ROUTE",
            timeout_s=2.0, leader_id="A01")
        req.assignment = TaskAssignment(uav_id="A01")
        req.assignment.formation_follow = bool(follower)
        wp = Pose3DYaw()
        req.assignment.waypoints.append(wp)
        wp.x, wp.y, wp.z, wp.yaw = 5.0, 6.0, 12.0, 0.0
        return req

    def test_leader_route_builds_goal(self):
        req = self._route()
        goal = build_goal(req)
        self.assertEqual(goal.command, "FOLLOW_ROUTE")
        self.assertEqual(goal.waypoints, ((5.0, 6.0, 12.0, 0.0),))

    def test_leader_route_keeps_leader_id(self):
        req = self._route()
        goal = build_goal(req)
        self.assertEqual(goal.leader_id, "A01")

    def test_route_leader_id_different_rejected(self):
        req = self._route()
        req.leader_id = "A02"
        with self.assertRaises(RequestValidationError) as caught:
            validate_task_request(req, ExecutorIdentity("A01", "UAV1"), ("MOVE_TO", "FOLLOW_ROUTE"))
        self.assertEqual(caught.exception.error_code, "NOT_IMPLEMENTED")

    def test_follower_route_rejected(self):
        req = self._route(follower=True)
        with self.assertRaises(RequestValidationError) as caught:
            validate_task_request(req, ExecutorIdentity("A01", "UAV1"), ("MOVE_TO", "FOLLOW_ROUTE"))
        self.assertEqual(caught.exception.error_code, "NOT_IMPLEMENTED")

    def test_route_requires_waypoints(self):
        req = self._route(); req.assignment.waypoints[:] = []
        with self.assertRaises(RequestValidationError) as caught:
            validate_task_request(req, ExecutorIdentity("A01", "UAV1"), ("MOVE_TO", "FOLLOW_ROUTE"))
        self.assertEqual(caught.exception.error_code, "INVALID_ASSIGNMENT")


if __name__ == "__main__": unittest.main()
