#!/usr/bin/env python3
import unittest
from swarm_uav_interfaces.srv import UavTaskRequest, UavHoldRequest
from swarm_uav_interfaces.msg import TaskAssignment
from swarm_uav_executor.models import ExecutorIdentity
from swarm_uav_executor.validation import RequestValidationError, request_fingerprint, validate_task_request


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
if __name__ == "__main__": unittest.main()
