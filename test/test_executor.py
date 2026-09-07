#!/usr/bin/env python3
import time, unittest
from swarm_uav_interfaces.msg import TaskAssignment
from swarm_uav_interfaces.srv import UavTaskControlRequest, UavTaskRequest, UavHoldRequest
from swarm_uav_executor.drivers.mock import MockMotionDriver
from swarm_uav_executor.executor import (GROUP_SAFETY_REASON,
                                         GROUP_SAFETY_RESET_REASON,
                                         UavTaskExecutor)
from swarm_uav_executor.models import ExecutorConfig, ExecutorIdentity
from swarm_uav_executor.state_reporter import StateReporter
from swarm_uav_executor.task_store import TaskStore
class Publisher:
    def __init__(self): self.items=[]
    def publish(self,item): self.items.append(item)
def request(command_id="c1"):
    req=UavTaskRequest(protocol_version="1.0",mission_id="m",group_id="GroupA",command_id=command_id,uav_id="A01",exec_target="UAV1",command="MOVE_TO",timeout_s=1,leader_id="")
    req.assignment=TaskAssignment(uav_id="A01"); return req
def make(delay=.05,success=True):
    identity=ExecutorIdentity("A01","UAV1"); config=ExecutorConfig("/task","/hold","/state",terminal_republish_count=0)
    store=TaskStore(); publisher=Publisher(); reporter=StateReporter(identity,store,publisher,republish_count=0)
    driver=MockMotionDriver(delay,success,"TEST_FAILED","done")
    return UavTaskExecutor(identity,config,store,reporter,driver),store,publisher,driver

class ExecutorTest(unittest.TestCase):
    def test_explicit_start_prepares_without_motion_then_starts(self):
        executor,store,_,driver=make(.01)
        executor.config = ExecutorConfig("/task", "/hold", "/state", "/control", True,
                                         terminal_republish_count=0)
        req=request(); self.assertTrue(executor.handle_task(req).accepted)
        self.assertEqual(driver.start_count, 0)
        control=UavTaskControlRequest(protocol_version="1.0", operation="START",
            mission_id="m", command_id="c1", uav_id="A01", exec_target="UAV1")
        self.assertTrue(executor.handle_task_control(control).accepted)
        time.sleep(.05); self.assertEqual(driver.start_count, 1)
        self.assertEqual(next(iter(store._records.values())).status, "COMPLETED")

    def test_abort_prepared_task_never_starts_motion(self):
        executor,store,_,driver=make(.01)
        executor.config = ExecutorConfig("/task", "/hold", "/state", "/control", True,
                                         terminal_republish_count=0)
        self.assertTrue(executor.handle_task(request()).accepted)
        control=UavTaskControlRequest(protocol_version="1.0", operation="ABORT",
            mission_id="m", command_id="c1", uav_id="A01", exec_target="UAV1", reason="group failed")
        self.assertTrue(executor.handle_task_control(control).accepted)
        self.assertEqual(driver.start_count, 0)
        record=next(iter(store._records.values()))
        self.assertEqual(record.status, "FAILED"); self.assertEqual(record.error_code, "TASK_ABORTED")

    def test_nonblocking_and_duplicate(self):
        executor,store,_,driver=make(.15); req=request(); start=time.time(); first=executor.handle_task(req)
        self.assertLess(time.time()-start,.1); second=executor.handle_task(req); self.assertTrue(second.accepted)
        time.sleep(.2); self.assertEqual(driver.start_count,1); self.assertEqual(store.get(next(iter(store._records))).status,"COMPLETED")
    def test_conflict_and_busy(self):
        executor,_,_,_=make(.2); req=request(); executor.handle_task(req); req.assignment.target_pose.x=1
        self.assertEqual(executor.handle_task(req).error_code,"DUPLICATE_CONFLICT")
        self.assertEqual(executor.handle_task(request("c2")).error_code,"BUSY")
    def test_hold_interrupts(self):
        executor,store,_,driver=make(.4); executor.handle_task(request())
        hold=UavHoldRequest(protocol_version="1.0",mission_id="m",command_id="c1",uav_id="A01",exec_target="UAV1",reason="stop")
        self.assertTrue(executor.handle_hold(hold).accepted); time.sleep(.08)
        self.assertEqual(store.active(),None); self.assertEqual(driver.hold_count,1)
        self.assertEqual(next(iter(store._records.values())).error_code,"COMMAND_HELD")
    def test_motion_failure_captures_hold_before_terminal(self):
        executor,store,_,driver=make(.01,False); executor.handle_task(request()); time.sleep(.08)
        record=next(iter(store._records.values()))
        self.assertEqual(record.status,"FAILED"); self.assertEqual(record.error_code,"TEST_FAILED")
        self.assertEqual(record.detail_stage,"HOLD"); self.assertEqual(driver.hold_count,1)
    def test_hold_failure_replaces_motion_error_with_safety_error(self):
        executor,store,_,driver=make(.01,False); driver.hold_success=False
        executor.handle_task(request()); time.sleep(.08); record=next(iter(store._records.values()))
        self.assertEqual(record.error_code,"HOLD_FAILED")
        self.assertIn("TEST_FAILED",record.message); self.assertEqual(driver.hold_count,1)
    def test_group_safety_lock_rejects_task_and_start_until_reset(self):
        executor,store,_,driver=make(.02)
        executor.config = ExecutorConfig("/task", "/hold", "/state", "/control", True,
                                         terminal_republish_count=0)
        self.assertTrue(executor.handle_task(request()).accepted)  # c1 prepared
        hold=UavHoldRequest(protocol_version="1.0",mission_id="m",command_id="c1",uav_id="A01",exec_target="UAV1",reason=GROUP_SAFETY_REASON)
        self.assertTrue(executor.handle_hold(hold).accepted)
        time.sleep(.06)
        # 锁存置位后拒绝新任务与 START。
        self.assertEqual(executor.handle_task(request("c2")).error_code, "SAFETY_LATCHED")
        control=UavTaskControlRequest(protocol_version="1.0", operation="START",
            mission_id="m", command_id="c1", uav_id="A01", exec_target="UAV1")
        self.assertEqual(executor.handle_task_control(control).error_code, "SAFETY_LATCHED")
        # 最小人工复位入口（GROUP_SAFETY_RESET）清除锁存，此后新任务可接收。
        reset=UavHoldRequest(protocol_version="1.0",mission_id="m",command_id="c1",uav_id="A01",exec_target="UAV1",reason=GROUP_SAFETY_RESET_REASON)
        self.assertTrue(executor.handle_hold(reset).accepted)
        time.sleep(.02)
        self.assertTrue(executor.handle_task(request("c2")).accepted)
    def test_ordinary_hold_does_not_set_safety_latch(self):
        executor,store,_,driver=make(.02)
        hold=UavHoldRequest(protocol_version="1.0",mission_id="m",command_id="c1",uav_id="A01",exec_target="UAV1",reason="stop")
        self.assertTrue(executor.handle_hold(hold).accepted); time.sleep(.05)
        self.assertTrue(executor.handle_task(request("c1")).accepted)  # 普通 HOLD 不锁存
if __name__ == "__main__": unittest.main()
