#!/usr/bin/env python3
import time, unittest
from swarm_uav_interfaces.msg import TaskAssignment
from swarm_uav_interfaces.srv import UavTaskRequest, UavHoldRequest
from swarm_uav_executor.drivers.mock import MockMotionDriver
from swarm_uav_executor.executor import UavTaskExecutor
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

class LeaseGuard:
    def __init__(self, permitted=False): self.permitted=permitted
    def permits_task(self, _mission_id): return self.permitted
class ExecutorTest(unittest.TestCase):
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
    def test_requires_active_lease_when_guard_is_installed(self):
        executor,_,_,driver=make(); guard=LeaseGuard(False); executor.lease_guard=guard
        self.assertEqual(executor.handle_task(request()).error_code,"LEASE_NOT_ACTIVE")
        self.assertEqual(driver.start_count,0); guard.permitted=True
        self.assertTrue(executor.handle_task(request()).accepted)
    def test_lease_expiry_holds_active_task_and_latches_new_work(self):
        executor,store,_,driver=make(.4); guard=LeaseGuard(True); executor.lease_guard=guard
        self.assertTrue(executor.handle_task(request()).accepted)
        result=executor.trigger_local_safety_hold("LEASE_EXPIRED","GCS lease expired")
        self.assertTrue(result.success); self.assertEqual(driver.hold_count,1)
        record=next(iter(store._records.values()))
        self.assertEqual(record.status,"FAILED"); self.assertEqual(record.error_code,"LEASE_EXPIRED")
        self.assertEqual(executor.handle_task(request("c2")).error_code,"LEASE_EXPIRED")
        executor.trigger_local_safety_hold("LEASE_EXPIRED","duplicate")
        self.assertEqual(driver.hold_count,1)
    def test_lease_expiry_holds_between_tasks(self):
        executor,_,_,driver=make(); executor.lease_guard=LeaseGuard(True)
        self.assertTrue(executor.trigger_local_safety_hold("LEASE_EXPIRED","expired").success)
        self.assertEqual(driver.hold_count,1)
    def test_safe_lease_end_releases_latch_without_reviving_old_task(self):
        executor,store,_,driver=make(.4); executor.lease_guard=LeaseGuard(True)
        self.assertTrue(executor.handle_task(request()).accepted)
        executor.trigger_local_safety_hold("LEASE_EXPIRED","expired")
        old_record=next(iter(store._records.values()))
        self.assertEqual(old_record.error_code,"LEASE_EXPIRED")
        executor.release_local_safety_latch()
        self.assertTrue(executor.handle_task(request("c2")).accepted)
        self.assertEqual(old_record.error_code,"LEASE_EXPIRED")
        self.assertEqual(driver.start_count,2)
if __name__ == "__main__": unittest.main()
