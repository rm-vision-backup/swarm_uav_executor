#!/usr/bin/env python3
import unittest
from swarm_uav_executor.models import *
from swarm_uav_executor.task_store import TaskStore

def record(key=TaskKey("m", "c", "A01"), fingerprint="f", updated=0.0):
    return TaskRecord(key, fingerprint, "UAV1", "MOVE_TO", 1.0, MotionGoal(0,0,0,0), updated_at=updated)
class StoreTest(unittest.TestCase):
    def test_duplicate_and_conflict(self):
        store=TaskStore(); self.assertEqual(store.register(record()).outcome, REGISTERED)
        self.assertEqual(store.register(record()).outcome, DUPLICATE)
        self.assertEqual(store.register(record(fingerprint="other")).outcome, CONFLICT)
    def test_terminal_pruning_never_removes_active(self):
        store=TaskStore(ttl_s=1,max_records=1); active=record(); store.register(active); store.prune(100)
        self.assertIsNotNone(store.active())
    def test_sequence_increments(self):
        store=TaskStore(); item=record(); store.register(item)
        done=store.update(item.key, TaskTransition(STATE_COMPLETED,"COMPLETED",updated_at=1))
        self.assertEqual(done.status_seq,2)
if __name__ == "__main__": unittest.main()
