#!/usr/bin/env python3
"""event_log 纯单元测试（无 ROS Master）。

重点：**非阻塞**契约——写盘卡住时 emit() 不能阻塞调用线程（跟随环/relay 的实时路径靠这个前提），
队列满丢最旧并计数。
"""
import json
import os
import shutil
import tempfile
import threading
import time
import unittest

from swarm_uav_executor.event_log import EventLog, default_event_log_path


class EventLogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="event_log_test_")
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def test_writes_events_with_time_and_identity(self):
        path = os.path.join(self.tmp, "nested", "UAV8_follower_events.jsonl")
        log = EventLog(path, uav="UAV8", component="follower")
        log.emit("follower_start", leader_id="UAV8", offset=[0.0, 5.0, 0.0])
        log.emit("follower_exit", reason="LEADER_LOST:warmup", leader_age_s=None)
        log.close()

        with open(path) as handle:
            records = [json.loads(line) for line in handle if line.strip()]
        self.assertEqual([r["event"] for r in records],
                         ["follower_start", "follower_exit"])
        first = records[0]
        self.assertEqual((first["uav"], first["component"]), ("UAV8", "follower"))
        self.assertIn("t", first)
        self.assertIn("mono", first)
        self.assertEqual(first["leader_id"], "UAV8")
        self.assertEqual(log.dropped, 0)
        self.assertEqual(log.errors, 0)

    def test_none_path_is_noop(self):
        log = EventLog(None, uav="UAV8", component="relay")
        log.emit("hold_enter", reason="no_candidate")   # 不应抛、不应建线程
        self.assertIsNone(log.path)
        self.assertIsNone(log._thread)
        log.close()

    def test_emit_never_blocks_when_writer_is_stuck(self):
        path = os.path.join(self.tmp, "UAV8_relay_events.jsonl")
        log = EventLog(path, uav="UAV8", component="relay", queue_size=16)
        # 占住写盘锁 → 写线程卡在第一条；随后 emit 只能进队列/被丢弃
        log._io_lock.acquire()
        try:
            started = time.monotonic()
            for index in range(2000):
                log.emit("tick", index=index)
            elapsed = time.monotonic() - started
        finally:
            log._io_lock.release()
        log.close()
        self.assertLess(elapsed, 0.5)                   # 非阻塞（不写盘只入队）
        self.assertGreater(log.dropped, 0)              # 队列满 → 丢最旧并计数

    def test_unwritable_path_counts_errors_and_warns(self):
        warnings = []
        log = EventLog("/proc/definitely/not/writable/e.jsonl", component="relay",
                       warn=warnings.append)
        log.emit("boom")
        deadline = time.monotonic() + 2.0
        while log.errors == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        log.close()
        self.assertGreaterEqual(log.errors, 1)
        self.assertGreaterEqual(len(warnings), 1)

    def test_close_drains_pending_events(self):
        path = os.path.join(self.tmp, "UAV9_follower_events.jsonl")
        log = EventLog(path, uav="UAV9", component="follower")
        for index in range(50):
            log.emit("burst", index=index)
        log.close()
        with open(path) as handle:
            self.assertEqual(sum(1 for line in handle if line.strip()), 50)

    def test_default_path_helper(self):
        self.assertEqual(default_event_log_path("/tmp/x", "UAV8", "follower"),
                         "/tmp/x/UAV8_follower_events.jsonl")
        self.assertIsNone(default_event_log_path("", "UAV8", "follower"))
        self.assertIsNone(default_event_log_path("off", "UAV8", "follower"))
        self.assertIsNone(default_event_log_path(None, "UAV8", "follower"))

    def test_emit_from_many_threads_does_not_raise(self):
        path = os.path.join(self.tmp, "UAV10_relay_events.jsonl")
        log = EventLog(path, uav="UAV10", component="relay")

        def worker(worker_id):
            for index in range(100):
                log.emit("multi", worker=worker_id, index=index)

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        log.close()
        with open(path) as handle:
            self.assertEqual(sum(1 for line in handle if line.strip()), 400)


if __name__ == "__main__":
    unittest.main()
