"""执行器侧事件日志（JSONL，非阻塞）。

为什么存在：2026-09-22 / 09-23 两轮实飞 coverage 失败时，机载侧**关键 20 s 无任何节点日志**——
`state_reporter.py` / `uav_executor_node.py` 运行期零日志语句，跟随环为何落 `LEADER_LOST`、
relay 何时切源/进 HOLD，全靠事后从 ulog 反推（见
quality_reports/2026-10-08_two_rounds_coverage_failure_root_cause.md §4）。本模块用**独立文件**
（不经 rosout，直接写盘，和 planner 诊断同思路）记录这些事件。

非阻塞契约（必须，不是优化）：``emit()`` 只入队，写盘在独立线程；队列满丢最旧并计数。
跟随环跑在 10/30 Hz、``setpoint_relay`` 以 30 Hz 独占 ``/mavros/setpoint_raw/local``——
绝不能把 SD 卡/文件系统时延加进这两条实时路径（``test_ego_swarm_driver.py`` 的 deadline 用例
就是靠这个前提）。
"""
from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time


class EventLog:
    """JSONL 事件日志。``path=None`` → 空实现（emit 只计数，不开线程）。"""

    def __init__(self, path=None, uav="", component="", queue_size=1024, warn=None,
                 clock=time.time, mono_clock=time.monotonic):
        self.path = str(path) if path else None
        self.uav = str(uav or "")
        self.component = str(component or "")
        self._warn = warn
        self._clock = clock
        self._mono = mono_clock
        self._queue = queue.Queue(maxsize=max(8, int(queue_size)))
        # 两把锁分工：_stat_lock 只护计数器（emit 溢出路径要用，必须极短），
        # _io_lock 护文件句柄与写入（可能慢，绝不能被 emit 触碰——否则磁盘卡住会把
        # "非阻塞"契约破坏在实时路径上）。
        self._stat_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._handle = None
        self._dropped = 0
        self._errors = 0
        self._last_warn = 0.0
        self._stop = threading.Event()
        self._thread = None
        if self.path is not None:
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()

    # ------------------------------------------------------------------ API
    @property
    def dropped(self):
        with self._stat_lock:
            return self._dropped

    @property
    def errors(self):
        with self._stat_lock:
            return self._errors

    def emit(self, event, **fields):
        """入队一条事件。**不阻塞**：队列满时丢最旧并计数。"""
        record = dict(fields)
        record["event"] = str(event)
        record["t"] = round(self._clock(), 6)
        record["mono"] = round(self._mono(), 6)
        if self.uav:
            record["uav"] = self.uav
        if self.component:
            record["component"] = self.component
        if self.path is None:
            return
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(record)
            except (queue.Empty, queue.Full):
                pass
            with self._stat_lock:
                self._dropped += 1

    def close(self, timeout=2.0):
        if self.path is None:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
        with self._io_lock:
            if self._handle is not None:
                try:
                    self._handle.close()
                except OSError:
                    pass
                self._handle = None

    # -------------------------------------------------------------- writer
    def _run(self):
        while not self._stop.is_set() or not self._queue.empty():
            try:
                record = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._write(record)

    def _write(self, record):
        try:
            with self._io_lock:
                if self._handle is None:
                    os.makedirs(os.path.dirname(self.path), exist_ok=True)
                    self._handle = open(self.path, "a")
                self._handle.write(json.dumps(record, ensure_ascii=False) + "\n")
                self._handle.flush()
        except Exception as exc:                      # noqa: BLE001 - 观测绝不抛出
            with self._stat_lock:
                self._errors += 1
            now = self._mono()
            if now - self._last_warn > 30.0:
                self._last_warn = now
                message = "event_log %s: write failed (%s)" % (self.component or self.path, exc)
                if self._warn is not None:
                    try:
                        self._warn(message)
                    except Exception:                 # noqa: BLE001
                        sys.stderr.write(message + "\n")
                else:
                    sys.stderr.write(message + "\n")


def default_event_log_path(directory, uav, component):
    """``<dir>/<uav>_<component>_events.jsonl``；``directory`` 为空则返回 None（关闭观测）。"""
    directory = str(directory or "").strip()
    if not directory or directory.lower() in ("none", "off", "disabled"):
        return None
    return os.path.join(directory, "%s_%s_events.jsonl" % (uav or "unknown", component))
