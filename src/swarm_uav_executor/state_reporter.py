"""Single authority for task transitions and ROS state publication."""
from __future__ import annotations
import threading
import time
from swarm_uav_interfaces.msg import UavTaskState
from .models import PROTOCOL_VERSION, TERMINAL_STATES, TaskTransition


class StateReporter:
    def __init__(self, identity, store, publisher, republish_count=3, republish_interval_s=0.2,
                 clock=time.time, timer_factory=None):
        self.identity = identity; self.store = store; self.publisher = publisher
        self.republish_count = int(republish_count); self.republish_interval_s = float(republish_interval_s)
        self.clock = clock; self.timer_factory = timer_factory or self._thread_timer
        self._timers = []; self._lock = threading.Lock(); self._shutdown = False

    @staticmethod
    def _thread_timer(delay, callback):
        timer = threading.Timer(delay, callback); timer.daemon = True; timer.start(); return timer

    def publish(self, record):
        if self._shutdown: return
        message = UavTaskState(protocol_version=PROTOCOL_VERSION, mission_id=record.key.mission_id,
            command_id=record.key.command_id, uav_id=record.key.uav_id, exec_target=record.exec_target,
            command=record.command, status=record.status, status_seq=record.status_seq,
            detail_stage=record.detail_stage, error_code=record.error_code, message=record.message,
            reported_at=float(self.clock()))
        self.publisher.publish(message)

    def publish_transition(self, key, status, detail_stage, error_code="", message=""):
        record = self.store.update(key, TaskTransition(status, detail_stage, error_code, message, self.clock()))
        self.publish(record)
        if status in TERMINAL_STATES: self.start_terminal_republish(record)
        return record

    def start_terminal_republish(self, record):
        for index in range(self.republish_count):
            timer = self.timer_factory(self.republish_interval_s * (index + 1), lambda item=record: self.publish(item))
            with self._lock: self._timers.append(timer)

    def shutdown(self):
        self._shutdown = True
        with self._lock:
            for timer in self._timers:
                if hasattr(timer, "cancel"): timer.cancel()
            self._timers = []
