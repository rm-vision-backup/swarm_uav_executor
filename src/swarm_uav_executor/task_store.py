"""Thread-safe bounded idempotency store."""
from __future__ import annotations

import threading
from dataclasses import replace
from typing import Dict, Optional

from .models import CONFLICT, DUPLICATE, REGISTERED, RegisterResult, TaskKey, TaskRecord, TaskTransition


class TaskStore:
    def __init__(self, ttl_s: float = 3600.0, max_records: int = 1024) -> None:
        self.ttl_s = float(ttl_s)
        self.max_records = int(max_records)
        self._records: Dict[TaskKey, TaskRecord] = {}
        self._lock = threading.RLock()

    def get(self, key: TaskKey) -> Optional[TaskRecord]:
        with self._lock:
            return self._records.get(key)

    def register(self, record: TaskRecord) -> RegisterResult:
        with self._lock:
            current = self._records.get(record.key)
            if current is not None:
                return RegisterResult(DUPLICATE if current.fingerprint == record.fingerprint else CONFLICT, current)
            if self.active() is not None:
                raise RuntimeError("active task already exists")
            self._records[record.key] = record
            self._trim_locked()
            return RegisterResult(REGISTERED, record)

    def update(self, key: TaskKey, transition: TaskTransition) -> TaskRecord:
        with self._lock:
            current = self._records[key]
            if current.terminal and transition.status != current.status:
                raise ValueError("terminal task cannot transition")
            updated = replace(current, status=transition.status, status_seq=current.status_seq + 1,
                              detail_stage=transition.detail_stage, error_code=transition.error_code,
                              message=transition.message, updated_at=transition.updated_at)
            self._records[key] = updated
            return updated

    def mark_started(self, key: TaskKey, now: float) -> TaskRecord:
        with self._lock:
            current = self._records[key]
            updated = replace(current, started=True, updated_at=now, message="task started")
            self._records[key] = updated
            return updated

    def active(self) -> Optional[TaskRecord]:
        with self._lock:
            return next((record for record in self._records.values() if not record.terminal), None)

    def prune(self, now: float) -> None:
        with self._lock:
            expired = [key for key, record in self._records.items()
                       if record.terminal and now - record.updated_at > self.ttl_s]
            for key in expired:
                self._records.pop(key, None)
            self._trim_locked()

    def _trim_locked(self) -> None:
        terminal = sorted((record for record in self._records.values() if record.terminal), key=lambda item: item.updated_at)
        while len(self._records) > self.max_records and terminal:
            self._records.pop(terminal.pop(0).key, None)
