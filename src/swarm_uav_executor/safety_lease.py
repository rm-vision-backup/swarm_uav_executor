"""Local monotonic safety-lease state machine."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import replace

from swarm_uav_interfaces.srv import UavSafetyLeaseResponse

from .models import (
    LEASE_ACTIVE,
    LEASE_END_PENDING,
    LEASE_EXPIRED_HOLD,
    LEASE_INACTIVE,
    PROTOCOL_VERSION,
    LeaseExpiryEvent,
    SafetyLeaseConfig,
    SafetyLeaseRecord,
)


OPERATIONS = frozenset(("START", "RENEW", "END"))


class LeaseValidationError(ValueError):
    def __init__(self, error_code, message):
        super().__init__(message)
        self.error_code = error_code
        self.message = message


class SafetyLeaseWatchdog:
    def __init__(self, identity, config: SafetyLeaseConfig, on_expire, can_end, clock=time.monotonic):
        self.identity = identity
        self.config = config
        self._on_expire = on_expire
        self._can_end = can_end
        self._clock = clock
        self._lock = threading.RLock()
        self._record = None
        self._last_epoch = ""
        self._last_seq = -1
        self._last_fingerprint = None
        self._last_response = None
        self._expiry_dispatched = False

    @property
    def state(self):
        with self._lock:
            return self._record.state if self._record is not None else LEASE_INACTIVE

    def record(self):
        with self._lock:
            return None if self._record is None else replace(self._record)

    def permits_task(self, mission_id):
        if not self.config.required_for_tasks:
            return True
        with self._lock:
            return (
                self._record is not None
                and self._record.state == LEASE_ACTIVE
                and self._record.mission_id == mission_id
                and self._clock() - self._record.last_renew_monotonic_s < self._record.ttl_s
            )

    def handle_lease(self, request):
        try:
            self._validate_common(request)
            fingerprint = self._fingerprint(request)
            with self._lock:
                duplicate = self._check_sequence(request, fingerprint)
                if duplicate is not None:
                    return duplicate
                if request.operation == "START":
                    response = self._handle_start(request)
                elif request.operation == "RENEW":
                    response = self._handle_renew(request)
                else:
                    response = self._handle_end(request)
                if response.accepted:
                    self._remember(request, fingerprint, response)
                return response
        except LeaseValidationError as error:
            return UavSafetyLeaseResponse(False, self.state, error.error_code, error.message)

    def check_expiry(self, now_monotonic_s=None):
        now = self._clock() if now_monotonic_s is None else float(now_monotonic_s)
        with self._lock:
            if self._record is None or self._record.state != LEASE_ACTIVE:
                return None
            if now - self._record.last_renew_monotonic_s < self._record.ttl_s:
                return None
            self._record = replace(
                self._record, state=LEASE_EXPIRED_HOLD, expired_at_monotonic_s=now
            )
            if self._expiry_dispatched:
                return None
            self._expiry_dispatched = True
            event = LeaseExpiryEvent(
                self._record.mission_id, self._record.session_epoch, now
            )
        try:
            result = self._on_expire(event)
            success = bool(getattr(result, "success", result))
            error_code = str(getattr(result, "error_code", ""))
            message = str(getattr(result, "message", ""))
        except Exception as error:  # Safety state stays latched if HOLD raises.
            success = False
            error_code = "LEASE_HOLD_FAILED"
            message = str(error)
        return replace(
            event, hold_success=success, error_code=error_code, message=message
        )

    def _validate_common(self, request):
        if request.protocol_version != PROTOCOL_VERSION:
            raise LeaseValidationError("INVALID_LEASE_PROTOCOL", "unsupported protocol_version")
        if request.operation not in OPERATIONS:
            raise LeaseValidationError("INVALID_LEASE_OPERATION", "unknown lease operation")
        if not request.mission_id or not request.session_epoch:
            raise LeaseValidationError("LEASE_EPOCH_CONFLICT", "mission_id and session_epoch are required")
        if request.uav_id != self.identity.uav_id or request.exec_target != self.identity.exec_target:
            raise LeaseValidationError("LEASE_IDENTITY_MISMATCH", "lease identity mismatch")
        ttl = float(request.ttl_s)
        if request.operation in ("START", "RENEW") and (
            not math.isfinite(ttl)
            or ttl < self.config.min_ttl_s
            or ttl > self.config.max_ttl_s
        ):
            raise LeaseValidationError("INVALID_LEASE_TTL", "lease ttl is outside allowed range")

    def _check_sequence(self, request, fingerprint):
        if request.session_epoch != self._last_epoch:
            return None
        seq = int(request.lease_seq)
        if seq < self._last_seq:
            raise LeaseValidationError("LEASE_SEQ_STALE", "lease_seq moved backwards")
        if seq == self._last_seq:
            if fingerprint == self._last_fingerprint:
                return self._copy_response(self._last_response)
            raise LeaseValidationError("LEASE_SEQ_STALE", "lease_seq content conflict")
        return None

    def _handle_start(self, request):
        if self._record is not None:
            raise LeaseValidationError("LEASE_EPOCH_CONFLICT", "a lease epoch is already active")
        if request.session_epoch == self._last_epoch:
            raise LeaseValidationError("LEASE_EPOCH_CONFLICT", "ended epoch cannot restart")
        now = self._clock()
        self._record = SafetyLeaseRecord(
            request.mission_id,
            request.session_epoch,
            int(request.lease_seq),
            float(request.ttl_s),
            now,
        )
        self._expiry_dispatched = False
        return UavSafetyLeaseResponse(True, LEASE_ACTIVE, "", "safety lease started")

    def _handle_renew(self, request):
        record = self._require_epoch(request)
        if record.state == LEASE_EXPIRED_HOLD:
            raise LeaseValidationError("LEASE_EXPIRED", "expired lease is latched")
        if record.state != LEASE_ACTIVE:
            raise LeaseValidationError("LEASE_NOT_ACTIVE", "lease is not active")
        self._record = replace(
            record,
            last_seq=int(request.lease_seq),
            ttl_s=float(request.ttl_s),
            last_renew_monotonic_s=self._clock(),
        )
        return UavSafetyLeaseResponse(True, LEASE_ACTIVE, "", "safety lease renewed")

    def _handle_end(self, request):
        record = self._require_epoch(request)
        self._record = replace(record, state=LEASE_END_PENDING)
        safe, message = self._can_end()
        if not safe:
            self._record = record
            raise LeaseValidationError("LEASE_END_UNSAFE", message or "lease cannot end safely")
        self._last_epoch = request.session_epoch
        self._record = None
        self._expiry_dispatched = False
        return UavSafetyLeaseResponse(True, LEASE_INACTIVE, "", "safety lease ended")

    def _require_epoch(self, request):
        if self._record is None:
            raise LeaseValidationError("LEASE_NOT_ACTIVE", "no active lease")
        if (
            request.session_epoch != self._record.session_epoch
            or request.mission_id != self._record.mission_id
        ):
            raise LeaseValidationError("LEASE_EPOCH_CONFLICT", "lease epoch mismatch")
        return self._record

    def _remember(self, request, fingerprint, response):
        self._last_epoch = request.session_epoch
        self._last_seq = int(request.lease_seq)
        self._last_fingerprint = fingerprint
        self._last_response = self._copy_response(response)

    @staticmethod
    def _fingerprint(request):
        return (
            request.protocol_version,
            request.operation,
            request.mission_id,
            request.session_epoch,
            int(request.lease_seq),
            float(request.ttl_s),
            request.uav_id,
            request.exec_target,
            request.reason,
        )

    @staticmethod
    def _copy_response(response):
        return UavSafetyLeaseResponse(
            response.accepted, response.state, response.error_code, response.message
        )