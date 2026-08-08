#!/usr/bin/env python3
import unittest

from swarm_uav_interfaces.srv import UavSafetyLeaseRequest
from swarm_uav_executor.models import (
    LEASE_ACTIVE,
    LEASE_EXPIRED_HOLD,
    LEASE_INACTIVE,
    ExecutorIdentity,
    SafetyLeaseConfig,
)
from swarm_uav_executor.safety_lease import SafetyLeaseWatchdog


class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now


def request(operation="START", epoch="epoch-1", seq=1, ttl=5.0, mission="mission-1"):
    return UavSafetyLeaseRequest("1.0", operation, mission, epoch, seq, ttl,
                                 "A01", "UAV1", "test")


class SafetyLeaseWatchdogTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(); self.expiries = []; self.safe_to_end = True; self.safe_ends = 0
        config = SafetyLeaseConfig("/UAV1/uav_safety_lease")
        self.watchdog = SafetyLeaseWatchdog(
            ExecutorIdentity("A01", "UAV1"), config,
            lambda event: self.expiries.append(event) or True,
            lambda: (self.safe_to_end, "still armed"), self.clock,
            lambda: setattr(self, "safe_ends", self.safe_ends + 1))

    def test_start_renew_and_exact_duplicate_are_idempotent(self):
        start = request(); self.assertTrue(self.watchdog.handle_lease(start).accepted)
        self.assertTrue(self.watchdog.handle_lease(start).accepted)
        self.clock.now = 4.0
        renew = request("RENEW", seq=2)
        self.assertTrue(self.watchdog.handle_lease(renew).accepted)
        self.assertTrue(self.watchdog.handle_lease(renew).accepted)
        self.assertEqual(LEASE_ACTIVE, self.watchdog.state)
        self.assertTrue(self.watchdog.permits_task("mission-1"))

    def test_rejects_stale_conflicting_epoch_identity_and_ttl(self):
        self.watchdog.handle_lease(request())
        self.assertEqual("LEASE_SEQ_STALE", self.watchdog.handle_lease(request("RENEW", seq=0)).error_code)
        conflict = request(); conflict.reason = "changed"
        self.assertEqual("LEASE_SEQ_STALE", self.watchdog.handle_lease(conflict).error_code)
        self.assertEqual("LEASE_EPOCH_CONFLICT", self.watchdog.handle_lease(request(epoch="epoch-2")).error_code)
        wrong = request("RENEW", seq=2); wrong.uav_id = "A02"
        self.assertEqual("LEASE_IDENTITY_MISMATCH", self.watchdog.handle_lease(wrong).error_code)
        self.assertEqual("INVALID_LEASE_TTL", self.watchdog.handle_lease(request("RENEW", seq=2, ttl=0.1)).error_code)

    def test_expiry_dispatches_hold_once_and_latches(self):
        self.watchdog.handle_lease(request())
        self.clock.now = 4.999; self.assertIsNone(self.watchdog.check_expiry())
        self.clock.now = 5.0; event = self.watchdog.check_expiry()
        self.assertTrue(event.hold_success); self.assertEqual(1, len(self.expiries))
        self.assertEqual(LEASE_EXPIRED_HOLD, self.watchdog.state)
        self.assertIsNone(self.watchdog.check_expiry())
        self.assertEqual("LEASE_EXPIRED", self.watchdog.handle_lease(request("RENEW", seq=2)).error_code)
        self.assertFalse(self.watchdog.permits_task("mission-1"))

    def test_hold_exception_is_reported_without_unlatching(self):
        self.watchdog._on_expire = lambda _event: (_ for _ in ()).throw(RuntimeError("hold failed"))
        self.watchdog.handle_lease(request()); self.clock.now = 5.0
        event = self.watchdog.check_expiry()
        self.assertFalse(event.hold_success); self.assertEqual("LEASE_HOLD_FAILED", event.error_code)
        self.assertEqual(LEASE_EXPIRED_HOLD, self.watchdog.state)

    def test_end_requires_safe_condition_then_allows_new_epoch(self):
        self.watchdog.handle_lease(request()); self.safe_to_end = False
        response = self.watchdog.handle_lease(request("END", seq=2))
        self.assertEqual("LEASE_END_UNSAFE", response.error_code)
        self.assertEqual(LEASE_ACTIVE, self.watchdog.state)
        self.assertEqual(0, self.safe_ends)
        self.safe_to_end = True
        response = self.watchdog.handle_lease(request("END", seq=2))
        self.assertTrue(response.accepted); self.assertEqual(LEASE_INACTIVE, self.watchdog.state)
        self.assertEqual(1, self.safe_ends)
        self.assertTrue(self.watchdog.handle_lease(request(epoch="epoch-2", seq=1)).accepted)

    def test_safe_end_callback_failure_keeps_lease_latched(self):
        self.watchdog.handle_lease(request())
        self.watchdog._on_safe_end = lambda: (_ for _ in ()).throw(RuntimeError("unlock failed"))
        response = self.watchdog.handle_lease(request("END", seq=2))
        self.assertEqual("LEASE_END_FAILED", response.error_code)
        self.assertEqual(LEASE_ACTIVE, self.watchdog.state)

    def test_expired_epoch_can_end_but_cannot_restart(self):
        self.watchdog.handle_lease(request()); self.clock.now = 5.0; self.watchdog.check_expiry()
        self.assertTrue(self.watchdog.handle_lease(request("END", seq=2)).accepted)
        self.assertEqual("LEASE_SEQ_STALE", self.watchdog.handle_lease(request(seq=1)).error_code)

    def test_validation_errors(self):
        bad = request(); bad.protocol_version = "2.0"
        self.assertEqual("INVALID_LEASE_PROTOCOL", self.watchdog.handle_lease(bad).error_code)
        bad = request(); bad.operation = "PAUSE"
        self.assertEqual("INVALID_LEASE_OPERATION", self.watchdog.handle_lease(bad).error_code)


if __name__ == "__main__": unittest.main()