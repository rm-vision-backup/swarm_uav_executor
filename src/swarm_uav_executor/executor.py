"""Application service coordinating validation, idempotency and local motion."""
from __future__ import annotations
import threading
import time
from swarm_uav_interfaces.srv import UavHoldResponse, UavTaskResponse
from .models import (CONFLICT, DUPLICATE, HoldGoal, SafetyHoldResult, STATE_ACCEPTED,
                     STATE_COMPLETED, STATE_FAILED, TaskRecord)
from .validation import (RequestValidationError, build_goal, request_fingerprint, task_key_from_request,
                         validate_hold_request, validate_task_request)


class UavTaskExecutor:
    def __init__(self, identity, config, store, reporter, driver, clock=time.time, lease_guard=None):
        self.identity = identity; self.config = config; self.store = store; self.reporter = reporter
        self.driver = driver; self.clock = clock; self._lock = threading.RLock(); self._cancel = None
        self._hold_in_progress = False; self._shutdown = False; self._threads = []
        self.lease_guard = lease_guard; self._local_safety_latched = False

    def handle_task(self, request):
        try:
            validate_task_request(request, self.identity, self.config.supported_commands)
        except RequestValidationError as error:
            return UavTaskResponse(False, STATE_FAILED, error.error_code, error.message)
        key = task_key_from_request(request); fingerprint = request_fingerprint(request)
        with self._lock:
            current = self.store.get(key)
            if current is not None:
                if current.fingerprint != fingerprint:
                    return UavTaskResponse(False, current.status, "DUPLICATE_CONFLICT", "same task key has different content")
                self.reporter.publish(current)
                return UavTaskResponse(True, current.status, current.error_code, current.message)
            if self._shutdown:
                return UavTaskResponse(False, STATE_FAILED, "SHUTTING_DOWN", "executor is shutting down")
            if self._local_safety_latched:
                return UavTaskResponse(False, STATE_FAILED, "LEASE_EXPIRED", "local safety HOLD is latched")
            if self.lease_guard is not None and not self.lease_guard.permits_task(request.mission_id):
                return UavTaskResponse(False, STATE_FAILED, "LEASE_NOT_ACTIVE", "matching safety lease is not active")
            if self._hold_in_progress or self.store.active() is not None:
                return UavTaskResponse(False, STATE_FAILED, "BUSY", "another task or HOLD is active")
            health = self.driver.health()
            if not health.ready:
                return UavTaskResponse(False, STATE_FAILED, health.error_code or "DRIVER_NOT_READY", health.message)
            goal = build_goal(request)
            record = TaskRecord(key, fingerprint, request.exec_target, request.command, float(request.timeout_s), goal,
                                updated_at=self.clock(), message="task accepted")
            self.store.register(record); self.reporter.publish(record); self._cancel = threading.Event()
            thread = threading.Thread(target=self._run_task, args=(key, goal, self._cancel), daemon=True)
            self._threads.append(thread); thread.start()
        return UavTaskResponse(True, STATE_ACCEPTED, "", "task accepted")

    def handle_hold(self, request):
        try: validate_hold_request(request, self.identity)
        except RequestValidationError as error: return UavHoldResponse(False, error.error_code, error.message)
        with self._lock:
            if self._hold_in_progress: return UavHoldResponse(True, "", "HOLD already active")
            interrupted = self.store.active(); self._hold_in_progress = True
            if self._cancel is not None: self._cancel.set()
            thread = threading.Thread(target=self._run_hold, args=(interrupted, request.reason), daemon=True)
            self._threads.append(thread); thread.start()
        return UavHoldResponse(True, "", "HOLD accepted")

    def _run_task(self, key, goal, cancel_event):
        record = self.store.get(key); deadline = self.clock() + record.timeout_s
        try: result = self.driver.start_move_to(goal, cancel_event, deadline)
        except Exception as error: result = type("Result", (), {"success": False, "error_code": "DRIVER_EXCEPTION", "message": str(error)})()
        if not result.success:
            with self._lock:
                current = self.store.get(key)
                if current is None or current.terminal or cancel_event.is_set(): return
                # Own the safety transition before replacing the motion target,
                # so no new task or concurrent HOLD can race this fallback.
                self._hold_in_progress = True
            try:
                hold_result = self.driver.hold(
                    HoldGoal("%s: %s" % (result.error_code, result.message)),
                    self.clock() + self.config.shutdown_hold_timeout_s,
                )
            except Exception as error:
                hold_result = type("Result", (), {
                    "success": False, "error_code": "HOLD_FAILED", "message": str(error)
                })()
            with self._lock:
                current = self.store.get(key)
                if current is not None and not current.terminal:
                    if hold_result.success:
                        error_code = result.error_code or "MOTION_FAILED"
                        message = result.message
                    else:
                        error_code = "HOLD_FAILED"
                        message = "%s: %s; safety HOLD failed: %s" % (
                            result.error_code or "MOTION_FAILED", result.message, hold_result.message
                        )
                    self.reporter.publish_transition(key, STATE_FAILED, "HOLD", error_code, message)
                self._cancel = None; self._hold_in_progress = False
            return
        with self._lock:
            current = self.store.get(key)
            if current is None or current.terminal: return
            if cancel_event.is_set(): return  # HOLD owns the terminal transition.
            self.reporter.publish_transition(key, STATE_COMPLETED, STATE_COMPLETED, result.error_code, result.message)
            self._cancel = None

    def _run_hold(self, interrupted, reason):
        try: result = self.driver.hold(HoldGoal(reason), self.clock() + self.config.shutdown_hold_timeout_s)
        except Exception as error: result = type("Result", (), {"success": False, "error_code": "HOLD_FAILED", "message": str(error)})()
        with self._lock:
            if interrupted is not None:
                current = self.store.get(interrupted.key)
                if current is not None and not current.terminal:
                    error_code = "COMMAND_HELD" if result.success else "HOLD_FAILED"
                    message = "task interrupted by HOLD" if result.success else result.message
                    self.reporter.publish_transition(current.key, STATE_FAILED, "HOLD", error_code, message)
            self._cancel = None; self._hold_in_progress = False

    def _fail_active_and_hold(self, error_code, message):
        active = self.store.active()
        if active is not None:
            class HoldRequest: pass
            request = HoldRequest(); request.protocol_version = "1.0"; request.mission_id = active.key.mission_id
            request.command_id = active.key.command_id; request.uav_id = self.identity.uav_id
            request.exec_target = self.identity.exec_target; request.reason = "%s: %s" % (error_code, message)
            self.handle_hold(request)

    def trigger_local_safety_hold(self, error_code, message, latch=True):
        """Atomically block new work and capture a local HOLD, even between tasks."""
        with self._lock:
            if self._local_safety_latched and latch:
                return SafetyHoldResult(True, "", "local safety HOLD already latched")
            interrupted = self.store.active()
            if latch: self._local_safety_latched = True
            if self._hold_in_progress:
                return SafetyHoldResult(True, "", "HOLD already active")
            self._hold_in_progress = True
            if self._cancel is not None: self._cancel.set()
        try:
            result = self.driver.hold(
                HoldGoal("%s: %s" % (error_code, message)),
                self.clock() + self.config.shutdown_hold_timeout_s,
            )
        except Exception as error:
            result = type("Result", (), {
                "success": False, "error_code": "HOLD_FAILED", "message": str(error)
            })()
        with self._lock:
            if interrupted is not None:
                current = self.store.get(interrupted.key)
                if current is not None and not current.terminal:
                    terminal_code = error_code if result.success else "HOLD_FAILED"
                    terminal_message = message if result.success else (
                        "%s; safety HOLD failed: %s" % (message, result.message)
                    )
                    self.reporter.publish_transition(
                        current.key, STATE_FAILED, "HOLD", terminal_code, terminal_message
                    )
            self._cancel = None; self._hold_in_progress = False
        return SafetyHoldResult(bool(result.success), result.error_code, result.message)

    def release_local_safety_latch(self):
        """Allow future work after the active lease epoch has ended safely."""
        with self._lock:
            self._local_safety_latched = False

    def shutdown(self):
        with self._lock: self._shutdown = True
        active = self.store.active()
        if active is not None: self._fail_active_and_hold("SHUTDOWN", "executor shutdown")
        end = self.clock() + self.config.shutdown_hold_timeout_s
        for thread in tuple(self._threads): thread.join(max(0.0, end - self.clock()))
        self.reporter.shutdown(); self.driver.shutdown()
