"""Application service coordinating validation, idempotency and local motion."""
from __future__ import annotations
import threading
import time
from swarm_uav_interfaces.srv import UavHoldResponse, UavTaskControlResponse, UavTaskResponse
from .models import (CONFLICT, DUPLICATE, HoldGoal, STATE_ACCEPTED,
                     STATE_COMPLETED, STATE_FAILED, TaskKey, TaskRecord)
from .validation import (RequestValidationError, build_goal, request_fingerprint, task_key_from_request,
                         validate_hold_request, validate_task_request)

# 变更 F（26090802）：组级安全 reason 约定（不改 msg/srv）。reason=GROUP_SAFETY
# 触发安全锁存（BRAKE_HOLD 语义 + 拒绝新任务/START）；GROUP_SAFETY_RESET 为最小
# 人工复位入口（节点重启亦复位）。只精确匹配字符串，不误伤普通收口 HOLD。
GROUP_SAFETY_REASON = "GROUP_SAFETY"
GROUP_SAFETY_RESET_REASON = "GROUP_SAFETY_RESET"


class UavTaskExecutor:
    def __init__(self, identity, config, store, reporter, driver, clock=time.time):
        self.identity = identity; self.config = config; self.store = store; self.reporter = reporter
        self.driver = driver; self.clock = clock; self._lock = threading.RLock(); self._cancel = None
        self._hold_in_progress = False; self._shutdown = False; self._threads = []
        # 变更 F：组级安全锁存位。置位后拒绝后续普通任务/START，仅人工复位
        # （节点重启或 UavHold reason=GROUP_SAFETY_RESET）。
        self._safety_latch = False

    def handle_task(self, request):
        try:
            validate_task_request(request, self.identity, self.config.supported_commands)
        except RequestValidationError as error:
            return UavTaskResponse(False, STATE_FAILED, error.error_code, error.message)
        with self._lock:
            if self._safety_latch:
                return UavTaskResponse(False, STATE_FAILED, "SAFETY_LATCHED",
                                       "group safety hold latched; manual reset required")
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
            if self._hold_in_progress or self.store.active() is not None:
                return UavTaskResponse(False, STATE_FAILED, "BUSY", "another task or HOLD is active")
            health = self.driver.health()
            if not health.ready:
                return UavTaskResponse(False, STATE_FAILED, health.error_code or "DRIVER_NOT_READY", health.message)
            goal = build_goal(request)
            prepared = self.driver.prepare(goal)
            if not prepared.ready:
                return UavTaskResponse(False, STATE_FAILED,
                                       prepared.error_code or "PREPARE_FAILED",
                                       prepared.message or "arm-before checks failed")
            record = TaskRecord(key, fingerprint, request.exec_target, request.command, float(request.timeout_s), goal,
                                updated_at=self.clock(), message="task prepared", prepared_at=self.clock())
            self.store.register(record); self.reporter.publish(record)
            if not self.config.require_explicit_start:
                self._start_record_locked(key)
        return UavTaskResponse(True, STATE_ACCEPTED, "",
                               "task prepared" if self.config.require_explicit_start else "task accepted")

    def _start_record_locked(self, key):
        record = self.store.mark_started(key, self.clock())
        self.reporter.publish(record)
        self._cancel = threading.Event()
        thread = threading.Thread(target=self._run_task, args=(key, record.goal, self._cancel), daemon=True)
        self._threads.append(thread); thread.start()
        return record

    def handle_task_control(self, request):
        operation = str(request.operation or "").upper()
        if request.protocol_version != "1.0":
            return UavTaskControlResponse(False, STATE_FAILED, "UNSUPPORTED_PROTOCOL", "protocol_version must be 1.0")
        if request.uav_id != self.identity.uav_id or request.exec_target != self.identity.exec_target:
            return UavTaskControlResponse(False, STATE_FAILED, "IDENTITY_MISMATCH", "request is not addressed to this UAV executor")
        key = TaskKey(request.mission_id, request.command_id, request.uav_id)
        if self.store.get(key) is None:
            return UavTaskControlResponse(False, STATE_FAILED, "TASK_NOT_PREPARED", "matching prepared task was not found")
        with self._lock:
            record = self.store.get(key)
            if operation == "ABORT":
                if record.terminal:
                    return UavTaskControlResponse(True, record.status, record.error_code, record.message)
                if record.started:
                    return UavTaskControlResponse(False, record.status, "TASK_ALREADY_STARTED",
                                                  "started task requires HOLD, not ABORT")
                if self._cancel is not None:
                    self._cancel.set()
                self.reporter.publish_transition(key, STATE_FAILED, "ABORTED", "TASK_ABORTED",
                                                 request.reason or "task aborted before group start")
                return UavTaskControlResponse(True, STATE_FAILED, "TASK_ABORTED", "task aborted")
            if operation != "START":
                return UavTaskControlResponse(False, record.status, "BAD_OPERATION", "operation must be START or ABORT")
            if self._safety_latch:
                return UavTaskControlResponse(False, record.status, "SAFETY_LATCHED",
                                              "group safety hold latched; manual reset required")
            if record.started:
                return UavTaskControlResponse(True, record.status, record.error_code,
                                              "task already started")
            if record.terminal:
                return UavTaskControlResponse(False, record.status, record.error_code or "TASK_TERMINAL",
                                              record.message or "task is already terminal")
            if self._hold_in_progress or self._cancel is not None:
                return UavTaskControlResponse(False, record.status, "BUSY", "another task or HOLD is active")
            # The record keeps the PREPARE timestamp, so the onboard deadline
            # includes time spent waiting at the group START barrier.
            self._start_record_locked(key)
        return UavTaskControlResponse(True, STATE_ACCEPTED, "", "task started")

    def handle_hold(self, request):
        try: validate_hold_request(request, self.identity)
        except RequestValidationError as error: return UavHoldResponse(False, error.error_code, error.message)
        with self._lock:
            if str(request.reason or "") == GROUP_SAFETY_RESET_REASON:
                # 最小人工复位入口：清除安全锁存位后仍执行一次 HOLD 确认（保持当前悬停），
                # 复位后新任务/START 可恢复接收。
                self._safety_latch = False
            elif str(request.reason or "") == GROUP_SAFETY_REASON:
                # 组级安全广播：置位本机安全锁存，进入 BRAKE_HOLD 锁存语义。
                self._safety_latch = True
            if self._hold_in_progress: return UavHoldResponse(True, "", "HOLD already active")
            interrupted = self.store.active(); self._hold_in_progress = True
            if self._cancel is not None: self._cancel.set()
            thread = threading.Thread(target=self._run_hold, args=(interrupted, request.reason), daemon=True)
            self._threads.append(thread); thread.start()
        return UavHoldResponse(True, "", "HOLD accepted")

    def _run_task(self, key, goal, cancel_event):
        record = self.store.get(key); deadline = record.prepared_at + record.timeout_s
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
                    self.clock() + self.config.ego_hold_timeout_s,
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
        try: result = self.driver.hold(HoldGoal(reason), self.clock() + self.config.ego_hold_timeout_s)
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

    def shutdown(self):
        with self._lock: self._shutdown = True
        active = self.store.active()
        if active is not None: self._fail_active_and_hold("SHUTDOWN", "executor shutdown")
        end = self.clock() + self.config.ego_hold_timeout_s
        for thread in tuple(self._threads): thread.join(max(0.0, end - self.clock()))
        self.reporter.shutdown(); self.driver.shutdown()
