"""Deterministic mock driver for unit and ROS integration tests."""
from __future__ import annotations
import time
from typing import Callable
from .base import MotionDriver
from ..models import DriverHealth, HoldGoal, MotionResult


class MockMotionDriver(MotionDriver):
    def __init__(self, result_delay_s=0.5, final_success=True, error_code="", message="mock UAV task finished",
                 ready=True, hold_success=True, clock: Callable[[], float] = time.time) -> None:
        self.result_delay_s = float(result_delay_s); self.final_success = bool(final_success)
        self.error_code = error_code; self.message = message; self.ready = bool(ready)
        self.hold_success = bool(hold_success); self.clock = clock; self.start_count = 0; self.hold_count = 0
        self._shutdown = False

    def start_move_to(self, goal, cancel_event, deadline):
        self.start_count += 1
        end = self.clock() + self.result_delay_s
        while self.clock() < end:
            if self._shutdown or cancel_event.is_set():
                return MotionResult(False, "COMMAND_HELD", "motion cancelled by HOLD")
            if self.clock() >= float(deadline):
                return MotionResult(False, "LOCAL_TIMEOUT", "motion deadline exceeded")
            time.sleep(0.01)
        return MotionResult(self.final_success, "" if self.final_success else (self.error_code or "MOCK_FAILED"), self.message)

    def hold(self, goal: HoldGoal, deadline):
        self.hold_count += 1
        return MotionResult(self.hold_success, "" if self.hold_success else "HOLD_FAILED",
                            "hold target captured" if self.hold_success else "mock HOLD failed")

    def health(self):
        return DriverHealth(self.ready, "" if self.ready else "DRIVER_NOT_READY", "" if self.ready else "mock driver is not ready")

    def shutdown(self): self._shutdown = True
