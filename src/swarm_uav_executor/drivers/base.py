"""Replaceable local-only motion driver port."""
from __future__ import annotations
from abc import ABC, abstractmethod

from ..models import DriverHealth, HoldGoal, MotionGoal, MotionResult


class MotionDriver(ABC):
    @abstractmethod
    def start_move_to(self, goal: MotionGoal, cancel_event, deadline) -> MotionResult: ...
    @abstractmethod
    def hold(self, goal: HoldGoal, deadline) -> MotionResult: ...
    @abstractmethod
    def health(self) -> DriverHealth: ...
    @abstractmethod
    def shutdown(self) -> None: ...
