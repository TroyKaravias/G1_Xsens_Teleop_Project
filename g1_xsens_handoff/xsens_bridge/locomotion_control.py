"""Bounded general-locomotion intent generation with no motor output."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LocomotionEvent(str, Enum):
    STOP = "STOP"
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"


@dataclass(frozen=True)
class LocomotionLimits:
    forward_mps: float = 0.20
    backward_mps: float = 0.12
    turn_rad_s: float = 0.35

    def __post_init__(self) -> None:
        if min(self.forward_mps, self.backward_mps, self.turn_rad_s) <= 0.0:
            raise ValueError("Locomotion limits must be positive")


@dataclass(frozen=True)
class LocomotionIntent:
    forward_velocity_mps: float = 0.0
    yaw_rate_rad_s: float = 0.0
    stopped: bool = True


class LocomotionSupervisor:
    def __init__(self, limits: LocomotionLimits | None = None) -> None:
        self.limits = LocomotionLimits() if limits is None else limits

    def update(
        self,
        event: LocomotionEvent,
        *,
        enabled: bool,
        stream_fresh: bool,
        operator_ready: bool,
    ) -> LocomotionIntent:
        if not enabled or not stream_fresh or not operator_ready:
            return LocomotionIntent()
        if event is LocomotionEvent.FORWARD:
            return LocomotionIntent(self.limits.forward_mps, 0.0, False)
        if event is LocomotionEvent.BACKWARD:
            return LocomotionIntent(-self.limits.backward_mps, 0.0, False)
        if event is LocomotionEvent.TURN_LEFT:
            return LocomotionIntent(0.0, self.limits.turn_rad_s, False)
        if event is LocomotionEvent.TURN_RIGHT:
            return LocomotionIntent(0.0, -self.limits.turn_rad_s, False)
        return LocomotionIntent()

