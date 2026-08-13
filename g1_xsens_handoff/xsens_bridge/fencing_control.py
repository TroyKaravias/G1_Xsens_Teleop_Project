"""Deterministic, robot-output-free fencing mode supervisor.

The supervisor separates expressive upper-body pose tracking from dynamic
footwork. It emits bounded intent values for a simulator/planner adapter but
contains no Unitree SDK, DDS, ROS, or motor-command code.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class FencingState(str, Enum):
    SAFE = "SAFE"
    EN_GARDE = "EN_GARDE"
    POSE_TRACKING = "POSE_TRACKING"
    ADVANCE = "ADVANCE"
    RETREAT = "RETREAT"
    LUNGE = "LUNGE"
    RECOVER = "RECOVER"
    ESTOP = "ESTOP"


class FencingEvent(str, Enum):
    NONE = "NONE"
    ENTER_GARDE = "ENTER_GARDE"
    ENABLE_POSE = "ENABLE_POSE"
    ADVANCE = "ADVANCE"
    RETREAT = "RETREAT"
    LUNGE = "LUNGE"
    RECOVER = "RECOVER"
    DISARM = "DISARM"
    ESTOP = "ESTOP"
    RESET_ESTOP = "RESET_ESTOP"


@dataclass(frozen=True)
class FencingLimits:
    """Conservative initial limits for restrained simulation/hardware tests."""

    advance_speed_mps: float = 0.15
    retreat_speed_mps: float = 0.12
    step_duration_s: float = 0.45
    lunge_duration_s: float = 0.65
    recovery_duration_s: float = 0.80
    minimum_guard_dwell_s: float = 0.50

    def __post_init__(self) -> None:
        values = (
            self.advance_speed_mps,
            self.retreat_speed_mps,
            self.step_duration_s,
            self.lunge_duration_s,
            self.recovery_duration_s,
            self.minimum_guard_dwell_s,
        )
        if any(value <= 0.0 for value in values):
            raise ValueError("All fencing limits must be positive")


@dataclass(frozen=True)
class FencingIntent:
    state: FencingState
    pose_tracking_enabled: bool
    forward_velocity_mps: float
    lunge_phase: float
    request_safe_return: bool
    estop_latched: bool


class FencingSupervisor:
    """Safety-gated fencing state machine for planner/pose intent generation."""

    def __init__(self, limits: FencingLimits | None = None) -> None:
        self.limits = FencingLimits() if limits is None else limits
        self.state = FencingState.SAFE
        self._state_started_at = 0.0
        self._estop_latched = False
        self._last_time: float | None = None

    def _transition(self, state: FencingState, now: float) -> None:
        self.state = state
        self._state_started_at = now

    def update(
        self,
        now: float,
        *,
        stream_fresh: bool,
        operator_ready: bool,
        event: FencingEvent = FencingEvent.NONE,
    ) -> FencingIntent:
        if self._last_time is not None and now < self._last_time:
            raise ValueError("Fencing supervisor time must not move backward")
        self._last_time = now

        if event is FencingEvent.ESTOP:
            self._estop_latched = True
            self._transition(FencingState.ESTOP, now)

        if self._estop_latched:
            if (
                event is FencingEvent.RESET_ESTOP
                and stream_fresh
                and operator_ready
            ):
                self._estop_latched = False
                self._transition(FencingState.SAFE, now)
            return self.intent(now)

        if event is FencingEvent.DISARM:
            self._transition(FencingState.SAFE, now)
            return self.intent(now)

        # Any loss of the live operator reference during an active state first
        # requests a controlled recovery. If freshness is not restored by the
        # end of recovery, the supervisor returns to SAFE.
        if not stream_fresh and self.state in {
            FencingState.EN_GARDE,
            FencingState.POSE_TRACKING,
            FencingState.ADVANCE,
            FencingState.RETREAT,
            FencingState.LUNGE,
        }:
            self._transition(FencingState.RECOVER, now)

        elapsed = now - self._state_started_at

        if self.state is FencingState.SAFE:
            if (
                event is FencingEvent.ENTER_GARDE
                and stream_fresh
                and operator_ready
            ):
                self._transition(FencingState.EN_GARDE, now)

        elif self.state is FencingState.EN_GARDE:
            if (
                event is FencingEvent.ENABLE_POSE
                and stream_fresh
                and operator_ready
                and elapsed >= self.limits.minimum_guard_dwell_s
            ):
                self._transition(FencingState.POSE_TRACKING, now)

        elif self.state is FencingState.POSE_TRACKING:
            if event is FencingEvent.ADVANCE and operator_ready:
                self._transition(FencingState.ADVANCE, now)
            elif event is FencingEvent.RETREAT and operator_ready:
                self._transition(FencingState.RETREAT, now)
            elif event is FencingEvent.LUNGE and operator_ready:
                self._transition(FencingState.LUNGE, now)
            elif event is FencingEvent.RECOVER:
                self._transition(FencingState.RECOVER, now)

        elif self.state in {FencingState.ADVANCE, FencingState.RETREAT}:
            if event is FencingEvent.RECOVER:
                self._transition(FencingState.RECOVER, now)
            elif elapsed >= self.limits.step_duration_s:
                self._transition(FencingState.POSE_TRACKING, now)

        elif self.state is FencingState.LUNGE:
            if (
                event is FencingEvent.RECOVER
                or elapsed >= self.limits.lunge_duration_s
            ):
                self._transition(FencingState.RECOVER, now)

        elif self.state is FencingState.RECOVER:
            if elapsed >= self.limits.recovery_duration_s:
                target = (
                    FencingState.EN_GARDE
                    if stream_fresh and operator_ready
                    else FencingState.SAFE
                )
                self._transition(target, now)

        return self.intent(now)

    def intent(self, now: float) -> FencingIntent:
        elapsed = max(0.0, now - self._state_started_at)
        velocity = 0.0
        if self.state is FencingState.ADVANCE:
            velocity = self.limits.advance_speed_mps
        elif self.state is FencingState.RETREAT:
            velocity = -self.limits.retreat_speed_mps

        lunge_phase = 0.0
        if self.state is FencingState.LUNGE:
            lunge_phase = min(
                1.0, elapsed / self.limits.lunge_duration_s
            )

        return FencingIntent(
            state=self.state,
            pose_tracking_enabled=self.state
            is FencingState.POSE_TRACKING,
            forward_velocity_mps=velocity,
            lunge_phase=lunge_phase,
            request_safe_return=self.state
            in {
                FencingState.SAFE,
                FencingState.EN_GARDE,
                FencingState.RECOVER,
                FencingState.ESTOP,
            },
            estop_latched=self._estop_latched,
        )
