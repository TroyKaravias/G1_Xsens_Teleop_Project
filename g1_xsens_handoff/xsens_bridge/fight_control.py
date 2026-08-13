"""Fail-closed fight-mode supervisor.

This module produces intent only.  It contains no ZMQ, Unitree SDK, DDS, or
motor output and is therefore suitable for deterministic simulation tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class FightState(str, Enum):
    DISARMED = "DISARMED"
    GUARD = "GUARD"
    POSE = "POSE"
    LOCOMOTION = "LOCOMOTION"
    BOXING = "BOXING"
    FALLEN = "FALLEN"
    GET_UP = "GET_UP"
    RECOVERING = "RECOVERING"
    ESTOP = "ESTOP"


class FightRoute(str, Enum):
    SAFE = "SAFE"
    POSE = "POSE"
    PLANNER = "PLANNER"
    REFERENCE = "REFERENCE"


class FightManeuver(str, Enum):
    IDLE_BOXING = "IDLE_BOXING"
    WALK_BOXING = "WALK_BOXING"
    LEFT_JAB = "LEFT_JAB"
    RIGHT_JAB = "RIGHT_JAB"
    LEFT_HOOK = "LEFT_HOOK"
    RIGHT_HOOK = "RIGHT_HOOK"


class Punch(str, Enum):
    NONE = "NONE"
    LEFT_JAB = "LEFT_JAB"
    RIGHT_JAB = "RIGHT_JAB"
    LEFT_HOOK = "LEFT_HOOK"
    RIGHT_HOOK = "RIGHT_HOOK"


@dataclass(frozen=True)
class FightRequest:
    armed: bool = False
    stream_fresh: bool = False
    operator_ready: bool = False
    pose_requested: bool = True
    forward_mps: float = 0.0
    lateral_mps: float = 0.0
    yaw_rate_rad_s: float = 0.0
    target_height_m: float = -1.0
    punch: Punch = Punch.NONE
    fall_detected: bool = False
    robot_stable: bool = False
    request_get_up: bool = False
    get_up_complete: bool = False
    estop: bool = False
    reset_estop: bool = False


@dataclass(frozen=True)
class FightOutput:
    state: FightState
    route: FightRoute
    maneuver: FightManeuver
    forward_mps: float = 0.0
    lateral_mps: float = 0.0
    yaw_rate_rad_s: float = 0.0
    target_height_m: float = -1.0
    upper_body_tracking: bool = False
    reference_name: str | None = None
    estop_latched: bool = False
    reason: str = ""


@dataclass(frozen=True)
class RobotBalanceSample:
    tilt_rad: float
    root_height_m: float
    angular_speed_rad_s: float


@dataclass(frozen=True)
class FallStatus:
    fallen: bool
    stable: bool


class FallDetector:
    """Debounce robot-state fall and settled conditions.

    This consumes *robot* estimator data, not the Xsens operator pose.  A
    supervisor latches the resulting fall until an explicit get-up completes.
    """

    def __init__(
        self,
        *,
        tilt_limit_rad: float = math.radians(60.0),
        height_limit_m: float = 0.45,
        fall_dwell_s: float = 0.15,
        stable_angular_speed_rad_s: float = 0.25,
        stable_dwell_s: float = 0.50,
    ) -> None:
        self.tilt_limit_rad = tilt_limit_rad
        self.height_limit_m = height_limit_m
        self.fall_dwell_s = fall_dwell_s
        self.stable_angular_speed_rad_s = stable_angular_speed_rad_s
        self.stable_dwell_s = stable_dwell_s
        self._fall_since: float | None = None
        self._stable_since: float | None = None

    def reset(self) -> None:
        self._fall_since = None
        self._stable_since = None

    def update(self, now: float, sample: RobotBalanceSample) -> FallStatus:
        values = (
            sample.tilt_rad,
            sample.root_height_m,
            sample.angular_speed_rad_s,
        )
        if not all(math.isfinite(value) for value in values):
            return FallStatus(fallen=True, stable=False)

        beyond_limit = (
            abs(sample.tilt_rad) >= self.tilt_limit_rad
            or sample.root_height_m <= self.height_limit_m
        )
        if beyond_limit:
            self._fall_since = now if self._fall_since is None else self._fall_since
        else:
            self._fall_since = None

        settled = abs(sample.angular_speed_rad_s) <= self.stable_angular_speed_rad_s
        if settled:
            self._stable_since = (
                now if self._stable_since is None else self._stable_since
            )
        else:
            self._stable_since = None

        return FallStatus(
            fallen=(
                self._fall_since is not None
                and now - self._fall_since >= self.fall_dwell_s
            ),
            stable=(
                self._stable_since is not None
                and now - self._stable_since >= self.stable_dwell_s
            ),
        )


class FightSupervisor:
    """Select exactly one safe, pose, planner, or reference route."""

    def __init__(
        self,
        *,
        max_forward_mps: float = 0.30,
        max_backward_mps: float = 0.25,
        max_lateral_mps: float = 0.25,
        max_translation_mps: float = 0.30,
        max_yaw_rate_rad_s: float = 0.50,
        punch_duration_s: float = 0.65,
        punch_cooldown_s: float = 0.45,
        fallen_settle_s: float = 0.50,
        get_up_timeout_s: float = 12.0,
        recovery_dwell_s: float = 1.0,
    ) -> None:
        if min(
            max_forward_mps,
            max_backward_mps,
            max_lateral_mps,
            max_translation_mps,
            max_yaw_rate_rad_s,
            punch_duration_s,
            fallen_settle_s,
            get_up_timeout_s,
            recovery_dwell_s,
        ) <= 0.0 or punch_cooldown_s < 0.0:
            raise ValueError("fight limits and dwell times are invalid")
        self.max_forward_mps = max_forward_mps
        self.max_backward_mps = max_backward_mps
        self.max_lateral_mps = max_lateral_mps
        self.max_translation_mps = max_translation_mps
        self.max_yaw_rate_rad_s = max_yaw_rate_rad_s
        self.punch_duration_s = punch_duration_s
        self.punch_cooldown_s = punch_cooldown_s
        self.fallen_settle_s = fallen_settle_s
        self.get_up_timeout_s = get_up_timeout_s
        self.recovery_dwell_s = recovery_dwell_s
        self.state = FightState.DISARMED
        self._estop_latched = False
        self._fallen_at: float | None = None
        self._stable_since: float | None = None
        self._get_up_started: float | None = None
        self._recovery_started: float | None = None
        self._punch_until = 0.0
        self._punch_cooldown_until = 0.0
        self._active_punch = Punch.NONE

    def _safe(self, reason: str) -> FightOutput:
        return FightOutput(
            state=self.state,
            route=FightRoute.SAFE,
            maneuver=FightManeuver.IDLE_BOXING,
            estop_latched=self._estop_latched,
            reason=reason,
        )

    @staticmethod
    def _finite_request(request: FightRequest) -> bool:
        return all(math.isfinite(value) for value in (
            request.forward_mps,
            request.lateral_mps,
            request.yaw_rate_rad_s,
            request.target_height_m,
        ))

    def update(self, now: float, request: FightRequest) -> FightOutput:
        if not math.isfinite(now):
            raise ValueError("now must be finite")

        if request.estop:
            self._estop_latched = True
            self.state = FightState.ESTOP

        if self._estop_latched:
            if (
                request.reset_estop
                and not request.estop
                and not request.armed
                and request.robot_stable
            ):
                self._estop_latched = False
                self.state = FightState.DISARMED
                return self._safe("estop-reset-disarmed")
            self.state = FightState.ESTOP
            return self._safe("estop-latched")

        if not self._finite_request(request):
            self.state = FightState.DISARMED
            return self._safe("non-finite-request")

        if request.fall_detected and self.state is not FightState.GET_UP:
            if self.state is not FightState.FALLEN:
                self._fallen_at = now
            self.state = FightState.FALLEN
            self._get_up_started = None
            self._recovery_started = None

        if self.state is FightState.GET_UP:
            assert self._get_up_started is not None
            if now - self._get_up_started > self.get_up_timeout_s:
                self.state = FightState.FALLEN
                self._fallen_at = now
                return self._safe("get-up-timeout")
            if request.get_up_complete and request.robot_stable:
                self.state = FightState.RECOVERING
                self._recovery_started = now
                return self._safe("get-up-complete")
            return FightOutput(
                state=self.state,
                route=FightRoute.REFERENCE,
                maneuver=FightManeuver.IDLE_BOXING,
                reference_name="get_up",
                reason="get-up-reference",
            )

        if self.state is FightState.FALLEN:
            if request.robot_stable:
                self._stable_since = now if self._stable_since is None else self._stable_since
            else:
                self._stable_since = None
            settled = (
                self._stable_since is not None
                and now - self._stable_since >= self.fallen_settle_s
            )
            if request.request_get_up and request.operator_ready and settled:
                self.state = FightState.GET_UP
                self._get_up_started = now
                return FightOutput(
                    state=self.state,
                    route=FightRoute.REFERENCE,
                    maneuver=FightManeuver.IDLE_BOXING,
                    reference_name="get_up",
                    reason="get-up-started",
                )
            return self._safe("fallen-waiting-settle" if not settled else "fallen-ready")

        if self.state is FightState.RECOVERING:
            assert self._recovery_started is not None
            if not request.robot_stable:
                self.state = FightState.FALLEN
                self._fallen_at = now
                return self._safe("recovery-unstable")
            if now - self._recovery_started < self.recovery_dwell_s:
                return self._safe("recovery-dwell")
            self.state = FightState.GUARD

        if not request.armed:
            self.state = FightState.DISARMED
            return self._safe("operator-disarmed")
        if not request.operator_ready:
            self.state = FightState.DISARMED
            return self._safe("operator-not-ready")
        if not request.stream_fresh:
            self.state = FightState.DISARMED
            return self._safe("xsens-stale")

        if self.state is FightState.BOXING and now < self._punch_until:
            return FightOutput(
                state=self.state,
                route=FightRoute.PLANNER,
                maneuver=FightManeuver(self._active_punch.value),
                reason="punch-active",
            )
        if self.state is FightState.BOXING:
            self._punch_cooldown_until = now + self.punch_cooldown_s
            self._active_punch = Punch.NONE
            self.state = FightState.GUARD

        if request.punch is not Punch.NONE and now >= self._punch_cooldown_until:
            self.state = FightState.BOXING
            self._active_punch = request.punch
            self._punch_until = now + self.punch_duration_s
            return FightOutput(
                state=self.state,
                route=FightRoute.PLANNER,
                maneuver=FightManeuver(request.punch.value),
                reason="punch-started",
            )

        forward = min(max(request.forward_mps, -self.max_backward_mps), self.max_forward_mps)
        lateral = min(max(request.lateral_mps, -self.max_lateral_mps), self.max_lateral_mps)
        translation_norm = math.hypot(forward, lateral)
        if translation_norm > self.max_translation_mps:
            scale = self.max_translation_mps / translation_norm
            forward *= scale
            lateral *= scale
        yaw_rate = min(max(request.yaw_rate_rad_s, -self.max_yaw_rate_rad_s), self.max_yaw_rate_rad_s)
        moving = max(abs(forward), abs(lateral), abs(yaw_rate)) > 1e-6
        if moving:
            self.state = FightState.LOCOMOTION
            return FightOutput(
                state=self.state,
                route=FightRoute.PLANNER,
                maneuver=FightManeuver.WALK_BOXING,
                forward_mps=forward,
                lateral_mps=lateral,
                yaw_rate_rad_s=yaw_rate,
                target_height_m=request.target_height_m,
                upper_body_tracking=True,
                reason="bounded-locomotion",
            )
        if request.pose_requested:
            self.state = FightState.POSE
            return FightOutput(
                state=self.state,
                route=FightRoute.POSE,
                maneuver=FightManeuver.IDLE_BOXING,
                upper_body_tracking=True,
                reason="guarded-xsens-pose",
            )
        self.state = FightState.GUARD
        return FightOutput(
            state=self.state,
            route=FightRoute.PLANNER,
            maneuver=FightManeuver.IDLE_BOXING,
            reason="planner-guard",
        )
