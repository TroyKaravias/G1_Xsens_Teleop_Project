"""Experimental, reference-only stabilization for single-leg kick tracking.

This module never talks to Unitree hardware.  It shapes SONIC joint references
and is intentionally kept separate from the proven Xsens publisher.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from .g1_retarget import G1_DEFAULT_POSE


class KickPhase(str, Enum):
    DOUBLE_SUPPORT = "DOUBLE_SUPPORT"
    LEFT_SWING = "LEFT_SWING"
    RIGHT_SWING = "RIGHT_SWING"


@dataclass(frozen=True)
class KickStabilityLimits:
    lift_on_score_rad: float = 0.22
    lift_off_score_rad: float = 0.12
    side_margin_rad: float = 0.07
    phase_dwell_s: float = 0.06
    stance_protection: float = 0.70
    waist_protection: float = 0.35
    support_hip_pitch_limit_rad: float = 0.20
    support_hip_roll_limit_rad: float = 0.18
    support_hip_yaw_limit_rad: float = 0.20
    support_knee_limit_rad: float = 0.25
    support_ankle_pitch_limit_rad: float = 0.15
    support_ankle_roll_limit_rad: float = 0.10
    hold_swing_hip_pitch: bool = True
    balance_assist: bool = True
    balance_activation_score_rad: float = 0.18
    balance_full_score_rad: float = 0.45
    adaptive_stance_protection: float = 0.97
    adaptive_waist_protection: float = 0.85
    adaptive_support_hip_pitch_limit_rad: float = 0.10
    adaptive_support_hip_roll_limit_rad: float = 0.12
    adaptive_support_hip_yaw_limit_rad: float = 0.14
    adaptive_support_knee_limit_rad: float = 0.12
    adaptive_support_ankle_pitch_limit_rad: float = 0.08
    adaptive_support_ankle_roll_limit_rad: float = 0.08
    adaptive_support_hip_pitch_forward_limit_rad: float = 0.06
    adaptive_waist_roll_limit_rad: float = 0.10
    adaptive_waist_pitch_limit_rad: float = 0.05
    support_hip_pitch_counter_bias_rad: float = 0.08
    support_knee_counter_bias_rad: float = 0.10
    support_ankle_pitch_counter_bias_rad: float = 0.05
    waist_pitch_counter_bias_rad: float = 0.04
    swing_hip_pitch_limit_rad: float = 0.65
    swing_hip_roll_limit_rad: float = 0.40
    swing_hip_yaw_limit_rad: float = 0.60
    swing_ankle_pitch_limit_rad: float = 0.35
    swing_ankle_roll_limit_rad: float = 0.18
    waist_roll_limit_rad: float = 0.22
    waist_pitch_limit_rad: float = 0.22
    lower_body_slew_rad_s: float = 4.0
    maximum_dt_s: float = 0.04

    def __post_init__(self) -> None:
        positive = (
            self.lift_on_score_rad,
            self.lift_off_score_rad,
            self.side_margin_rad,
            self.phase_dwell_s,
            self.support_hip_pitch_limit_rad,
            self.support_hip_roll_limit_rad,
            self.support_hip_yaw_limit_rad,
            self.support_knee_limit_rad,
            self.support_ankle_pitch_limit_rad,
            self.support_ankle_roll_limit_rad,
            self.balance_activation_score_rad,
            self.balance_full_score_rad,
            self.adaptive_support_hip_pitch_limit_rad,
            self.adaptive_support_hip_roll_limit_rad,
            self.adaptive_support_hip_yaw_limit_rad,
            self.adaptive_support_knee_limit_rad,
            self.adaptive_support_ankle_pitch_limit_rad,
            self.adaptive_support_ankle_roll_limit_rad,
            self.adaptive_support_hip_pitch_forward_limit_rad,
            self.adaptive_waist_roll_limit_rad,
            self.adaptive_waist_pitch_limit_rad,
            self.support_hip_pitch_counter_bias_rad,
            self.support_knee_counter_bias_rad,
            self.support_ankle_pitch_counter_bias_rad,
            self.waist_pitch_counter_bias_rad,
            self.swing_hip_roll_limit_rad,
            self.swing_hip_yaw_limit_rad,
            self.swing_hip_pitch_limit_rad,
            self.swing_ankle_pitch_limit_rad,
            self.swing_ankle_roll_limit_rad,
            self.waist_roll_limit_rad,
            self.waist_pitch_limit_rad,
            self.lower_body_slew_rad_s,
            self.maximum_dt_s,
        )
        if min(positive) <= 0.0:
            raise ValueError("kick stability limits must be positive")
        if self.lift_off_score_rad >= self.lift_on_score_rad:
            raise ValueError("lift-off score must be lower than lift-on score")
        if not 0.0 <= self.stance_protection <= 1.0:
            raise ValueError("stance protection must be within [0, 1]")
        if not 0.0 <= self.waist_protection <= 1.0:
            raise ValueError("waist protection must be within [0, 1]")
        if not 0.0 <= self.adaptive_stance_protection <= 1.0:
            raise ValueError("adaptive stance protection must be within [0, 1]")
        if not 0.0 <= self.adaptive_waist_protection <= 1.0:
            raise ValueError("adaptive waist protection must be within [0, 1]")
        if self.balance_full_score_rad < self.balance_activation_score_rad:
            raise ValueError("balance full score must exceed activation score")


@dataclass(frozen=True)
class StabilizedKickReference:
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    phase: KickPhase
    support_side: str | None
    left_score_rad: float
    right_score_rad: float


class KickStabilityGovernor:
    """Protect the likely stance leg while retaining swing-leg kick intent."""

    # Once a likely support leg is identified, anchor the full leg rather than
    # only its yaw/ankle channels. That keeps the stance hip pitch and knee
    # from chasing the kicking leg and pitching the torso forward.
    _STANCE_LOCAL_INDICES = np.asarray([0, 1, 2, 3, 4, 5], dtype=np.int64)

    def __init__(self, limits: KickStabilityLimits | None = None) -> None:
        self.limits = limits or KickStabilityLimits()
        self.phase = KickPhase.DOUBLE_SUPPORT
        self._candidate_phase = self.phase
        self._candidate_since: float | None = None
        self._stance_reference = G1_DEFAULT_POSE.copy()
        self._swing_pitch_peak = np.zeros(2, dtype=np.float64)
        self._previous_position: np.ndarray | None = None
        self._previous_time: float | None = None

    def reset(self) -> None:
        self.phase = KickPhase.DOUBLE_SUPPORT
        self._candidate_phase = self.phase
        self._candidate_since = None
        self._stance_reference = G1_DEFAULT_POSE.copy()
        self._swing_pitch_peak.fill(0.0)
        self._previous_position = None
        self._previous_time = None

    @staticmethod
    def _validate_vector(name: str, value: np.ndarray) -> np.ndarray:
        result = np.asarray(value, dtype=np.float64)
        if result.shape != (29,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be a finite 29-element vector")
        return result

    @staticmethod
    def _leg_score(position: np.ndarray, start: int) -> float:
        delta = position[start : start + 6] - G1_DEFAULT_POSE[start : start + 6]
        # Hip pitch and knee flexion dominate a front-kick chamber/extension.
        # Roll and yaw retain sensitivity to diagonal and roundhouse chambers.
        return float(
            0.55 * abs(delta[0])
            + 0.30 * abs(delta[3])
            + 0.10 * abs(delta[1])
            + 0.05 * abs(delta[2])
        )

    def _desired_phase(self, left: float, right: float) -> KickPhase:
        limits = self.limits
        if self.phase is KickPhase.LEFT_SWING and left >= limits.lift_off_score_rad:
            return KickPhase.LEFT_SWING
        if self.phase is KickPhase.RIGHT_SWING and right >= limits.lift_off_score_rad:
            return KickPhase.RIGHT_SWING
        if (
            left >= limits.lift_on_score_rad
            and left - right >= limits.side_margin_rad
        ):
            return KickPhase.LEFT_SWING
        if (
            right >= limits.lift_on_score_rad
            and right - left >= limits.side_margin_rad
        ):
            return KickPhase.RIGHT_SWING
        return KickPhase.DOUBLE_SUPPORT

    def _update_phase(self, desired: KickPhase, now: float) -> None:
        if desired is self.phase:
            self._candidate_phase = self.phase
            self._candidate_since = None
            return
        if desired is not self._candidate_phase:
            self._candidate_phase = desired
            self._candidate_since = now
            return
        if (
            self._candidate_since is not None
            and now - self._candidate_since >= self.limits.phase_dwell_s
        ):
            self.phase = desired
            if desired is KickPhase.DOUBLE_SUPPORT:
                self._swing_pitch_peak.fill(0.0)
            self._candidate_phase = desired
            self._candidate_since = None

    def _hold_swing_chamber(self, target: np.ndarray, swing_start: int) -> None:
        if not self.limits.hold_swing_hip_pitch:
            return
        swing_index = 0 if swing_start == 0 else 1
        hip_pitch_index = swing_start
        neutral = G1_DEFAULT_POSE[hip_pitch_index]
        current_deviation = float(target[hip_pitch_index] - neutral)
        peak_deviation = self._swing_pitch_peak[swing_index]
        if abs(current_deviation) >= abs(peak_deviation):
            self._swing_pitch_peak[swing_index] = current_deviation
            return
        target[hip_pitch_index] = neutral + peak_deviation

    def _balance_weight(self, swing_score: float) -> float:
        if not self.limits.balance_assist:
            return 0.0
        start = self.limits.balance_activation_score_rad
        stop = self.limits.balance_full_score_rad
        if stop <= start:
            return 1.0 if swing_score >= stop else 0.0
        return float(np.clip((swing_score - start) / (stop - start), 0.0, 1.0))

    @staticmethod
    def _interpolate_limit(base_limit: float, adaptive_limit: float, weight: float) -> float:
        return float((1.0 - weight) * base_limit + weight * min(base_limit, adaptive_limit))

    def _shape_target(self, live: np.ndarray) -> np.ndarray:
        if self.phase is KickPhase.DOUBLE_SUPPORT:
            # Freeze the last confirmed double-support reference as soon as a
            # swing candidate appears. Otherwise support-foot motion during
            # the phase dwell would leak into the stance anchor.
            if self._candidate_phase is KickPhase.DOUBLE_SUPPORT:
                self._stance_reference = live.copy()
            return live.copy()

        target = live.copy()
        swing_start = 0 if self.phase is KickPhase.LEFT_SWING else 6
        support_start = 6 if swing_start == 0 else 0
        swing_score = self._leg_score(live, swing_start)
        balance_weight = self._balance_weight(swing_score)
        support_indices = support_start + self._STANCE_LOCAL_INDICES
        swing_hip_pitch_index = swing_start
        support_hip_pitch_index = support_start
        support_knee_index = support_start + 3
        support_ankle_pitch_index = support_start + 4
        protection = max(
            self.limits.stance_protection,
            balance_weight * self.limits.adaptive_stance_protection,
        )
        target[support_indices] = (
            (1.0 - protection) * target[support_indices]
            + protection * self._stance_reference[support_indices]
        )
        for local_index, limit in (
            (
                0,
                self._interpolate_limit(
                    self.limits.support_hip_pitch_limit_rad,
                    self.limits.adaptive_support_hip_pitch_limit_rad,
                    balance_weight,
                ),
            ),
            (
                1,
                self._interpolate_limit(
                    self.limits.support_hip_roll_limit_rad,
                    self.limits.adaptive_support_hip_roll_limit_rad,
                    balance_weight,
                ),
            ),
            (
                2,
                self._interpolate_limit(
                    self.limits.support_hip_yaw_limit_rad,
                    self.limits.adaptive_support_hip_yaw_limit_rad,
                    balance_weight,
                ),
            ),
            (
                3,
                self._interpolate_limit(
                    self.limits.support_knee_limit_rad,
                    self.limits.adaptive_support_knee_limit_rad,
                    balance_weight,
                ),
            ),
            (
                4,
                self._interpolate_limit(
                    self.limits.support_ankle_pitch_limit_rad,
                    self.limits.adaptive_support_ankle_pitch_limit_rad,
                    balance_weight,
                ),
            ),
            (
                5,
                self._interpolate_limit(
                    self.limits.support_ankle_roll_limit_rad,
                    self.limits.adaptive_support_ankle_roll_limit_rad,
                    balance_weight,
                ),
            ),
        ):
            index = support_start + local_index
            reference = self._stance_reference[index]
            target[index] = np.clip(
                target[index],
                reference - limit,
                reference + limit,
            )
        support_hip_pitch_reference = self._stance_reference[support_hip_pitch_index]
        forward_flexion_limit = self._interpolate_limit(
            self.limits.support_hip_pitch_limit_rad,
            self.limits.adaptive_support_hip_pitch_forward_limit_rad,
            balance_weight,
        )
        # Front kicks drive the swing hip pitch more negative; keep the
        # support hip from following that same forward-folding pattern.
        target[support_hip_pitch_index] = max(
            target[support_hip_pitch_index],
            support_hip_pitch_reference - forward_flexion_limit,
        )
        swing_hip_pitch_delta = float(
            live[swing_hip_pitch_index] - G1_DEFAULT_POSE[swing_hip_pitch_index]
        )
        if abs(swing_hip_pitch_delta) > 1e-6:
            counter_pitch_sign = -math.copysign(1.0, swing_hip_pitch_delta)
            support_pitch_limit = self._interpolate_limit(
                self.limits.support_hip_pitch_limit_rad,
                self.limits.adaptive_support_hip_pitch_limit_rad,
                balance_weight,
            )
            support_hip_pitch_target = (
                support_hip_pitch_reference
                + counter_pitch_sign
                * balance_weight
                * self.limits.support_hip_pitch_counter_bias_rad
            )
            target[support_hip_pitch_index] = np.clip(
                max(target[support_hip_pitch_index], support_hip_pitch_target),
                support_hip_pitch_reference - forward_flexion_limit,
                support_hip_pitch_reference + support_pitch_limit,
            )

            support_knee_reference = self._stance_reference[support_knee_index]
            support_knee_limit = self._interpolate_limit(
                self.limits.support_knee_limit_rad,
                self.limits.adaptive_support_knee_limit_rad,
                balance_weight,
            )
            support_knee_target = (
                support_knee_reference
                + balance_weight * self.limits.support_knee_counter_bias_rad
            )
            target[support_knee_index] = np.clip(
                max(target[support_knee_index], support_knee_target),
                support_knee_reference - support_knee_limit,
                support_knee_reference + support_knee_limit,
            )

            support_ankle_pitch_reference = self._stance_reference[
                support_ankle_pitch_index
            ]
            support_ankle_pitch_limit = self._interpolate_limit(
                self.limits.support_ankle_pitch_limit_rad,
                self.limits.adaptive_support_ankle_pitch_limit_rad,
                balance_weight,
            )
            support_ankle_pitch_target = (
                support_ankle_pitch_reference
                - balance_weight * self.limits.support_ankle_pitch_counter_bias_rad
            )
            target[support_ankle_pitch_index] = np.clip(
                min(target[support_ankle_pitch_index], support_ankle_pitch_target),
                support_ankle_pitch_reference - support_ankle_pitch_limit,
                support_ankle_pitch_reference + support_ankle_pitch_limit,
            )

        for local_index, limit in (
            (0, self.limits.swing_hip_pitch_limit_rad),
            (1, self.limits.swing_hip_roll_limit_rad),
            (2, self.limits.swing_hip_yaw_limit_rad),
            (4, self.limits.swing_ankle_pitch_limit_rad),
            (5, self.limits.swing_ankle_roll_limit_rad),
        ):
            index = swing_start + local_index
            neutral = G1_DEFAULT_POSE[index]
            target[index] = np.clip(target[index], neutral - limit, neutral + limit)
        self._hold_swing_chamber(target, swing_start)

        waist_indices = np.asarray([13, 14], dtype=np.int64)
        waist_protection = max(
            self.limits.waist_protection,
            balance_weight * self.limits.adaptive_waist_protection,
        )
        target[waist_indices] *= 1.0 - waist_protection
        waist_roll_limit = self._interpolate_limit(
            self.limits.waist_roll_limit_rad,
            self.limits.adaptive_waist_roll_limit_rad,
            balance_weight,
        )
        waist_pitch_limit = self._interpolate_limit(
            self.limits.waist_pitch_limit_rad,
            self.limits.adaptive_waist_pitch_limit_rad,
            balance_weight,
        )
        target[13] = np.clip(
            target[13], -waist_roll_limit,
            waist_roll_limit,
        )
        target[14] = np.clip(
            target[14], -waist_pitch_limit,
            waist_pitch_limit,
        )
        if abs(swing_hip_pitch_delta) > 1e-6:
            target[14] = np.clip(
                target[14]
                + (
                    -math.copysign(1.0, swing_hip_pitch_delta)
                    * balance_weight
                    * self.limits.waist_pitch_counter_bias_rad
                ),
                -waist_pitch_limit,
                waist_pitch_limit,
            )
        return target

    def update(
        self,
        now: float,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> StabilizedKickReference:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        live_position = self._validate_vector("joint_pos", joint_pos)
        live_velocity = self._validate_vector("joint_vel", joint_vel)
        if self._previous_time is not None and now <= self._previous_time:
            raise ValueError("kick stability timestamps must increase")

        left_score = self._leg_score(live_position, 0)
        right_score = self._leg_score(live_position, 6)
        self._update_phase(self._desired_phase(left_score, right_score), now)
        target = self._shape_target(live_position)

        output_velocity = live_velocity.copy()
        if self._previous_position is None or self._previous_time is None:
            output_position = target
        else:
            dt = now - self._previous_time
            max_step = self.limits.lower_body_slew_rad_s * min(
                dt, self.limits.maximum_dt_s
            )
            output_position = target.copy()
            output_position[:15] = self._previous_position[:15] + np.clip(
                target[:15] - self._previous_position[:15],
                -max_step,
                max_step,
            )
            output_velocity[:15] = np.clip(
                (output_position[:15] - self._previous_position[:15]) / dt,
                -self.limits.lower_body_slew_rad_s,
                self.limits.lower_body_slew_rad_s,
            )

        self._previous_position = output_position.copy()
        self._previous_time = now
        support_side = None
        if self.phase is KickPhase.LEFT_SWING:
            support_side = "right"
        elif self.phase is KickPhase.RIGHT_SWING:
            support_side = "left"
        return StabilizedKickReference(
            joint_pos=output_position.astype(np.float32),
            joint_vel=output_velocity.astype(np.float32),
            phase=self.phase,
            support_side=support_side,
            left_score_rad=left_score,
            right_score_rad=right_score,
        )
