"""Frame-to-frame operator translation for SONIC planner locomotion."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .g1_retarget import _conjugate, _rotate_vectors


def _yaw_quaternion(quaternion: np.ndarray) -> np.ndarray:
    """Return heading only, so pelvis lean cannot alter horizontal motion."""
    w, x, y, z = np.asarray(quaternion, dtype=np.float64)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.asarray(
        [math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class GlobalPositionCommand:
    forward_mps: float
    lateral_mps: float
    delta_m: tuple[float, float]


@dataclass(frozen=True)
class SquatCommand:
    active: bool
    height_m: float
    pelvis_drop_m: float


class SquatController:
    """Recognize a planted-foot squat and map depth to SONIC height."""

    def __init__(
        self,
        neutral_pelvis_z: float,
        neutral_left_foot_z: float,
        neutral_right_foot_z: float,
        *,
        enter_drop_m: float = 0.08,
        exit_drop_m: float = 0.04,
        full_drop_m: float = 0.24,
        standing_height_m: float = 0.80,
        minimum_height_m: float = 0.52,
        foot_lift_limit_m: float = 0.04,
        horizontal_speed_limit_mps: float = 0.40,
    ) -> None:
        if not 0.0 < exit_drop_m < enter_drop_m < full_drop_m:
            raise ValueError("squat thresholds must satisfy exit < enter < full")
        if not 0.2 <= minimum_height_m < standing_height_m:
            raise ValueError("invalid squat height range")
        self.neutral_pelvis_z = float(neutral_pelvis_z)
        self.neutral_foot_z = {
            "left": float(neutral_left_foot_z),
            "right": float(neutral_right_foot_z),
        }
        self.enter_drop_m = enter_drop_m
        self.exit_drop_m = exit_drop_m
        self.full_drop_m = full_drop_m
        self.standing_height_m = standing_height_m
        self.minimum_height_m = minimum_height_m
        self.foot_lift_limit_m = foot_lift_limit_m
        self.horizontal_speed_limit_mps = horizontal_speed_limit_mps
        self.active = False

    def reset(self) -> None:
        self.active = False

    def update(
        self,
        pelvis_z: float,
        left_foot_z: float,
        right_foot_z: float,
        horizontal_speed_mps: float,
    ) -> SquatCommand:
        drop = max(0.0, self.neutral_pelvis_z - float(pelvis_z))
        feet_planted = (
            abs(float(left_foot_z) - self.neutral_foot_z["left"])
            <= self.foot_lift_limit_m
            and abs(float(right_foot_z) - self.neutral_foot_z["right"])
            <= self.foot_lift_limit_m
        )
        still = horizontal_speed_mps <= self.horizontal_speed_limit_mps
        if self.active:
            if drop <= self.exit_drop_m or not feet_planted:
                self.active = False
        elif drop >= self.enter_drop_m and feet_planted and still:
            self.active = True

        if not self.active:
            return SquatCommand(False, self.standing_height_m, drop)
        scale = np.clip(
            (drop - self.enter_drop_m) / (self.full_drop_m - self.enter_drop_m),
            0.0,
            1.0,
        )
        height = self.standing_height_m + float(scale) * (
            self.minimum_height_m - self.standing_height_m
        )
        return SquatCommand(True, height, drop)


class GlobalPositionController:
    """Convert consecutive pelvis samples to bounded horizontal velocity."""

    def __init__(
        self,
        *,
        velocity_deadzone_mps: float = 0.015,
        translation_gain: float = 1.0,
        forward_gain: float = 1.50,
        backward_gain: float = 1.70,
        left_gain: float = 1.20,
        right_gain: float = 1.15,
        max_forward_mps: float = 0.50,
        max_backward_mps: float = 0.45,
        max_lateral_mps: float = 0.40,
        max_left_mps: float | None = 0.35,
        max_right_mps: float | None = 0.45,
        filter_alpha: float = 0.70,
        max_sample_gap_s: float = 0.20,
        command_hold_s: float = 0.18,
    ) -> None:
        if velocity_deadzone_mps < 0.0:
            raise ValueError("velocity deadzone cannot be negative")
        if translation_gain <= 0.0:
            raise ValueError("translation gain must be positive")
        if min(forward_gain, backward_gain, left_gain, right_gain) <= 0.0:
            raise ValueError("directional gains must be positive")
        if min(max_forward_mps, max_backward_mps, max_lateral_mps) <= 0.0:
            raise ValueError("speed limits must be positive")
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError("filter alpha must be within (0, 1]")
        if max_sample_gap_s <= 0.0:
            raise ValueError("max sample gap must be positive")
        if command_hold_s < 0.0:
            raise ValueError("command hold cannot be negative")
        self.velocity_deadzone_mps = velocity_deadzone_mps
        self.translation_gain = translation_gain
        self.forward_gain = forward_gain
        self.backward_gain = backward_gain
        self.left_gain = left_gain
        self.right_gain = right_gain
        self.max_forward_mps = max_forward_mps
        self.max_backward_mps = max_backward_mps
        self.max_lateral_mps = max_lateral_mps
        self.max_left_mps = (
            max_lateral_mps if max_left_mps is None else max_left_mps
        )
        self.max_right_mps = (
            max_lateral_mps if max_right_mps is None else max_right_mps
        )
        if min(self.max_left_mps, self.max_right_mps) <= 0.0:
            raise ValueError("directional lateral limits must be positive")
        self.filter_alpha = filter_alpha
        self.max_sample_gap_s = max_sample_gap_s
        self.command_hold_s = command_hold_s
        self._previous_position: np.ndarray | None = None
        self._previous_time: float | None = None
        self._filtered_velocity = np.zeros(2, dtype=np.float64)
        self._held_velocity = np.zeros(2, dtype=np.float64)
        self._last_motion_time: float | None = None

    def reset(self) -> None:
        self._previous_position = None
        self._previous_time = None
        self._filtered_velocity.fill(0.0)
        self._held_velocity.fill(0.0)
        self._last_motion_time = None

    def update(
        self,
        pelvis_position: np.ndarray,
        pelvis_quaternion: np.ndarray,
        sample_time: float,
    ) -> GlobalPositionCommand:
        position = np.asarray(pelvis_position, dtype=np.float64)
        if self._previous_position is None or self._previous_time is None:
            self._previous_position = position.copy()
            self._previous_time = float(sample_time)
            return GlobalPositionCommand(0.0, 0.0, (0.0, 0.0))

        delta_world = position - self._previous_position
        dt = float(sample_time) - self._previous_time
        self._previous_position = position.copy()
        self._previous_time = float(sample_time)
        if dt <= 0.0 or dt > self.max_sample_gap_s:
            self._filtered_velocity.fill(0.0)
            self._held_velocity.fill(0.0)
            self._last_motion_time = None
            return GlobalPositionCommand(0.0, 0.0, (0.0, 0.0))

        local_delta = _rotate_vectors(
            _conjugate(_yaw_quaternion(pelvis_quaternion)[None]),
            delta_world[None],
        )[0]
        measured_velocity = local_delta[:2] / dt * self.translation_gain
        measured_velocity[0] *= (
            self.forward_gain if measured_velocity[0] >= 0.0 else self.backward_gain
        )
        measured_velocity[1] *= (
            self.left_gain if measured_velocity[1] >= 0.0 else self.right_gain
        )
        self._filtered_velocity = (
            self.filter_alpha * measured_velocity
            + (1.0 - self.filter_alpha) * self._filtered_velocity
        )
        velocity = self._filtered_velocity.copy()
        velocity[np.abs(velocity) < self.velocity_deadzone_mps] = 0.0
        if np.any(velocity != 0.0):
            self._held_velocity = velocity.copy()
            self._last_motion_time = float(sample_time)
        elif (
            self._last_motion_time is not None
            and float(sample_time) - self._last_motion_time <= self.command_hold_s
        ):
            velocity = self._held_velocity.copy()
        else:
            self._held_velocity.fill(0.0)
        forward = float(
            np.clip(velocity[0], -self.max_backward_mps, self.max_forward_mps)
        )
        lateral = float(np.clip(
            velocity[1], -self.max_right_mps, self.max_left_mps
        ))
        return GlobalPositionCommand(
            forward, lateral, (float(local_delta[0]), float(local_delta[1]))
        )
