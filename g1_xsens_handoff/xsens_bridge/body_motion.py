"""Conservative body-motion intent extraction for live Xsens simulation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .g1_retarget import _conjugate, _multiply, _rotate_vectors
from .xudp import PoseFrame


@dataclass(frozen=True)
class BodyMotionIntent:
    forward_velocity_mps: float
    lateral_velocity_mps: float
    yaw_rate_rad_s: float
    target_height_m: float
    label: str


class BodyMotionController:
    """Map measured pelvis translation to bounded SONIC footwork commands.

    The controller deliberately ignores a single-frame spike. Motion must
    persist for several samples, and stopping the body returns the command to
    zero. It produces references only; it does not publish robot commands.
    """

    def __init__(
        self,
        *,
        velocity_alpha: float = 0.18,
        deadband_mps: float = 0.08,
        confirm_frames: int = 4,
        head_turn_rate_deadband_rad_s: float = 0.20,
        head_turn_gain: float = 0.50,
        head_turn_max_rate_rad_s: float = 0.50,
    ) -> None:
        self.velocity_alpha = velocity_alpha
        self.deadband_mps = deadband_mps
        self.confirm_frames = confirm_frames
        if head_turn_rate_deadband_rad_s < 0.0:
            raise ValueError(
                "head_turn_rate_deadband_rad_s must be non-negative"
            )
        if head_turn_gain <= 0.0 or head_turn_max_rate_rad_s <= 0.0:
            raise ValueError("head turn gain and maximum rate must be positive")
        self.head_turn_rate_deadband_rad_s = head_turn_rate_deadband_rad_s
        self.head_turn_gain = head_turn_gain
        self.head_turn_max_rate_rad_s = head_turn_max_rate_rad_s
        self._previous_position: np.ndarray | None = None
        self._previous_time: float | None = None
        self._filtered_forward = 0.0
        self._filtered_lateral = 0.0
        self._candidate_sign = 0
        self._candidate_frames = 0
        self._neutral_height: float | None = None
        self._previous_pelvis_head_yaw: float | None = None
        self._neutral_foot_height: dict[str, float] = {}
        self._step_cooldown_until = 0.0
        self._step_active_until = 0.0
        self._foot_lift_armed = True

    def reset(self) -> None:
        self._previous_position = None
        self._previous_time = None
        self._filtered_forward = 0.0
        self._filtered_lateral = 0.0
        self._candidate_sign = 0
        self._candidate_frames = 0
        self._neutral_height = None
        self._previous_pelvis_head_yaw = None
        self._neutral_foot_height.clear()
        self._step_cooldown_until = 0.0
        self._step_active_until = 0.0
        self._foot_lift_armed = True

    @staticmethod
    def _segments(frame: PoseFrame) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        return {
            segment.name: (
                np.asarray(segment.position_m, dtype=np.float64),
                np.asarray(segment.quaternion_wxyz, dtype=np.float64),
            )
            for segment in frame.segments
        }

    @staticmethod
    def _yaw(quaternion: np.ndarray) -> float:
        w, x, y, z = quaternion
        return float(np.arctan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        ))

    @staticmethod
    def _pelvis(frame: PoseFrame) -> tuple[np.ndarray, np.ndarray]:
        for segment in frame.segments:
            if segment.name == "pelvis":
                return (
                    np.asarray(segment.position_m, dtype=np.float64),
                    np.asarray(segment.quaternion_wxyz, dtype=np.float64),
                )
        raise ValueError("Xsens frame is missing pelvis")

    def _pelvis_head_yaw(
        self,
        pelvis_quaternion: np.ndarray,
        segments: dict[str, tuple[np.ndarray, np.ndarray]],
    ) -> float | None:
        """Return head yaw in the pelvis frame, or None without head data."""
        if "head" not in segments:
            return None
        relative = _multiply(
            _conjugate(pelvis_quaternion[None]),
            segments["head"][1][None],
        )[0]
        return self._yaw(relative)

    def _head_turn_rate(
        self,
        pelvis_quaternion: np.ndarray,
        segments: dict[str, tuple[np.ndarray, np.ndarray]],
        dt: float,
    ) -> float:
        relative_yaw = self._pelvis_head_yaw(pelvis_quaternion, segments)
        if relative_yaw is None:
            return 0.0
        if self._previous_pelvis_head_yaw is None:
            self._previous_pelvis_head_yaw = relative_yaw
            return 0.0
        yaw_delta = float(np.arctan2(
            np.sin(relative_yaw - self._previous_pelvis_head_yaw),
            np.cos(relative_yaw - self._previous_pelvis_head_yaw),
        ))
        self._previous_pelvis_head_yaw = relative_yaw
        measured_rate = yaw_delta / dt
        magnitude = abs(measured_rate)
        if magnitude <= self.head_turn_rate_deadband_rad_s:
            return 0.0
        commanded = self.head_turn_gain * (
            magnitude - self.head_turn_rate_deadband_rad_s
        )
        return float(np.copysign(
            min(commanded, self.head_turn_max_rate_rad_s), measured_rate
        ))

    def update(
        self,
        frame: PoseFrame,
        timestamp: float,
        *,
        fencing: bool,
    ) -> BodyMotionIntent:
        position, quaternion = self._pelvis(frame)
        if self._previous_position is None or self._previous_time is None:
            self._previous_position = position
            self._previous_time = timestamp
            return BodyMotionIntent(0.0, 0.0, 0.0, -1.0, "body-neutral")

        dt = timestamp - self._previous_time
        delta_world = position - self._previous_position
        self._previous_position = position
        self._previous_time = timestamp
        if dt <= 0.0 or dt > 0.20:
            self._filtered_forward = 0.0
            return BodyMotionIntent(0.0, 0.0, 0.0, -1.0, "body-neutral")

        translation_intent = self.update_from_delta(
            delta_world, dt, quaternion, fencing=fencing
        )
        if fencing:
            return translation_intent

        segments = self._segments(frame)
        if self._neutral_height is None:
            self._neutral_height = float(position[2])
        height_drop = self._neutral_height - float(position[2])
        target_height = (
            float(np.clip(0.79 - height_drop, 0.62, 0.79))
            if height_drop > 0.035
            else -1.0
        )

        # Turn only while head yaw is changing relative to the pelvis. Holding
        # a left/right look must not remain a continuous yaw-rate command.
        yaw_rate = self._head_turn_rate(quaternion, segments, dt)

        lifted = False
        for side in ("left", "right"):
            name = f"{side}_foot"
            if name not in segments:
                continue
            relative_height = float(segments[name][0][2] - position[2])
            baseline = self._neutral_foot_height.setdefault(
                side, relative_height
            )
            if relative_height - baseline > 0.065:
                lifted = True
        if not lifted:
            self._foot_lift_armed = True
        if (
            lifted
            and self._foot_lift_armed
            and timestamp >= self._step_cooldown_until
        ):
            self._step_active_until = timestamp + 0.55
            self._step_cooldown_until = timestamp + 0.80
            self._foot_lift_armed = False
        if timestamp < self._step_active_until:
            return BodyMotionIntent(
                0.20, 0.0, yaw_rate, target_height, "body-step"
            )

        labels = []
        if target_height > 0.0:
            labels.append("squat")
        if yaw_rate != 0.0:
            labels.append("turn")
        if translation_intent.forward_velocity_mps != 0.0:
            labels.append(
                "forward"
                if translation_intent.forward_velocity_mps > 0
                else "backward"
            )
        if translation_intent.lateral_velocity_mps != 0.0:
            labels.append(
                "left"
                if translation_intent.lateral_velocity_mps > 0
                else "right"
            )
        return BodyMotionIntent(
            translation_intent.forward_velocity_mps,
            translation_intent.lateral_velocity_mps,
            yaw_rate,
            target_height,
            "body-" + "+".join(labels) if labels else "body-neutral",
        )

    def update_from_delta(
        self,
        delta_world: np.ndarray,
        dt: float,
        pelvis_quaternion: np.ndarray,
        *,
        fencing: bool,
    ) -> BodyMotionIntent:
        if dt <= 0.0 or dt > 0.20:
            self._filtered_forward = 0.0
            return BodyMotionIntent(0.0, 0.0, 0.0, -1.0, "body-neutral")
        return self._update_velocity(
            np.asarray(delta_world, dtype=np.float64) / dt,
            pelvis_quaternion,
            fencing=fencing,
        )

    def _update_velocity(
        self,
        world_velocity: np.ndarray,
        pelvis_quaternion: np.ndarray,
        *,
        fencing: bool,
    ) -> BodyMotionIntent:
        local_velocity = _rotate_vectors(
            _conjugate(np.asarray(pelvis_quaternion)[None]),
            np.asarray(world_velocity)[None],
        )[0]
        measured = float(local_velocity[0])
        # Backing up is usually a shorter, more cautious human movement than
        # advancing, so amplify it before filtering.
        if measured < 0.0:
            measured *= 1.45
        measured_lateral = float(local_velocity[1])
        self._filtered_forward = (
            self.velocity_alpha * measured
            + (1.0 - self.velocity_alpha) * self._filtered_forward
        )
        self._filtered_lateral = (
            self.velocity_alpha * measured_lateral
            + (1.0 - self.velocity_alpha) * self._filtered_lateral
        )
        sign = (
            1
            if self._filtered_forward > self.deadband_mps
            else -1
            if self._filtered_forward < -0.055
            else 0
        )
        if sign == self._candidate_sign:
            self._candidate_frames += 1
        else:
            self._candidate_sign = sign
            self._candidate_frames = 1
        if self._candidate_frames < self.confirm_frames:
            return BodyMotionIntent(0.0, 0.0, 0.0, -1.0, "body-neutral")
        if sign == 0:
            lateral = (
                float(np.clip(0.8 * self._filtered_lateral, -0.25, 0.25))
                if not fencing and abs(self._filtered_lateral) > 0.08
                else 0.0
            )
            return BodyMotionIntent(
                0.0,
                lateral,
                0.0,
                -1.0,
                (
                    "body-left" if lateral > 0.0 else "body-right"
                    if lateral < 0.0 else "body-neutral"
                ),
            )

        magnitude = abs(self._filtered_forward)
        if fencing:
            if sign > 0 and magnitude >= 0.45:
                return BodyMotionIntent(
                    0.35, 0.0, 0.0, -1.0, "body-lunge"
                )
            return BodyMotionIntent(
                0.20 if sign > 0 else -0.30,
                0.0,
                0.0,
                -1.0,
                "body-advance" if sign > 0 else "body-retreat",
            )
        return BodyMotionIntent(
            float(np.clip(0.9 * self._filtered_forward, -0.35, 0.30)),
            float(np.clip(0.8 * self._filtered_lateral, -0.25, 0.25))
            if abs(self._filtered_lateral) > 0.08
            else 0.0,
            0.0,
            -1.0,
            "body-forward" if sign > 0 else "body-backward",
        )
