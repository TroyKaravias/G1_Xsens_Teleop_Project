"""Safety-bounded global pelvis following for the SONIC locomotion planner.

This module converts Xsens global pelvis position and heading into planner-level
horizontal velocity and facing commands.  It never produces joint commands or
talks to robot hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GlobalPelvisLimits:
    engage_distance_m: float = 0.08
    release_distance_m: float = 0.04
    position_gain_s: float = 1.20
    minimum_speed_mps: float = 0.20
    maximum_speed_mps: float = 0.20
    maximum_sample_jump_m: float = 0.20
    maximum_excursion_m: float = 2.0
    maximum_dt_s: float = 0.20
    heading_gain: float = 1.0
    heading_deadband_rad: float = math.radians(3.0)
    heading_max_rate_rad_s: float = math.radians(30.0)

    def __post_init__(self) -> None:
        if self.engage_distance_m <= 0.0:
            raise ValueError("engage distance must be positive")
        if not 0.0 <= self.release_distance_m < self.engage_distance_m:
            raise ValueError("release distance must be below engage distance")
        if self.position_gain_s <= 0.0:
            raise ValueError("position gain must be positive")
        if not 0.0 < self.minimum_speed_mps <= self.maximum_speed_mps:
            raise ValueError("speed limits are invalid")
        # This is an explicit software commissioning ceiling, independent of
        # any broader limits accepted by the SONIC planner itself.
        if self.maximum_speed_mps > 0.35:
            raise ValueError("physical planner speed cannot exceed 0.35 m/s")
        if min(
            self.maximum_sample_jump_m,
            self.maximum_excursion_m,
            self.maximum_dt_s,
            self.heading_gain,
            self.heading_max_rate_rad_s,
        ) <= 0.0:
            raise ValueError("pelvis safety limits must be positive")
        if self.heading_deadband_rad < 0.0:
            raise ValueError("heading deadband must be non-negative")


@dataclass(frozen=True)
class GlobalPelvisCommand:
    velocity_x_mps: float = 0.0
    velocity_y_mps: float = 0.0
    heading_rad: float = 0.0
    desired_x_m: float = 0.0
    desired_y_m: float = 0.0
    estimated_x_m: float = 0.0
    estimated_y_m: float = 0.0
    active: bool = False
    faulted: bool = False
    label: str = "pelvis-idle"


def quaternion_yaw(quaternion_wxyz: np.ndarray) -> float:
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("pelvis quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-8:
        raise ValueError("pelvis quaternion must be nonzero")
    w, x, y, z = quaternion / norm
    return float(math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    ))


class GlobalPelvisFollower:
    """Follow calibrated Xsens XY displacement with bounded planner motion.

    SONIC does not expose robot odometry through this publisher, so the robot
    position is conservatively estimated by integrating the bounded velocity
    commands.  This makes a finite human displacement produce a finite catch-up
    command instead of continuous walking while the person stands still.
    """

    def __init__(self, limits: GlobalPelvisLimits | None = None) -> None:
        self.limits = GlobalPelvisLimits() if limits is None else limits
        self._origin_position: np.ndarray | None = None
        self._origin_yaw = 0.0
        self._previous_position: np.ndarray | None = None
        self._previous_time: float | None = None
        self._estimated_position = np.zeros(2, dtype=np.float64)
        self._previous_velocity = np.zeros(2, dtype=np.float64)
        self._heading_command = 0.0
        self._active = False
        self._faulted = False

    @property
    def armed(self) -> bool:
        return self._origin_position is not None and not self._faulted

    @property
    def faulted(self) -> bool:
        return self._faulted

    @staticmethod
    def _position(raw_position: np.ndarray) -> np.ndarray:
        position = np.asarray(raw_position, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("pelvis position must contain three finite values")
        return position

    def arm(
        self,
        position_m: np.ndarray,
        quaternion_wxyz: np.ndarray,
        timestamp: float,
    ) -> None:
        position = self._position(position_m)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        self._origin_position = position.copy()
        self._origin_yaw = quaternion_yaw(quaternion_wxyz)
        self._previous_position = position.copy()
        self._previous_time = timestamp
        self._estimated_position.fill(0.0)
        self._previous_velocity.fill(0.0)
        self._heading_command = 0.0
        self._active = False
        self._faulted = False

    def stop(self) -> None:
        self._previous_velocity.fill(0.0)
        self._active = False

    def _stopped(self, label: str, *, faulted: bool = False) -> GlobalPelvisCommand:
        self.stop()
        if faulted:
            self._faulted = True
        return GlobalPelvisCommand(
            heading_rad=self._heading_command,
            estimated_x_m=float(self._estimated_position[0]),
            estimated_y_m=float(self._estimated_position[1]),
            faulted=self._faulted,
            label=label,
        )

    def update(
        self,
        position_m: np.ndarray,
        quaternion_wxyz: np.ndarray,
        timestamp: float,
    ) -> GlobalPelvisCommand:
        if self._origin_position is None:
            self.arm(position_m, quaternion_wxyz, timestamp)
            return GlobalPelvisCommand(label="pelvis-armed")
        if self._faulted:
            return self._stopped("pelvis-fault", faulted=True)

        position = self._position(position_m)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        assert self._previous_position is not None
        assert self._previous_time is not None
        dt = timestamp - self._previous_time
        sample_jump = float(np.linalg.norm(position[:2] - self._previous_position[:2]))
        self._previous_position = position.copy()
        self._previous_time = timestamp
        if dt <= 0.0 or dt > self.limits.maximum_dt_s:
            return self._stopped("pelvis-timing-pause")
        if sample_jump > self.limits.maximum_sample_jump_m:
            return self._stopped("pelvis-jump-fault", faulted=True)

        # Integrate only commands actually emitted by this limiter.  The
        # result is an open-loop estimate, not a claim of measured robot pose.
        self._estimated_position += self._previous_velocity * dt

        delta_world = position - self._origin_position
        cosine = math.cos(self._origin_yaw)
        sine = math.sin(self._origin_yaw)
        desired = np.asarray((
            cosine * delta_world[0] + sine * delta_world[1],
            -sine * delta_world[0] + cosine * delta_world[1],
        ))
        if float(np.linalg.norm(desired)) > self.limits.maximum_excursion_m:
            return self._stopped("pelvis-excursion-fault", faulted=True)

        error = desired - self._estimated_position
        distance = float(np.linalg.norm(error))
        if self._active:
            self._active = distance > self.limits.release_distance_m
        else:
            self._active = distance >= self.limits.engage_distance_m

        velocity = np.zeros(2, dtype=np.float64)
        if self._active and distance > 1e-8:
            speed = float(np.clip(
                self.limits.position_gain_s
                * (distance - self.limits.release_distance_m),
                self.limits.minimum_speed_mps,
                self.limits.maximum_speed_mps,
            ))
            velocity = speed * error / distance

        relative_yaw = math.atan2(
            math.sin(quaternion_yaw(quaternion_wxyz) - self._origin_yaw),
            math.cos(quaternion_yaw(quaternion_wxyz) - self._origin_yaw),
        )
        heading_target = self.limits.heading_gain * relative_yaw
        if abs(heading_target) <= self.limits.heading_deadband_rad:
            heading_target = 0.0
        heading_delta = math.atan2(
            math.sin(heading_target - self._heading_command),
            math.cos(heading_target - self._heading_command),
        )
        maximum_heading_step = self.limits.heading_max_rate_rad_s * dt
        self._heading_command += float(np.clip(
            heading_delta, -maximum_heading_step, maximum_heading_step
        ))
        self._previous_velocity = velocity

        if not self._active:
            label = "pelvis-idle"
        else:
            forward = abs(velocity[0]) >= abs(velocity[1])
            if forward:
                label = "pelvis-forward" if velocity[0] > 0.0 else "pelvis-backward"
            else:
                label = "pelvis-left" if velocity[1] > 0.0 else "pelvis-right"
        return GlobalPelvisCommand(
            velocity_x_mps=float(velocity[0]),
            velocity_y_mps=float(velocity[1]),
            heading_rad=self._heading_command,
            desired_x_m=float(desired[0]),
            desired_y_m=float(desired[1]),
            estimated_x_m=float(self._estimated_position[0]),
            estimated_y_m=float(self._estimated_position[1]),
            active=self._active,
            label=label,
        )
