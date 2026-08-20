"""Fixed-base MuJoCo tracking helpers for simulation-only AgiBot X2 work."""

from __future__ import annotations

import math

import numpy as np

from .g1_retarget import _conjugate, _rotate_vectors
from .x2_retarget import X2_JOINT_NAMES


def _heading_quaternion(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=np.float64)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.asarray(
        [math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)],
        dtype=np.float64,
    )


class X2GlobalRootTracker:
    """Integrate bounded Xsens pelvis displacement into visual world X/Y."""

    def __init__(
        self,
        calibration_quaternion: np.ndarray,
        initial_xy: np.ndarray,
        *,
        gain: float = 1.0,
        max_speed_mps: float = 0.75,
        max_jump_m: float = 0.25,
        deadband_mps: float = 0.015,
        filter_alpha: float = 0.60,
        max_sample_gap_s: float = 0.20,
    ) -> None:
        if gain <= 0.0 or max_speed_mps <= 0.0 or max_jump_m <= 0.0:
            raise ValueError("global root gain and limits must be positive")
        if deadband_mps < 0.0 or not 0.0 < filter_alpha <= 1.0:
            raise ValueError("invalid global root filter settings")
        self.calibration_heading_inverse = _conjugate(
            _heading_quaternion(calibration_quaternion)[None]
        )
        self.xy = np.asarray(initial_xy, dtype=np.float64).copy()
        if self.xy.shape != (2,):
            raise ValueError(f"initial_xy must have shape (2,), got {self.xy.shape}")
        self.gain = gain
        self.max_speed_mps = max_speed_mps
        self.max_jump_m = max_jump_m
        self.deadband_mps = deadband_mps
        self.filter_alpha = filter_alpha
        self.max_sample_gap_s = max_sample_gap_s
        self._previous_position: np.ndarray | None = None
        self._previous_time: float | None = None
        self._filtered_velocity = np.zeros(2, dtype=np.float64)
        self.rejected_jumps = 0

    def update(
        self,
        pelvis_position: np.ndarray,
        sample_time: float,
    ) -> np.ndarray:
        position = np.asarray(pelvis_position, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("pelvis position must contain three finite values")
        if self._previous_position is None or self._previous_time is None:
            self._previous_position = position.copy()
            self._previous_time = float(sample_time)
            return self.xy.copy()
        delta_world = position - self._previous_position
        dt = float(sample_time) - self._previous_time
        self._previous_position = position.copy()
        self._previous_time = float(sample_time)
        if dt <= 0.0 or dt > self.max_sample_gap_s:
            self._filtered_velocity.fill(0.0)
            return self.xy.copy()
        if float(np.linalg.norm(delta_world[:2])) > self.max_jump_m:
            self.rejected_jumps += 1
            self._filtered_velocity.fill(0.0)
            return self.xy.copy()
        local_delta = _rotate_vectors(
            self.calibration_heading_inverse, delta_world[None]
        )[0, :2]
        velocity = self.gain * local_delta / dt
        speed = float(np.linalg.norm(velocity))
        if speed > self.max_speed_mps:
            velocity *= self.max_speed_mps / speed
        self._filtered_velocity = (
            self.filter_alpha * velocity
            + (1.0 - self.filter_alpha) * self._filtered_velocity
        )
        if float(np.linalg.norm(self._filtered_velocity)) < self.deadband_mps:
            self._filtered_velocity.fill(0.0)
        self.xy += self._filtered_velocity * dt
        return self.xy.copy()


def default_x2_pd_gains() -> tuple[np.ndarray, np.ndarray]:
    """Return conservative simulation gains ordered by ``X2_JOINT_NAMES``."""
    kp = np.empty(len(X2_JOINT_NAMES), dtype=np.float64)
    kd = np.empty(len(X2_JOINT_NAMES), dtype=np.float64)
    for index, name in enumerate(X2_JOINT_NAMES):
        if name.startswith(("left_hip", "right_hip", "left_knee", "right_knee")):
            kp[index], kd[index] = 80.0, 4.0
        elif name.startswith(("left_ankle", "right_ankle")):
            kp[index], kd[index] = 45.0, 2.5
        elif name.startswith("waist"):
            kp[index], kd[index] = 55.0, 3.0
        elif name.startswith(("left_wrist", "right_wrist")):
            kp[index], kd[index] = 8.0, 0.6
        elif name.startswith("head"):
            kp[index], kd[index] = 5.0, 0.4
        else:
            kp[index], kd[index] = 25.0, 1.5
    return kp, kd


def bounded_pd_torque(
    position: np.ndarray,
    velocity: np.ndarray,
    target_position: np.ndarray,
    target_velocity: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    torque_limits: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute PD torque and report which joints reached their force limit."""
    position = np.asarray(position, dtype=np.float64)
    velocity = np.asarray(velocity, dtype=np.float64)
    target_position = np.asarray(target_position, dtype=np.float64)
    target_velocity = np.asarray(target_velocity, dtype=np.float64)
    kp = np.asarray(kp, dtype=np.float64)
    kd = np.asarray(kd, dtype=np.float64)
    limits = np.asarray(torque_limits, dtype=np.float64)
    expected = (len(X2_JOINT_NAMES),)
    for label, values in (
        ("position", position),
        ("velocity", velocity),
        ("target_position", target_position),
        ("target_velocity", target_velocity),
        ("kp", kp),
        ("kd", kd),
    ):
        if values.shape != expected:
            raise ValueError(f"{label} must have shape {expected}, got {values.shape}")
    if limits.shape != (len(X2_JOINT_NAMES), 2):
        raise ValueError(
            f"torque_limits must have shape ({len(X2_JOINT_NAMES)}, 2), "
            f"got {limits.shape}"
        )
    raw = kp * (target_position - position) + kd * (target_velocity - velocity)
    torque = np.clip(raw, limits[:, 0], limits[:, 1])
    saturated = np.abs(torque - raw) > 1e-10
    return torque, saturated


def interpolate_x2_reference(
    trajectory: np.ndarray,
    frame_dt: float,
    elapsed_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Linearly interpolate a finite recorded reference and its velocity."""
    values = np.asarray(trajectory, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != len(X2_JOINT_NAMES):
        raise ValueError(
            f"Expected trajectory shape (frames, {len(X2_JOINT_NAMES)}), "
            f"got {values.shape}"
        )
    if len(values) == 0:
        raise ValueError("trajectory must contain at least one frame")
    if frame_dt <= 0.0:
        raise ValueError("frame_dt must be positive")
    elapsed = float(np.clip(elapsed_s, 0.0, (len(values) - 1) * frame_dt))
    lower = min(int(elapsed / frame_dt), len(values) - 1)
    upper = min(lower + 1, len(values) - 1)
    fraction = (elapsed - lower * frame_dt) / frame_dt if upper != lower else 0.0
    position = (1.0 - fraction) * values[lower] + fraction * values[upper]
    velocity = (
        (values[upper] - values[lower]) / frame_dt
        if upper != lower
        else np.zeros(len(X2_JOINT_NAMES), dtype=np.float64)
    )
    return position, velocity
