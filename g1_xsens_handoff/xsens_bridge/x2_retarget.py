"""Simulation-only Xsens retargeting for official AgiBot X2 Ultra MJCFs."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .g1_retarget import (
    G1_JOINT_NAMES,
    G1_T_POSE,
    clamp_to_model_limits,
    retarget_quaternions,
)


X2_MODEL_COMMIT = "77f43eb0904dae4c48ccd9154fee824f8ffd4d38"

X2_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "head_yaw_joint",
    "head_pitch_joint",
)

X2_T_POSE = np.zeros(len(X2_JOINT_NAMES), dtype=np.float64)
X2_T_POSE[X2_JOINT_NAMES.index("left_shoulder_roll_joint")] = 1.50
X2_T_POSE[X2_JOINT_NAMES.index("right_shoulder_roll_joint")] = -1.50

# The calibrated G1-layout reference and both official X2 MJCF variants use
# negative elbow flexion. Keep this table for any future, visually verified
# X2-specific axis differences; none are currently required.
X2_DELTA_SIGN: dict[str, float] = {}

X2_VARIANT_XML = {
    "v1.3": Path("X2_URDF-v1.3.0/x2_ultra.xml"),
    "v1.4": Path("X2_URDF-v1.4.0/X2-Ultra.xml"),
}


def model_xml(model_root: str | Path, variant: str) -> Path:
    """Return the pinned official MJCF path for an X2 Ultra variant."""
    try:
        relative = X2_VARIANT_XML[variant]
    except KeyError as exc:
        supported = ", ".join(sorted(X2_VARIANT_XML))
        raise ValueError(f"Unsupported X2 variant {variant!r}; use {supported}") from exc
    path = Path(model_root) / relative
    if not path.is_file():
        raise FileNotFoundError(f"Missing official X2 model: {path}")
    return path


def g1_deltas_to_x2(g1_trajectory: np.ndarray) -> np.ndarray:
    """Convert calibrated G1-layout deltas into the X2 semantic joint layout."""
    trajectory = np.asarray(g1_trajectory, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] != len(G1_JOINT_NAMES):
        raise ValueError(
            f"Expected trajectory shape (frames, {len(G1_JOINT_NAMES)}), "
            f"got {trajectory.shape}"
        )

    g1_index = {name: index for index, name in enumerate(G1_JOINT_NAMES)}
    result = np.broadcast_to(X2_T_POSE, (len(trajectory), len(X2_JOINT_NAMES))).copy()
    for x2_index, name in enumerate(X2_JOINT_NAMES):
        if name not in g1_index:
            continue
        source_index = g1_index[name]
        delta = trajectory[:, source_index] - G1_T_POSE[source_index]
        result[:, x2_index] += X2_DELTA_SIGN.get(name, 1.0) * delta
    return result


def g1_velocities_to_x2(g1_velocities: np.ndarray) -> np.ndarray:
    """Map G1-layout joint velocities into the semantic X2 layout."""
    velocities = np.asarray(g1_velocities, dtype=np.float64)
    if velocities.ndim != 1 or velocities.shape[0] != len(G1_JOINT_NAMES):
        raise ValueError(
            f"Expected velocity shape ({len(G1_JOINT_NAMES)},), "
            f"got {velocities.shape}"
        )
    g1_index = {name: index for index, name in enumerate(G1_JOINT_NAMES)}
    result = np.zeros(len(X2_JOINT_NAMES), dtype=np.float64)
    for x2_index, name in enumerate(X2_JOINT_NAMES):
        if name in g1_index:
            result[x2_index] = (
                X2_DELTA_SIGN.get(name, 1.0) * velocities[g1_index[name]]
            )
    return result


def retarget_xsens_to_x2(
    quaternions: dict[str, np.ndarray],
    *,
    motion_gain: float = 0.75,
    torso_gain: float = 0.50,
    calibration_frame: int = 0,
    positions: dict[str, np.ndarray] | None = None,
) -> np.ndarray:
    """Retarget an Xsens sequence to a kinematic X2 reference trajectory."""
    g1_layout = retarget_quaternions(
        quaternions,
        motion_gain=motion_gain,
        torso_gain=torso_gain,
        calibration_frame=calibration_frame,
        base_pose=G1_T_POSE,
        positions=positions,
    )
    return g1_deltas_to_x2(g1_layout)


def validate_model_joint_names(model_joint_names: tuple[str, ...]) -> None:
    """Fail on missing, duplicate, or reordered controlled joints."""
    if len(set(model_joint_names)) != len(model_joint_names):
        raise ValueError("X2 model joint list contains duplicate names")
    missing = [name for name in X2_JOINT_NAMES if name not in model_joint_names]
    if missing:
        raise ValueError(f"X2 model is missing joints: {', '.join(missing)}")
    controlled = tuple(name for name in model_joint_names if name in X2_JOINT_NAMES)
    if controlled != X2_JOINT_NAMES:
        raise ValueError("X2 controlled-joint ordering does not match the pinned contract")


def lowpass_x2_trajectory(
    trajectory: np.ndarray,
    frame_dt: float,
    cutoff_hz: float = 4.0,
) -> np.ndarray:
    """Apply a causal one-pole low-pass filter to an offline X2 reference.

    This is deliberately X2-local preprocessing. It does not modify the G1
    retargeter or either robot's control path.
    """
    values = np.asarray(trajectory, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != len(X2_JOINT_NAMES):
        raise ValueError(
            f"Expected trajectory shape (frames, {len(X2_JOINT_NAMES)}), "
            f"got {values.shape}"
        )
    if frame_dt <= 0.0:
        raise ValueError("frame_dt must be positive")
    if cutoff_hz < 0.0:
        raise ValueError("cutoff_hz must be non-negative")
    if cutoff_hz == 0.0 or len(values) < 2:
        return values.copy()

    alpha = 1.0 - np.exp(-2.0 * np.pi * cutoff_hz * frame_dt)
    filtered = values.copy()
    for frame in range(1, len(filtered)):
        filtered[frame] = (
            alpha * values[frame] + (1.0 - alpha) * filtered[frame - 1]
        )
    return filtered


def clamp_x2_trajectory(
    trajectory: np.ndarray,
    joint_ranges: np.ndarray,
    margin_rad: float = 0.0,
) -> tuple[np.ndarray, int]:
    return clamp_to_model_limits(
        trajectory, joint_ranges, margin_rad=margin_rad
    )
