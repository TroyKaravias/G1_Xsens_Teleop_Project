"""Conservative offline Xsens-to-G1 retargeting for kinematic preview.

This module deliberately produces preview poses, not motor commands.  It uses
the first CSV frame as the subject's calibration pose, measures later segment
rotations relative to that pose, and maps those rotation deltas onto the G1's
29 joint axes.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np


G1_JOINT_NAMES = (
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
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

G1_DEFAULT_POSE = np.array(
    [
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        0.0, 0.0, 0.0,
        0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
        0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    ],
    dtype=np.float64,
)

# Upright G1 pose corresponding to a human T-pose. This is used only for
# kinematic sensor-alignment previews; it is not a dynamically stable stance.
G1_T_POSE = np.zeros(29, dtype=np.float64)
G1_T_POSE[16] = 1.50
G1_T_POSE[18] = 1.40
G1_T_POSE[23] = -1.50
G1_T_POSE[25] = 1.40

REQUIRED_SEGMENTS = (
    "pelvis",
    "t8",
    "left_upper_leg",
    "left_lower_leg",
    "left_foot",
    "right_upper_leg",
    "right_lower_leg",
    "right_foot",
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
)


def _normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.maximum(norm, 1.0e-12)


def _conjugate(q: np.ndarray) -> np.ndarray:
    result = q.copy()
    result[..., 1:] *= -1.0
    return result


def _multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        axis=-1,
    )


def _rotation_vector(q: np.ndarray) -> np.ndarray:
    """Convert normalized wxyz quaternions to shortest-path rotation vectors."""
    q = _normalize(q)
    q = np.where(q[..., :1] < 0.0, -q, q)
    xyz = q[..., 1:]
    length = np.linalg.norm(xyz, axis=-1)
    angle = 2.0 * np.arctan2(length, np.clip(q[..., 0], -1.0, 1.0))
    scale = np.divide(angle, length, out=np.full_like(angle, 2.0), where=length > 1e-8)
    return xyz * scale[..., None]


def _relative_delta(
    parent: np.ndarray,
    child: np.ndarray,
    parent_zero: np.ndarray,
    child_zero: np.ndarray,
) -> np.ndarray:
    current = _multiply(_conjugate(parent), child)
    zero = _multiply(_conjugate(parent_zero), child_zero)
    return _rotation_vector(_multiply(current, _conjugate(zero)))


def load_xsens_csv(path: str | Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Load time and required segment quaternions from an exported bridge CSV."""
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise ValueError(f"No frames found in {path}")

    missing = [
        f"{segment}.qw"
        for segment in REQUIRED_SEGMENTS
        if f"{segment}.qw" not in rows[0]
    ]
    if missing:
        raise ValueError(f"CSV is missing Xsens columns: {', '.join(missing)}")

    times = np.array([float(row["time_s"]) for row in rows], dtype=np.float64)
    quaternions: dict[str, np.ndarray] = {}
    for segment in REQUIRED_SEGMENTS:
        quaternions[segment] = _normalize(
            np.array(
                [
                    [
                        float(row[f"{segment}.qw"]),
                        float(row[f"{segment}.qx"]),
                        float(row[f"{segment}.qy"]),
                        float(row[f"{segment}.qz"]),
                    ]
                    for row in rows
                ],
                dtype=np.float64,
            )
        )
    return times, quaternions


def load_pelvis_positions(path: str | Path) -> np.ndarray:
    """Load the Xsens pelvis XYZ positions from an exported bridge CSV."""
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No frames found in {path}")
    return np.array(
        [
            [
                float(row["pelvis.px_m"]),
                float(row["pelvis.py_m"]),
                float(row["pelvis.pz_m"]),
            ]
            for row in rows
        ],
        dtype=np.float64,
    )


def load_segment_positions(
    path: str | Path,
    segments: tuple[str, ...],
) -> dict[str, np.ndarray]:
    """Load XYZ positions for selected Xsens segments."""
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No frames found in {path}")
    return {
        segment: np.array(
            [
                [
                    float(row[f"{segment}.px_m"]),
                    float(row[f"{segment}.py_m"]),
                    float(row[f"{segment}.pz_m"]),
                ]
                for row in rows
            ],
            dtype=np.float64,
        )
        for segment in segments
    }


def _rotate_vectors(q: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    pure = np.concatenate((np.zeros((len(vectors), 1)), vectors), axis=1)
    return _multiply(_multiply(q, pure), _conjugate(q))[..., 1:]


def _arm_frame_alignment(t_pose_arm: np.ndarray, side_sign: float) -> np.ndarray:
    """Align a measured T-pose while preserving the subject's forward axis."""
    lateral = t_pose_arm * side_sign
    lateral /= np.linalg.norm(lateral)
    forward = np.array((1.0, 0.0, 0.0))
    forward -= lateral * np.dot(forward, lateral)
    forward /= np.linalg.norm(forward)
    up = np.cross(forward, lateral)
    source_basis = np.column_stack((forward, lateral, up))
    # Target basis is the canonical X-forward, Y-left, Z-up frame.
    return source_basis.T


def detect_t_pose_frame(path: str | Path) -> int:
    """Find the frame with maximum horizontal hand span near shoulder height."""
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No frames found in {path}")

    def positions(segment: str) -> np.ndarray:
        return np.array(
            [
                [
                    float(row[f"{segment}.px_m"]),
                    float(row[f"{segment}.py_m"]),
                    float(row[f"{segment}.pz_m"]),
                ]
                for row in rows
            ],
            dtype=np.float64,
        )

    left = positions("left_hand") - positions("left_shoulder")
    right = positions("right_hand") - positions("right_shoulder")
    hand_span = np.linalg.norm(
        positions("left_hand") - positions("right_hand"),
        axis=1,
    )
    vertical_error = np.abs(left[:, 2]) + np.abs(right[:, 2])
    return int(np.argmax(hand_span - 2.0 * vertical_error))


def scaled_pelvis_motion(
    positions: np.ndarray,
    quaternions: np.ndarray,
    *,
    gain: float = 0.65,
) -> tuple[np.ndarray, np.ndarray]:
    """Return pelvis translation and orientation deltas from the first frame."""
    position_delta = (positions - positions[:1]) * gain
    orientation_delta = _multiply(quaternions, _conjugate(quaternions[:1]))
    rotation_delta = _rotation_vector(orientation_delta) * gain
    angle = np.linalg.norm(rotation_delta, axis=-1)
    half_angle = 0.5 * angle
    xyz_scale = np.divide(
        np.sin(half_angle),
        angle,
        out=np.full_like(angle, 0.5),
        where=angle > 1e-8,
    )
    scaled_orientation = np.concatenate(
        (np.cos(half_angle)[:, None], rotation_delta * xyz_scale[:, None]),
        axis=1,
    )
    return position_delta, _normalize(scaled_orientation)


def retarget_quaternions(
    quaternions: dict[str, np.ndarray],
    *,
    motion_gain: float = 0.75,
    torso_gain: float = 0.50,
    calibration_frame: int = 0,
    base_pose: np.ndarray = G1_DEFAULT_POSE,
    positions: dict[str, np.ndarray] | None = None,
) -> np.ndarray:
    """Return a provisional (frames, 29) G1 trajectory.

    Xsens and G1 both use an X-forward, Y-left, Z-up convention.  Rotation-vector
    components are mapped to the G1 X-roll, Y-pitch, and Z-yaw joint axes.
    Wrists remain at their defaults because the current project has no hand
    control.
    """
    frame_count = quaternions["pelvis"].shape[0]
    if not 0 <= calibration_frame < frame_count:
        raise ValueError(f"Calibration frame {calibration_frame} is out of range")
    result = np.repeat(np.asarray(base_pose)[None, :], frame_count, axis=0)

    def delta(parent: str, child: str, gain: float = motion_gain) -> np.ndarray:
        p = quaternions[parent]
        c = quaternions[child]
        zero = slice(calibration_frame, calibration_frame + 1)
        return _relative_delta(p, c, p[zero], c[zero]) * gain

    for side, start in (("left", 0), ("right", 6)):
        hip = delta("pelvis", f"{side}_upper_leg")
        knee = delta(f"{side}_upper_leg", f"{side}_lower_leg")
        ankle = delta(
            f"{side}_lower_leg",
            f"{side}_foot",
            gain=min(motion_gain, 0.50),
        )
        result[:, start + 0] += hip[:, 1]
        result[:, start + 1] += hip[:, 0]
        result[:, start + 2] += hip[:, 2]
        result[:, start + 3] += knee[:, 1]
        result[:, start + 4] += ankle[:, 1]
        result[:, start + 5] += ankle[:, 0]

    waist = delta("pelvis", "t8", gain=torso_gain)
    result[:, 12] += waist[:, 2]
    result[:, 13] += waist[:, 0]
    # Preserve the recorded trunk-pitch direction. In the supplied calibration
    # sequence the subject begins leaning forward over a computer before moving
    # into the upright N- and T-poses.
    result[:, 14] += waist[:, 1]

    for side, start in (("left", 15), ("right", 22)):
        shoulder = delta("t8", f"{side}_upper_arm")
        elbow = delta(f"{side}_upper_arm", f"{side}_forearm")
        result[:, start + 0] += shoulder[:, 1]
        result[:, start + 1] += shoulder[:, 0]
        result[:, start + 2] += shoulder[:, 2]
        result[:, start + 3] += elbow[:, 1]

    if positions is not None:
        # Position-based arm IK correctly distinguishes the calibration poses:
        # down (N), lateral (T), and forward. Vectors are expressed relative to
        # the pelvis so global subject heading does not affect shoulder angles.
        pelvis_inverse = _conjugate(quaternions["pelvis"])
        for side, start in (("left", 15), ("right", 22)):
            upper_world = (
                positions[f"{side}_forearm"] - positions[f"{side}_upper_arm"]
            )
            lower_world = positions[f"{side}_hand"] - positions[f"{side}_forearm"]
            upper = _rotate_vectors(pelvis_inverse, upper_world)
            lower = _rotate_vectors(pelvis_inverse, lower_world)
            upper /= np.maximum(np.linalg.norm(upper, axis=1, keepdims=True), 1e-8)
            lower /= np.maximum(np.linalg.norm(lower, axis=1, keepdims=True), 1e-8)

            side_sign = 1.0 if side == "left" else -1.0
            alignment = _arm_frame_alignment(
                upper[calibration_frame],
                side_sign,
            )
            upper = upper @ alignment.T
            lower = lower @ alignment.T

            roll = np.arcsin(np.clip(upper[:, 1], -1.0, 1.0))
            horizontal = np.hypot(upper[:, 0], upper[:, 2])
            pitch = np.arctan2(-upper[:, 0], -upper[:, 2])
            pitch = np.unwrap(pitch)
            valid = horizontal >= 0.20
            valid_indices = np.flatnonzero(valid)
            if len(valid_indices) >= 2:
                pitch = np.interp(np.arange(frame_count), valid_indices, pitch[valid])
            pitch -= pitch[calibration_frame]
            # With near-forward arms, the G1's shoulder/link offsets bring the
            # wrists toward the centerline even when the human arms are
            # parallel. Add a smooth, side-specific roll bias only after the
            # shoulder pitch passes roughly 45 degrees. This preserves N/T
            # poses while preventing crossed hands in the forward pose.
            forward_weight = np.clip(
                (np.abs(pitch) - 0.75) / 0.75,
                0.0,
                1.0,
            )
            roll += side_sign * 0.30 * forward_weight
            elbow_flex = np.arccos(
                np.clip(np.sum(upper * lower, axis=1), -1.0, 1.0)
            )

            result[:, start + 0] = pitch
            result[:, start + 1] = roll
            result[:, start + 2] = 0.0
            # The G1's elbow actuator zero is an approximately 80-degree
            # visible bend. Around +1.4 rad is visually straight, so convert
            # the human flexion angle into the robot's offset convention.
            result[:, start + 3] = 1.40 - elbow_flex

    return result


def clamp_to_model_limits(
    trajectory: np.ndarray,
    joint_ranges: np.ndarray,
    *,
    margin_rad: float = 0.02,
) -> tuple[np.ndarray, int]:
    """Clamp a trajectory and return it with the number of changed values."""
    lower = joint_ranges[:, 0] + margin_rad
    upper = joint_ranges[:, 1] - margin_rad
    clamped = np.clip(trajectory, lower, upper)
    return clamped, int(np.count_nonzero(np.abs(clamped - trajectory) > 1e-10))
