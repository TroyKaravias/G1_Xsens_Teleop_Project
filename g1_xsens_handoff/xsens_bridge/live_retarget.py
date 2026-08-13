"""Online Xsens-to-G1 retargeting with N/T and optional forward calibration.

This module produces simulation references only.  It has no ROS, Unitree SDK,
network-publishing, or motor-command code.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterable

import numpy as np

from .g1_retarget import (
    G1_DEFAULT_POSE,
    G1_T_POSE,
    REQUIRED_SEGMENTS,
    _conjugate,
    _multiply,
    _relative_delta,
    _rotate_vectors,
    clamp_to_model_limits,
    retarget_quaternions,
)
from .xudp import PoseFrame


# Position-based arm IK additionally needs the hand positions.
ARM_POSITION_SEGMENTS = (
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
)

HAND_ORIENTATION_SEGMENTS = (
    "left_hand",
    "right_hand",
)

CALIBRATION_QUATERNION_SEGMENTS = (
    *REQUIRED_SEGMENTS,
    *HAND_ORIENTATION_SEGMENTS,
)

CALIBRATION_POSITION_SEGMENTS = (
    *ARM_POSITION_SEGMENTS,
    "left_shoulder",
    "right_shoulder",
    "pelvis",
    "left_upper_leg",
    "left_lower_leg",
    "left_foot",
    "right_upper_leg",
    "right_lower_leg",
    "right_foot",
)


def _nearest_equivalent_angle(target: float, reference: float) -> float:
    """Return target modulo 2π using the shortest path from reference."""
    delta = np.arctan2(
        np.sin(target - reference),
        np.cos(target - reference),
    )
    return float(reference + delta)


@dataclass(frozen=True)
class CalibrationResult:
    """Frozen sensor reference chosen from an N→T startup sequence."""

    quaternions: dict[str, np.ndarray]
    positions: dict[str, np.ndarray]
    n_positions: dict[str, np.ndarray]
    n_pose_index: int
    t_pose_index: int
    t_pose_score: float
    captured_frames: int
    forward_positions: dict[str, np.ndarray] | None = None
    forward_pose_index: int | None = None


@dataclass(frozen=True)
class RetargetedPose:
    """One timestamped 29-DOF G1 simulation reference."""

    timestamp: float
    dof_pos: np.ndarray
    dof_vel: np.ndarray


def _frame_maps(
    frame: PoseFrame,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    quaternions: dict[str, np.ndarray] = {}
    positions: dict[str, np.ndarray] = {}
    for segment in frame.segments:
        quaternions[segment.name] = np.asarray(
            segment.quaternion_wxyz, dtype=np.float64
        )
        positions[segment.name] = np.asarray(segment.position_m, dtype=np.float64)
    return quaternions, positions


def _stack_frames(
    frames: Iterable[PoseFrame],
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], int]:
    frame_list = list(frames)
    if not frame_list:
        raise ValueError("Calibration requires at least one Xsens frame")

    quaternion_rows: dict[str, list[np.ndarray]] = {
        name: [] for name in CALIBRATION_QUATERNION_SEGMENTS
    }
    position_rows: dict[str, list[np.ndarray]] = {
        name: [] for name in CALIBRATION_POSITION_SEGMENTS
    }
    for index, frame in enumerate(frame_list):
        quaternions, positions = _frame_maps(frame)
        missing = [
            *(name for name in REQUIRED_SEGMENTS if name not in quaternions),
            *(
                name
                for name in CALIBRATION_POSITION_SEGMENTS
                if name not in positions
            ),
        ]
        if missing:
            raise ValueError(
                f"Xsens frame {index} is missing segments: {', '.join(missing)}"
            )
        for name in quaternion_rows:
            quaternion_rows[name].append(quaternions[name])
        for name in position_rows:
            position_rows[name].append(positions[name])

    quaternion_arrays = {
        name: np.stack(values) for name, values in quaternion_rows.items()
    }
    position_arrays = {
        name: np.stack(values) for name, values in position_rows.items()
    }
    return quaternion_arrays, position_arrays, len(frame_list)


def _t_pose_scores(positions: dict[str, np.ndarray]) -> np.ndarray:
    left = positions["left_hand"] - positions["left_shoulder"]
    right = positions["right_hand"] - positions["right_shoulder"]
    span = np.linalg.norm(
        positions["left_hand"] - positions["right_hand"], axis=1
    )
    vertical_error = np.abs(left[:, 2]) + np.abs(right[:, 2])
    return span - 2.0 * vertical_error


def calibrate_nt_sequence(frames: Iterable[PoseFrame]) -> CalibrationResult:
    """Select the strongest T-pose from a captured N→T calibration sequence."""
    quaternions, positions, frame_count = _stack_frames(frames)
    scores = _t_pose_scores(positions)
    # The guided sequence holds N-pose during its first 40%. Use the midpoint
    # of that hold, avoiding both startup settling and the transition to T.
    n_window = max(1, int(frame_count * 0.40))
    n_pose_index = n_window // 2
    t_pose_index = int(np.argmax(scores))
    return CalibrationResult(
        quaternions={
            name: values[t_pose_index].copy()
            for name, values in quaternions.items()
        },
        positions={
            name: values[t_pose_index].copy()
            for name, values in positions.items()
        },
        n_positions={
            name: values[n_pose_index].copy()
            for name, values in positions.items()
        },
        n_pose_index=n_pose_index,
        t_pose_index=t_pose_index,
        t_pose_score=float(scores[t_pose_index]),
        captured_frames=frame_count,
    )


def calibrate_ntf_sequence(frames: Iterable[PoseFrame]) -> CalibrationResult:
    """Select N, T, and arms-forward references from a guided sequence.

    Timing is intentionally part of the calibration contract: N occupies the
    first third, T the middle third, and forward reach the final third. The
    strongest T frame is selected only from the middle window; a settled frame
    near the end of the final hold becomes the forward reference.
    """
    quaternions, positions, frame_count = _stack_frames(frames)
    if frame_count < 90:
        raise ValueError("N/T/forward calibration requires at least 90 frames")
    scores = _t_pose_scores(positions)
    n_pose_index = max(0, int(frame_count * 0.16))
    t_start = int(frame_count * 0.36)
    t_stop = max(t_start + 1, int(frame_count * 0.64))
    t_pose_index = t_start + int(np.argmax(scores[t_start:t_stop]))
    forward_pose_index = min(frame_count - 1, int(frame_count * 0.88))
    return CalibrationResult(
        quaternions={
            name: values[t_pose_index].copy()
            for name, values in quaternions.items()
        },
        positions={
            name: values[t_pose_index].copy()
            for name, values in positions.items()
        },
        n_positions={
            name: values[n_pose_index].copy()
            for name, values in positions.items()
        },
        n_pose_index=n_pose_index,
        t_pose_index=t_pose_index,
        t_pose_score=float(scores[t_pose_index]),
        captured_frames=frame_count,
        forward_positions={
            name: values[forward_pose_index].copy()
            for name, values in positions.items()
        },
        forward_pose_index=forward_pose_index,
    )


class OnlineG1Retargeter:
    """Convert independent live Xsens frames into smooth 29-DOF references."""

    def __init__(
        self,
        calibration: CalibrationResult,
        *,
        motion_gain: float = 1.00,
        torso_gain: float = 0.50,
        neutral_shoulder_bias_deg: float = 6.0,
        reach_shoulder_bias_deg: float = 11.0,
        shoulder_yaw_gain: float = 0.75,
        velocity_alpha: float = 0.35,
        max_joint_speed: float = 12.0,
        max_position_speed: float = 4.0,
        max_position_dt: float = 0.04,
        track_wrists: bool = False,
        wrist_roll_gain: float = 1.65,
        track_wrist_pitch: bool = False,
        wrist_pitch_gain: float = 0.50,
        wrist_pitch_limit_rad: float = 0.35,
        wrist_pitch_sign: float = 1.0,
        left_wrist_pitch_sign: float = 1.0,
        right_wrist_pitch_sign: float = 1.0,
        track_pelvis_yaw: bool = False,
        pelvis_yaw_gain: float = 0.50,
        pelvis_yaw_limit_rad: float = 0.50,
        track_foot_heading: bool = False,
        foot_heading_gain: float = 1.00,
        foot_heading_limit_rad: float = 0.70,
        foot_lift_threshold_m: float = 0.05,
        track_lifted_hip_roll: bool = False,
        lifted_hip_roll_gain: float = 0.80,
        lifted_hip_roll_limit_rad: float = 0.35,
        joint_ranges: np.ndarray | None = None,
    ) -> None:
        self.calibration = calibration
        self.motion_gain = motion_gain
        self.torso_gain = torso_gain
        self.neutral_shoulder_bias = np.deg2rad(neutral_shoulder_bias_deg)
        self.reach_shoulder_bias = np.deg2rad(reach_shoulder_bias_deg)
        self.shoulder_yaw_gain = shoulder_yaw_gain
        self.velocity_alpha = velocity_alpha
        self.max_joint_speed = max_joint_speed
        self.max_position_speed = max_position_speed
        self.max_position_dt = max_position_dt
        self.track_wrists = track_wrists
        self.wrist_roll_gain = wrist_roll_gain
        self.track_wrist_pitch = track_wrist_pitch
        self.wrist_pitch_gain = wrist_pitch_gain
        self.wrist_pitch_limit_rad = wrist_pitch_limit_rad
        self.wrist_pitch_sign = wrist_pitch_sign
        self.wrist_pitch_signs = wrist_pitch_sign * np.asarray(
            [left_wrist_pitch_sign, right_wrist_pitch_sign],
            dtype=np.float64,
        )
        self.track_pelvis_yaw = track_pelvis_yaw
        self.pelvis_yaw_gain = pelvis_yaw_gain
        self.pelvis_yaw_limit_rad = pelvis_yaw_limit_rad
        self.track_foot_heading = track_foot_heading
        self.foot_heading_gain = foot_heading_gain
        self.foot_heading_limit_rad = foot_heading_limit_rad
        self.foot_lift_threshold_m = foot_lift_threshold_m
        self.track_lifted_hip_roll = track_lifted_hip_roll
        self.lifted_hip_roll_gain = lifted_hip_roll_gain
        self.lifted_hip_roll_limit_rad = lifted_hip_roll_limit_rad
        if self.max_position_speed <= 0.0 or self.max_position_dt <= 0.0:
            raise ValueError(
                "max_position_speed and max_position_dt must be positive"
            )
        if self.wrist_roll_gain <= 0.0:
            raise ValueError("wrist_roll_gain must be positive")
        if self.wrist_pitch_gain <= 0.0 or self.wrist_pitch_limit_rad <= 0.0:
            raise ValueError("wrist pitch gain and limit must be positive")
        if self.wrist_pitch_sign not in (-1.0, 1.0):
            raise ValueError("wrist_pitch_sign must be -1 or 1")
        if any(sign not in (-1.0, 1.0) for sign in self.wrist_pitch_signs):
            raise ValueError("per-arm wrist pitch signs must be -1 or 1")
        if self.pelvis_yaw_gain <= 0.0 or self.pelvis_yaw_limit_rad <= 0.0:
            raise ValueError("pelvis yaw gain and limit must be positive")
        if (
            self.foot_heading_gain <= 0.0
            or self.foot_heading_limit_rad <= 0.0
            or self.foot_lift_threshold_m <= 0.0
        ):
            raise ValueError("foot heading gains, limits, and lift threshold must be positive")
        if (
            self.lifted_hip_roll_gain <= 0.0
            or self.lifted_hip_roll_limit_rad <= 0.0
        ):
            raise ValueError("lifted hip-roll gain and limit must be positive")
        if not 0.0 <= self.shoulder_yaw_gain <= 1.0:
            raise ValueError("shoulder_yaw_gain must be within [0, 1]")
        self.joint_ranges = (
            None if joint_ranges is None else np.asarray(joint_ranges, dtype=np.float64)
        )
        if self.joint_ranges is not None and self.joint_ranges.shape != (29, 2):
            raise ValueError("joint_ranges must have shape (29, 2)")
        self._previous_time: float | None = None
        self._previous_pos: np.ndarray | None = None
        self._filtered_vel = np.zeros(29, dtype=np.float64)
        self._previous_arm_pitch = np.array(
            [G1_DEFAULT_POSE[15], G1_DEFAULT_POSE[22]], dtype=np.float64
        )
        self._previous_arm_yaw = np.zeros(2, dtype=np.float64)
        self._wrist_twist_axes = np.zeros((2, 3), dtype=np.float64)
        self._wrist_pitch_axes = np.zeros((2, 3), dtype=np.float64)
        for arm_index, side in enumerate(("left", "right")):
            forearm_axis_world = (
                calibration.positions[f"{side}_hand"]
                - calibration.positions[f"{side}_forearm"]
            )
            forearm_axis_local = _rotate_vectors(
                _conjugate(
                    calibration.quaternions[f"{side}_forearm"][None]
                ),
                forearm_axis_world[None],
            )[0]
            self._wrist_twist_axes[arm_index] = (
                forearm_axis_local
                / max(np.linalg.norm(forearm_axis_local), 1e-8)
            )
            pelvis_up_world = _rotate_vectors(
                self.calibration.quaternions["pelvis"][None],
                np.asarray([[0.0, 0.0, 1.0]], dtype=np.float64),
            )[0]
            pelvis_up_local = _rotate_vectors(
                _conjugate(
                    calibration.quaternions[f"{side}_forearm"][None]
                ),
                pelvis_up_world[None],
            )[0]
            pitch_axis = np.cross(
                pelvis_up_local, self._wrist_twist_axes[arm_index]
            )
            self._wrist_pitch_axes[arm_index] = (
                pitch_axis / max(np.linalg.norm(pitch_axis), 1e-8)
            )
        self._neutral_arm_angles = self._arm_angles(
            calibration.quaternions["pelvis"],
            calibration.n_positions,
        )
        self._forward_arm_offsets = np.zeros((2, 3), dtype=np.float64)
        if calibration.forward_positions is not None:
            forward_angles = self._arm_angles(
                calibration.quaternions["pelvis"],
                calibration.forward_positions,
            )
            desired_pitch = -1.35
            for arm_index, (side, start) in enumerate(
                (("left", 15), ("right", 22))
            ):
                pitch, roll, elbow, _ = forward_angles[arm_index]
                neutral_pitch, neutral_roll, _, _ = self._neutral_arm_angles[
                    arm_index
                ]
                raw_pitch = G1_DEFAULT_POSE[start] + pitch - neutral_pitch
                side_sign = 1.0 if side == "left" else -1.0
                raw_roll = (
                    G1_DEFAULT_POSE[start + 1]
                    + roll
                    - neutral_roll
                    + side_sign * 0.30
                )
                # These targets keep straight forward arms parallel and
                # shoulder-width apart on the G1. Pitch still receives the
                # normal reach correction later, so compensate for it here.
                desired_roll = side_sign * 0.12
                desired_elbow = 1.40
                reach_correction = self.reach_shoulder_bias
                self._forward_arm_offsets[arm_index] = (
                    desired_pitch - reach_correction - raw_pitch,
                    desired_roll - raw_roll,
                    desired_elbow - elbow,
                )

    def reset_history(self) -> None:
        self._previous_time = None
        self._previous_pos = None
        self._filtered_vel.fill(0.0)
        self._previous_arm_pitch[:] = (
            G1_DEFAULT_POSE[15],
            G1_DEFAULT_POSE[22],
        )
        self._previous_arm_yaw.fill(0.0)

    @staticmethod
    def _arm_angles(
        pelvis_quaternion: np.ndarray,
        positions: dict[str, np.ndarray],
    ) -> np.ndarray:
        """Return direct pelvis-frame pitch/roll/elbow angles for both arms."""
        pelvis_inverse = _conjugate(
            np.asarray(pelvis_quaternion, dtype=np.float64)[None]
        )
        result = np.zeros((2, 4), dtype=np.float64)
        for arm_index, side in enumerate(("left", "right")):
            upper_world = (
                positions[f"{side}_forearm"] - positions[f"{side}_upper_arm"]
            )[None]
            lower_world = (
                positions[f"{side}_hand"] - positions[f"{side}_forearm"]
            )[None]
            upper = _rotate_vectors(pelvis_inverse, upper_world)[0]
            lower = _rotate_vectors(pelvis_inverse, lower_world)[0]
            upper /= max(np.linalg.norm(upper), 1e-8)
            lower /= max(np.linalg.norm(lower), 1e-8)
            horizontal = np.hypot(upper[0], upper[2])
            pitch = (
                np.arctan2(-upper[0], -upper[2])
                if horizontal >= 0.20
                else np.nan
            )
            roll = np.arcsin(np.clip(upper[1], -1.0, 1.0))
            elbow = 1.40 - np.arccos(
                np.clip(np.dot(upper, lower), -1.0, 1.0)
            )
            # Shoulder yaw controls the plane in which the G1's single-axis
            # elbow bends. Derive that plane from the human upper/lower-arm
            # geometry. With arms lateral, a vertical forearm yields positive
            # yaw on the left and negative yaw on the right, matching the G1.
            bend = lower - np.dot(lower, upper) * upper
            forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            reference = forward - np.dot(forward, upper) * upper
            bend_norm = np.linalg.norm(bend)
            reference_norm = np.linalg.norm(reference)
            if bend_norm < 0.10 or reference_norm < 0.20:
                plane_yaw = np.nan
            else:
                bend /= bend_norm
                reference /= reference_norm
                plane_yaw = np.arctan2(
                    -np.dot(np.cross(reference, bend), upper),
                    np.dot(reference, bend),
                )
            result[arm_index] = (pitch, roll, elbow, plane_yaw)
        return result

    def _apply_live_arm_mapping(
        self,
        dof_pos: np.ndarray,
        pelvis_quaternion: np.ndarray,
        positions: dict[str, np.ndarray],
    ) -> np.ndarray:
        """Anchor N-pose exactly while retaining direct T/forward geometry."""
        corrected = dof_pos.copy()
        angles = self._arm_angles(pelvis_quaternion, positions)
        for arm_index, (side, start) in enumerate(
            (("left", 15), ("right", 22))
        ):
            pitch, roll, elbow, plane_yaw = angles[arm_index]
            neutral_pitch, neutral_roll, _, _ = self._neutral_arm_angles[
                arm_index
            ]
            if np.isnan(pitch):
                target_pitch = self._previous_arm_pitch[arm_index]
            else:
                target_pitch = (
                    G1_DEFAULT_POSE[start] + pitch - neutral_pitch
                )
                self._previous_arm_pitch[arm_index] = target_pitch
            corrected[start] = target_pitch
            side_sign = 1.0 if side == "left" else -1.0
            forward_weight = np.clip(
                (abs(target_pitch) - 0.75) / 0.75,
                0.0,
                1.0,
            )
            if self.calibration.forward_positions is not None:
                current_pelvis_inverse = _conjugate(
                    np.asarray(pelvis_quaternion, dtype=np.float64)[None]
                )
                calibration_pelvis_inverse = _conjugate(
                    self.calibration.quaternions["pelvis"][None]
                )
                current_upper = (
                    np.asarray(
                        positions[f"{side}_forearm"], dtype=np.float64
                    )
                    - np.asarray(
                        positions[f"{side}_upper_arm"], dtype=np.float64
                    )
                )
                current_upper = _rotate_vectors(
                    current_pelvis_inverse, current_upper[None]
                )[0]
                t_upper = (
                    self.calibration.positions[f"{side}_forearm"]
                    - self.calibration.positions[f"{side}_upper_arm"]
                )
                t_upper = _rotate_vectors(
                    calibration_pelvis_inverse, t_upper[None]
                )[0]
                forward_upper = (
                    self.calibration.forward_positions[f"{side}_forearm"]
                    - self.calibration.forward_positions[
                        f"{side}_upper_arm"
                    ]
                )
                forward_upper = _rotate_vectors(
                    calibration_pelvis_inverse, forward_upper[None]
                )[0]
                current_upper /= max(np.linalg.norm(current_upper), 1e-8)
                t_upper /= max(np.linalg.norm(t_upper), 1e-8)
                forward_upper /= max(
                    np.linalg.norm(forward_upper), 1e-8
                )
                distance_forward = np.linalg.norm(
                    current_upper - forward_upper
                )
                distance_t = np.linalg.norm(current_upper - t_upper)
                forward_weight = float(
                    distance_t
                    / max(distance_t + distance_forward, 1e-8)
                )
            corrected[start + 1] = (
                G1_DEFAULT_POSE[start + 1] + roll - neutral_roll
                + side_sign * 0.30 * forward_weight
            )
            elbow_flexion = 1.40 - elbow
            bend_weight = np.clip(
                (elbow_flexion - 0.35) / 0.65,
                0.0,
                1.0,
            )
            quaternion_yaw = self.shoulder_yaw_gain * dof_pos[start + 2]
            target_yaw = (
                quaternion_yaw
                if np.isnan(plane_yaw)
                else (
                    (1.0 - bend_weight) * quaternion_yaw
                    + bend_weight * plane_yaw
                )
            )
            target_yaw = _nearest_equivalent_angle(
                target_yaw,
                self._previous_arm_yaw[arm_index],
            )
            if side == "left" and elbow_flexion > 0.70:
                pelvis_inverse = _conjugate(
                    np.asarray(pelvis_quaternion, dtype=np.float64)[None]
                )
                left_hand_local = _rotate_vectors(
                    pelvis_inverse,
                    np.asarray(positions["left_hand"], dtype=np.float64)[None],
                )[0]
                pelvis_local = _rotate_vectors(
                    pelvis_inverse,
                    np.asarray(positions["pelvis"], dtype=np.float64)[None],
                )[0]
                # A deeply flexed left arm near the body center is the
                # two-forearm face guard. The human has enough wrist/forearm
                # freedom to cross farther than the G1's arm chain can safely
                # reproduce. Limit only that medial guard case; T-pose,
                # reaching, and the validated strongman motion are unchanged.
                if left_hand_local[1] - pelvis_local[1] < 0.06:
                    target_yaw = min(target_yaw, 0.80)
                    target_yaw = (
                        0.65 * self._previous_arm_yaw[arm_index]
                        + 0.35 * target_yaw
                    )
            # Preserve the already validated left arm. The right strongman
            # forearm needs additional negative yaw to reach vertical; both
            # remain well inside the model's ±2.618 rad hardware limits.
            yaw_limit = 1.20 if side == "left" else 1.80
            target_yaw = float(
                np.clip(target_yaw, -yaw_limit, yaw_limit)
            )
            corrected[start + 2] = target_yaw
            self._previous_arm_yaw[arm_index] = target_yaw
            corrected[start + 3] = elbow
            if self.calibration.forward_positions is not None:
                pitch_offset, roll_offset, elbow_offset = (
                    self._forward_arm_offsets[arm_index]
                )
                corrected[start] += forward_weight * pitch_offset
                corrected[start + 1] += forward_weight * roll_offset
                corrected[start + 3] += forward_weight * elbow_offset
                # Shoulder yaw is poorly observable with a straight human
                # elbow. Fade it toward zero at the measured forward pose so
                # the two G1 forearms remain parallel instead of crossing.
                corrected[start + 2] *= 1.0 - forward_weight
                self._previous_arm_yaw[arm_index] = corrected[start + 2]
        return corrected

    def _correct_shoulders(self, dof_pos: np.ndarray) -> np.ndarray:
        corrected = dof_pos.copy()
        for pitch_id, roll_id in ((15, 16), (22, 23)):
            pitch = corrected[pitch_id]
            roll = abs(corrected[roll_id])
            reach_weight = np.clip((abs(pitch) - 0.5) / 1.0, 0.0, 1.0)
            correction = self.neutral_shoulder_bias + reach_weight * (
                self.reach_shoulder_bias - self.neutral_shoulder_bias
            )
            preserve_t_pose = 1.0 - np.clip((roll - 0.9) / 0.4, 0.0, 1.0)
            corrected[pitch_id] = pitch + correction * preserve_t_pose
        return corrected

    def _apply_wrist_mapping(
        self,
        dof_pos: np.ndarray,
        current_quaternions: dict[str, np.ndarray],
    ) -> np.ndarray:
        """Map hand orientation relative to forearm onto the G1 wrists."""
        if not self.track_wrists:
            return dof_pos
        corrected = dof_pos.copy()
        for arm_index, (side, start) in enumerate(
            (("left", 15), ("right", 22))
        ):
            delta = _relative_delta(
                current_quaternions[f"{side}_forearm"][None],
                current_quaternions[f"{side}_hand"][None],
                self.calibration.quaternions[f"{side}_forearm"][None],
                self.calibration.quaternions[f"{side}_hand"][None],
            )[0]
            # Palm-up/down rotation drives wrist roll only. Wrist pitch and
            # yaw deliberately remain at their neutral direct-pose values.
            palm_roll = float(np.dot(
                delta, self._wrist_twist_axes[arm_index]
            ))
            corrected[start + 4] = np.clip(
                self.wrist_roll_gain * palm_roll, -1.40, 1.40
            )
            if self.track_wrist_pitch:
                wrist_pitch = float(np.dot(
                    delta, self._wrist_pitch_axes[arm_index]
                ))
                corrected[start + 5] = G1_DEFAULT_POSE[start + 5]
                corrected[start + 6] = np.clip(
                    self.wrist_pitch_signs[arm_index]
                    * self.wrist_pitch_gain
                    * wrist_pitch,
                    -self.wrist_pitch_limit_rad,
                    self.wrist_pitch_limit_rad,
                )
            else:
                corrected[start + 5] = G1_DEFAULT_POSE[start + 5]
                corrected[start + 6] = G1_DEFAULT_POSE[start + 6]
        return corrected

    @staticmethod
    def _yaw(quaternion: np.ndarray) -> float:
        w, x, y, z = quaternion
        return float(np.arctan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        ))

    def _apply_pelvis_yaw(
        self,
        dof_pos: np.ndarray,
        pelvis_quaternion: np.ndarray,
    ) -> np.ndarray:
        """Map calibrated pelvis heading onto bounded G1 waist yaw only."""
        if not self.track_pelvis_yaw:
            return dof_pos
        relative = _multiply(
            _conjugate(self.calibration.quaternions["pelvis"][None]),
            pelvis_quaternion[None],
        )[0]
        corrected = dof_pos.copy()
        corrected[12] = np.clip(
            self.pelvis_yaw_gain * self._yaw(relative),
            -self.pelvis_yaw_limit_rad,
            self.pelvis_yaw_limit_rad,
        )
        return corrected

    def _apply_lifted_foot_heading(
        self,
        dof_pos: np.ndarray,
        current_quaternions: dict[str, np.ndarray],
        current_positions: dict[str, np.ndarray],
    ) -> np.ndarray:
        """Map lifted-foot heading to hip yaw; never twist a planted foot."""
        if not self.track_foot_heading:
            return dof_pos
        corrected = dof_pos.copy()
        current_pelvis_z = float(current_positions["pelvis"][2])
        neutral_pelvis_z = float(self.calibration.positions["pelvis"][2])
        for side, hip_yaw_index in (("left", 2), ("right", 8)):
            current_relative_height = (
                float(current_positions[f"{side}_foot"][2])
                - current_pelvis_z
            )
            neutral_relative_height = (
                float(self.calibration.positions[f"{side}_foot"][2])
                - neutral_pelvis_z
            )
            if (
                current_relative_height - neutral_relative_height
                <= self.foot_lift_threshold_m
            ):
                continue
            foot_delta = _relative_delta(
                current_quaternions["pelvis"][None],
                current_quaternions[f"{side}_foot"][None],
                self.calibration.quaternions["pelvis"][None],
                self.calibration.quaternions[f"{side}_foot"][None],
            )[0]
            corrected[hip_yaw_index] = np.clip(
                self.foot_heading_gain * foot_delta[2],
                -self.foot_heading_limit_rad,
                self.foot_heading_limit_rad,
            )
        return corrected

    def _apply_lifted_hip_roll(
        self,
        dof_pos: np.ndarray,
        pelvis_quaternion: np.ndarray,
        current_positions: dict[str, np.ndarray],
    ) -> np.ndarray:
        """Use lateral knee position to refine hip roll on a lifted leg."""
        if not self.track_lifted_hip_roll:
            return dof_pos
        corrected = dof_pos.copy()
        current_pelvis_z = float(current_positions["pelvis"][2])
        neutral_pelvis_z = float(self.calibration.positions["pelvis"][2])
        current_inverse = _conjugate(pelvis_quaternion[None])
        neutral_inverse = _conjugate(
            self.calibration.quaternions["pelvis"][None]
        )
        for side, hip_roll_index in (("left", 1), ("right", 7)):
            current_relative_height = (
                float(current_positions[f"{side}_foot"][2])
                - current_pelvis_z
            )
            neutral_relative_height = (
                float(self.calibration.positions[f"{side}_foot"][2])
                - neutral_pelvis_z
            )
            if (
                current_relative_height - neutral_relative_height
                <= self.foot_lift_threshold_m
            ):
                continue
            current_upper = _rotate_vectors(
                current_inverse,
                (
                    current_positions[f"{side}_lower_leg"]
                    - current_positions[f"{side}_upper_leg"]
                )[None],
            )[0]
            neutral_upper = _rotate_vectors(
                neutral_inverse,
                (
                    self.calibration.positions[f"{side}_lower_leg"]
                    - self.calibration.positions[f"{side}_upper_leg"]
                )[None],
            )[0]
            current_roll = float(np.arctan2(
                current_upper[1], -current_upper[2]
            ))
            neutral_roll = float(np.arctan2(
                neutral_upper[1], -neutral_upper[2]
            ))
            roll_delta = float(np.arctan2(
                np.sin(current_roll - neutral_roll),
                np.cos(current_roll - neutral_roll),
            ))
            corrected[hip_roll_index] = np.clip(
                self.lifted_hip_roll_gain * roll_delta,
                -self.lifted_hip_roll_limit_rad,
                self.lifted_hip_roll_limit_rad,
            )
        return corrected

    def retarget(self, frame: PoseFrame, timestamp: float) -> RetargetedPose:
        current_quaternions, current_positions = _frame_maps(frame)
        missing = [
            *(
                name
                for name in REQUIRED_SEGMENTS
                if name not in current_quaternions
            ),
            *(
                name
                for name in ARM_POSITION_SEGMENTS
                if name not in current_positions
            ),
        ]
        if missing:
            raise ValueError(f"Live Xsens frame is missing: {', '.join(missing)}")

        quaternions = {
            name: np.stack(
                (self.calibration.quaternions[name], current_quaternions[name])
            )
            for name in REQUIRED_SEGMENTS
        }
        positions = {
            name: np.stack(
                (self.calibration.positions[name], current_positions[name])
            )
            for name in ARM_POSITION_SEGMENTS
        }
        dof_pos = retarget_quaternions(
            quaternions,
            motion_gain=self.motion_gain,
            torso_gain=self.torso_gain,
            calibration_frame=0,
            base_pose=G1_T_POSE,
            positions=positions,
        )[1]
        dof_pos = self._apply_live_arm_mapping(
            dof_pos,
            current_quaternions["pelvis"],
            current_positions,
        )
        dof_pos = self._apply_wrist_mapping(dof_pos, current_quaternions)
        dof_pos = self._apply_pelvis_yaw(
            dof_pos, current_quaternions["pelvis"]
        )
        dof_pos = self._apply_lifted_foot_heading(
            dof_pos, current_quaternions, current_positions
        )
        dof_pos = self._apply_lifted_hip_roll(
            dof_pos, current_quaternions["pelvis"], current_positions
        )
        dof_pos = self._correct_shoulders(dof_pos)
        if self.joint_ranges is not None:
            dof_pos, _ = clamp_to_model_limits(dof_pos[None], self.joint_ranges)
            dof_pos = dof_pos[0]

        if self._previous_time is None or self._previous_pos is None:
            dof_vel = np.zeros(29, dtype=np.float64)
        else:
            dt = timestamp - self._previous_time
            if dt <= 0.0:
                raise ValueError("Live timestamps must be strictly increasing")
            # Limit the position reference itself, not only the velocity field.
            # This prevents a shoulder pitch that was held through a geometric
            # singularity—or a pose received after a stream stall—from
            # snapping directly to the newest target. Cap the effective dt so
            # a long outage cannot authorize one oversized catch-up step.
            max_step = self.max_position_speed * min(
                dt, self.max_position_dt
            )
            dof_pos = self._previous_pos + np.clip(
                dof_pos - self._previous_pos,
                -max_step,
                max_step,
            )
            raw_velocity = np.clip(
                (dof_pos - self._previous_pos) / dt,
                -self.max_joint_speed,
                self.max_joint_speed,
            )
            self._filtered_vel = (
                self.velocity_alpha * raw_velocity
                + (1.0 - self.velocity_alpha) * self._filtered_vel
            )
            dof_vel = self._filtered_vel.copy()

        self._previous_time = timestamp
        self._previous_pos = dof_pos.copy()
        return RetargetedPose(
            timestamp=timestamp,
            dof_pos=dof_pos.astype(np.float32),
            dof_vel=dof_vel.astype(np.float32),
        )
