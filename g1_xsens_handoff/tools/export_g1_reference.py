#!/usr/bin/env python3
"""Export a calibrated Xsens recording as a G1 motion-tracking NPZ."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from xsens_bridge.g1_retarget import (
    G1_JOINT_NAMES,
    G1_T_POSE,
    clamp_to_model_limits,
    detect_t_pose_frame,
    load_pelvis_positions,
    load_segment_positions,
    load_xsens_csv,
    retarget_quaternions,
    scaled_pelvis_motion,
)


ARM_POSITION_SEGMENTS = (
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
)


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    result = q.copy()
    result[..., 1:] *= -1.0
    return result


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
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


def _angular_velocity(quaternions: np.ndarray, dt: float) -> np.ndarray:
    """Finite-difference wxyz orientations into world-frame angular velocity."""
    current = quaternions[:-1]
    following = quaternions[1:].copy()
    following = np.where(
        np.sum(current * following, axis=-1, keepdims=True) < 0.0,
        -following,
        following,
    )
    delta = _quat_multiply(following, _quat_conjugate(current))
    delta = np.where(delta[..., :1] < 0.0, -delta, delta)
    xyz = delta[..., 1:]
    length = np.linalg.norm(xyz, axis=-1)
    angle = 2.0 * np.arctan2(length, np.clip(delta[..., 0], -1.0, 1.0))
    scale = np.divide(
        angle,
        length,
        out=np.full_like(angle, 2.0),
        where=length > 1e-8,
    )
    velocity = xyz * scale[..., None] / dt
    return np.concatenate((velocity, velocity[-1:]), axis=0)


def export_reference(
    csv_path: Path,
    xml_path: Path,
    output_path: Path,
    *,
    motion_gain: float,
    torso_gain: float,
    root_gain: float,
) -> dict[str, float | int]:
    times, quaternions = load_xsens_csv(csv_path)
    positions = load_segment_positions(csv_path, ARM_POSITION_SEGMENTS)
    calibration_frame = detect_t_pose_frame(csv_path)
    joint_pos = retarget_quaternions(
        quaternions,
        motion_gain=motion_gain,
        torso_gain=torso_gain,
        calibration_frame=calibration_frame,
        base_pose=G1_T_POSE,
        positions=positions,
    )
    root_delta, root_quat = scaled_pelvis_motion(
        load_pelvis_positions(csv_path),
        quaternions["pelvis"],
        gain=root_gain,
    )

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in G1_JOINT_NAMES]
    )
    if np.any(joint_ids < 0):
        raise ValueError("MuJoCo model does not contain all expected G1 joints")
    qpos_addresses = model.jnt_qposadr[joint_ids]
    joint_pos, clamp_count = clamp_to_model_limits(
        joint_pos,
        model.jnt_range[joint_ids],
    )

    # The bridge CSV is resampled onto a nominal 50 Hz frame sequence, but each
    # row retains its nearest packet timestamp and therefore contains sub-frame
    # jitter. Recover the intended uniform rate from the complete duration.
    fps = float(round((len(times) - 1) / (times[-1] - times[0])))
    dt = 1.0 / fps
    joint_vel = np.gradient(joint_pos, dt, axis=0)
    root_origin = np.array((0.0, 0.0, 0.793))

    frame_count = len(times)
    # MuJoCo body 0 is the world. Tracking references contain robot bodies only,
    # with the pelvis at index 0 (30 bodies for the G1 model).
    robot_body_count = model.nbody - 1
    body_pos = np.empty((frame_count, robot_body_count, 3), dtype=np.float64)
    body_quat = np.empty((frame_count, robot_body_count, 4), dtype=np.float64)
    for frame in range(frame_count):
        data.qpos[:3] = root_origin + root_delta[frame]
        data.qpos[3:7] = root_quat[frame]
        data.qpos[qpos_addresses] = joint_pos[frame]
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        body_pos[frame] = data.xpos[1:]
        body_quat[frame] = data.xquat[1:]

    body_lin_vel = np.gradient(body_pos, dt, axis=0)
    body_ang_vel = _angular_velocity(body_quat, dt)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        fps=np.array([fps], dtype=np.float64),
        time_s=times.astype(np.float64),
        joint_pos=joint_pos.astype(np.float32),
        joint_vel=joint_vel.astype(np.float32),
        body_pos_w=body_pos.astype(np.float32),
        body_quat_w=body_quat.astype(np.float32),
        body_lin_vel_w=body_lin_vel.astype(np.float32),
        body_ang_vel_w=body_ang_vel.astype(np.float32),
        joint_names=np.asarray(G1_JOINT_NAMES),
        body_names=np.asarray(
            [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) or "world"
                for index in range(1, model.nbody)
            ]
        ),
        calibration_frame=np.array([calibration_frame], dtype=np.int32),
        source_csv=np.array([str(csv_path)]),
    )

    return {
        "frames": frame_count,
        "fps": fps,
        "duration_s": float(times[-1]),
        "clamped_values": clamp_count,
        "peak_joint_speed": float(np.max(np.abs(joint_vel))),
        "bodies": robot_body_count,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--gain", type=float, default=0.75)
    parser.add_argument("--torso-gain", type=float, default=0.50)
    parser.add_argument("--root-gain", type=float, default=0.65)
    args = parser.parse_args()

    stats = export_reference(
        args.csv,
        args.xml,
        args.output,
        motion_gain=args.gain,
        torso_gain=args.torso_gain,
        root_gain=args.root_gain,
    )
    print(f"Exported: {args.output}")
    for name, value in stats.items():
        print(f"{name}: {value}")


if __name__ == "__main__":
    main()
