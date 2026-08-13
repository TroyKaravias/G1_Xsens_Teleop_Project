#!/usr/bin/env python3
"""Convert a validated G1 reference NPZ into a ProtoMotions MotionPlayer cache."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch


def _quat_wxyz_to_xyzw(quat: np.ndarray) -> np.ndarray:
    return quat[..., [1, 2, 3, 0]]


def _differentiate(values: np.ndarray, dt: float) -> np.ndarray:
    return np.gradient(values, dt, axis=0).astype(np.float32)


def _quat_conjugate_xyzw(quat: np.ndarray) -> np.ndarray:
    result = quat.copy()
    result[..., :3] *= -1.0
    return result


def _quat_multiply_xyzw(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = np.moveaxis(lhs, -1, 0)
    rx, ry, rz, rw = np.moveaxis(rhs, -1, 0)
    return np.stack(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ),
        axis=-1,
    )


def _angular_velocity_xyzw(quat: np.ndarray, dt: float) -> np.ndarray:
    """Estimate world-frame angular velocity from normalized xyzw quaternions."""
    quat = quat.astype(np.float64, copy=True)
    quat /= np.linalg.norm(quat, axis=-1, keepdims=True)
    dots = np.sum(quat[1:] * quat[:-1], axis=-1)
    quat[1:][dots < 0.0] *= -1.0

    delta = _quat_multiply_xyzw(quat[1:], _quat_conjugate_xyzw(quat[:-1]))
    delta /= np.linalg.norm(delta, axis=-1, keepdims=True)
    vector = delta[..., :3]
    scalar = np.clip(delta[..., 3], -1.0, 1.0)
    vector_norm = np.linalg.norm(vector, axis=-1)
    angle = 2.0 * np.arctan2(vector_norm, scalar)
    axis = np.divide(
        vector,
        vector_norm[..., None],
        out=np.zeros_like(vector),
        where=vector_norm[..., None] > 1e-9,
    )
    interval_velocity = axis * (angle[..., None] / dt)
    velocity = np.empty(quat.shape[:-1] + (3,), dtype=np.float64)
    velocity[0] = interval_velocity[0]
    velocity[-1] = interval_velocity[-1]
    if len(quat) > 2:
        velocity[1:-1] = 0.5 * (interval_velocity[:-1] + interval_velocity[1:])
    return velocity.astype(np.float32)


def _joint_names(model: mujoco.MjModel) -> list[str]:
    names: list[str] = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id))
    return names


def _apply_shoulder_pitch_correction(
    joint_pos: np.ndarray,
    joint_names: list[str],
    neutral_bias_deg: float,
    reach_bias_deg: float,
) -> np.ndarray:
    """Lower forward-raised arms while preserving high-roll T-pose frames."""
    corrected = joint_pos.copy()
    neutral_bias = np.deg2rad(neutral_bias_deg)
    reach_bias = np.deg2rad(reach_bias_deg)
    for side in ("left", "right"):
        pitch_id = joint_names.index(f"{side}_shoulder_pitch_joint")
        roll_id = joint_names.index(f"{side}_shoulder_roll_joint")
        pitch = corrected[:, pitch_id]
        roll = np.abs(corrected[:, roll_id])

        # Blend from the neutral correction to the larger forward-reach
        # correction as shoulder pitch magnitude approaches 1.5 rad.
        reach_weight = np.clip((np.abs(pitch) - 0.5) / 1.0, 0.0, 1.0)
        correction = neutral_bias + reach_weight * (reach_bias - neutral_bias)

        # Preserve the visually validated T-pose: fade the pitch correction out
        # as shoulder roll moves from 0.9 rad toward the T-pose range.
        preserve_t_pose = 1.0 - np.clip((roll - 0.9) / 0.4, 0.0, 1.0)
        corrected[:, pitch_id] = pitch + correction * preserve_t_pose
    return corrected


def _load_protomotions_model(mjcf_path: Path) -> mujoco.MjModel:
    """Apply the same standalone-MuJoCo patch as ProtoMotions deployment."""
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    for sensor in root.findall("sensor"):
        root.remove(sensor)

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"MJCF has no worldbody: {mjcf_path}")
    has_floor = any(geom.get("name") == "floor" for geom in worldbody.findall("geom"))
    if not has_floor:
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "floor",
                "type": "plane",
                "size": "0 0 0.05",
                "rgba": "0.7 0.7 0.7 1",
            },
        )

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".xml",
        dir=mjcf_path.parent,
        delete=False,
    ) as temporary:
        tree.write(temporary, encoding="unicode")
        temporary_path = Path(temporary.name)
    try:
        return mujoco.MjModel.from_xml_path(str(temporary_path))
    finally:
        os.unlink(temporary_path)


def convert(
    reference_path: Path,
    mjcf_path: Path,
    output_path: Path,
    neutral_shoulder_bias_deg: float = 0.0,
    reach_shoulder_bias_deg: float = 0.0,
) -> None:
    reference = np.load(reference_path)
    fps = float(np.asarray(reference["fps"]).reshape(-1)[0])
    dt = 1.0 / fps
    joint_pos = np.asarray(reference["joint_pos"], dtype=np.float32)
    joint_vel = np.asarray(reference["joint_vel"], dtype=np.float32)
    source_joint_names = [str(name) for name in reference["joint_names"]]
    source_body_pos = np.asarray(reference["body_pos_w"], dtype=np.float32)
    source_body_quat_wxyz = np.asarray(reference["body_quat_w"], dtype=np.float32)

    if joint_pos.ndim != 2 or joint_pos.shape[1] != 29:
        raise ValueError(f"Expected joint_pos shape (frames, 29), got {joint_pos.shape}")
    if joint_vel.shape != joint_pos.shape:
        raise ValueError(f"joint_vel shape {joint_vel.shape} does not match joint_pos")
    if source_body_pos.shape[0] != len(joint_pos):
        raise ValueError("Body and joint frame counts do not match")
    if neutral_shoulder_bias_deg or reach_shoulder_bias_deg:
        joint_pos = _apply_shoulder_pitch_correction(
            joint_pos,
            source_joint_names,
            neutral_shoulder_bias_deg,
            reach_shoulder_bias_deg,
        )
        joint_vel = _differentiate(joint_pos, dt)

    model = _load_protomotions_model(mjcf_path)
    data = mujoco.MjData(model)
    model_joint_names = _joint_names(model)
    if model_joint_names != source_joint_names:
        differences = [
            f"{index}: source={source!r}, model={target!r}"
            for index, (source, target) in enumerate(
                zip(source_joint_names, model_joint_names, strict=False)
            )
            if source != target
        ]
        raise ValueError(
            "Joint order mismatch between reference and ProtoMotions MJCF:\n"
            + "\n".join(differences[:10])
        )
    if model.nq != 7 + 29 or model.nv != 6 + 29:
        raise ValueError(f"Expected free root + 29 joints, got nq={model.nq}, nv={model.nv}")

    num_frames = len(joint_pos)
    num_bodies = model.nbody - 1
    body_pos = np.empty((num_frames, num_bodies, 3), dtype=np.float32)
    body_rot = np.empty((num_frames, num_bodies, 4), dtype=np.float32)

    root_pos = source_body_pos[:, 0]
    root_quat_wxyz = source_body_quat_wxyz[:, 0]
    for frame in range(num_frames):
        data.qpos[:3] = root_pos[frame]
        data.qpos[3:7] = root_quat_wxyz[frame]
        data.qpos[7:] = joint_pos[frame]
        mujoco.mj_forward(model, data)
        body_pos[frame] = data.xpos[1:]
        body_rot[frame] = _quat_wxyz_to_xyzw(data.xquat[1:])

    body_vel = _differentiate(body_pos, dt)
    body_ang_vel = _angular_velocity_xyzw(body_rot, dt)
    cache = {
        "dof_pos": joint_pos,
        "dof_vel": joint_vel,
        "body_rot": body_rot,
        "body_pos": body_pos,
        "body_vel": body_vel,
        "body_ang_vel": body_ang_vel,
        "control_dt": dt,
        "num_frames": num_frames,
    }

    for key, value in cache.items():
        if isinstance(value, np.ndarray) and not np.isfinite(value).all():
            raise ValueError(f"Non-finite values found in {key}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(cache, output_path)
    print(
        f"Saved ProtoMotions cache: {output_path}\n"
        f"  frames={num_frames}, fps={fps:.1f}, dofs={joint_pos.shape[1]}, "
        f"bodies={num_bodies}, duration={num_frames / fps:.2f}s"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_npz", type=Path)
    parser.add_argument("mjcf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--neutral-shoulder-bias-deg",
        type=float,
        default=0.0,
        help="Positive shoulder-pitch correction near neutral poses.",
    )
    parser.add_argument(
        "--reach-shoulder-bias-deg",
        type=float,
        default=0.0,
        help="Positive shoulder-pitch correction for large forward reaches.",
    )
    args = parser.parse_args()
    convert(
        args.reference_npz,
        args.mjcf,
        args.output,
        args.neutral_shoulder_bias_deg,
        args.reach_shoulder_bias_deg,
    )


if __name__ == "__main__":
    main()
