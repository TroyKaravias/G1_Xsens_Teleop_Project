#!/usr/bin/env python3
"""Preview an exported Xsens recording on a torque-free, kinematic G1."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="50 Hz Xsens bridge CSV")
    parser.add_argument("--xml", type=Path, required=True, help="G1 29-DOF MuJoCo XML")
    parser.add_argument("--gain", type=float, default=0.75, help="Limb motion amplitude, 0..1")
    parser.add_argument(
        "--torso-gain",
        type=float,
        default=0.50,
        help="Torso motion amplitude, 0..1",
    )
    parser.add_argument(
        "--root-gain",
        type=float,
        default=0.65,
        help="Pelvis translation/orientation amplitude, 0..1",
    )
    parser.add_argument(
        "--fixed-root",
        action="store_true",
        help="Keep the pelvis fixed (useful for diagnosing individual joints)",
    )
    parser.add_argument(
        "--calibration-frame",
        type=int,
        help="Xsens T-pose frame; detected automatically when omitted",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed")
    parser.add_argument("--summary-only", action="store_true")
    args = parser.parse_args()

    if not 0.0 <= args.gain <= 1.0:
        parser.error("--gain must be between 0 and 1")
    if not 0.0 <= args.torso_gain <= 1.0:
        parser.error("--torso-gain must be between 0 and 1")
    if not 0.0 <= args.root_gain <= 1.0:
        parser.error("--root-gain must be between 0 and 1")
    if args.speed <= 0.0:
        parser.error("--speed must be positive")

    times, quaternions = load_xsens_csv(args.csv)
    segment_positions = load_segment_positions(
        args.csv,
        (
            "left_upper_arm",
            "left_forearm",
            "left_hand",
            "right_upper_arm",
            "right_forearm",
            "right_hand",
        ),
    )
    calibration_frame = (
        args.calibration_frame
        if args.calibration_frame is not None
        else detect_t_pose_frame(args.csv)
    )
    trajectory = retarget_quaternions(
        quaternions,
        motion_gain=args.gain,
        torso_gain=args.torso_gain,
        calibration_frame=calibration_frame,
        base_pose=G1_T_POSE,
        positions=segment_positions,
    )
    pelvis_positions = load_pelvis_positions(args.csv)
    root_positions, root_orientations = scaled_pelvis_motion(
        pelvis_positions,
        quaternions["pelvis"],
        gain=args.root_gain,
    )
    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)

    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in G1_JOINT_NAMES]
    )
    if np.any(joint_ids < 0):
        missing = [name for name, joint_id in zip(G1_JOINT_NAMES, joint_ids) if joint_id < 0]
        raise ValueError(f"XML is missing G1 joints: {', '.join(missing)}")
    qpos_addresses = model.jnt_qposadr[joint_ids]
    joint_ranges = model.jnt_range[joint_ids]
    trajectory, clamp_count = clamp_to_model_limits(trajectory, joint_ranges)

    frame_dt = float(np.median(np.diff(times)))
    velocities = np.diff(trajectory, axis=0) / frame_dt
    print(f"Frames: {len(times)}, rate: {1.0 / frame_dt:.1f} Hz, duration: {times[-1]:.1f}s")
    print(
        f"T-pose calibration: frame {calibration_frame}, "
        f"time {times[calibration_frame]:.2f}s"
    )
    print(f"Joint values clamped: {clamp_count} / {trajectory.size}")
    print(f"Peak preview joint speed: {np.max(np.abs(velocities)):.2f} rad/s")
    print("KINEMATIC PREVIEW ONLY: actuators and physics stepping are disabled.")

    if args.summary_only:
        return

    # We update qpos and forward kinematics but never call mj_step, so no policy,
    # torque, contact, or dynamic instability is involved.
    root_origin = np.array((0.0, 0.0, 0.793))
    frame = 0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            started = time.monotonic()
            if args.fixed_root:
                data.qpos[:3] = root_origin
                data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
            else:
                data.qpos[:3] = root_origin + root_positions[frame]
                data.qpos[3:7] = root_orientations[frame]
            data.qpos[qpos_addresses] = trajectory[frame]
            data.qvel[:] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)
            viewer.sync()
            frame = (frame + 1) % len(trajectory)
            remaining = frame_dt / args.speed - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
