#!/usr/bin/env python3
"""Replay recorded Xsens motion on an actuator-free AgiBot X2 in MuJoCo."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xsens_bridge.g1_retarget import (
    detect_t_pose_frame,
    load_segment_positions,
    load_xsens_csv,
)
from xsens_bridge.x2_retarget import (
    X2_JOINT_NAMES,
    clamp_x2_trajectory,
    lowpass_x2_trajectory,
    model_xml,
    retarget_xsens_to_x2,
    validate_model_joint_names,
)


def _render_snapshots(
    mujoco,
    model,
    data,
    qpos_addresses: np.ndarray,
    trajectory: np.ndarray,
    frames: list[int],
    output_dir: Path,
) -> None:
    from PIL import Image

    output_dir.mkdir(parents=True, exist_ok=True)
    # Both pinned vendor models declare the default 640x480 offscreen buffer.
    renderer = mujoco.Renderer(model, height=480, width=640)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, 0.0, 0.85)
    camera.distance = 3.2
    camera.elevation = -8.0
    for frame in frames:
        data.qpos[qpos_addresses] = trajectory[frame]
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        for label, azimuth in (("front", 180.0), ("right", 90.0)):
            camera.azimuth = azimuth
            renderer.update_scene(data, camera=camera)
            image = renderer.render()
            Image.fromarray(image).save(
                output_dir / f"frame_{frame:04d}_{label}.png"
            )
    renderer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--variant", choices=("v1.3", "v1.4"), default="v1.3")
    parser.add_argument("--gain", type=float, default=0.75)
    parser.add_argument("--torso-gain", type=float, default=0.50)
    parser.add_argument("--calibration-frame", type=int)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--filter-cutoff-hz",
        type=float,
        default=4.0,
        help="X2-only one-pole joint filter; zero disables it (default: 4)",
    )
    parser.add_argument(
        "--render-dir",
        type=Path,
        help="write headless front/right PNG snapshots instead of opening a viewer",
    )
    parser.add_argument(
        "--render-frames",
        type=int,
        nargs="*",
        help="snapshot frame indices (default: calibration and four time samples)",
    )
    parser.add_argument("--summary-only", action="store_true")
    args = parser.parse_args()

    if not 0.0 <= args.gain <= 1.0:
        parser.error("--gain must be between 0 and 1")
    if not 0.0 <= args.torso_gain <= 1.0:
        parser.error("--torso-gain must be between 0 and 1")
    if args.speed <= 0.0:
        parser.error("--speed must be positive")
    if args.filter_cutoff_hz < 0.0:
        parser.error("--filter-cutoff-hz must be non-negative")

    import mujoco

    times, quaternions = load_xsens_csv(args.csv)
    positions = load_segment_positions(
        args.csv,
        (
            "left_upper_arm", "left_forearm", "left_hand",
            "right_upper_arm", "right_forearm", "right_hand",
        ),
    )
    calibration_frame = (
        args.calibration_frame
        if args.calibration_frame is not None
        else detect_t_pose_frame(args.csv)
    )
    trajectory = retarget_xsens_to_x2(
        quaternions,
        motion_gain=args.gain,
        torso_gain=args.torso_gain,
        calibration_frame=calibration_frame,
        positions=positions,
    )
    frame_dt = float(np.median(np.diff(times)))
    unfiltered_peak_speed = float(
        np.max(np.abs(np.diff(trajectory, axis=0) / frame_dt))
    )
    trajectory = lowpass_x2_trajectory(
        trajectory, frame_dt=frame_dt, cutoff_hz=args.filter_cutoff_hz
    )

    xml = model_xml(args.models, args.variant)
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    model_joint_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        != "floating_base_joint"
    )
    validate_model_joint_names(model_joint_names)
    joint_ids = np.asarray([
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in X2_JOINT_NAMES
    ])
    qpos_addresses = model.jnt_qposadr[joint_ids]
    joint_ranges = model.jnt_range[joint_ids]
    below_or_above = (trajectory < joint_ranges[:, 0]) | (
        trajectory > joint_ranges[:, 1]
    )
    clamp_counts = np.count_nonzero(below_or_above, axis=0)
    trajectory, clamp_count = clamp_x2_trajectory(trajectory, joint_ranges)
    velocities = np.diff(trajectory, axis=0) / frame_dt
    print(f"Model: {xml}")
    print(f"Frames: {len(times)}, rate: {1.0 / frame_dt:.1f} Hz")
    print(f"Calibration frame: {calibration_frame}")
    print(f"X2 filter cutoff: {args.filter_cutoff_hz:.1f} Hz")
    print(f"Joint values clamped: {clamp_count} / {trajectory.size}")
    for name, count in zip(X2_JOINT_NAMES, clamp_counts):
        if count:
            print(f"  {name}: {count}")
    print(f"Unfiltered peak kinematic joint speed: {unfiltered_peak_speed:.2f} rad/s")
    print(f"Peak kinematic joint speed: {np.max(np.abs(velocities)):.2f} rad/s")
    print("SIMULATION ONLY: actuator commands and physics stepping are disabled.")
    if args.render_dir:
        frames = args.render_frames
        if frames is None:
            frames = sorted({
                0,
                calibration_frame,
                len(trajectory) // 4,
                len(trajectory) // 2,
                3 * len(trajectory) // 4,
                len(trajectory) - 1,
            })
        invalid = [frame for frame in frames if not 0 <= frame < len(trajectory)]
        if invalid:
            parser.error(f"render frame indices out of range: {invalid}")
        _render_snapshots(
            mujoco, model, data, qpos_addresses, trajectory, frames, args.render_dir
        )
        print(f"Rendered {len(frames) * 2} snapshots to {args.render_dir}")
        return
    if args.summary_only:
        return

    import mujoco.viewer

    frame = 0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            started = time.monotonic()
            data.qpos[qpos_addresses] = trajectory[frame]
            data.qvel[:] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)
            viewer.sync()
            frame = (frame + 1) % len(trajectory)
            time.sleep(max(0.0, frame_dt / args.speed - (time.monotonic() - started)))


if __name__ == "__main__":
    main()
