#!/usr/bin/env python3
"""Generate a chained X2 planner reference for SONIC baseline validation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xsens_bridge.x2_planner import X2KinematicPlanner, pack_mujoco_qpos, unpack_mujoco_qpos


def load_xsens_upper_body(path: Path) -> tuple[np.ndarray, float]:
    from xsens_bridge.g1_retarget import (
        detect_t_pose_frame, load_segment_positions, load_xsens_csv,
    )
    from xsens_bridge.x2_retarget import lowpass_x2_trajectory, retarget_xsens_to_x2

    times, quaternions = load_xsens_csv(path)
    positions = load_segment_positions(path, (
        "left_upper_arm", "left_forearm", "left_hand",
        "right_upper_arm", "right_forearm", "right_hand",
    ))
    frame_dt = float(np.median(np.diff(times)))
    calibration = detect_t_pose_frame(path)
    trajectory = retarget_xsens_to_x2(
        quaternions, calibration_frame=calibration, positions=positions,
    )
    trajectory = lowpass_x2_trajectory(trajectory, frame_dt, 4.0)
    return trajectory[calibration:] - trajectory[calibration], frame_dt


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", type=Path, required=True)
    parser.add_argument("--seed-motion", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--mode", type=int, default=0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--velocity", type=float, nargs=4, default=(0, 0, 0, 0))
    parser.add_argument(
        "--xsens-csv", type=Path,
        help="inject recorded Xsens upper-body deltas into planner context",
    )
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    import joblib
    motions = joblib.load(args.seed_motion)
    motion = next(iter(motions.values()))
    context = pack_mujoco_qpos(
        np.asarray(motion["root_trans_offset"][:4]),
        np.asarray(motion["root_rot"][:4]),
        np.asarray(motion["dof"][:4]),
    )
    xsens_upper_origin = context[0, 19:].copy()
    planner = X2KinematicPlanner(args.planner)
    xsens = load_xsens_upper_body(args.xsens_csv) if args.xsens_csv else None
    required = int(np.ceil(args.seconds * 30.0))
    chunks = []
    previous_xsens_delta = np.zeros((4, 19), dtype=np.float32)
    while sum(len(chunk) for chunk in chunks) < required:
        if xsens is not None:
            xsens_trajectory, xsens_dt = xsens
            elapsed = sum(len(chunk) for chunk in chunks) / 30.0
            indices = np.minimum(
                ((elapsed + np.arange(4) / 30.0) / xsens_dt).astype(int),
                len(xsens_trajectory) - 1,
            )
            # Preserve planner root and locomotion. Xsens supplies calibrated
            # waist/arm/head deltas to the four-frame generative context.
            current_xsens_delta = xsens_trajectory[indices, 12:]
            context[:, 7 + 12:] += current_xsens_delta - previous_xsens_delta
            previous_xsens_delta = current_xsens_delta
        prediction = planner.generate(context, np.asarray(args.velocity), args.mode, args.seed + len(chunks))
        chunks.append(prediction)
        context = prediction[-4:]
    qpos = np.concatenate(chunks)[:required]
    if xsens is not None:
        xsens_trajectory, xsens_dt = xsens
        indices = np.minimum(
            (np.arange(required, dtype=np.float64) / 30.0 / xsens_dt).astype(int),
            len(xsens_trajectory) - 1,
        )
        qpos[:, 19:] = xsens_upper_origin[None, :] + xsens_trajectory[indices, 12:]
    root_position, root_quaternion, joints = unpack_mujoco_qpos(qpos)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump({"x2_planner": {
        "root_trans_offset": root_position,
        "root_rot": root_quaternion,
        "dof": joints,
        "fps": 30.0,
    }}, args.output)
    print(f"Generated {len(qpos)} X2 planner frames ({len(qpos) / 30.0:.2f}s): {args.output}")
    print("SIMULATION REFERENCE ONLY: no policy, dynamics, or hardware validation.")


if __name__ == "__main__":
    main()
