#!/usr/bin/env python3
"""Convert a validated G1 reference NPZ to mjlab's 36-column input CSV."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_npz", type=Path)
    parser.add_argument("output_csv", type=Path)
    args = parser.parse_args()

    reference = np.load(args.reference_npz)
    joint_pos = reference["joint_pos"]
    body_pos = reference["body_pos_w"]
    body_quat_wxyz = reference["body_quat_w"]

    if joint_pos.ndim != 2 or joint_pos.shape[1] != 29:
        raise ValueError(f"Expected joint_pos shape (frames, 29), got {joint_pos.shape}")
    if body_pos.shape[:2] != (len(joint_pos), 30):
        raise ValueError(f"Expected body_pos_w shape (frames, 30, 3), got {body_pos.shape}")
    if body_quat_wxyz.shape != (len(joint_pos), 30, 4):
        raise ValueError(
            f"Expected body_quat_w shape (frames, 30, 4), got {body_quat_wxyz.shape}"
        )

    root_pos = body_pos[:, 0]
    root_quat = body_quat_wxyz[:, 0]
    # mjlab CSV convention is x, y, z, then quaternion xyzw, then 29 joints.
    root_quat_xyzw = root_quat[:, [1, 2, 3, 0]]
    csv_data = np.concatenate((root_pos, root_quat_xyzw, joint_pos), axis=1)
    if csv_data.shape[1] != 36 or not np.isfinite(csv_data).all():
        raise ValueError("Generated mjlab CSV failed shape/finite validation")

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output_csv, csv_data, delimiter=",", fmt="%.9f")
    print(f"Exported {len(csv_data)} frames × 36 columns: {args.output_csv}")


if __name__ == "__main__":
    main()
