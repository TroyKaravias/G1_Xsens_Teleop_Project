#!/usr/bin/env python3
"""Create a ProtoMotions cache containing a simulated live-data dropout."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np
import torch

from export_protomotions_cache import (
    _angular_velocity_xyzw,
    _differentiate,
    _load_protomotions_model,
    _quat_wxyz_to_xyzw,
)
from xsens_bridge.live_reference import LiveReferenceBuffer, ReferenceFrame


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_cache", type=Path)
    parser.add_argument("mjcf", type=Path)
    parser.add_argument("output_cache", type=Path)
    parser.add_argument("--drop-start", type=float, default=5.0)
    parser.add_argument("--drop-duration", type=float, default=0.70)
    args = parser.parse_args()

    source = torch.load(args.input_cache, map_location="cpu", weights_only=False)
    dt = float(source["control_dt"])
    source_pos = np.asarray(source["dof_pos"], dtype=np.float32)
    source_vel = np.asarray(source["dof_vel"], dtype=np.float32)
    source_body_rot = np.asarray(source["body_rot"], dtype=np.float32)
    source_body_pos = np.asarray(source["body_pos"], dtype=np.float32)

    buffer = LiveReferenceBuffer(control_dt=dt)
    output_pos = np.empty_like(source_pos)
    states: list[str] = []
    for index in range(len(source_pos)):
        now = index * dt
        dropping = args.drop_start <= now < args.drop_start + args.drop_duration
        if not dropping:
            buffer.push(
                ReferenceFrame(
                    now, source_pos[index], source_vel[index], source_body_rot[index]
                )
            )
        states.append(buffer.state(now).value)
        output_pos[index] = buffer.future(now)["dof_pos"][0]

    output_vel = _differentiate(output_pos, dt)
    model = _load_protomotions_model(args.mjcf)
    data = mujoco.MjData(model)
    body_pos = np.empty_like(source_body_pos)
    body_rot = np.empty_like(source_body_rot)
    for index in range(len(output_pos)):
        data.qpos[:3] = source_body_pos[index, 0]
        root_xyzw = source_body_rot[index, 0]
        data.qpos[3:7] = root_xyzw[[3, 0, 1, 2]]
        data.qpos[7:] = output_pos[index]
        mujoco.mj_forward(model, data)
        body_pos[index] = data.xpos[1:]
        body_rot[index] = _quat_wxyz_to_xyzw(data.xquat[1:])

    output = {
        "dof_pos": output_pos,
        "dof_vel": output_vel,
        "body_rot": body_rot,
        "body_pos": body_pos,
        "body_vel": _differentiate(body_pos, dt),
        "body_ang_vel": _angular_velocity_xyzw(body_rot, dt),
        "control_dt": dt,
        "num_frames": len(output_pos),
    }
    args.output_cache.parent.mkdir(parents=True, exist_ok=True)
    torch.save(output, args.output_cache)
    print(f"Saved: {args.output_cache}")
    for state in dict.fromkeys(states):
        print(f"  {state}: {states.count(state)} ticks")


if __name__ == "__main__":
    main()
