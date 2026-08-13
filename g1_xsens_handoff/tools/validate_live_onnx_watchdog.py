#!/usr/bin/env python3
"""Exercise ProtoMotions ONNX inference with a simulated live-data dropout."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

from xsens_bridge.live_reference import (
    LiveReferenceBuffer,
    ReferenceFrame,
    WatchdogState,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion_cache", type=Path)
    parser.add_argument("onnx", type=Path)
    parser.add_argument("--drop-start", type=float, default=5.0)
    parser.add_argument("--drop-duration", type=float, default=0.70)
    args = parser.parse_args()

    motion = torch.load(args.motion_cache, map_location="cpu", weights_only=False)
    dt = float(motion["control_dt"])
    dof_pos = np.asarray(motion["dof_pos"], dtype=np.float32)
    dof_vel = np.asarray(motion["dof_vel"], dtype=np.float32)
    body_rot = np.asarray(motion["body_rot"], dtype=np.float32)
    if dof_pos.shape[1] != 29 or body_rot.shape[1] != 33:
        raise ValueError(
            f"Expected 29 DOFs and 33 bodies, got {dof_pos.shape}, {body_rot.shape}"
        )

    session = ort.InferenceSession(
        str(args.onnx), providers=["CPUExecutionProvider"]
    )
    buffer = LiveReferenceBuffer(control_dt=dt)
    previous_actions = np.zeros((1, 1, 29), dtype=np.float32)
    state_counts = {state: 0 for state in WatchdogState}
    transitions: list[tuple[float, WatchdogState]] = []
    last_state: WatchdogState | None = None
    inference_times: list[float] = []

    import time

    for index in range(len(dof_pos)):
        now = index * dt
        dropping = args.drop_start <= now < args.drop_start + args.drop_duration
        if not dropping:
            buffer.push(
                ReferenceFrame(now, dof_pos[index], dof_vel[index], body_rot[index])
            )

        state = buffer.state(now)
        state_counts[state] += 1
        if state != last_state:
            transitions.append((now, state))
            last_state = state

        if state is WatchdogState.ESTOP:
            continue
        future = buffer.future(now)
        inputs = {
            "current_anchor_rot": body_rot[index, 16][None],
            "current_dof_pos": dof_pos[index][None],
            "current_dof_vel": dof_vel[index][None],
            "current_root_local_ang_vel": np.zeros((1, 3), dtype=np.float32),
            "historical_processed_actions": previous_actions,
            "mimic_future_anchor_rot": future["body_rot"][:, 16][None],
            "mimic_future_dof_pos": future["dof_pos"][None],
            "mimic_future_dof_vel": future["dof_vel"][None],
        }
        started = time.perf_counter()
        outputs = session.run(None, inputs)
        inference_times.append((time.perf_counter() - started) * 1000.0)
        previous_actions = np.asarray(outputs[0], dtype=np.float32)[:, None, :]

    required = {
        WatchdogState.LIVE,
        WatchdogState.HOLD,
        WatchdogState.SAFE_RETURN,
        WatchdogState.RECOVERING,
    }
    observed = {state for _, state in transitions}
    if not required.issubset(observed):
        raise RuntimeError(f"Missing watchdog transitions: {required - observed}")

    print("Watchdog transitions:")
    for timestamp, state in transitions:
        print(f"  t={timestamp:6.2f}s  {state.value}")
    print("State ticks:")
    for state, count in state_counts.items():
        print(f"  {state.value:6s}: {count}")
    print(f"ONNX calls: {len(inference_times)}")
    print(f"Mean ONNX latency: {np.mean(inference_times):.3f} ms")
    buffer.trigger_estop()
    if buffer.state(len(dof_pos) * dt) is not WatchdogState.ESTOP:
        raise RuntimeError("Hard ESTOP did not latch")
    print("PASS: HOLD, SAFE_RETURN, RECOVERING, LIVE, and hard ESTOP verified")


if __name__ == "__main__":
    main()
