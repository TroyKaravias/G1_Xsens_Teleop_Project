#!/usr/bin/env python3
"""Run live Xsens → G1 retargeting → watchdog → ONNX → MuJoCo.

This executable is simulation-only.  It contains no Unitree, ROS, DDS, or
motor-command adapter and never opens a connection to the physical robot.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

from xsens_bridge.g1_retarget import G1_JOINT_NAMES
from xsens_bridge.live_reference import (
    LiveReferenceBuffer,
    ReferenceFrame,
    WatchdogState,
)
from xsens_bridge.live_retarget import (
    OnlineG1Retargeter,
    calibrate_nt_sequence,
)
from xsens_bridge.stream import LatestPoseReceiver


log = logging.getLogger("live_xsens_mujoco")
logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protomotions-root", type=Path, required=True)
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9763)
    parser.add_argument("--calibration-seconds", type=float, default=9.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument(
        "--runtime-seconds",
        type=float,
        default=30.0,
        help="Simulation time after calibration; 0 runs until Ctrl+C.",
    )
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--neutral-shoulder-bias-deg", type=float, default=6.0)
    parser.add_argument("--reach-shoulder-bias-deg", type=float, default=11.0)
    parser.add_argument(
        "--leg-motion-gain",
        type=float,
        default=1.0,
        help="Scale live hip and knee motion; 1.0 preserves measured amplitude.",
    )
    return parser.parse_args()


def _load_deployment_helpers(protomotions_root: Path):
    root = str(protomotions_root.resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    from deployment.state_utils import (  # type: ignore
        apply_heading_offset_np,
        compute_yaw_offset_np,
    )
    from deployment.test_tracker_mujoco import (  # type: ignore
        build_onnx_inputs,
        load_mujoco_model,
        read_robot_state,
    )

    return (
        apply_heading_offset_np,
        compute_yaw_offset_np,
        build_onnx_inputs,
        load_mujoco_model,
        read_robot_state,
    )


def _model_joint_ranges(model: mujoco.MjModel) -> np.ndarray:
    joint_ids = np.array(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in G1_JOINT_NAMES
        ]
    )
    if np.any(joint_ids < 0):
        raise ValueError("ProtoMotions MJCF does not contain all 29 G1 joints")
    return model.jnt_range[joint_ids].copy()


class ReferenceKinematics:
    """Compute policy body rotations from a retargeted joint reference."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.data = mujoco.MjData(model)

    def body_rot_xyzw(self, dof_pos: np.ndarray) -> np.ndarray:
        self.data.qpos[:3] = (0.0, 0.0, 0.793)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[7:] = dof_pos
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return self.data.xquat[1:][:, [1, 2, 3, 0]].astype(np.float32)


def _capture_calibration(
    receiver: LatestPoseReceiver,
    seconds: float,
    wait_timeout: float,
):
    log.info("Waiting for live MXTP02 frames...")
    wait_started = time.monotonic()
    while receiver.latest() is None:
        if time.monotonic() - wait_started > wait_timeout:
            raise TimeoutError("No Xsens frame arrived before the wait timeout")
        time.sleep(0.02)

    log.info(
        "Calibration started (%.1fs): hold N-pose, move to T-pose, then hold T.",
        seconds,
    )
    started = time.monotonic()
    frames = []
    last_counter: int | None = None
    announced_t = False
    while time.monotonic() - started < seconds:
        elapsed = time.monotonic() - started
        if not announced_t and elapsed >= seconds * 0.40:
            log.info("Move smoothly into T-pose and hold it.")
            announced_t = True
        latest = receiver.latest()
        if latest is not None:
            counter = latest.frame.header.sample_counter
            if counter != last_counter:
                frames.append(latest.frame)
                last_counter = counter
        time.sleep(0.002)
    if len(frames) < 50:
        raise RuntimeError(f"Calibration captured only {len(frames)} unique frames")
    calibration = calibrate_nt_sequence(frames)
    log.info(
        "Calibration complete: selected T frame %d/%d, score=%.3f",
        calibration.t_pose_index,
        calibration.captured_frames,
        calibration.t_pose_score,
    )
    return calibration


def main() -> None:
    args = _parse_args()
    (
        apply_heading_offset_np,
        compute_yaw_offset_np,
        build_onnx_inputs,
        load_mujoco_model,
        read_robot_state,
    ) = _load_deployment_helpers(args.protomotions_root)

    onnx_path = args.onnx.resolve()
    with onnx_path.with_suffix(".yaml").open() as stream:
        meta = yaml.safe_load(stream)
    robot_meta = meta["robot"]
    timing = meta["timing"]
    motion_meta = meta["motion"]
    control = meta["control"]
    runtime = meta["_runtime"]
    num_dofs = int(robot_meta["num_dofs"])
    num_bodies = int(robot_meta["num_bodies"])
    if num_dofs != 29 or num_bodies != 33:
        raise ValueError(
            f"This integration expects G1 29 DOFs/33 bodies, got "
            f"{num_dofs}/{num_bodies}"
        )

    control_dt = float(timing["control_dt"])
    physics_dt = float(timing["physics_dt"])
    decimation = int(timing["decimation"])
    future_steps = tuple(int(value) for value in motion_meta["future_step_indices"])
    anchor_body_index = int(robot_meta["anchor_body_index"])
    root_body_index = int(robot_meta["root_body_index"])
    onnx_name_to_key = runtime["onnx_name_to_in_key"]
    pd_target_max_accel = control.get("pd_target_max_accel")
    ema_alpha = float(control.get("action_ema_alpha", 1.0))

    model, data = load_mujoco_model(
        robot_meta["mjcf_path"],
        control["stiffness"],
        control["damping"],
        physics_dt,
    )
    reference_kinematics = ReferenceKinematics(model)
    session = ort.InferenceSession(
        str(onnx_path), providers=["CPUExecutionProvider"]
    )
    output_names = [output.name for output in session.get_outputs()]
    pd_output_index = output_names.index("joint_pos_targets")

    viewer = None
    if args.render:
        from mujoco import viewer as mj_viewer

        viewer = mj_viewer.launch_passive(
            model, data, show_left_ui=False, show_right_ui=False
        )
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -10.0
        viewer.cam.azimuth = 180.0
        viewer.cam.trackbodyid = 1
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING

    with LatestPoseReceiver(args.bind, args.port) as receiver:
        log.info("Listening on UDP %s:%d", args.bind, args.port)
        calibration = _capture_calibration(
            receiver, args.calibration_seconds, args.wait_timeout
        )
        retargeter = OnlineG1Retargeter(
            calibration,
            motion_gain=args.leg_motion_gain,
            neutral_shoulder_bias_deg=args.neutral_shoulder_bias_deg,
            reach_shoulder_bias_deg=args.reach_shoulder_bias_deg,
            joint_ranges=_model_joint_ranges(model),
        )
        buffer = LiveReferenceBuffer(
            future_steps=future_steps,
            control_dt=control_dt,
        )

        latest = receiver.latest()
        if latest is None:
            raise RuntimeError("Xsens stream disappeared after calibration")
        first_pose = retargeter.retarget(latest.frame, latest.received_at)
        first_body_rot = reference_kinematics.body_rot_xyzw(first_pose.dof_pos)
        buffer.push(
            ReferenceFrame(
                first_pose.timestamp,
                first_pose.dof_pos,
                first_pose.dof_vel,
                first_body_rot,
            )
        )

        data.qpos[:3] = (0.0, 0.0, 0.793)
        data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        data.qpos[7:] = first_pose.dof_pos
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        last_counter = latest.frame.header.sample_counter
        previous_actions = None
        previous_pd = None
        previous_previous_pd = None
        ema_previous = None
        heading_offset = None
        last_state = None
        ticks = 0
        inference_ms = []
        started = time.monotonic()
        next_tick = started

        log.info("LIVE SIMULATION STARTED — physical robot output is absent.")
        try:
            while args.runtime_seconds <= 0 or time.monotonic() - started < args.runtime_seconds:
                tick_started = time.monotonic()
                latest = receiver.latest()
                if (
                    latest is not None
                    and latest.frame.header.sample_counter != last_counter
                ):
                    pose = retargeter.retarget(
                        latest.frame, latest.received_at
                    )
                    body_rot = reference_kinematics.body_rot_xyzw(pose.dof_pos)
                    buffer.push(
                        ReferenceFrame(
                            pose.timestamp,
                            pose.dof_pos,
                            pose.dof_vel,
                            body_rot,
                        )
                    )
                    last_counter = latest.frame.header.sample_counter

                now = time.monotonic()
                state = buffer.state(now)
                if state != last_state:
                    log.info("Watchdog: %s", state.value)
                    last_state = state
                if state is WatchdogState.ESTOP:
                    raise RuntimeError("Hard ESTOP latched")

                future = buffer.future(now)
                robot_state = read_robot_state(
                    data, anchor_body_index, root_body_index
                )
                if heading_offset is None:
                    heading_offset = compute_yaw_offset_np(
                        robot_state["body_rot"][anchor_body_index],
                        future["body_rot"][0, anchor_body_index],
                    )
                future["body_rot"] = apply_heading_offset_np(
                    heading_offset, future["body_rot"]
                )
                inputs = build_onnx_inputs(
                    robot_state,
                    future,
                    onnx_name_to_key,
                    anchor_body_index,
                    num_dofs,
                    previous_actions,
                )
                inference_started = time.perf_counter()
                outputs = session.run(output_names, inputs)
                inference_ms.append(
                    (time.perf_counter() - inference_started) * 1000.0
                )
                pd_targets = np.asarray(
                    outputs[pd_output_index], dtype=np.float32
                ).squeeze()

                if (
                    pd_target_max_accel is not None
                    and previous_pd is not None
                    and previous_previous_pd is not None
                ):
                    delta = pd_targets - previous_pd
                    previous_delta = previous_pd - previous_previous_pd
                    acceleration = delta - previous_delta
                    pd_targets = (
                        previous_pd
                        + previous_delta
                        + np.clip(
                            acceleration,
                            -pd_target_max_accel,
                            pd_target_max_accel,
                        )
                    )
                previous_previous_pd = previous_pd
                previous_pd = pd_targets.copy()

                if ema_alpha < 1.0:
                    if ema_previous is None:
                        ema_previous = pd_targets.copy()
                    pd_targets = (
                        ema_alpha * pd_targets
                        + (1.0 - ema_alpha) * ema_previous
                    )
                    ema_previous = pd_targets.copy()
                previous_actions = pd_targets.copy()
                data.ctrl[:] = pd_targets
                for _ in range(decimation):
                    mujoco.mj_step(model, data)

                if not 0.45 <= float(data.qpos[2]) <= 1.10:
                    raise RuntimeError(
                        f"Simulation root-height limit exceeded: {data.qpos[2]:.3f}m"
                    )
                if viewer is not None:
                    if not viewer.is_running():
                        break
                    viewer.sync()

                ticks += 1
                if ticks % 50 == 0:
                    age_ms = 1000.0 * max(
                        0.0, now - (receiver.health.last_packet_monotonic or now)
                    )
                    log.info(
                        "ticks=%d state=%s root_h=%.3f packet_age=%.1fms "
                        "frames=%d missing=%d malformed=%d onnx=%.3fms",
                        ticks,
                        state.value,
                        data.qpos[2],
                        age_ms,
                        receiver.health.received_frames,
                        receiver.health.missing_frames,
                        receiver.health.malformed_packets,
                        float(np.mean(inference_ms[-50:])),
                    )

                next_tick += control_dt
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                elif time.monotonic() - next_tick > control_dt:
                    next_tick = time.monotonic()
        finally:
            if viewer is not None:
                viewer.close()

    log.info(
        "Done: ticks=%d mean ONNX=%.3fms root_h=%.3f",
        ticks,
        float(np.mean(inference_ms)) if inference_ms else float("nan"),
        data.qpos[2],
    )


if __name__ == "__main__":
    main()
