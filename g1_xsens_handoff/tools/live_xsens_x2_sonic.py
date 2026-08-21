#!/usr/bin/env python3
"""Continuously drive free-base X2 MuJoCo from streamed Xsens through SONIC."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.live_xsens_x2 import capture_calibration
from tools.live_xsens_sonic import relative_pelvis_yaw, unwrap_yaw
from tools.simulate_xsens_x2_sonic import (
    CONTROL_DT, DECIMATION, SIM_DT, load_model_with_floor,
    policy_control_constants, quat_rotate_inverse,
)
from xsens_bridge.live_retarget import OnlineG1Retargeter
from xsens_bridge.stream import LatestPoseReceiver
from xsens_bridge.x2_retarget import (
    X2_JOINT_NAMES, clamp_x2_trajectory, g1_deltas_to_x2,
    g1_velocities_to_x2, model_xml,
)
from xsens_bridge.x2_sonic import (
    IL_TO_MJ_DOF, MJ_TO_IL_DOF, X2SonicDelayedReferenceBuffer,
    X2SonicOnnxPolicy, X2SonicProprioceptionBuffer, X2SonicReferenceFrame,
)

log = logging.getLogger("live_xsens_x2_sonic")
logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--seed-motion", type=Path, required=True)
    parser.add_argument("--variant", choices=("v1.3",), default="v1.3")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9764)
    parser.add_argument("--calibration-seconds", type=float, default=9.0)
    parser.add_argument("--forward-calibration-seconds", type=float, default=3.0)
    parser.add_argument("--wait-timeout", type=float, default=60.0)
    parser.add_argument("--stale-ms", type=float, default=250.0)
    parser.add_argument("--reference-delay", type=float, default=1.0)
    parser.add_argument("--runtime-seconds", type=float, default=0.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument(
        "--track-xsens-legs", action=argparse.BooleanOptionalAction, default=False,
        help="apply calibrated Xsens leg deltas around SONIC's stable idle legs",
    )
    args = parser.parse_args()
    if args.reference_delay < 1.0:
        parser.error("--reference-delay must be at least 1.0 second")

    import mujoco
    import joblib

    seed_library = joblib.load(args.seed_motion)
    idle = next(iter(seed_library.values()))
    idle_joints = np.asarray(idle["dof"], dtype=np.float32)
    idle_root_position = np.asarray(idle["root_trans_offset"], dtype=np.float64)
    idle_root_quaternion_xyzw = np.asarray(idle["root_rot"], dtype=np.float64)
    idle_fps = float(idle["fps"])

    stale_s = args.stale_ms / 1000.0
    with LatestPoseReceiver(args.bind, args.port, counter_reset_after_seconds=stale_s) as receiver:
        log.info("X2 SONIC SIMULATION ONLY: waiting on %s:%d", args.bind, args.port)
        calibration = capture_calibration(
            receiver, args.calibration_seconds, args.forward_calibration_seconds,
            args.wait_timeout,
        )
        retargeter = OnlineG1Retargeter(
            calibration, motion_gain=.75, torso_gain=.50,
            max_position_speed=4.0, max_joint_speed=10.0,
        )

        path = model_xml(args.models, args.variant)
        model = load_model_with_floor(mujoco, path)
        model.opt.timestep = SIM_DT
        data = mujoco.MjData(model)
        joint_ids = np.asarray([model.joint(name).id for name in X2_JOINT_NAMES])
        qpos_adr = model.jnt_qposadr[joint_ids]
        dof_adr = model.jnt_dofadr[joint_ids]
        actuator_ids = np.asarray([
            model.actuator(f"motor_{name}").id for name in X2_JOINT_NAMES
        ])
        limits = model.jnt_range[joint_ids]
        torque_limits = model.actuator_ctrlrange[actuator_ids]
        policy = X2SonicOnnxPolicy(args.policy)
        history = X2SonicProprioceptionBuffer()
        reference = X2SonicDelayedReferenceBuffer(args.reference_delay)
        kp, kd, action_scale, default = policy_control_constants()

        origin_x2 = None
        stream_motion_started = None
        previous_wrapped_yaw = None
        target_yaw = 0.0
        last_counter = None
        last_packet_at = None
        last_reference_end = None
        initialized = False
        last_action_mj = np.zeros(31, dtype=np.float32)
        processed = 0
        policy_steps = 0
        saturation_events = 0
        fall_reason = None
        last_buffer_log_count = -1

        viewer = None
        started = time.monotonic()
        next_policy = started
        try:
            while args.runtime_seconds <= 0 or time.monotonic() - started < args.runtime_seconds:
                now = time.monotonic()
                latest = receiver.latest()
                fresh = latest is not None and now - latest.received_at <= stale_s
                if fresh and latest.frame.header.sample_counter != last_counter:
                    pose = retargeter.retarget(latest.frame, latest.received_at)
                    direct_x2 = g1_deltas_to_x2(pose.dof_pos[None])[0]
                    if origin_x2 is None:
                        origin_x2 = direct_x2.copy()
                        stream_motion_started = latest.received_at
                    idle_index = int(
                        (latest.received_at - stream_motion_started) * idle_fps
                    ) % len(idle_joints)
                    # Continuous stationary reference: validated idle owns the
                    # root/legs; every received Xsens frame owns waist/arms/head.
                    position = idle_joints[idle_index].copy()
                    position[12:] = direct_x2[12:]
                    if args.track_xsens_legs:
                        position[:12] += direct_x2[:12] - origin_x2[:12]
                    position, _ = clamp_x2_trajectory(position[None], limits)
                    velocity = g1_velocities_to_x2(pose.dof_vel)
                    if not args.track_xsens_legs:
                        velocity[:12] = 0.0
                    pelvis_segment = next(
                        segment for segment in latest.frame.segments
                        if segment.name == "pelvis"
                    )
                    wrapped_yaw = relative_pelvis_yaw(
                        calibration.quaternions["pelvis"],
                        np.asarray(pelvis_segment.quaternion_wxyz),
                    )
                    previous_wrapped_yaw, target_yaw = unwrap_yaw(
                        wrapped_yaw, previous_wrapped_yaw, target_yaw,
                    )
                    reference.push(X2SonicReferenceFrame(
                        latest.received_at, position[0], velocity, target_yaw,
                    ))
                    last_packet_at = latest.received_at
                    last_reference_end = latest.received_at
                    last_counter = latest.frame.header.sample_counter
                    processed += 1

                if not initialized:
                    if last_reference_end is None or not reference.ready(last_reference_end):
                        if processed and processed % 25 == 0 and processed != last_buffer_log_count:
                            log.info("Buffering deterministic SONIC lookahead: %d frames", processed)
                            last_buffer_log_count = processed
                        time.sleep(.002)
                        continue
                    initial_pos, initial_vel = reference.delayed_current(last_reference_end)
                    mujoco.mj_resetData(model, data)
                    data.qpos[:3] = idle_root_position[0]
                    q = idle_root_quaternion_xyzw[0]
                    data.qpos[3:7] = (q[3], q[0], q[1], q[2])
                    data.qpos[qpos_adr] = initial_pos
                    data.qvel[dof_adr] = initial_vel
                    mujoco.mj_forward(model, data)
                    initialized = True
                    started = now
                    next_policy = now
                    log.info("SONIC control active after %.2f s reference delay", args.reference_delay)
                    if args.viewer:
                        import mujoco.viewer
                        viewer = mujoco.viewer.launch_passive(model, data)
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.lookat[:] = (0.0, 0.0, 0.65)
                        viewer.cam.distance = 5.0
                        viewer.cam.azimuth = 135.0
                        viewer.cam.elevation = -28.0

                if now < next_policy:
                    time.sleep(min(.002, next_policy - now))
                    continue
                next_policy += CONTROL_DT
                if last_packet_at is None or now - last_packet_at > stale_s:
                    log.warning("Xsens stream stale; ending free-base policy test")
                    fall_reason = "stale Xsens input"
                    break

                qpos_mj = data.qpos[qpos_adr].copy()
                qvel_mj = data.qvel[dof_adr].copy()
                base_quat = data.qpos[3:7].copy()
                current_yaw = float(np.arctan2(
                    2.0 * (base_quat[0] * base_quat[3] + base_quat[1] * base_quat[2]),
                    1.0 - 2.0 * (base_quat[2] ** 2 + base_quat[3] ** 2),
                ))
                gravity = quat_rotate_inverse(base_quat, np.asarray([0., 0., -1.]))
                history.append(
                    data.qvel[3:6],
                    qpos_mj[IL_TO_MJ_DOF] - default[IL_TO_MJ_DOF],
                    qvel_mj[IL_TO_MJ_DOF],
                    last_action_mj[IL_TO_MJ_DOF],
                    gravity,
                )
                tokenizer = reference.tokenizer(last_reference_end, current_yaw)
                try:
                    action_il = policy.infer(tokenizer, history.flattened())
                except RuntimeError as exc:
                    fall_reason = str(exc)
                    break
                action_il = np.clip(action_il, -20.0, 20.0)
                action_mj = action_il[MJ_TO_IL_DOF]
                # Published frozen-G1-core v2 runtime requirement.
                action_mj[[19, 20, 21, 26, 27, 28]] = 0.0
                target = default + action_mj * action_scale
                for _ in range(DECIMATION):
                    raw = kp * (target - data.qpos[qpos_adr]) - kd * data.qvel[dof_adr]
                    torque = np.clip(raw, torque_limits[:, 0], torque_limits[:, 1])
                    saturation_events += int(np.count_nonzero(raw != torque))
                    data.ctrl[:] = 0
                    data.ctrl[actuator_ids] = torque
                    mujoco.mj_step(model, data)
                last_action_mj = action_mj
                policy_steps += 1

                if data.qpos[2] < .4:
                    fall_reason = f"pelvis height {data.qpos[2]:.3f} m"
                    break
                if gravity[2] > -.3:
                    fall_reason = f"body tilt gravity_z={gravity[2]:.3f}"
                    break
                if not np.isfinite(data.qpos).all():
                    fall_reason = "non-finite MuJoCo state"
                    break
                if viewer is not None:
                    if not viewer.is_running():
                        break
                    viewer.sync()
        except KeyboardInterrupt:
            log.info("Stopped by operator")
        finally:
            if viewer is not None:
                viewer.close()

        health = receiver.health
        print("Mode: LIVE XSENS -> X2 SONIC -> FREE-BASE MUJOCO")
        print(f"Policy: {args.policy}")
        print(f"Processed Xsens frames: {processed}")
        print(f"Policy steps: {policy_steps}")
        print(f"Missing Xsens frames: {health.missing_frames}")
        print(f"Malformed packets: {health.malformed_packets}")
        print(f"Torque saturation events: {saturation_events}")
        print(f"Termination: {fall_reason or 'operator/viewer/runtime limit'}")
        print("SIMULATION ONLY: no X2 hardware connection or hardware validation.")


if __name__ == "__main__":
    main()
