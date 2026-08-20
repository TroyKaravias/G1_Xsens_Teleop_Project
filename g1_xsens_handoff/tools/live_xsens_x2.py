#!/usr/bin/env python3
"""Drive fixed-base X2 MuJoCo dynamics from live or replayed Xsens UDP."""

from __future__ import annotations

import argparse
from dataclasses import replace
import logging
import math
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xsens_bridge.live_retarget import OnlineG1Retargeter, calibrate_nt_sequence
from xsens_bridge.stream import LatestPoseReceiver
from xsens_bridge.x2_dynamics import (
    X2GlobalRootTracker,
    bounded_pd_torque,
    default_x2_pd_gains,
)
from xsens_bridge.x2_retarget import (
    X2_JOINT_NAMES,
    X2_T_POSE,
    clamp_x2_trajectory,
    g1_deltas_to_x2,
    g1_velocities_to_x2,
    model_xml,
    validate_model_joint_names,
)
from tools.live_xsens_sonic import (
    relative_pelvis_yaw,
    unwrap_yaw,
    yaw_quaternion,
)


log = logging.getLogger("live_xsens_x2")
logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")


def capture_calibration(
    receiver: LatestPoseReceiver,
    nt_seconds: float,
    forward_seconds: float,
    wait_timeout: float,
):
    log.info("Waiting for Xsens MXTP02 on UDP...")
    deadline = time.monotonic() + wait_timeout
    while receiver.latest() is None:
        if time.monotonic() >= deadline:
            raise TimeoutError("No Xsens frame arrived before the wait timeout")
        time.sleep(0.02)
    log.info(
        "N/T calibration for %.1f s: hold N-pose for the first 40%%, then "
        "move to T-pose and hold. A %.1f s arms-forward hold follows.",
        nt_seconds,
        forward_seconds,
    )
    started = time.monotonic()
    announced_t = False
    nt_frames = []
    last_counter = None
    while time.monotonic() - started < nt_seconds:
        elapsed = time.monotonic() - started
        if not announced_t and elapsed >= nt_seconds * 0.40:
            log.info("Move to T-pose now and hold.")
            announced_t = True
        latest = receiver.latest()
        if (
            latest is not None
            and latest.frame.header.sample_counter != last_counter
        ):
            nt_frames.append(latest.frame)
            last_counter = latest.frame.header.sample_counter
        time.sleep(0.002)
    if len(nt_frames) < 50:
        raise RuntimeError(
            f"N/T calibration captured only {len(nt_frames)} frames"
        )
    result = calibrate_nt_sequence(nt_frames)

    log.info(
        "Move both arms straight forward at shoulder height, shoulder-width "
        "apart, and hold."
    )
    forward_started = time.monotonic()
    forward_frames = []
    while time.monotonic() - forward_started < forward_seconds:
        latest = receiver.latest()
        if (
            latest is not None
            and latest.frame.header.sample_counter != last_counter
        ):
            forward_frames.append(latest.frame)
            last_counter = latest.frame.header.sample_counter
        time.sleep(0.002)
    if len(forward_frames) < 10:
        raise RuntimeError(
            f"Arms-forward calibration captured only {len(forward_frames)} frames"
        )
    forward_index = min(len(forward_frames) - 1, int(len(forward_frames) * 0.88))
    forward_frame = forward_frames[forward_index]
    forward_positions = {
        segment.name: np.asarray(segment.position_m, dtype=np.float64)
        for segment in forward_frame.segments
    }
    result = replace(
        result,
        forward_positions=forward_positions,
        forward_pose_index=len(nt_frames) + forward_index,
        captured_frames=len(nt_frames) + len(forward_frames),
    )
    log.info(
        "Calibration complete: N=%d T=%d FORWARD=%d score=%.3f frames=%d",
        result.n_pose_index,
        result.t_pose_index,
        result.forward_pose_index,
        result.t_pose_score,
        result.captured_frames,
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--variant", choices=("v1.3", "v1.4"), default="v1.3")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9764)
    parser.add_argument("--calibration-seconds", type=float, default=9.0)
    parser.add_argument(
        "--forward-calibration-seconds",
        type=float,
        default=3.0,
        help="arms-forward hold appended after the unchanged N/T phase",
    )
    parser.add_argument("--wait-timeout", type=float, default=60.0)
    parser.add_argument("--stale-ms", type=float, default=250.0)
    parser.add_argument("--motion-gain", type=float, default=0.75)
    parser.add_argument("--torso-gain", type=float, default=0.50)
    parser.add_argument(
        "--track-global-yaw",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="rotate the pinned X2 base with calibrated pelvis heading",
    )
    parser.add_argument("--global-yaw-gain", type=float, default=1.0)
    parser.add_argument(
        "--global-yaw-max-rate-deg-s",
        type=float,
        default=180.0,
        help="maximum simulated root turn rate",
    )
    parser.add_argument(
        "--track-global-position",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="move the pinned X2 root across world X/Y from pelvis displacement",
    )
    parser.add_argument("--global-position-gain", type=float, default=1.0)
    parser.add_argument("--global-position-max-speed-mps", type=float, default=0.75)
    parser.add_argument("--global-position-max-jump-m", type=float, default=0.25)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument(
        "--runtime-seconds",
        type=float,
        default=0.0,
        help="zero runs until the viewer closes or Ctrl+C",
    )
    args = parser.parse_args()
    if args.variant == "v1.4":
        log.warning("v1.4 has known persistent vendor-model self-contacts")
    if (
        args.calibration_seconds <= 0.0
        or args.forward_calibration_seconds <= 0.0
        or args.wait_timeout <= 0.0
    ):
        parser.error("calibration and wait times must be positive")
    if args.stale_ms <= 0.0:
        parser.error("--stale-ms must be positive")
    if not 0.0 <= args.motion_gain <= 1.0:
        parser.error("--motion-gain must be between 0 and 1")
    if not 0.0 <= args.torso_gain <= 1.0:
        parser.error("--torso-gain must be between 0 and 1")
    if args.global_yaw_gain <= 0.0 or args.global_yaw_max_rate_deg_s <= 0.0:
        parser.error("global yaw gain and maximum rate must be positive")
    if min(
        args.global_position_gain,
        args.global_position_max_speed_mps,
        args.global_position_max_jump_m,
    ) <= 0.0:
        parser.error("global position gain and limits must be positive")

    import mujoco

    stale_seconds = args.stale_ms / 1000.0
    with LatestPoseReceiver(
        args.bind, args.port, counter_reset_after_seconds=stale_seconds
    ) as receiver:
        log.info("SIMULATION ONLY: listening on %s:%d", args.bind, args.port)
        calibration = capture_calibration(
            receiver,
            args.calibration_seconds,
            args.forward_calibration_seconds,
            args.wait_timeout,
        )
        retargeter = OnlineG1Retargeter(
            calibration,
            motion_gain=args.motion_gain,
            torso_gain=args.torso_gain,
            max_position_speed=4.0,
            max_joint_speed=10.0,
        )

        xml = model_xml(args.models, args.variant)
        # Add bright scene context without editing the pinned vendor model or
        # adding physical support. The plane is visual-only and cannot contact.
        spec = mujoco.MjSpec.from_file(str(xml))
        spec.add_texture(
            name="visual_floor_checker",
            type=mujoco.mjtTexture.mjTEXTURE_2D,
            builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
            rgb1=(0.72, 0.76, 0.80),
            rgb2=(0.35, 0.39, 0.44),
            width=512,
            height=512,
        )
        spec.add_material(
            name="visual_floor_material",
            textures=("visual_floor_checker",),
            texrepeat=(12.0, 12.0),
            reflectance=0.10,
            roughness=0.80,
        )
        spec.add_texture(
            name="visual_sky",
            type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
            builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
            rgb1=(0.82, 0.87, 0.93),
            rgb2=(0.42, 0.50, 0.60),
            width=512,
            height=512,
        )
        spec.worldbody.add_geom(
            name="visual_floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0.0, 0.0, -0.07),
            size=(5.0, 5.0, 0.1),
            contype=0,
            conaffinity=0,
            material="visual_floor_material",
        )
        spec.visual.headlight.ambient = (0.45, 0.45, 0.45)
        spec.visual.headlight.diffuse = (0.75, 0.75, 0.75)
        spec.visual.headlight.specular = (0.10, 0.10, 0.10)
        model = spec.compile()
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
        actuator_ids = np.asarray([
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{name}"
            )
            for name in X2_JOINT_NAMES
        ])
        if np.any(actuator_ids < 0):
            raise RuntimeError("Official X2 model is missing a named joint motor")
        qpos_addresses = model.jnt_qposadr[joint_ids]
        dof_addresses = model.jnt_dofadr[joint_ids]
        joint_ranges = model.jnt_range[joint_ids]
        torque_limits = model.actuator_ctrlrange[actuator_ids]
        kp, kd = default_x2_pd_gains()
        target, _ = clamp_x2_trajectory(X2_T_POSE[None], joint_ranges)
        target = target[0]
        target_velocity = np.zeros(len(X2_JOINT_NAMES), dtype=np.float64)

        mujoco.mj_resetData(model, data)
        data.qpos[qpos_addresses] = target
        base_qpos = data.qpos[:7].copy()
        root_tracker = X2GlobalRootTracker(
            calibration.quaternions["pelvis"],
            base_qpos[:2],
            gain=args.global_position_gain,
            max_speed_mps=args.global_position_max_speed_mps,
            max_jump_m=args.global_position_max_jump_m,
        )
        root_quaternion = base_qpos[3:7].copy()
        global_heading = 0.0
        previous_wrapped_yaw = None
        unwrapped_pelvis_yaw = 0.0
        previous_heading_time = None
        mujoco.mj_forward(model, data)

        viewer = None
        if args.viewer:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(model, data)
        started = time.monotonic()
        next_render = started
        render_period = 1.0 / 60.0
        last_counter = None
        state = None
        processed = 0
        stale_transitions = 0
        saturation_events = 0
        try:
            while (
                args.runtime_seconds <= 0.0
                or time.monotonic() - started < args.runtime_seconds
            ):
                now = time.monotonic()
                latest = receiver.latest()
                fresh = latest is not None and now - latest.received_at <= stale_seconds
                new_state = "LIVE" if fresh else "STALE-HOLD"
                if new_state != state:
                    log.info("Stream state: %s", new_state)
                    if state == "LIVE" and not fresh:
                        stale_transitions += 1
                    state = new_state
                if (
                    fresh
                    and latest is not None
                    and latest.frame.header.sample_counter != last_counter
                ):
                    pose = retargeter.retarget(latest.frame, latest.received_at)
                    target = g1_deltas_to_x2(pose.dof_pos[None])[0]
                    target, _ = clamp_x2_trajectory(target[None], joint_ranges)
                    target = target[0]
                    target_velocity = g1_velocities_to_x2(pose.dof_vel)
                    if args.track_global_position:
                        pelvis_position = np.asarray(
                            next(
                                segment.position_m
                                for segment in latest.frame.segments
                                if segment.name == "pelvis"
                            ),
                            dtype=np.float64,
                        )
                        base_qpos[:2] = root_tracker.update(
                            pelvis_position, latest.received_at
                        )
                    if args.track_global_yaw:
                        pelvis = next(
                            segment
                            for segment in latest.frame.segments
                            if segment.name == "pelvis"
                        )
                        wrapped_yaw = relative_pelvis_yaw(
                            calibration.quaternions["pelvis"],
                            np.asarray(pelvis.quaternion_wxyz),
                        )
                        previous_wrapped_yaw, unwrapped_pelvis_yaw = unwrap_yaw(
                            wrapped_yaw,
                            previous_wrapped_yaw,
                            unwrapped_pelvis_yaw,
                        )
                        heading_dt = (
                            0.0
                            if previous_heading_time is None
                            else min(
                                latest.received_at - previous_heading_time,
                                0.04,
                            )
                        )
                        desired_heading = (
                            args.global_yaw_gain * unwrapped_pelvis_yaw
                        )
                        max_heading_step = math.radians(
                            args.global_yaw_max_rate_deg_s
                        ) * max(heading_dt, 0.0)
                        global_heading += float(np.clip(
                            desired_heading - global_heading,
                            -max_heading_step,
                            max_heading_step,
                        ))
                        root_quaternion = yaw_quaternion(
                            global_heading, root_quaternion
                        ).astype(np.float64)
                        base_qpos[3:7] = root_quaternion
                        previous_heading_time = latest.received_at
                    last_counter = latest.frame.header.sample_counter
                    processed += 1
                elif not fresh:
                    target_velocity.fill(0.0)

                # Keep simulation time synchronized to the live wall clock.
                # Rendering every 1 ms made the physics clock lag badly on
                # ordinary displays, so catch up physics in 1 ms substeps and
                # synchronize the viewer independently at about 60 Hz.
                target_sim_time = now - started
                while data.time + 0.5 * model.opt.timestep < target_sim_time:
                    torque, saturated = bounded_pd_torque(
                        data.qpos[qpos_addresses],
                        data.qvel[dof_addresses],
                        target,
                        target_velocity,
                        kp,
                        kd,
                        torque_limits,
                    )
                    saturation_events += int(np.count_nonzero(saturated))
                    data.ctrl[:] = 0.0
                    data.ctrl[actuator_ids] = torque
                    mujoco.mj_step(model, data)
                    data.qpos[:7] = base_qpos
                    data.qvel[:6] = 0.0
                if not np.all(np.isfinite(data.qpos)):
                    raise RuntimeError("Non-finite MuJoCo state")
                if viewer is not None and now >= next_render:
                    if not viewer.is_running():
                        break
                    # qpos was restored after the last dynamics step; refresh
                    # derived body transforms before drawing the pinned pose.
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    next_render = max(next_render + render_period, now)
                time.sleep(0.001)
        except KeyboardInterrupt:
            log.info("Stopped by operator")
        finally:
            if viewer is not None:
                viewer.close()

        health = receiver.health
        print(f"Model: {xml}")
        print("Mode: LIVE FIXED-BASE X2 DYNAMICS")
        print(f"Processed live reference frames: {processed}")
        print(f"Received frames: {health.received_frames}")
        print(f"Missing frames: {health.missing_frames}")
        print(f"Malformed packets: {health.malformed_packets}")
        print(f"Stale transitions: {stale_transitions}")
        print(f"Torque saturation events: {saturation_events}")
        print(f"Final global root yaw: {math.degrees(global_heading):.1f} deg")
        print(
            f"Final global root X/Y: {base_qpos[0]:.3f}, {base_qpos[1]:.3f} m"
        )
        print(f"Rejected global position jumps: {root_tracker.rejected_jumps}")
        wall_elapsed = max(time.monotonic() - started, 1e-9)
        print(f"Simulation real-time factor: {data.time / wall_elapsed:.3f}x")
        print("SIMULATION ONLY: no balance, locomotion, or hardware output.")


if __name__ == "__main__":
    main()
