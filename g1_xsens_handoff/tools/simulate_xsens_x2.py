#!/usr/bin/env python3
"""Track recorded Xsens motion with a fixed-base X2 MuJoCo PD controller."""

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
from xsens_bridge.x2_dynamics import (
    bounded_pd_torque,
    default_x2_pd_gains,
    interpolate_x2_reference,
)
from xsens_bridge.x2_retarget import (
    X2_JOINT_NAMES,
    clamp_x2_trajectory,
    lowpass_x2_trajectory,
    model_xml,
    retarget_xsens_to_x2,
    validate_model_joint_names,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--variant", choices=("v1.3", "v1.4"), default="v1.3")
    parser.add_argument("--gain", type=float, default=0.75)
    parser.add_argument("--torso-gain", type=float, default=0.50)
    parser.add_argument("--filter-cutoff-hz", type=float, default=4.0)
    parser.add_argument("--calibration-frame", type=int)
    parser.add_argument("--duration", type=float, help="simulate only this many seconds")
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="recording playback multiplier; 2 is twice real time (default: 1)",
    )
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()
    if not 0.0 <= args.gain <= 1.0:
        parser.error("--gain must be between 0 and 1")
    if not 0.0 <= args.torso_gain <= 1.0:
        parser.error("--torso-gain must be between 0 and 1")
    if args.filter_cutoff_hz < 0.0:
        parser.error("--filter-cutoff-hz must be non-negative")
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.playback_speed <= 0.0:
        parser.error("--playback-speed must be positive")

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
    frame_dt = float(np.median(np.diff(times)))
    trajectory = retarget_xsens_to_x2(
        quaternions,
        motion_gain=args.gain,
        torso_gain=args.torso_gain,
        calibration_frame=calibration_frame,
        positions=positions,
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
    torque_limits = model.actuator_ctrlrange[actuator_ids]
    trajectory, clamp_count = clamp_x2_trajectory(
        trajectory, model.jnt_range[joint_ids]
    )
    kp, kd = default_x2_pd_gains()

    # Initialize directly on the first bounded reference, then preserve the
    # vendor model's initial floating-base pose throughout this fixed-base test.
    mujoco.mj_resetData(model, data)
    data.qpos[qpos_addresses] = trajectory[0]
    base_qpos = data.qpos[:7].copy()
    mujoco.mj_forward(model, data)
    recording_duration = (len(trajectory) - 1) * frame_dt
    duration = min(
        args.duration if args.duration is not None else np.inf,
        recording_duration / args.playback_speed,
    )
    squared_error = np.zeros(len(X2_JOINT_NAMES), dtype=np.float64)
    max_error = np.zeros(len(X2_JOINT_NAMES), dtype=np.float64)
    saturated_steps = np.zeros(len(X2_JOINT_NAMES), dtype=np.int64)
    contact_steps = 0
    contact_body_pairs: set[tuple[str, str]] = set()
    steps = 0

    viewer_context = None
    if args.viewer:
        import mujoco.viewer
        viewer_context = mujoco.viewer.launch_passive(model, data)
    try:
        while data.time < duration:
            started = time.monotonic()
            target, target_velocity = interpolate_x2_reference(
                trajectory, frame_dt, data.time * args.playback_speed
            )
            target_velocity *= args.playback_speed
            position = data.qpos[qpos_addresses]
            velocity = data.qvel[dof_addresses]
            torque, saturated = bounded_pd_torque(
                position, velocity, target, target_velocity, kp, kd, torque_limits
            )
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids] = torque
            mujoco.mj_step(model, data)

            # Numerical fixture: remove all six floating-base velocities and
            # restore its pose. This is intentionally not a balance controller.
            data.qpos[:7] = base_qpos
            data.qvel[:6] = 0.0
            error = target - data.qpos[qpos_addresses]
            squared_error += error * error
            max_error = np.maximum(max_error, np.abs(error))
            saturated_steps += saturated
            if data.ncon:
                contact_steps += 1
                for contact in data.contact:
                    body1 = model.geom_bodyid[contact.geom1]
                    body2 = model.geom_bodyid[contact.geom2]
                    name1 = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_BODY, body1
                    ) or f"body_{body1}"
                    name2 = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_BODY, body2
                    ) or f"body_{body2}"
                    contact_body_pairs.add(tuple(sorted((name1, name2))))
            steps += 1
            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                raise RuntimeError(f"Non-finite simulation state at {data.time:.3f} s")
            if viewer_context is not None:
                if not viewer_context.is_running():
                    break
                viewer_context.sync()
            if args.realtime:
                time.sleep(max(0.0, model.opt.timestep - (time.monotonic() - started)))
    finally:
        if viewer_context is not None:
            viewer_context.close()

    rms_error = np.sqrt(squared_error / max(steps, 1))
    print(f"Model: {xml}")
    print(f"Mode: FIXED-BASE DYNAMICS (floating base numerically pinned)")
    print(f"Playback speed: {args.playback_speed:.2f}x")
    print(f"Simulated: {data.time:.3f} s, steps: {steps}, dt: {model.opt.timestep:.4f} s")
    print(f"Reference values clamped before control: {clamp_count} / {trajectory.size}")
    print(f"Overall RMS tracking error: {np.sqrt(np.mean(rms_error ** 2)):.4f} rad")
    print(f"Peak tracking error: {np.max(max_error):.4f} rad")
    print(f"Torque saturation events: {int(np.sum(saturated_steps))} / {steps * len(X2_JOINT_NAMES)}")
    print(f"Steps with contacts: {contact_steps} / {steps}")
    for body1, body2 in sorted(contact_body_pairs):
        print(f"  contact: {body1} <-> {body2}")
    for name, rms, maximum, saturated in zip(
        X2_JOINT_NAMES, rms_error, max_error, saturated_steps
    ):
        if maximum > 0.10 or saturated:
            print(
                f"  {name}: rms={rms:.4f}, max={maximum:.4f}, "
                f"saturated_steps={saturated}"
            )
    print("SIMULATION ONLY: no balance, locomotion policy, or hardware output.")


if __name__ == "__main__":
    main()
