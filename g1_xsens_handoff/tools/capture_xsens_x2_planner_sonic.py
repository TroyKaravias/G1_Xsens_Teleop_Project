#!/usr/bin/env python3
"""Capture MXTP02 Xsens and track it with stationary or planned v2 SONIC."""

from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.live_xsens_x2 import capture_calibration
from xsens_bridge.live_retarget import OnlineG1Retargeter
from xsens_bridge.stream import LatestPoseReceiver
from xsens_bridge.x2_planner import (
    X2KinematicPlanner, clamp_planner_joints, pack_mujoco_qpos,
    unpack_mujoco_qpos,
)
from xsens_bridge.x2_retarget import g1_deltas_to_x2


log = logging.getLogger("capture_xsens_x2_planner_sonic")
logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")


def _sample_deltas(
    timestamps: np.ndarray, deltas: np.ndarray, sample_times: np.ndarray
) -> np.ndarray:
    result = np.empty((len(sample_times), deltas.shape[1]), dtype=np.float32)
    for joint in range(deltas.shape[1]):
        result[:, joint] = np.interp(sample_times, timestamps, deltas[:, joint])
    return result


def main() -> None:
    external = WORKSPACE_ROOT / "external"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9764)
    parser.add_argument("--calibration-seconds", type=float, default=9.0)
    parser.add_argument("--forward-calibration-seconds", type=float, default=3.0)
    parser.add_argument(
        "--capture-seconds", type=float, default=0.0,
        help="post-calibration seconds to capture; 0 captures until playback ends",
    )
    parser.add_argument(
        "--motion-start-delay", type=float, default=0.0,
        help="discard this many post-calibration seconds before capturing motion",
    )
    parser.add_argument(
        "--auto-trim-quiet", action=argparse.BooleanOptionalAction, default=True,
        help="remove a long quiet lead-in before the final motion sequence",
    )
    parser.add_argument("--wait-timeout", type=float, default=60.0)
    parser.add_argument("--stale-ms", type=float, default=500.0)
    parser.add_argument("--planner", type=Path, default=external / "sonic_x2/x2_planner_frozen_g1core_v1.onnx")
    parser.add_argument("--seed-motion", type=Path, default=external / "sonic_x2_quickplay/motions/x2_idle_stand.pkl")
    parser.add_argument("--sonic", type=Path, default=external / "sonic_x2_quickplay/models/x2_sonic_frozen_g1core_lora_v2.onnx")
    parser.add_argument("--evaluator", type=Path, default=external / "sonic_x2_quickplay/scripts/eval_x2_mujoco_onnx.py")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "data/x2_planner_xsens_live.pkl")
    parser.add_argument(
        "--mode", type=int, default=-1,
        help="planner locomotion mode; -1 selects idle/slow-walk/walk from pelvis speed",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--stationary", action=argparse.BooleanOptionalAction, default=True,
        help="hold validated idle root/legs and apply Xsens only above the hips",
    )
    parser.add_argument("--viewer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--verify-reference", action=argparse.BooleanOptionalAction, default=True,
        help="show kinematic reference first; close it to start SONIC dynamics",
    )
    args = parser.parse_args()
    if min(args.calibration_seconds, args.forward_calibration_seconds) <= 0:
        parser.error("calibration durations must be positive")
    if args.capture_seconds < 0 or args.motion_start_delay < 0:
        parser.error("capture duration and motion start delay must be non-negative")

    stale_s = args.stale_ms / 1000.0
    samples: list[tuple[float, np.ndarray, np.ndarray]] = []
    with LatestPoseReceiver(args.bind, args.port, counter_reset_after_seconds=stale_s) as receiver:
        log.info("X2 SIMULATION ONLY: waiting for MVN playback/live suit on %s:%d", args.bind, args.port)
        calibration = capture_calibration(
            receiver, args.calibration_seconds,
            args.forward_calibration_seconds, args.wait_timeout,
        )
        retargeter = OnlineG1Retargeter(
            calibration, motion_gain=.75, torso_gain=.50,
            max_position_speed=4.0, max_joint_speed=10.0,
        )
        origin = None
        last_counter = None
        started = time.monotonic()
        last_fresh = started
        announced_capture = False
        while args.capture_seconds == 0 or time.monotonic() - started < args.motion_start_delay + args.capture_seconds:
            latest = receiver.latest()
            now = time.monotonic()
            if latest is not None and now - latest.received_at <= stale_s:
                last_fresh = now
                counter = latest.frame.header.sample_counter
                if counter != last_counter:
                    pose = retargeter.retarget(latest.frame, latest.received_at)
                    direct = g1_deltas_to_x2(pose.dof_pos[None])[0]
                    if origin is None:
                        origin = direct.copy()
                    if now - started >= args.motion_start_delay:
                        if not announced_capture:
                            log.info("Capturing motion now")
                            announced_capture = True
                        # Preserve the absolute calibrated X2 pose. Subtracting
                        # this from an idle-template origin and later adding it
                        # to the planner's hands-behind-back pose changes the
                        # semantics (e.g. punches become salute-like gestures).
                        pelvis_position = np.asarray(next(
                            segment.position_m for segment in latest.frame.segments
                            if segment.name == "pelvis"
                        ), dtype=np.float32)
                        samples.append((latest.received_at, direct.astype(np.float32), pelvis_position))
                    last_counter = counter
            elif latest is not None and now - last_fresh > stale_s:
                log.warning("Xsens stream ended/stale; planning from %d captured frames", len(samples))
                break
            time.sleep(.002)
    if len(samples) < 4:
        raise RuntimeError(f"Captured only {len(samples)} post-calibration Xsens frames")

    timestamps = np.asarray([item[0] for item in samples], dtype=np.float64)
    timestamps -= timestamps[0]
    positions = np.stack([item[1] for item in samples])
    pelvis_positions = np.stack([item[2] for item in samples])
    pelvis_positions -= pelvis_positions[0]
    duration = max(4 / 30.0, float(timestamps[-1]))
    frames = max(4, int(np.floor(duration * 30.0)) + 1)
    sample_times = np.arange(frames, dtype=np.float64) / 30.0
    xsens_positions = _sample_deltas(timestamps, positions, sample_times)
    sampled_pelvis = _sample_deltas(timestamps, pelvis_positions, sample_times)
    if args.auto_trim_quiet and len(xsens_positions) > 60:
        upper_speed = np.linalg.norm(
            np.diff(xsens_positions[:, 12:], axis=0), axis=1
        ) * 30.0
        active = np.flatnonzero(upper_speed > 0.5)
        if len(active) >= 2:
            long_gaps = np.flatnonzero(np.diff(active) > 5 * 30)
            if len(long_gaps):
                motion_start = int(active[long_gaps[-1] + 1] + 1)
                trim_start = max(0, motion_start - 30)
                log.info(
                    "Auto-trimming %.2f s quiet lead-in; retaining 1 s pre-motion",
                    trim_start / 30.0,
                )
                xsens_positions = xsens_positions[trim_start:]
                sampled_pelvis = sampled_pelvis[trim_start:]
                frames = len(xsens_positions)
    xsens_deltas = xsens_positions - xsens_positions[0]

    import joblib
    seed_data = joblib.load(args.seed_motion)
    seed_motion = next(iter(seed_data.values()))
    context = pack_mujoco_qpos(
        np.asarray(seed_motion["root_trans_offset"][:4]),
        np.asarray(seed_motion["root_rot"][:4]),
        np.asarray(seed_motion["dof"][:4]),
    )
    if args.stationary:
        seed_fps = float(seed_motion["fps"])
        seed_count = len(seed_motion["dof"])
        seed_indices = (
            np.arange(frames, dtype=np.float64) * seed_fps / 30.0
        ).astype(int) % seed_count
        root_position = np.asarray(seed_motion["root_trans_offset"])[seed_indices].copy()
        # Stationary mode deliberately removes world translation while keeping
        # the validated idle reference's contact height.
        root_position[:, :2] = root_position[0, :2]
        qpos = pack_mujoco_qpos(
            root_position,
            np.asarray(seed_motion["root_rot"])[seed_indices],
            np.asarray(seed_motion["dof"])[seed_indices],
        )
        log.info("Stationary mode: idle root/legs, Xsens waist/arms/head")
    else:
        planner = X2KinematicPlanner(args.planner)
        chunks = []
        previous_delta = np.zeros((4, 19), dtype=np.float32)
        previous_facing = np.asarray([1.0, 0.0], dtype=np.float32)
        while sum(len(chunk) for chunk in chunks) < frames:
            start = sum(len(chunk) for chunk in chunks)
            indices = np.minimum(start + np.arange(4), frames - 1)
            current_delta = xsens_deltas[indices, 12:]
            context[:, 19:] += current_delta - previous_delta
            previous_delta = current_delta
            stop = min(frames - 1, start + 63)
            span_s = max((stop - start) / 30.0, 1.0 / 30.0)
            velocity_xy = (sampled_pelvis[stop, :2] - sampled_pelvis[start, :2]) / span_s
            speed = float(np.linalg.norm(velocity_xy))
            if speed > 0.8:
                velocity_xy *= 0.8 / speed
                speed = 0.8
            if speed > 0.05:
                previous_facing = velocity_xy / speed
            mode = args.mode if args.mode >= 0 else (0 if speed < 0.05 else 1 if speed < 0.35 else 2)
            velocity_intent = np.asarray([
                previous_facing[0], velocity_xy[0], velocity_xy[1], previous_facing[1]
            ], dtype=np.float32)
            log.info(
                "Planner chunk %d: mode=%d speed=%.2f m/s velocity=(%.2f, %.2f)",
                len(chunks), mode, speed, velocity_xy[0], velocity_xy[1],
            )
            prediction = planner.generate(
                context, velocity_intent=velocity_intent, mode=mode,
                random_seed=args.seed + len(chunks),
            )
            chunks.append(prediction)
            context = prediction[-4:].copy()
        qpos = np.concatenate(chunks)[:frames]
    # The template planner owns dynamically coherent root and lower-body
    # motion. Its mode templates must not overwrite the operator's recorded
    # waist/arm/head intent (mode 0 otherwise produces its own idle gesture).
    # Apply Xsens to every policy reference frame, not merely four context
    # frames. SONIC still owns actuation and must track this under physics.
    qpos[:, 19:] = xsens_positions[:frames, 12:]
    import mujoco
    x2_model = mujoco.MjModel.from_xml_path(
        str(external / "sonic_x2_quickplay/assets/mjcf/x2_ultra.xml")
    )
    joint_limits = np.asarray([
        x2_model.jnt_range[x2_model.joint(name).id]
        for name in (
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
            "head_yaw_joint", "head_pitch_joint",
        )
    ])
    qpos, clamp_count = clamp_planner_joints(qpos, joint_limits)
    root_position, root_quaternion, joints = unpack_mujoco_qpos(qpos)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump({"x2_xsens_planner": {
        "root_trans_offset": root_position,
        "root_rot": root_quaternion,
        "dof": joints,
        "fps": 30.0,
    }}, args.output)
    log.info("Captured %d Xsens frames; generated %.2f s X2 planner reference", len(samples), frames / 30.0)
    log.info("X2 joint-limit clamps: %d", clamp_count)
    log.info("Reference: %s", args.output)

    if args.viewer:
        environment = os.environ.copy()
        dependency_path = str(external / "x2_python_deps")
        environment["PYTHONPATH"] = dependency_path + os.pathsep + environment.get("PYTHONPATH", "")
        if args.verify_reference:
            log.info("Opening KINEMATIC reference first; verify punches, then close the window")
            subprocess.run([
                sys.executable, str(args.evaluator),
                "--motion", str(args.output), "--kinematic",
            ], check=True, env=environment)
            log.info("Kinematic window closed; opening v2 SONIC free-base dynamics")
        subprocess.run([
            sys.executable, str(args.evaluator),
            "--onnx", str(args.sonic),
            "--motion", str(args.output),
            "--tuning", "", "--action-clip", "20", "--freeze-wrist",
        ], check=True, env=environment)
    print("SIMULATION ONLY: no X2 hardware connection or hardware validation.")


if __name__ == "__main__":
    main()
