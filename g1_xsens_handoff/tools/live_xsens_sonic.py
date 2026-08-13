#!/usr/bin/env python3
"""Publish calibrated live Xsens poses to SONIC ZMQ protocol v1.

This bridge publishes motion references only. It contains no Unitree SDK,
DDS, ROS, or motor-command output. Use it with SONIC in MuJoCo first.
"""

from __future__ import annotations

import argparse
from collections import deque
import logging
import math
import time

import numpy as np

from xsens_bridge.g1_retarget import G1_DEFAULT_POSE, _conjugate, _multiply
from xsens_bridge.live_retarget import (
    OnlineG1Retargeter,
    calibrate_nt_sequence,
    calibrate_ntf_sequence,
)
from xsens_bridge.sonic_zmq import (
    mujoco_to_isaaclab,
    pack_sonic_pose_message,
    protocol_v1_fields,
)
from xsens_bridge.stream import LatestPoseReceiver


log = logging.getLogger("live_xsens_sonic")
logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")

FULL_TURN_RAD = 2.0 * math.pi


def relative_pelvis_yaw(
    calibration_quaternion: np.ndarray,
    current_quaternion: np.ndarray,
) -> float:
    """Return current pelvis heading relative to calibrated heading."""
    relative = _multiply(
        _conjugate(np.asarray(calibration_quaternion)[None]),
        np.asarray(current_quaternion)[None],
    )[0]
    w, x, y, z = relative
    return float(np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    ))


def bounded_root_heading(
    pelvis_yaw: float,
    previous_heading: float,
    dt: float,
    *,
    gain: float,
    deadband_rad: float,
    limit_rad: float,
    max_rate_rad_s: float,
) -> float:
    """Scale and slew-limit a calibrated root-heading reference.

    Limits smaller than one full turn retain the conservative hard-clamp
    behavior. A full-turn limit enables continuous rotation: positive and
    negative headings wrap through zero after each complete revolution, and
    the slew limiter follows the shortest circular delta across that wrap.
    """
    target = 0.0 if abs(pelvis_yaw) <= deadband_rad else gain * pelvis_yaw
    max_step = max_rate_rad_s * dt
    if limit_rad + 1e-9 < FULL_TURN_RAD:
        target = float(np.clip(target, -limit_rad, limit_rad))
        return float(previous_heading + np.clip(
            target - previous_heading, -max_step, max_step
        ))

    target = wrap_full_turn(target)
    previous_heading = wrap_full_turn(previous_heading)
    circular_delta = math.atan2(
        math.sin(target - previous_heading),
        math.cos(target - previous_heading),
    )
    next_heading = previous_heading + float(np.clip(
        circular_delta, -max_step, max_step
    ))
    return wrap_full_turn(next_heading)


def wrap_full_turn(angle: float) -> float:
    """Wrap complete turns to zero while retaining rotation direction."""
    if -FULL_TURN_RAD < angle < FULL_TURN_RAD:
        return float(angle)
    wrapped = math.fmod(angle, FULL_TURN_RAD)
    return 0.0 if abs(wrapped) < 1e-12 else float(wrapped)


def unwrap_yaw(
    wrapped_yaw: float,
    previous_wrapped_yaw: float | None,
    previous_unwrapped_yaw: float,
) -> tuple[float, float]:
    """Accumulate continuous yaw without reversing at the ±π boundary."""
    if previous_wrapped_yaw is None:
        return wrapped_yaw, wrapped_yaw
    delta = float(np.arctan2(
        np.sin(wrapped_yaw - previous_wrapped_yaw),
        np.cos(wrapped_yaw - previous_wrapped_yaw),
    ))
    return wrapped_yaw, previous_unwrapped_yaw + delta


def yaw_quaternion(
    yaw: float,
    previous: np.ndarray | None = None,
) -> np.ndarray:
    quaternion = np.asarray(
        [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
        dtype=np.float32,
    )
    if previous is not None and float(np.dot(quaternion, previous)) < 0.0:
        quaternion = -quaternion
    return quaternion


def clamp_physical_leg_angles(
    dof_pos: np.ndarray,
    dof_vel: np.ndarray,
    *,
    hip_limit_rad: float,
    foot_limit_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Clamp physical hip and ankle references around the neutral stance."""
    position = np.asarray(dof_pos, dtype=np.float32).copy()
    velocity = np.asarray(dof_vel, dtype=np.float32).copy()
    if position.shape != (29,) or velocity.shape != (29,):
        raise ValueError("physical leg references must have shape (29,)")

    for indices, limit in (
        (np.asarray([0, 1, 2, 6, 7, 8]), hip_limit_rad),
        (np.asarray([4, 5, 10, 11]), foot_limit_rad),
    ):
        if limit <= 0.0:
            continue
        lower = G1_DEFAULT_POSE[indices] - limit
        upper = G1_DEFAULT_POSE[indices] + limit
        requested = position[indices].copy()
        position[indices] = np.clip(requested, lower, upper)
        outward = ((requested > upper) & (velocity[indices] > 0.0)) | (
            (requested < lower) & (velocity[indices] < 0.0)
        )
        velocity[indices[outward]] = 0.0
    return position, velocity


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--xsens-port", type=int, default=9763)
    parser.add_argument("--zmq-bind", default="tcp://*:5556")
    parser.add_argument("--topic", default="pose")
    parser.add_argument("--calibration-seconds", type=float, default=9.0)
    parser.add_argument(
        "--calibration-sequence",
        choices=("nt", "ntf"),
        default="nt",
        help="Use legacy N→T or measured N→T→forward calibration.",
    )
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--stale-ms", type=float, default=250.0)
    parser.add_argument("--fps", type=float, default=50.0)
    # Live protocol-v1 messages are appended to SONIC's streaming motion.
    # Sending overlapping multi-frame history on every 50 Hz tick makes the
    # reference queue grow faster than playback and produces increasing lag.
    parser.add_argument("--batch-frames", type=int, default=1)
    parser.add_argument("--runtime-seconds", type=float, default=0.0)
    parser.add_argument("--leg-motion-gain", type=float, default=1.0)
    parser.add_argument(
        "--torso-gain",
        type=float,
        default=0.50,
        help="Gain for pelvis-relative chest yaw/roll/pitch retargeting.",
    )
    parser.add_argument(
        "--arm-max-speed",
        type=float,
        default=4.0,
        help="Maximum live joint-reference slew rate in rad/s.",
    )
    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=12.0,
        help="Internal joint-velocity cap in rad/s.",
    )
    parser.add_argument(
        "--physical-leg-blend",
        type=float,
        default=0.35,
        help=(
            "Fraction of calibrated Xsens leg motion used by "
            "--physical-full-body (0..1.0). Waist remains neutral."
        ),
    )
    parser.add_argument(
        "--physical-hip-limit-deg",
        type=float,
        default=0.0,
        help=(
            "Optional symmetric hip pitch/roll/yaw limit in degrees around "
            "the neutral stance; zero disables the additional clamp."
        ),
    )
    parser.add_argument(
        "--physical-foot-limit-deg",
        type=float,
        default=0.0,
        help=(
            "Optional symmetric ankle pitch/roll limit in degrees around "
            "the neutral stance; zero disables the additional clamp."
        ),
    )
    parser.add_argument("--neutral-shoulder-bias-deg", type=float, default=6.0)
    parser.add_argument("--reach-shoulder-bias-deg", type=float, default=11.0)
    parser.add_argument(
        "--track-wrists",
        action="store_true",
        help=(
            "SIMULATION/COMMISSIONING: track Xsens hand rotation relative "
            "to each forearm as palm-up/down wrist roll only. Wrist pitch "
            "and yaw remain neutral."
        ),
    )
    parser.add_argument(
        "--track-wrist-pitch",
        action="store_true",
        help=(
            "Track bounded anatomical wrist flexion/extension on a "
            "calibrated axis and route it to the adjacent G1 wrist-yaw "
            "motor. The G1 wrist-pitch motor remains neutral."
        ),
    )
    parser.add_argument(
        "--wrist-pitch-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
        help="Coordinate-convention sign for G1 wrist pitch.",
    )
    parser.add_argument(
        "--left-wrist-pitch-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
    )
    parser.add_argument(
        "--right-wrist-pitch-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
    )
    parser.add_argument(
        "--track-pelvis-yaw",
        action="store_true",
        help=(
            "Track calibrated pelvis heading as bounded waist yaw. Waist "
            "roll/pitch remain neutral in physical full-body mode."
        ),
    )
    parser.add_argument(
        "--track-relative-waist",
        action="store_true",
        help=(
            "Track pelvis-to-chest waist yaw/roll/pitch with conservative "
            "limits while ignoring uniform global body orientation."
        ),
    )
    parser.add_argument(
        "--track-foot-heading",
        action="store_true",
        help=(
            "Map a clearly lifted foot's pelvis-relative yaw to the matching "
            "hip-yaw joint. Planted-foot heading is ignored."
        ),
    )
    parser.add_argument(
        "--track-lifted-hip-roll",
        action="store_true",
        help=(
            "Use pelvis-relative lateral knee position to refine hip roll "
            "only while the corresponding foot is lifted."
        ),
    )
    parser.add_argument(
        "--track-root-heading",
        action="store_true",
        help="SIMULATION ONLY: send a bounded pelvis-yaw body_quat reference.",
    )
    parser.add_argument("--root-heading-gain", type=float, default=0.20)
    parser.add_argument(
        "--root-heading-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
        help="Coordinate-convention sign applied to calibrated pelvis yaw.",
    )
    parser.add_argument("--root-heading-limit-deg", type=float, default=15.0)
    parser.add_argument("--root-heading-deadband-deg", type=float, default=5.0)
    parser.add_argument("--root-heading-max-rate-deg-s", type=float, default=10.0)
    parser.add_argument(
        "--physical-arm-only",
        action="store_true",
        help=(
            "Hold all 12 leg and 3 waist references at the validated G1 "
            "neutral pose; stream only the two arms. Intended for restrained "
            "physical neutral validation before planner locomotion."
        ),
    )
    parser.add_argument(
        "--physical-full-body",
        action="store_true",
        help=(
            "Experimental restrained physical mode: stream both arms and "
            "reduced-gain calibrated leg motion while holding the waist at "
            "neutral. This is raw pose tracking, not planner locomotion."
        ),
    )
    return parser.parse_args()


def capture_calibration(
    receiver: LatestPoseReceiver,
    seconds: float,
    wait_timeout: float,
    sequence: str,
):
    log.info("Waiting for Xsens MXTP02 frames...")
    deadline = time.monotonic() + wait_timeout
    while receiver.latest() is None:
        if time.monotonic() >= deadline:
            raise TimeoutError("No Xsens frame arrived before the wait timeout")
        time.sleep(0.02)
    if sequence == "ntf":
        log.info(
            "Calibration: hold N-pose, then T-pose, then arms straight "
            "forward at shoulder height."
        )
    else:
        log.info("Calibration: hold N-pose, then move to T-pose and hold.")
    started = time.monotonic()
    frames = []
    last_counter = None
    announced_t = False
    announced_forward = False
    while time.monotonic() - started < seconds:
        elapsed = time.monotonic() - started
        t_fraction = 0.33 if sequence == "ntf" else 0.40
        if not announced_t and elapsed >= seconds * t_fraction:
            log.info("Move smoothly into T-pose and hold it.")
            announced_t = True
        if (
            sequence == "ntf"
            and not announced_forward
            and elapsed >= seconds * 0.67
        ):
            log.info(
                "Move both arms straight forward, shoulder-width apart, "
                "and hold."
            )
            announced_forward = True
        latest = receiver.latest()
        if latest is not None:
            counter = latest.frame.header.sample_counter
            if counter != last_counter:
                frames.append(latest.frame)
                last_counter = counter
        time.sleep(0.002)
    if len(frames) < 50:
        raise RuntimeError(f"Calibration captured only {len(frames)} frames")
    result = (
        calibrate_ntf_sequence(frames)
        if sequence == "ntf"
        else calibrate_nt_sequence(frames)
    )
    if result.forward_pose_index is None:
        log.info(
            "Calibration complete: N=%d T=%d score=%.3f frames=%d",
            result.n_pose_index,
            result.t_pose_index,
            result.t_pose_score,
            result.captured_frames,
        )
    else:
        log.info(
            "Calibration complete: N=%d T=%d FORWARD=%d score=%.3f "
            "frames=%d",
            result.n_pose_index,
            result.t_pose_index,
            result.forward_pose_index,
            result.t_pose_score,
            result.captured_frames,
        )
    return result


def main() -> None:
    args = parse_args()
    if (
        args.fps <= 0
        or args.batch_frames <= 0
        or args.arm_max_speed <= 0
        or args.torso_gain <= 0
    ):
        raise ValueError("fps and batch-frames must be positive")
    if args.physical_arm_only and args.physical_full_body:
        raise ValueError(
            "physical-arm-only and physical-full-body are mutually exclusive"
        )
    if args.track_relative_waist and args.track_pelvis_yaw:
        raise ValueError(
            "track-relative-waist and track-pelvis-yaw are mutually exclusive"
        )
    if not 0.0 <= args.physical_leg_blend <= 1.0:
        raise ValueError("physical-leg-blend must be within [0, 1.0]")
    if args.physical_hip_limit_deg < 0.0 or args.physical_foot_limit_deg < 0.0:
        raise ValueError("physical hip and foot limits cannot be negative")
    if (
        args.root_heading_gain <= 0.0
        or args.root_heading_limit_deg <= 0.0
        or args.root_heading_deadband_deg < 0.0
        or args.root_heading_max_rate_deg_s <= 0.0
    ):
        raise ValueError("root-heading gain, limits, and rate are invalid")
    try:
        import zmq
    except ImportError as error:
        raise SystemExit("pyzmq is required: python -m pip install pyzmq") from error

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    # This is a live control reference, not a recording transport. Retaining
    # old poses adds unsafe latency, so keep at most one unsent message and
    # drop a tick instead of blocking the 50 Hz loop.
    publisher.setsockopt(zmq.SNDHWM, 1)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.zmq_bind)
    period = 1.0 / args.fps
    stale_seconds = args.stale_ms / 1000.0

    try:
        with LatestPoseReceiver(
            args.bind,
            args.xsens_port,
            counter_reset_after_seconds=stale_seconds,
        ) as receiver:
            log.info("Xsens UDP %s:%d", args.bind, args.xsens_port)
            log.info("SONIC ZMQ %s topic=%s (simulation references only)", args.zmq_bind, args.topic)
            calibration = capture_calibration(
                receiver,
                args.calibration_seconds,
                args.wait_timeout,
                args.calibration_sequence,
            )
            retargeter = OnlineG1Retargeter(
                calibration,
                motion_gain=args.leg_motion_gain,
                torso_gain=args.torso_gain,
                neutral_shoulder_bias_deg=args.neutral_shoulder_bias_deg,
                reach_shoulder_bias_deg=args.reach_shoulder_bias_deg,
                max_position_speed=args.arm_max_speed,
                max_joint_speed=args.max_joint_speed,
                track_wrists=args.track_wrists,
                track_wrist_pitch=args.track_wrist_pitch,
                wrist_pitch_sign=args.wrist_pitch_sign,
                left_wrist_pitch_sign=args.left_wrist_pitch_sign,
                right_wrist_pitch_sign=args.right_wrist_pitch_sign,
                track_pelvis_yaw=args.track_pelvis_yaw,
                track_foot_heading=args.track_foot_heading,
                track_lifted_hip_roll=args.track_lifted_hip_roll,
            )
            positions: deque[np.ndarray] = deque(maxlen=args.batch_frames)
            velocities: deque[np.ndarray] = deque(maxlen=args.batch_frames)
            body_quaternions: deque[np.ndarray] = deque(
                maxlen=args.batch_frames
            )
            indices: deque[int] = deque(maxlen=args.batch_frames)
            root_heading_command = 0.0
            previous_root_quaternion = yaw_quaternion(0.0)
            previous_wrapped_pelvis_yaw: float | None = None
            unwrapped_pelvis_yaw = 0.0
            frame_number = 0
            last_counter = None
            last_pose = None
            state = None
            started = time.monotonic()
            next_tick = started
            sent = 0
            dropped = 0
            while (
                args.runtime_seconds <= 0
                or time.monotonic() - started < args.runtime_seconds
            ):
                now = time.monotonic()
                latest = receiver.latest()
                fresh = (
                    latest is not None
                    and now - latest.received_at <= stale_seconds
                )
                new_state = "LIVE" if fresh else "STALE"
                if new_state != state:
                    log.info("Stream state: %s", new_state)
                    if not fresh:
                        root_heading_command = 0.0
                        previous_root_quaternion = yaw_quaternion(0.0)
                        previous_wrapped_pelvis_yaw = None
                        unwrapped_pelvis_yaw = 0.0
                    state = new_state
                if (
                    fresh
                    and latest is not None
                    and latest.frame.header.sample_counter != last_counter
                ):
                    last_pose = retargeter.retarget(
                        latest.frame, latest.received_at
                    )
                    root_quaternion = yaw_quaternion(
                        0.0, previous_root_quaternion
                    )
                    if args.track_root_heading:
                        pelvis = next(
                            segment for segment in latest.frame.segments
                            if segment.name == "pelvis"
                        )
                        pelvis_yaw = relative_pelvis_yaw(
                            calibration.quaternions["pelvis"],
                            np.asarray(pelvis.quaternion_wxyz),
                        )
                        pelvis_yaw *= args.root_heading_sign
                        (
                            previous_wrapped_pelvis_yaw,
                            unwrapped_pelvis_yaw,
                        ) = unwrap_yaw(
                            pelvis_yaw,
                            previous_wrapped_pelvis_yaw,
                            unwrapped_pelvis_yaw,
                        )
                        root_heading_command = bounded_root_heading(
                            unwrapped_pelvis_yaw,
                            root_heading_command,
                            period,
                            gain=args.root_heading_gain,
                            deadband_rad=math.radians(
                                args.root_heading_deadband_deg
                            ),
                            limit_rad=math.radians(
                                args.root_heading_limit_deg
                            ),
                            max_rate_rad_s=math.radians(
                                args.root_heading_max_rate_deg_s
                            ),
                        )
                        root_quaternion = yaw_quaternion(
                            root_heading_command,
                            previous_root_quaternion,
                        )
                    previous_root_quaternion = root_quaternion
                    if args.physical_arm_only:
                        # MuJoCo/bridge order is legs [0:12], waist [12:15],
                        # arms [15:29]. Never pass live Xsens lower-body or
                        # torso deltas during the restrained arm-only stage.
                        last_pose.dof_pos[:15] = G1_DEFAULT_POSE[:15]
                        last_pose.dof_vel[:15] = 0.0
                    elif args.physical_full_body:
                        # Allow only a conservative fraction of calibrated
                        # leg motion on hardware. Keeping the waist neutral
                        # avoids feeding pelvis/torso lean directly into the
                        # raw lower-body reference.
                        last_pose.dof_pos[:12] = (
                            G1_DEFAULT_POSE[:12]
                            + args.physical_leg_blend
                            * (
                                last_pose.dof_pos[:12]
                                - G1_DEFAULT_POSE[:12]
                            )
                        )
                        last_pose.dof_vel[:12] *= args.physical_leg_blend
                        limited_pos, limited_vel = clamp_physical_leg_angles(
                            last_pose.dof_pos,
                            last_pose.dof_vel,
                            hip_limit_rad=math.radians(
                                args.physical_hip_limit_deg
                            ),
                            foot_limit_rad=math.radians(
                                args.physical_foot_limit_deg
                            ),
                        )
                        last_pose.dof_pos[:] = limited_pos
                        last_pose.dof_vel[:] = limited_vel
                        if args.track_relative_waist:
                            waist_limits = np.asarray(
                                [0.55, 0.28, 0.34], dtype=np.float32
                            )
                            last_pose.dof_pos[12:15] = np.clip(
                                last_pose.dof_pos[12:15],
                                G1_DEFAULT_POSE[12:15] - waist_limits,
                                G1_DEFAULT_POSE[12:15] + waist_limits,
                            )
                        elif args.track_pelvis_yaw:
                            last_pose.dof_pos[13:15] = G1_DEFAULT_POSE[13:15]
                            last_pose.dof_vel[13:15] = 0.0
                        else:
                            last_pose.dof_pos[12:15] = G1_DEFAULT_POSE[12:15]
                            last_pose.dof_vel[12:15] = 0.0
                    last_counter = latest.frame.header.sample_counter
                if fresh and last_pose is not None:
                    positions.append(last_pose.dof_pos)
                    velocities.append(last_pose.dof_vel)
                    body_quaternions.append(root_quaternion)
                    indices.append(frame_number)
                    frame_number += 1
                    while len(positions) < args.batch_frames:
                        positions.appendleft(last_pose.dof_pos)
                        velocities.appendleft(np.zeros(29, dtype=np.float32))
                        body_quaternions.appendleft(root_quaternion)
                        indices.appendleft(indices[0])
                    fields = protocol_v1_fields(
                        mujoco_to_isaaclab(np.stack(positions)),
                        mujoco_to_isaaclab(np.stack(velocities)),
                        np.asarray(indices, dtype=np.int64),
                        body_quat=np.stack(body_quaternions),
                    )
                    try:
                        publisher.send(
                            pack_sonic_pose_message(
                                fields, topic=args.topic, version=1
                            ),
                            flags=zmq.NOBLOCK,
                        )
                        sent += 1
                    except zmq.Again:
                        dropped += 1
                    if sent % int(max(1, args.fps)) == 0:
                        age_ms = 1000.0 * (now - latest.received_at)
                        log.info(
                            "sent=%d dropped=%d age=%.1fms received=%d "
                            "missing=%d resets=%d malformed=%d "
                            "root_heading=%.1fdeg",
                            sent,
                            dropped,
                            age_ms,
                            receiver.health.received_frames,
                            receiver.health.missing_frames,
                            receiver.health.counter_resets,
                            receiver.health.malformed_packets,
                            math.degrees(root_heading_command),
                        )

                next_tick += period
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif time.monotonic() - next_tick > period:
                    next_tick = time.monotonic()
    finally:
        publisher.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
