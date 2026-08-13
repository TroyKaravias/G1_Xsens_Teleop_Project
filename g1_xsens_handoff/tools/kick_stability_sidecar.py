#!/usr/bin/env python3
"""Experimental latest-only SONIC reference stability sidecar.

This process consumes the proven publisher on an internal endpoint, shapes
only lower-body references, and republishes protocol-v1 messages to SONIC.
It contains no Unitree SDK or direct motor output.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import time

import numpy as np

from xsens_bridge.kick_stability import KickStabilityGovernor, KickStabilityLimits
from xsens_bridge.sonic_zmq import (
    G1_MUJOCO_TO_ISAACLAB,
    SONIC_ZMQ_HEADER_SIZE,
    pack_sonic_pose_message,
)


log = logging.getLogger("kick_stability_sidecar")
logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")

DTYPES = {
    "u8": np.dtype("<u1"),
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"),
    "i64": np.dtype("<i8"),
    "bool": np.dtype("?"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="tcp://127.0.0.1:5567")
    parser.add_argument("--output", default="tcp://127.0.0.1:5556")
    parser.add_argument("--topic", default="pose")
    parser.add_argument("--stale-ms", type=float, default=750.0)
    parser.add_argument("--lift-on-rad", type=float, default=0.22)
    parser.add_argument("--lift-off-rad", type=float, default=0.12)
    parser.add_argument("--side-margin-rad", type=float, default=0.07)
    parser.add_argument("--phase-dwell-s", type=float, default=0.06)
    parser.add_argument("--stance-protection", type=float, default=0.70)
    parser.add_argument("--waist-protection", type=float, default=0.35)
    parser.add_argument("--support-hip-pitch-limit-deg", type=float, default=11.5)
    parser.add_argument("--support-hip-roll-limit-deg", type=float, default=10.0)
    parser.add_argument("--support-hip-yaw-limit-deg", type=float, default=11.5)
    parser.add_argument("--support-knee-limit-deg", type=float, default=14.0)
    parser.add_argument("--support-ankle-pitch-limit-deg", type=float, default=8.5)
    parser.add_argument("--support-ankle-roll-limit-deg", type=float, default=5.5)
    parser.add_argument("--balance-activation-rad", type=float, default=0.18)
    parser.add_argument("--balance-full-rad", type=float, default=0.45)
    parser.add_argument("--adaptive-stance-protection", type=float, default=0.97)
    parser.add_argument("--adaptive-waist-protection", type=float, default=0.85)
    parser.add_argument(
        "--adaptive-support-hip-pitch-limit-deg", type=float, default=5.7
    )
    parser.add_argument(
        "--adaptive-support-hip-roll-limit-deg", type=float, default=6.9
    )
    parser.add_argument(
        "--adaptive-support-hip-yaw-limit-deg", type=float, default=8.0
    )
    parser.add_argument("--adaptive-support-knee-limit-deg", type=float, default=6.9)
    parser.add_argument(
        "--adaptive-support-ankle-pitch-limit-deg", type=float, default=4.6
    )
    parser.add_argument(
        "--adaptive-support-ankle-roll-limit-deg", type=float, default=4.6
    )
    parser.add_argument(
        "--adaptive-support-hip-pitch-forward-limit-deg",
        type=float,
        default=3.4,
    )
    parser.add_argument("--adaptive-waist-roll-limit-deg", type=float, default=5.7)
    parser.add_argument("--adaptive-waist-pitch-limit-deg", type=float, default=2.9)
    parser.add_argument(
        "--support-hip-pitch-counter-bias-deg", type=float, default=4.6
    )
    parser.add_argument("--support-knee-counter-bias-deg", type=float, default=5.7)
    parser.add_argument(
        "--support-ankle-pitch-counter-bias-deg", type=float, default=2.9
    )
    parser.add_argument("--waist-pitch-counter-bias-deg", type=float, default=2.3)
    parser.add_argument("--swing-hip-pitch-limit-deg", type=float, default=37.0)
    parser.add_argument("--swing-hip-roll-limit-deg", type=float, default=23.0)
    parser.add_argument("--swing-hip-yaw-limit-deg", type=float, default=34.0)
    parser.add_argument("--swing-ankle-pitch-limit-deg", type=float, default=20.0)
    parser.add_argument("--swing-ankle-roll-limit-deg", type=float, default=10.0)
    parser.add_argument("--waist-roll-limit-deg", type=float, default=13.0)
    parser.add_argument("--waist-pitch-limit-deg", type=float, default=13.0)
    parser.add_argument("--lower-body-slew-rad-s", type=float, default=4.0)
    parser.add_argument("--maximum-dt-s", type=float, default=0.04)
    return parser.parse_args()


def unpack_fields(message: bytes, topic: bytes) -> tuple[int, dict[str, np.ndarray]]:
    if not message.startswith(topic):
        raise ValueError("unexpected SONIC topic")
    header_start = len(topic)
    payload_start = header_start + SONIC_ZMQ_HEADER_SIZE
    header = json.loads(
        message[header_start:payload_start].rstrip(b"\x00").decode("utf-8")
    )
    fields: dict[str, np.ndarray] = {}
    offset = payload_start
    for description in header["fields"]:
        dtype = DTYPES[description["dtype"]]
        shape = tuple(description["shape"])
        count = int(np.prod(shape, dtype=np.int64))
        size = count * dtype.itemsize
        fields[description["name"]] = np.frombuffer(
            message, dtype=dtype, count=count, offset=offset
        ).reshape(shape).copy()
        offset += size
    return int(header["v"]), fields


def isaaclab_to_mujoco(values: np.ndarray) -> np.ndarray:
    array = np.asarray(values)
    if array.shape[-1] != 29:
        raise ValueError("SONIC joint values must end with 29 elements")
    result = np.empty_like(array)
    result[..., G1_MUJOCO_TO_ISAACLAB] = array
    return result


def mujoco_to_isaaclab(values: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(values[..., G1_MUJOCO_TO_ISAACLAB])


def stabilize_message(
    message: bytes,
    topic: bytes,
    governor: KickStabilityGovernor,
    now: float,
) -> tuple[bytes, object]:
    version, fields = unpack_fields(message, topic)
    if version != 1:
        raise ValueError(f"unsupported SONIC protocol version: {version}")
    position = isaaclab_to_mujoco(fields["joint_pos"][-1])
    velocity = isaaclab_to_mujoco(fields["joint_vel"][-1])
    stabilized = governor.update(now, position, velocity)
    fields["joint_pos"][-1] = mujoco_to_isaaclab(stabilized.joint_pos)
    fields["joint_vel"][-1] = mujoco_to_isaaclab(stabilized.joint_vel)
    return (
        pack_sonic_pose_message(fields, topic=topic.decode("utf-8"), version=1),
        stabilized,
    )


def main() -> int:
    args = parse_args()
    if args.stale_ms <= 0.0:
        raise ValueError("stale-ms must be positive")
    limits = KickStabilityLimits(
        lift_on_score_rad=args.lift_on_rad,
        lift_off_score_rad=args.lift_off_rad,
        side_margin_rad=args.side_margin_rad,
        phase_dwell_s=args.phase_dwell_s,
        stance_protection=args.stance_protection,
        waist_protection=args.waist_protection,
        support_hip_pitch_limit_rad=math.radians(args.support_hip_pitch_limit_deg),
        support_hip_roll_limit_rad=math.radians(args.support_hip_roll_limit_deg),
        support_hip_yaw_limit_rad=math.radians(args.support_hip_yaw_limit_deg),
        support_knee_limit_rad=math.radians(args.support_knee_limit_deg),
        support_ankle_pitch_limit_rad=math.radians(args.support_ankle_pitch_limit_deg),
        support_ankle_roll_limit_rad=math.radians(args.support_ankle_roll_limit_deg),
        balance_activation_score_rad=args.balance_activation_rad,
        balance_full_score_rad=args.balance_full_rad,
        adaptive_stance_protection=args.adaptive_stance_protection,
        adaptive_waist_protection=args.adaptive_waist_protection,
        adaptive_support_hip_pitch_limit_rad=math.radians(
            args.adaptive_support_hip_pitch_limit_deg
        ),
        adaptive_support_hip_roll_limit_rad=math.radians(
            args.adaptive_support_hip_roll_limit_deg
        ),
        adaptive_support_hip_yaw_limit_rad=math.radians(
            args.adaptive_support_hip_yaw_limit_deg
        ),
        adaptive_support_knee_limit_rad=math.radians(
            args.adaptive_support_knee_limit_deg
        ),
        adaptive_support_ankle_pitch_limit_rad=math.radians(
            args.adaptive_support_ankle_pitch_limit_deg
        ),
        adaptive_support_ankle_roll_limit_rad=math.radians(
            args.adaptive_support_ankle_roll_limit_deg
        ),
        adaptive_support_hip_pitch_forward_limit_rad=math.radians(
            args.adaptive_support_hip_pitch_forward_limit_deg
        ),
        adaptive_waist_roll_limit_rad=math.radians(
            args.adaptive_waist_roll_limit_deg
        ),
        adaptive_waist_pitch_limit_rad=math.radians(
            args.adaptive_waist_pitch_limit_deg
        ),
        support_hip_pitch_counter_bias_rad=math.radians(
            args.support_hip_pitch_counter_bias_deg
        ),
        support_knee_counter_bias_rad=math.radians(
            args.support_knee_counter_bias_deg
        ),
        support_ankle_pitch_counter_bias_rad=math.radians(
            args.support_ankle_pitch_counter_bias_deg
        ),
        waist_pitch_counter_bias_rad=math.radians(
            args.waist_pitch_counter_bias_deg
        ),
        swing_hip_pitch_limit_rad=math.radians(args.swing_hip_pitch_limit_deg),
        swing_hip_roll_limit_rad=math.radians(args.swing_hip_roll_limit_deg),
        swing_hip_yaw_limit_rad=math.radians(args.swing_hip_yaw_limit_deg),
        swing_ankle_pitch_limit_rad=math.radians(args.swing_ankle_pitch_limit_deg),
        swing_ankle_roll_limit_rad=math.radians(args.swing_ankle_roll_limit_deg),
        waist_roll_limit_rad=math.radians(args.waist_roll_limit_deg),
        waist_pitch_limit_rad=math.radians(args.waist_pitch_limit_deg),
        lower_body_slew_rad_s=args.lower_body_slew_rad_s,
        maximum_dt_s=args.maximum_dt_s,
    )
    import zmq

    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.RCVHWM, 1)
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, args.topic)
    subscriber.connect(args.input)
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 1)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.output)
    poller = zmq.Poller()
    poller.register(subscriber, zmq.POLLIN)
    governor = KickStabilityGovernor(limits)
    topic = args.topic.encode("utf-8")
    stale_seconds = args.stale_ms / 1000.0
    last_message_at: float | None = None
    last_phase = None
    sent = 0
    started = time.monotonic()
    log.info("EXPERIMENTAL kick stability: %s -> %s", args.input, args.output)
    log.info(
        "limits: stance=%.2f waist=%.2f adaptive_stance=%.2f adaptive_waist=%.2f "
        "support_pitch=%.1f adaptive_support_pitch=%.1f adaptive_forward_pitch=%.1f "
        "support_knee=%.1f adaptive_support_knee=%.1f swing_pitch=%.1f "
        "swing_roll=%.1f swing_yaw=%.1f ankle_pitch=%.1f ankle_roll=%.1f "
        "adaptive_waist_pitch=%.1f support_counter_pitch=%.1f "
        "support_counter_knee=%.1f support_counter_ankle=%.1f "
        "waist_counter_pitch=%.1f",
        limits.stance_protection,
        limits.waist_protection,
        limits.adaptive_stance_protection,
        limits.adaptive_waist_protection,
        math.degrees(limits.support_hip_pitch_limit_rad),
        math.degrees(limits.adaptive_support_hip_pitch_limit_rad),
        math.degrees(limits.adaptive_support_hip_pitch_forward_limit_rad),
        math.degrees(limits.support_knee_limit_rad),
        math.degrees(limits.adaptive_support_knee_limit_rad),
        math.degrees(limits.swing_hip_pitch_limit_rad),
        math.degrees(limits.swing_hip_roll_limit_rad),
        math.degrees(limits.swing_hip_yaw_limit_rad),
        math.degrees(limits.swing_ankle_pitch_limit_rad),
        math.degrees(limits.swing_ankle_roll_limit_rad),
        math.degrees(limits.adaptive_waist_pitch_limit_rad),
        math.degrees(limits.support_hip_pitch_counter_bias_rad),
        math.degrees(limits.support_knee_counter_bias_rad),
        math.degrees(limits.support_ankle_pitch_counter_bias_rad),
        math.degrees(limits.waist_pitch_counter_bias_rad),
    )

    try:
        while True:
            events = dict(poller.poll(100))
            now = time.monotonic()
            if subscriber not in events:
                if last_message_at is not None and now - last_message_at > stale_seconds:
                    governor.reset()
                    last_message_at = None
                    last_phase = None
                    log.info("Input STALE; governor reset and output paused")
                continue
            message = subscriber.recv()
            if last_message_at is None:
                log.info("Input LIVE")
            last_message_at = now
            output, status = stabilize_message(message, topic, governor, now)
            try:
                publisher.send(output, flags=zmq.NOBLOCK)
                sent += 1
            except zmq.Again:
                continue
            if status.phase is not last_phase:
                log.info(
                    "phase=%s support=%s left_score=%.3f right_score=%.3f",
                    status.phase.value,
                    status.support_side or "both",
                    status.left_score_rad,
                    status.right_score_rad,
                )
                last_phase = status.phase
            if sent % 50 == 0:
                elapsed = max(now - started, 1e-9)
                log.info("sent=%d average_rate=%.1fHz", sent, sent / elapsed)
    except KeyboardInterrupt:
        return 0
    finally:
        subscriber.close(linger=0)
        publisher.close(linger=0)
        context.term()


if __name__ == "__main__":
    raise SystemExit(main())
