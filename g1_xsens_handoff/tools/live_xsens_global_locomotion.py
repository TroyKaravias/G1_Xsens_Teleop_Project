#!/usr/bin/env python3
"""Publish safety-bounded global Xsens pelvis following to SONIC.

This process emits SONIC ``zmq_manager`` planner messages only.  Xsens global
pelvis X/Y displacement controls walking direction, calibrated pelvis yaw
controls global facing, and the planner owns the balance-critical leg gait.
Xsens continues to provide bounded waist/arm targets through SONIC's supported
17-DOF upper-body override.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from xsens_bridge.global_pelvis import GlobalPelvisFollower, GlobalPelvisLimits
from xsens_bridge.live_retarget import OnlineG1Retargeter
from xsens_bridge.sonic_manager import (
    SonicLocomotionMode,
    pack_command_message,
    pack_planner_message,
    sonic_upper_body_from_isaaclab,
)
from xsens_bridge.sonic_zmq import mujoco_to_isaaclab
from xsens_bridge.stream import LatestPoseReceiver
from tools.live_xsens_sonic import capture_calibration


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--xsens-port", type=int, default=9763)
    parser.add_argument("--zmq-bind", default="tcp://127.0.0.1:5556")
    parser.add_argument("--calibration-seconds", type=float, default=15.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--stale-ms", type=float, default=300.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--upper-max-speed", type=float, default=4.0)
    parser.add_argument("--engage-distance-m", type=float, default=0.08)
    parser.add_argument("--release-distance-m", type=float, default=0.04)
    parser.add_argument("--position-gain", type=float, default=1.20)
    parser.add_argument("--minimum-speed-mps", type=float, default=0.20)
    parser.add_argument("--maximum-speed-mps", type=float, default=0.20)
    parser.add_argument("--maximum-sample-jump-m", type=float, default=0.20)
    parser.add_argument("--maximum-excursion-m", type=float, default=2.0)
    parser.add_argument("--heading-gain", type=float, default=1.0)
    parser.add_argument("--heading-deadband-deg", type=float, default=3.0)
    parser.add_argument("--heading-max-rate-deg-s", type=float, default=30.0)
    parser.add_argument("--torso-gain", type=float, default=0.50)
    parser.add_argument("--waist-blend", type=float, default=0.25)
    parser.add_argument("--neutral-shoulder-bias-deg", type=float, default=6.0)
    parser.add_argument("--reach-shoulder-bias-deg", type=float, default=11.0)
    return parser.parse_args()


def pelvis_pose(frame):
    for segment in frame.segments:
        if segment.name == "pelvis":
            return (
                np.asarray(segment.position_m, dtype=np.float64),
                np.asarray(segment.quaternion_wxyz, dtype=np.float64),
            )
    raise ValueError("Xsens frame is missing pelvis")


def planner_packet(command, upper_body):
    velocity = np.asarray(
        (command.velocity_x_mps, command.velocity_y_mps), dtype=np.float64
    )
    speed = float(np.linalg.norm(velocity))
    moving = speed > 1e-6
    movement = (
        (float(velocity[0] / speed), float(velocity[1] / speed), 0.0)
        if moving
        else (0.0, 0.0, 0.0)
    )
    facing = (
        math.cos(command.heading_rad),
        math.sin(command.heading_rad),
        0.0,
    )
    return pack_planner_message(
        mode=(SonicLocomotionMode.SLOW_WALK if moving else SonicLocomotionMode.IDLE),
        movement=movement,
        facing=facing,
        speed=speed if moving else -1.0,
        height=-1.0,
        upper_body_position=upper_body,
    )


def send_burst(publisher, message, *, count=20, interval_s=0.025):
    for _ in range(count):
        publisher.send(message)
        time.sleep(interval_s)


def main():
    args = parse_args()
    if args.fps <= 0.0 or args.upper_max_speed <= 0.0:
        raise ValueError("fps and upper-body speed must be positive")
    if not 0.0 <= args.waist_blend <= 0.35:
        raise ValueError("waist blend must be within [0, 0.35]")

    limits = GlobalPelvisLimits(
        engage_distance_m=args.engage_distance_m,
        release_distance_m=args.release_distance_m,
        position_gain_s=args.position_gain,
        minimum_speed_mps=args.minimum_speed_mps,
        maximum_speed_mps=args.maximum_speed_mps,
        maximum_sample_jump_m=args.maximum_sample_jump_m,
        maximum_excursion_m=args.maximum_excursion_m,
        heading_gain=args.heading_gain,
        heading_deadband_rad=math.radians(args.heading_deadband_deg),
        heading_max_rate_rad_s=math.radians(args.heading_max_rate_deg_s),
    )

    try:
        import zmq
    except ImportError as error:
        raise SystemExit("pyzmq is required") from error

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 1)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.zmq_bind)
    period = 1.0 / args.fps
    stale_seconds = args.stale_ms / 1000.0
    follower = GlobalPelvisFollower(limits)
    started = False

    try:
        with LatestPoseReceiver(
            args.bind,
            args.xsens_port,
            counter_reset_after_seconds=stale_seconds,
        ) as receiver:
            print(f"Xsens UDP {args.bind}:{args.xsens_port}")
            print(f"SONIC planner {args.zmq_bind}")
            calibration = capture_calibration(
                receiver,
                args.calibration_seconds,
                args.wait_timeout,
                "ntf",
            )
            retargeter = OnlineG1Retargeter(
                calibration,
                motion_gain=1.0,
                torso_gain=args.torso_gain,
                neutral_shoulder_bias_deg=args.neutral_shoulder_bias_deg,
                reach_shoulder_bias_deg=args.reach_shoulder_bias_deg,
                max_position_speed=args.upper_max_speed,
                max_joint_speed=12.0,
                track_wrists=False,
            )
            confirmation = input(
                "Robot harnessed; E-stop operator ready; SONIC says Init Done. "
                "Stand centered and still, then type ARM: "
            )
            if confirmation.strip() != "ARM":
                raise SystemExit("Not armed; exact confirmation ARM was not entered")

            # Allow subscriber connections to settle, then send both IDLE and
            # START repeatedly.  This avoids the PUB/SUB slow-joiner loss that
            # made a one-shot network activation unreliable.
            time.sleep(0.75)
            latest = receiver.latest()
            if latest is None or time.monotonic() - latest.received_at > stale_seconds:
                raise RuntimeError("Xsens stream is stale at arming")
            position, quaternion = pelvis_pose(latest.frame)
            follower.arm(position, quaternion, latest.received_at)
            pose = retargeter.retarget(latest.frame, latest.received_at)
            upper = sonic_upper_body_from_isaaclab(
                mujoco_to_isaaclab(pose.dof_pos)
            )
            upper[:3] *= args.waist_blend
            idle = follower.update(position, quaternion, latest.received_at + 1e-6)
            send_burst(publisher, planner_packet(idle, upper), count=10)
            send_burst(
                publisher,
                pack_command_message(start=True, stop=False, planner=True),
            )
            started = True
            print("SONIC START sent in PLANNER mode; Enter is not required.")
            print("Global pelvis XY + yaw following ARMED. Ctrl+C stops publishing.")

            upper_command = upper.copy()
            last_counter = latest.frame.header.sample_counter
            last_state = "LIVE"
            last_label = "pelvis-idle"
            last_log = time.monotonic()
            next_tick = time.monotonic()
            command = idle
            while True:
                now = time.monotonic()
                latest = receiver.latest()
                fresh = latest is not None and now - latest.received_at <= stale_seconds
                state = "LIVE" if fresh else "STALE-IDLE"
                if state != last_state:
                    print(f"Stream state: {state}")
                    last_state = state
                if fresh and latest is not None:
                    counter = latest.frame.header.sample_counter
                    if counter != last_counter:
                        position, quaternion = pelvis_pose(latest.frame)
                        if last_state == "LIVE":
                            command = follower.update(
                                position, quaternion, latest.received_at
                            )
                        pose = retargeter.retarget(
                            latest.frame, latest.received_at
                        )
                        upper_target = sonic_upper_body_from_isaaclab(
                            mujoco_to_isaaclab(pose.dof_pos)
                        )
                        upper_target[:3] *= args.waist_blend
                        maximum_step = args.upper_max_speed * period
                        upper_command += np.clip(
                            upper_target - upper_command,
                            -maximum_step,
                            maximum_step,
                        )
                        last_counter = counter
                else:
                    follower.stop()
                    command = type(command)(
                        heading_rad=command.heading_rad,
                        estimated_x_m=command.estimated_x_m,
                        estimated_y_m=command.estimated_y_m,
                        label="pelvis-stale-idle",
                    )
                publisher.send(planner_packet(command, upper_command))
                if command.label != last_label or now - last_log >= 1.0:
                    print(
                        f"{command.label} desired=({command.desired_x_m:+.2f},"
                        f"{command.desired_y_m:+.2f})m estimate=("
                        f"{command.estimated_x_m:+.2f},{command.estimated_y_m:+.2f})m "
                        f"velocity=({command.velocity_x_mps:+.2f},"
                        f"{command.velocity_y_mps:+.2f})m/s "
                        f"heading={math.degrees(command.heading_rad):+.1f}deg"
                    )
                    last_label = command.label
                    last_log = now
                if command.faulted:
                    raise RuntimeError(
                        f"Pelvis safety latch: {command.label}; restart and recenter"
                    )
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                elif -delay > period:
                    next_tick = time.monotonic()
    finally:
        if started:
            try:
                stopped = GlobalPelvisFollower(limits)
                stopped.arm(np.zeros(3), np.asarray((1.0, 0.0, 0.0, 0.0)), 0.0)
                command = stopped.update(
                    np.zeros(3), np.asarray((1.0, 0.0, 0.0, 0.0)), 1e-6
                )
                send_burst(publisher, planner_packet(command, None), count=10)
            except Exception:
                pass
        publisher.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
