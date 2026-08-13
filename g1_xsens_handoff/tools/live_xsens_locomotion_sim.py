#!/usr/bin/env python3
"""Blend live Xsens upper-body tracking with SONIC locomotion references.

Simulation only: this publisher contains no Unitree SDK, DDS, ROS, or motor
command output. Physical use is permitted only with ``--physical-constrained``
and an independently configured, restrained SONIC controller.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import math
import select
import sys
import termios
import time
import tty

import numpy as np

from xsens_bridge.body_motion import BodyMotionController
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
    parser.add_argument("--zmq-bind", default="tcp://*:5556")
    parser.add_argument("--calibration-seconds", type=float, default=12.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--stale-ms", type=float, default=250.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument(
        "--upper-max-speed",
        type=float,
        default=1.5,
        help="Maximum commanded upper-joint slew rate in rad/s.",
    )
    parser.add_argument(
        "--fencing-upper-max-speed",
        type=float,
        default=4.0,
        help="Upper-joint slew limit used while fencing mode is selected.",
    )
    parser.add_argument(
        "--arms-only",
        action="store_true",
        help="Track arms in stationary IDLE indefinitely; no locomotion.",
    )
    parser.add_argument(
        "--manual-locomotion",
        action="store_true",
        help="Enable bounded w/s/a/d locomotion pulses while tracking arms.",
    )
    parser.add_argument(
        "--body-motion",
        action="store_true",
        help=(
            "Map Xsens pelvis translation and pelvis-relative head yaw to "
            "bounded locomotion/fencing footwork."
        ),
    )
    parser.add_argument(
        "--head-turn-rate-deadband",
        type=float,
        default=0.20,
        help="Ignore pelvis-relative head yaw rates below this rad/s value.",
    )
    parser.add_argument("--head-turn-gain", type=float, default=0.50)
    parser.add_argument("--head-turn-max-rate", type=float, default=0.50)
    parser.add_argument(
        "--track-wrists",
        action="store_true",
        help="Track palm-up/down wrist roll only; wrist pitch/yaw stay neutral.",
    )
    parser.add_argument(
        "--physical-constrained",
        action="store_true",
        help=(
            "Require explicit start confirmation, disable automatic body "
            "gesture locomotion, and cap manual commands to one slow, short "
            "pulse for a supported physical validation."
        ),
    )
    parser.add_argument("--neutral-shoulder-bias-deg", type=float, default=6.0)
    parser.add_argument("--reach-shoulder-bias-deg", type=float, default=11.0)
    return parser.parse_args()


@contextmanager
def raw_keyboard(enabled: bool):
    """Temporarily enable single-key terminal commands."""
    if not enabled:
        yield
        return
    if not sys.stdin.isatty():
        raise RuntimeError("manual locomotion requires an interactive terminal")
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def read_key() -> str | None:
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    return sys.stdin.read(1).lower() if readable else None


def planner_packet(
    velocity: float,
    lateral_velocity: float,
    yaw_rate: float,
    heading: float,
    upper_body,
    *,
    fencing_guarded: bool = False,
    target_height: float = -1.0,
):
    facing = (math.cos(heading), math.sin(heading), 0.0)
    direction = 0.0 if velocity == 0.0 else math.copysign(1.0, velocity)
    if velocity == 0.0 and yaw_rate != 0.0:
        # A facing-only request twists the trunk in place. A minimum slow
        # forward component makes SONIC reposition the feet in a shallow arc.
        direction = 1.0
    left = (-facing[1], facing[0])
    movement_norm = math.hypot(velocity, lateral_velocity)
    movement = (
        (velocity * facing[0] + lateral_velocity * left[0])
        / max(movement_norm, 1e-8),
        (velocity * facing[1] + lateral_velocity * left[1])
        / max(movement_norm, 1e-8),
        0.0,
    )
    if movement_norm == 0.0 and yaw_rate != 0.0:
        movement = (facing[0], facing[1], 0.0)
    moving = movement_norm != 0.0 or yaw_rate != 0.0
    if fencing_guarded and not moving:
        locomotion_mode = SonicLocomotionMode.IDLE_BOXING
    elif moving:
        locomotion_mode = SonicLocomotionMode.SLOW_WALK
    else:
        locomotion_mode = SonicLocomotionMode.IDLE
    return pack_planner_message(
        mode=locomotion_mode,
        movement=movement,
        facing=facing,
        speed=movement_norm if movement_norm != 0.0 else (-1.0 if not moving else 0.2),
        height=target_height,
        upper_body_position=upper_body,
    )


def main():
    args = parse_args()
    if (
        args.fps <= 0
        or args.upper_max_speed <= 0
        or args.fencing_upper_max_speed <= 0
        or args.head_turn_rate_deadband < 0
        or args.head_turn_gain <= 0
        or args.head_turn_max_rate <= 0
    ):
        raise ValueError("fps, speed limits, and head-turn limits are invalid")
    if args.physical_constrained:
        if not args.manual_locomotion:
            raise ValueError(
                "--physical-constrained requires --manual-locomotion"
            )
        if args.body_motion:
            raise ValueError(
                "--physical-constrained cannot use --body-motion"
            )
    try:
        import zmq
    except ImportError as error:
        raise SystemExit("Install pyzmq first") from error

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 1)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.zmq_bind)
    period = 1.0 / args.fps
    stale_seconds = args.stale_ms / 1000.0
    segments = [
        ("idle", 2.0, 0.0, 0.0),
        ("forward", 8.0, 0.20, 0.0),
        ("idle", 1.5, 0.0, 0.0),
        ("backward", 5.0, -0.35, 0.0),
        ("idle", 1.5, 0.0, 0.0),
        ("turn-left", 2.0, 0.20, 0.80),
        ("idle", 2.0, 0.0, 0.0),
    ]

    try:
        with LatestPoseReceiver(args.bind, args.xsens_port) as receiver:
            print(f"Xsens UDP {args.bind}:{args.xsens_port}")
            print(f"SONIC manager {args.zmq_bind} (SIMULATION ONLY)")
            calibration = capture_calibration(
                receiver,
                args.calibration_seconds,
                args.wait_timeout,
                "ntf",
            )
            retargeter = OnlineG1Retargeter(
                calibration,
                motion_gain=1.0,
                neutral_shoulder_bias_deg=args.neutral_shoulder_bias_deg,
                reach_shoulder_bias_deg=args.reach_shoulder_bias_deg,
                track_wrists=args.track_wrists,
            )
            body_controller = BodyMotionController(
                head_turn_rate_deadband_rad_s=(
                    args.head_turn_rate_deadband
                ),
                head_turn_gain=args.head_turn_gain,
                head_turn_max_rate_rad_s=args.head_turn_max_rate,
            )
            print("Calibration complete. Connecting publisher...")
            time.sleep(1.0)
            if args.physical_constrained:
                input(
                    "PHYSICAL CONSTRAINED TEST: robot supported, E-stop "
                    "operator ready, SONIC initialized but not engaged. "
                    "Press ENTER to publish stationary planner IDLE."
                )
            elif args.arms_only:
                input(
                    "READY: press ENTER to start stationary arm tracking. "
                    "The robot will not walk."
                )
            publisher.send(pack_command_message(
                start=True, stop=False, planner=True
            ))
            print("START sent. Click MuJoCo and press 9 once when ready.")
            heading = 0.0
            last_counter = None
            upper_body_target = None
            upper_body_command = None
            state = None
            stale_started = None
            next_tick = time.monotonic()
            active_segments = (
                [("arms-only idle", float("inf"), 0.0, 0.0)]
                if args.arms_only or args.manual_locomotion
                else segments
            )
            if args.manual_locomotion:
                print(
                    "Modes: 1=locomotion, 2=fencing"
                )
                print(
                    "Locomotion: w=forward, s=backward, "
                    "a=left, d=right"
                )
                print(
                    "Fencing: g=en-garde, j=advance, "
                    "k=retreat, l=shallow-lunge prototype"
                )
                print("Global: x=stop/recover, q=quit")
                print("Every movement is bounded and auto-stops.")
            demo_mode = "locomotion"
            fencing_guarded = False
            body_label = "body-neutral"
            with raw_keyboard(args.manual_locomotion):
              for name, duration, segment_velocity, segment_yaw_rate in active_segments:
                print(name)
                live_ticks = 0
                command_velocity = segment_velocity
                command_lateral_velocity = 0.0
                command_yaw_rate = segment_yaw_rate
                command_height = -1.0
                command_deadline = 0.0
                command_label = "idle"
                required_ticks = (
                    float("inf")
                    if math.isinf(duration)
                    else round(duration * args.fps)
                )
                quit_requested = False
                while live_ticks < required_ticks and not quit_requested:
                    now = time.monotonic()
                    if args.manual_locomotion:
                        key = read_key()
                        command_map = {
                            "w": (
                                "forward",
                                0.10 if args.physical_constrained else 0.20,
                                0.0,
                                0.35 if args.physical_constrained else 1.0,
                            ),
                            "s": (
                                "backward",
                                -0.10 if args.physical_constrained else -0.35,
                                0.0,
                                0.35 if args.physical_constrained else 0.8,
                            ),
                            "a": (
                                "turn-left",
                                0.10 if args.physical_constrained else 0.20,
                                0.25 if args.physical_constrained else 0.80,
                                0.35 if args.physical_constrained else 0.7,
                            ),
                            "d": (
                                "turn-right",
                                0.10 if args.physical_constrained else 0.20,
                                -0.25 if args.physical_constrained else -0.80,
                                0.35 if args.physical_constrained else 0.7,
                            ),
                        }
                        fencing_map = {
                            # SONIC needs enough time to plan and initiate a
                            # complete support-foot transfer. Shorter 0.35 s
                            # pulses expired before the first step developed.
                            "j": ("fencing advance", 0.20, 0.0, 0.90),
                            "k": ("fencing retreat", -0.30, 0.0, 0.85),
                            # This is intentionally a bounded planner step,
                            # not SONIC's bundled reference-motion lunge.
                            "l": (
                                "shallow-lunge prototype",
                                0.35,
                                0.0,
                                1.10,
                            ),
                        }
                        selected = None
                        if key == "1":
                            demo_mode = "locomotion"
                            fencing_guarded = False
                            command_velocity = 0.0
                            command_lateral_velocity = 0.0
                            command_yaw_rate = 0.0
                            command_height = -1.0
                            command_deadline = 0.0
                            command_label = "idle"
                            print("\nMODE: LOCOMOTION")
                        elif key == "2":
                            if args.physical_constrained:
                                print(
                                    "\nFencing disabled during first "
                                    "constrained physical step test"
                                )
                            else:
                                demo_mode = "fencing"
                                fencing_guarded = False
                                command_velocity = 0.0
                                command_lateral_velocity = 0.0
                                command_yaw_rate = 0.0
                                command_height = -1.0
                                command_deadline = 0.0
                                command_label = "idle"
                                print("\nMODE: FENCING—press g for en-garde")
                        elif key == "g" and demo_mode == "fencing":
                            fencing_guarded = True
                            command_velocity = 0.0
                            command_lateral_velocity = 0.0
                            command_yaw_rate = 0.0
                            command_height = -1.0
                            command_deadline = 0.0
                            command_label = "en-garde"
                            print("\nEN-GARDE—footwork enabled")
                        elif demo_mode == "locomotion" and key in command_map:
                            selected = command_map[key]
                        elif (
                            demo_mode == "fencing"
                            and fencing_guarded
                            and key in fencing_map
                        ):
                            selected = fencing_map[key]
                        elif (
                            demo_mode == "fencing"
                            and key in fencing_map
                            and not fencing_guarded
                        ):
                            print("\nIgnored: press g to enter en-garde first")
                        if selected is not None:
                            (
                                command_label,
                                command_velocity,
                                command_yaw_rate,
                                pulse,
                            ) = selected
                            command_deadline = now + pulse
                            print(f"\n{command_label} pulse")
                        elif key == "x":
                            command_velocity = 0.0
                            command_lateral_velocity = 0.0
                            command_yaw_rate = 0.0
                            command_height = -1.0
                            command_deadline = 0.0
                            command_label = (
                                "en-garde"
                                if demo_mode == "fencing" and fencing_guarded
                                else "idle"
                            )
                            print("\nstop/recover")
                        elif key == "q":
                            quit_requested = True
                            continue
                        if command_deadline and now >= command_deadline:
                            command_velocity = 0.0
                            command_lateral_velocity = 0.0
                            command_yaw_rate = 0.0
                            command_height = -1.0
                            command_deadline = 0.0
                            if command_label == "shallow-lunge prototype":
                                print("\nauto-stop; recovering to en-garde")
                            else:
                                print("\nauto-stop")
                            command_label = (
                                "en-garde"
                                if demo_mode == "fencing" and fencing_guarded
                                else "idle"
                            )
                    latest = receiver.latest()
                    fresh = (
                        latest is not None
                        and now - latest.received_at <= stale_seconds
                    )
                    new_state = "LIVE" if fresh else "STALE—LOCOMOTION PAUSED"
                    if new_state != state:
                        age_ms = (
                            float("inf")
                            if latest is None
                            else 1000.0 * (now - latest.received_at)
                        )
                        if fresh and stale_started is not None:
                            print(
                                f"LIVE—recovered after "
                                f"{now - stale_started:.3f}s; "
                                f"age={age_ms:.1f}ms "
                                f"frames={receiver.health.received_frames} "
                                f"missing={receiver.health.missing_frames} "
                                f"malformed={receiver.health.malformed_packets}"
                            )
                            stale_started = None
                        elif not fresh:
                            command_velocity = 0.0
                            command_lateral_velocity = 0.0
                            command_yaw_rate = 0.0
                            command_height = -1.0
                            command_deadline = 0.0
                            fencing_guarded = False
                            command_label = "idle"
                            body_controller.reset()
                            body_label = "body-neutral"
                            stale_started = now
                            print(
                                f"STALE—holding arms and pausing locomotion; "
                                f"age={age_ms:.1f}ms "
                                f"frames={receiver.health.received_frames} "
                                f"missing={receiver.health.missing_frames} "
                                f"malformed={receiver.health.malformed_packets}"
                            )
                        else:
                            print(
                                f"LIVE age={age_ms:.1f}ms "
                                f"frames={receiver.health.received_frames}"
                            )
                        state = new_state
                    if fresh and latest is not None:
                        counter = latest.frame.header.sample_counter
                        if counter != last_counter:
                            pose = retargeter.retarget(
                                latest.frame, latest.received_at
                            )
                            if args.body_motion and command_deadline == 0.0:
                                body_intent = body_controller.update(
                                    latest.frame,
                                    latest.received_at,
                                    fencing=(demo_mode == "fencing"),
                                )
                                body_enabled = (
                                    demo_mode == "locomotion"
                                    or (
                                        demo_mode == "fencing"
                                        and fencing_guarded
                                    )
                                )
                                command_velocity = (
                                    body_intent.forward_velocity_mps
                                    if body_enabled
                                    else 0.0
                                )
                                command_lateral_velocity = (
                                    body_intent.lateral_velocity_mps
                                    if body_enabled
                                    else 0.0
                                )
                                command_yaw_rate = (
                                    body_intent.yaw_rate_rad_s
                                    if body_enabled
                                    else 0.0
                                )
                                command_height = (
                                    body_intent.target_height_m
                                    if body_enabled
                                    else -1.0
                                )
                                if body_intent.label != body_label:
                                    body_label = body_intent.label
                                    print(f"\n{body_label}")
                            isaaclab = mujoco_to_isaaclab(pose.dof_pos)
                            upper_body_target = sonic_upper_body_from_isaaclab(
                                isaaclab
                            )
                            # SONIC's 17-value field begins with waist yaw,
                            # roll, and pitch. Keep those neutral so the
                            # locomotion planner retains a stable trunk while
                            # Xsens controls both seven-DOF arms.
                            upper_body_target[:3] = 0.0
                            last_counter = counter
                        if upper_body_target is not None:
                            if upper_body_command is None:
                                upper_body_command = upper_body_target.copy()
                            else:
                                upper_speed = (
                                    args.fencing_upper_max_speed
                                    if demo_mode == "fencing"
                                    else args.upper_max_speed
                                )
                                max_step = upper_speed * period
                                upper_body_command += np.clip(
                                    upper_body_target - upper_body_command,
                                    -max_step,
                                    max_step,
                                )
                            heading += command_yaw_rate * period
                            packet = planner_packet(
                                command_velocity,
                                command_lateral_velocity,
                                command_yaw_rate,
                                heading,
                                upper_body_command,
                                fencing_guarded=(
                                    demo_mode == "fencing"
                                    and fencing_guarded
                                ),
                                target_height=command_height,
                            )
                            try:
                                publisher.send(packet, flags=zmq.NOBLOCK)
                            except zmq.Again:
                                pass
                            live_ticks += 1
                    else:
                        # Keep sending the last commanded arm pose through a
                        # short stream outage. Omitting the upper-body field
                        # makes SONIC fall back to its motion reference, then
                        # jump back to Xsens when packets resume.
                        publisher.send(
                            planner_packet(
                                0.0,
                                0.0,
                                0.0,
                                heading,
                                upper_body_command,
                                fencing_guarded=False,
                                target_height=-1.0,
                            ),
                            flags=zmq.NOBLOCK,
                        )
                    next_tick += period
                    delay = next_tick - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    elif -delay > period:
                        next_tick = time.monotonic()
            print("Combined simulation demo complete.")
    finally:
        try:
            publisher.send(pack_command_message(
                start=False, stop=True, planner=True
            ))
            time.sleep(0.05)
        except Exception:
            pass
        publisher.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
