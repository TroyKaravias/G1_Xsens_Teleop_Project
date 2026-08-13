#!/usr/bin/env python3
"""Publish a conservative scripted locomotion demo to SONIC simulation.

This is a ZMQ reference publisher only. Run SONIC with ``zmq_manager sim``.
Do not use this script with a physical robot.
"""

from __future__ import annotations

import argparse
import math
import time

from xsens_bridge.sonic_manager import (
    SonicLocomotionMode,
    pack_command_message,
    pack_planner_message,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="tcp://*:5556")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--forward-speed", type=float, default=0.20)
    parser.add_argument("--backward-speed", type=float, default=0.35)
    parser.add_argument("--turn-speed", type=float, default=0.20)
    parser.add_argument("--turn-yaw-rate", type=float, default=0.80)
    parser.add_argument("--forward-seconds", type=float, default=8.0)
    parser.add_argument("--backward-seconds", type=float, default=5.0)
    parser.add_argument("--turn-seconds", type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.fps <= 0:
        raise ValueError("fps must be positive")
    try:
        import zmq
    except ImportError as error:
        raise SystemExit("Install pyzmq first") from error

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.bind)
    print(f"SONIC simulation demo publishing on {args.bind}")
    print("Sequence: idle, forward, idle, backward, idle, left turn, idle")
    time.sleep(1.0)
    socket.send(pack_command_message(start=True, stop=False, planner=True))

    segments = [
        ("idle", 1.5, 0.0, 0.0),
        ("forward", args.forward_seconds, args.forward_speed, 0.0),
        ("idle", 1.0, 0.0, 0.0),
        ("backward", args.backward_seconds, -args.backward_speed, 0.0),
        ("idle", 1.0, 0.0, 0.0),
        ("turn-left", args.turn_seconds, args.turn_speed, args.turn_yaw_rate),
        ("idle", 1.5, 0.0, 0.0),
    ]
    heading = 0.0
    period = 1.0 / args.fps
    for name, duration, velocity, yaw_rate in segments:
        print(name)
        ticks = round(duration * args.fps)
        for _ in range(ticks):
            heading += yaw_rate * period
            facing = (math.cos(heading), math.sin(heading), 0.0)
            direction = 0.0 if velocity == 0.0 else math.copysign(1.0, velocity)
            movement = (
                direction * facing[0],
                direction * facing[1],
                0.0,
            )
            socket.send(
                pack_planner_message(
                    mode=(
                        SonicLocomotionMode.SLOW_WALK
                        if velocity != 0.0 or yaw_rate != 0.0
                        else SonicLocomotionMode.IDLE
                    ),
                    movement=movement,
                    facing=facing,
                    speed=(
                        abs(velocity)
                        if velocity != 0.0
                        else (0.2 if yaw_rate != 0.0 else -1.0)
                    ),
                )
            )
            time.sleep(period)
    socket.send(pack_command_message(start=False, stop=True, planner=True))
    print("Demo complete; STOP sent.")
    socket.close(linger=0)
    context.term()


if __name__ == "__main__":
    main()
