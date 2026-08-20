#!/usr/bin/env python3
"""Xsens arms + global pelvis displacement through SONIC planner mode."""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from xsens_bridge.global_position import (
    GlobalPositionCommand,
    GlobalPositionController,
    SquatCommand,
    SquatController,
)
from xsens_bridge.g1_retarget_working_20260820 import G1_DEFAULT_POSE
from xsens_bridge.live_retarget_working_20260820 import OnlineG1Retargeter
from xsens_bridge.sonic_manager import (
    SonicLocomotionMode,
    pack_planner_message,
    sonic_upper_body_from_isaaclab,
)
from xsens_bridge.sonic_zmq import mujoco_to_isaaclab
from xsens_bridge.stream import LatestPoseReceiver
from tools.live_xsens_sonic import capture_calibration, relative_pelvis_yaw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--xsens-port", type=int, default=9763)
    parser.add_argument("--zmq-bind", default="tcp://127.0.0.1:5556")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--stale-ms", type=float, default=250.0)
    parser.add_argument("--calibration-seconds", type=float, default=15.0)
    parser.add_argument("--velocity-deadzone-mps", type=float, default=0.015)
    parser.add_argument("--translation-gain", type=float, default=1.0)
    parser.add_argument("--forward-gain", type=float, default=1.50)
    parser.add_argument("--backward-gain", type=float, default=1.70)
    parser.add_argument("--left-gain", type=float, default=1.20)
    parser.add_argument("--right-gain", type=float, default=1.15)
    parser.add_argument("--max-forward-mps", type=float, default=0.50)
    parser.add_argument("--max-backward-mps", type=float, default=0.45)
    parser.add_argument("--max-lateral-mps", type=float, default=0.40)
    parser.add_argument("--max-left-mps", type=float, default=0.35)
    parser.add_argument("--max-right-mps", type=float, default=0.45)
    parser.add_argument("--position-filter-alpha", type=float, default=0.70)
    parser.add_argument("--command-hold-s", type=float, default=0.18)
    parser.add_argument("--enable-squat", type=int, choices=(0, 1), default=1)
    parser.add_argument("--squat-enter-drop-m", type=float, default=0.08)
    parser.add_argument("--squat-exit-drop-m", type=float, default=0.04)
    parser.add_argument("--squat-full-drop-m", type=float, default=0.24)
    parser.add_argument("--squat-min-height-m", type=float, default=0.52)
    parser.add_argument(
        "--squat-horizontal-limit-mps", type=float, default=0.40
    )
    parser.add_argument("--arm-max-speed", type=float, default=4.0)
    parser.add_argument("--elbow-plane-yaw-gain", type=float, default=1.00)
    parser.add_argument("--wrist-roll-axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--wrist-roll-gain", type=float, default=2.60)
    parser.add_argument("--wrist-roll-limit-deg", type=float, default=45.0)
    return parser.parse_args()


def pelvis_segment(frame):
    return next(segment for segment in frame.segments if segment.name == "pelvis")


def planner_message(
    command,
    heading: float,
    upper_body: np.ndarray,
    squat: SquatCommand | None = None,
) -> bytes:
    facing = (math.cos(heading), math.sin(heading), 0.0)
    left = (-facing[1], facing[0])
    speed = math.hypot(command.forward_mps, command.lateral_mps)
    if squat is not None and squat.active:
        movement = (0.0, 0.0, 0.0)
        mode = 4  # LocomotionMode::IDEL_SQUAT in installed SONIC.
        speed = 0.0
        height = squat.height_m
    elif speed > 0.0:
        movement = (
            (command.forward_mps * facing[0] + command.lateral_mps * left[0]) / speed,
            (command.forward_mps * facing[1] + command.lateral_mps * left[1]) / speed,
            0.0,
        )
        mode = SonicLocomotionMode.SLOW_WALK
        height = -1.0
    else:
        movement = (0.0, 0.0, 0.0)
        mode = SonicLocomotionMode.IDLE
        height = -1.0
    return pack_planner_message(
        mode=mode,
        movement=movement,
        facing=facing,
        speed=speed if speed > 0.0 else -1.0,
        height=height,
        upper_body_position=upper_body,
    )


def main() -> int:
    args = parse_args()
    import zmq

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 1)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.zmq_bind)
    period = 1.0 / args.fps
    stale_s = args.stale_ms / 1000.0
    try:
        with LatestPoseReceiver(args.bind, args.xsens_port) as receiver:
            print("GLOBAL POSITION TELEOP: one-frame arms + SONIC planner legs")
            calibration = capture_calibration(
                receiver, args.calibration_seconds, 30.0, "ntf"
            )
            retargeter = OnlineG1Retargeter(
                calibration,
                motion_gain=1.0,
                max_position_speed=args.arm_max_speed,
                elbow_plane_yaw_gain=args.elbow_plane_yaw_gain,
                wrist_roll_axis=args.wrist_roll_axis,
                wrist_roll_gain=args.wrist_roll_gain,
                wrist_roll_limit_deg=args.wrist_roll_limit_deg,
            )
            controller = GlobalPositionController(
                velocity_deadzone_mps=args.velocity_deadzone_mps,
                translation_gain=args.translation_gain,
                forward_gain=args.forward_gain,
                backward_gain=args.backward_gain,
                left_gain=args.left_gain,
                right_gain=args.right_gain,
                max_forward_mps=args.max_forward_mps,
                max_backward_mps=args.max_backward_mps,
                max_lateral_mps=args.max_lateral_mps,
                max_left_mps=args.max_left_mps,
                max_right_mps=args.max_right_mps,
                filter_alpha=args.position_filter_alpha,
                command_hold_s=args.command_hold_s,
            )
            squat_controller = SquatController(
                calibration.positions["pelvis"][2],
                calibration.positions["left_foot"][2],
                calibration.positions["right_foot"][2],
                enter_drop_m=args.squat_enter_drop_m,
                exit_drop_m=args.squat_exit_drop_m,
                full_drop_m=args.squat_full_drop_m,
                minimum_height_m=args.squat_min_height_m,
                horizontal_speed_limit_mps=args.squat_horizontal_limit_mps,
            )
            time.sleep(1.0)
            neutral_upper = sonic_upper_body_from_isaaclab(
                mujoco_to_isaaclab(G1_DEFAULT_POSE)
            )
            print("PUBLISHING: use ] then Enter in the SONIC terminal")
            last_frame_time = None
            last_command = GlobalPositionCommand(0.0, 0.0, (0.0, 0.0))
            last_upper = neutral_upper
            last_heading = 0.0
            last_squat = SquatCommand(False, 0.80, 0.0)
            last_live = False
            next_tick = time.monotonic()
            sent = 0
            while True:
                now = time.monotonic()
                latest = receiver.latest()
                fresh = latest is not None and now - latest.received_at <= stale_s
                if fresh and latest is not None:
                    if (
                        last_frame_time is None
                        or latest.received_at > last_frame_time
                    ):
                        pelvis = pelvis_segment(latest.frame)
                        pose = retargeter.retarget(
                            latest.frame, latest.received_at
                        )
                        last_upper = sonic_upper_body_from_isaaclab(
                            mujoco_to_isaaclab(pose.dof_pos)
                        )
                        last_command = controller.update(
                            np.asarray(pelvis.position_m),
                            np.asarray(pelvis.quaternion_wxyz),
                            latest.received_at,
                        )
                        segments = {
                            segment.name: segment for segment in latest.frame.segments
                        }
                        last_squat = squat_controller.update(
                            pelvis.position_m[2],
                            segments["left_foot"].position_m[2],
                            segments["right_foot"].position_m[2],
                            math.hypot(
                                last_command.forward_mps,
                                last_command.lateral_mps,
                            ),
                        )
                        if last_squat.active:
                            controller.reset()
                            last_command = GlobalPositionCommand(
                                0.0, 0.0, last_command.delta_m
                            )
                        last_heading = relative_pelvis_yaw(
                            calibration.quaternions["pelvis"],
                            np.asarray(pelvis.quaternion_wxyz),
                        )
                        last_frame_time = latest.received_at
                    command = last_command
                    heading = last_heading
                    output = planner_message(
                        command,
                        heading,
                        last_upper,
                        last_squat if args.enable_squat else None,
                    )
                    last_live = True
                else:
                    if last_live:
                        controller.reset()
                        last_command = GlobalPositionCommand(
                            0.0, 0.0, (0.0, 0.0)
                        )
                        last_upper = sonic_upper_body_from_isaaclab(
                            mujoco_to_isaaclab(G1_DEFAULT_POSE)
                        )
                        last_heading = 0.0
                        last_squat = SquatCommand(False, 0.80, 0.0)
                        squat_controller.reset()
                        print("Input STALE: commanding planner IDLE")
                    neutral_upper = sonic_upper_body_from_isaaclab(
                        mujoco_to_isaaclab(G1_DEFAULT_POSE)
                    )
                    zero = GlobalPositionCommand(0.0, 0.0, (0.0, 0.0))
                    output = planner_message(zero, 0.0, neutral_upper)
                    last_live = False
                try:
                    publisher.send(output, flags=zmq.NOBLOCK)
                    sent += 1
                except zmq.Again:
                    pass
                if sent and sent % 50 == 0 and fresh:
                    print(
                        f"LIVE forward={command.forward_mps:+.2f} "
                        f"lateral={command.lateral_mps:+.2f} "
                        f"frame_delta=({command.delta_m[0]:+.4f},"
                        f"{command.delta_m[1]:+.4f}) "
                        f"heading={math.degrees(heading):+.1f}deg"
                        f" squat={'ON' if last_squat.active else 'off'}"
                        f" drop={last_squat.pelvis_drop_m:.2f}m"
                    )
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        return 0
    finally:
        publisher.close(linger=0)
        context.term()


if __name__ == "__main__":
    raise SystemExit(main())
