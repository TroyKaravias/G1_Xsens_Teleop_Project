"""SONIC ZMQ-manager protocol and safe mode-intent adapter.

Wire format matches NVIDIA GR00T-WholeBodyControl commit 4141c34:
topic + 1280-byte JSON header + packed little-endian payload.
"""

from __future__ import annotations

from enum import IntEnum
import math

import numpy as np

from .mode_control import DemoMode, ModeIntent
from .sonic_zmq import pack_sonic_pose_message
from .fight_control import FightManeuver, FightOutput, FightRoute


SONIC_UPPER_BODY_ISAACLAB_INDICES = np.asarray(
    [2, 5, 8, 11, 12, 15, 16, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28],
    dtype=np.int64,
)


class SonicLocomotionMode(IntEnum):
    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3
    IDLE_BOXING = 9
    WALK_BOXING = 10
    LEFT_JAB = 11
    RIGHT_JAB = 12
    RANDOM_PUNCHES = 13
    LEFT_HOOK = 15
    RIGHT_HOOK = 16


def command_fields(
    *, start: bool, stop: bool, planner: bool
) -> dict[str, np.ndarray]:
    return {
        "start": np.asarray([start], dtype=np.uint8),
        "stop": np.asarray([stop], dtype=np.uint8),
        "planner": np.asarray([planner], dtype=np.uint8),
    }


def pack_command_message(
    *, start: bool, stop: bool, planner: bool
) -> bytes:
    return pack_sonic_pose_message(
        command_fields(start=start, stop=stop, planner=planner),
        topic="command",
        version=1,
    )


def sonic_upper_body_from_isaaclab(
    joint_position: np.ndarray,
) -> np.ndarray:
    """Extract SONIC's 17 upper-body targets from a 29-DOF IsaacLab pose."""
    position = np.asarray(joint_position, dtype=np.float32)
    if position.shape != (29,):
        raise ValueError("joint_position must have shape [29]")
    return np.ascontiguousarray(
        position[SONIC_UPPER_BODY_ISAACLAB_INDICES], dtype=np.float32
    )


def planner_fields(
    *,
    mode: SonicLocomotionMode,
    movement: tuple[float, float, float],
    facing: tuple[float, float, float],
    speed: float,
    height: float = -1.0,
    upper_body_position: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    movement_array = np.asarray(movement, dtype=np.float32)
    facing_array = np.asarray(facing, dtype=np.float32)
    if movement_array.shape != (3,) or facing_array.shape != (3,):
        raise ValueError("movement and facing must each have shape [3]")
    if not np.all(np.isfinite(movement_array)):
        raise ValueError("movement must be finite")
    if not np.all(np.isfinite(facing_array)):
        raise ValueError("facing must be finite")
    fields = {
        "mode": np.asarray([int(mode)], dtype=np.int32),
        "movement": movement_array,
        "facing": facing_array,
        "speed": np.asarray([speed], dtype=np.float32),
        "height": np.asarray([height], dtype=np.float32),
    }
    if upper_body_position is not None:
        upper = np.asarray(upper_body_position, dtype=np.float32)
        if upper.shape != (17,):
            raise ValueError("upper_body_position must have shape [17]")
        fields["upper_body_position"] = upper
    return fields


def pack_planner_message(**kwargs) -> bytes:
    return pack_sonic_pose_message(
        planner_fields(**kwargs), topic="planner", version=1
    )


def mode_intent_to_planner(
    intent: ModeIntent,
    *,
    heading_rad: float = 0.0,
    upper_body_position: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Convert a safe high-level intent into one SONIC planner message."""
    speed = 0.0
    forward = 0.0
    yaw_rate = 0.0
    if intent.mode is DemoMode.MOVING and intent.armed:
        forward = intent.locomotion.forward_velocity_mps
        yaw_rate = intent.locomotion.yaw_rate_rad_s
    elif intent.mode is DemoMode.FENCING and intent.armed:
        forward = intent.fencing.forward_velocity_mps

    # The SONIC planner expects a direction plus a nonnegative speed.
    direction = 0.0 if abs(forward) < 1e-6 else math.copysign(1.0, forward)
    speed = abs(forward)
    effective_heading = heading_rad + yaw_rate * 0.02
    facing = (
        math.cos(effective_heading),
        math.sin(effective_heading),
        0.0,
    )
    turning = abs(yaw_rate) > 1e-6
    # A pure facing change twists the waist without moving the feet. During
    # a turn request, use SONIC's minimum slow step to create a shallow arc.
    if turning and direction == 0.0:
        direction = 1.0
    movement = (
        direction * facing[0],
        direction * facing[1],
        0.0,
    )
    mode = (
        SonicLocomotionMode.SLOW_WALK
        if speed > 0.0 or turning
        else SonicLocomotionMode.IDLE
    )
    # SONIC clamps SLOW_WALK to at least 0.2 m/s. Supplying that mode during
    # a pure facing change allows foot repositioning instead of only twisting
    # the waist while IDLE.
    planner_speed = speed if speed > 0.0 else (0.2 if turning else -1.0)
    upper = (
        sonic_upper_body_from_isaaclab(upper_body_position)
        if intent.upper_body_tracking_enabled
        and upper_body_position is not None
        else None
    )
    return planner_fields(
        mode=mode,
        movement=movement,
        facing=facing,
        speed=planner_speed,
        upper_body_position=upper,
    )


_FIGHT_MODE_TO_SONIC = {
    FightManeuver.IDLE_BOXING: SonicLocomotionMode.IDLE_BOXING,
    FightManeuver.WALK_BOXING: SonicLocomotionMode.WALK_BOXING,
    FightManeuver.LEFT_JAB: SonicLocomotionMode.LEFT_JAB,
    FightManeuver.RIGHT_JAB: SonicLocomotionMode.RIGHT_JAB,
    FightManeuver.LEFT_HOOK: SonicLocomotionMode.LEFT_HOOK,
    FightManeuver.RIGHT_HOOK: SonicLocomotionMode.RIGHT_HOOK,
}


class SonicFightAdapter:
    """Convert exclusive fight-planner intent to the pinned SONIC schema.

    Heading is integrated from bounded frame-to-frame yaw-rate commands.  The
    internal angle may cross any number of full turns; only sin/cos are sent,
    avoiding the discontinuity of a wrapped absolute heading.
    """

    def __init__(self, *, initial_heading_rad: float = 0.0) -> None:
        if not math.isfinite(initial_heading_rad):
            raise ValueError("initial heading must be finite")
        self.heading_rad = initial_heading_rad
        self._previous_time: float | None = None

    def reset(self, *, heading_rad: float = 0.0) -> None:
        if not math.isfinite(heading_rad):
            raise ValueError("heading must be finite")
        self.heading_rad = heading_rad
        self._previous_time = None

    def planner_fields(
        self,
        output: FightOutput,
        now: float,
        *,
        upper_body_position: np.ndarray | None = None,
    ) -> dict[str, np.ndarray]:
        if output.route is not FightRoute.PLANNER:
            raise ValueError("fight output is not routed to the SONIC planner")
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        dt = 0.0 if self._previous_time is None else now - self._previous_time
        self._previous_time = now
        if dt < 0.0 or dt > 0.20:
            dt = 0.0
        self.heading_rad += output.yaw_rate_rad_s * dt

        facing = (
            math.cos(self.heading_rad),
            math.sin(self.heading_rad),
            0.0,
        )
        left = (-facing[1], facing[0])
        movement_norm = math.hypot(output.forward_mps, output.lateral_mps)
        if movement_norm > 1e-8:
            movement = (
                (
                    output.forward_mps * facing[0]
                    + output.lateral_mps * left[0]
                ) / movement_norm,
                (
                    output.forward_mps * facing[1]
                    + output.lateral_mps * left[1]
                ) / movement_norm,
                0.0,
            )
            speed = movement_norm
        elif abs(output.yaw_rate_rad_s) > 1e-8:
            # The pinned planner needs a minimum slow step to turn its feet.
            movement = facing
            speed = 0.20
        else:
            movement = (0.0, 0.0, 0.0)
            speed = -1.0

        upper = None
        if output.upper_body_tracking and upper_body_position is not None:
            upper = sonic_upper_body_from_isaaclab(upper_body_position)
        return planner_fields(
            mode=_FIGHT_MODE_TO_SONIC[output.maneuver],
            movement=movement,
            facing=facing,
            speed=speed,
            height=output.target_height_m,
            upper_body_position=upper,
        )
