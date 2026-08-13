#!/usr/bin/env python3
"""Run the end-of-day controller demo without robot motor output."""

from xsens_bridge.fencing_control import FencingEvent
from xsens_bridge.locomotion_control import LocomotionEvent
from xsens_bridge.mode_control import ModeEvent, ModeSupervisor


def main() -> None:
    supervisor = ModeSupervisor()
    timeline = [
        (0.0, ModeEvent.SELECT_MOVING, FencingEvent.NONE,
         LocomotionEvent.STOP, "select locomotion"),
        (0.1, ModeEvent.ARM, FencingEvent.NONE,
         LocomotionEvent.STOP, "arm; Xsens upper body enabled"),
        (0.2, ModeEvent.NONE, FencingEvent.NONE,
         LocomotionEvent.FORWARD, "walk while tracking arms"),
        (0.6, ModeEvent.NONE, FencingEvent.NONE,
         LocomotionEvent.TURN_LEFT, "turn left while tracking arms"),
        (1.0, ModeEvent.DISARM, FencingEvent.NONE,
         LocomotionEvent.STOP, "stop locomotion"),
        (1.2, ModeEvent.SELECT_FENCING, FencingEvent.NONE,
         LocomotionEvent.STOP, "select fencing"),
        (1.3, ModeEvent.ARM, FencingEvent.NONE,
         LocomotionEvent.STOP, "arm fencing"),
        (1.4, ModeEvent.NONE, FencingEvent.ENTER_GARDE,
         LocomotionEvent.STOP, "enter en garde"),
        (2.0, ModeEvent.NONE, FencingEvent.ENABLE_POSE,
         LocomotionEvent.STOP, "enable Xsens pose"),
        (2.4, ModeEvent.NONE, FencingEvent.ADVANCE,
         LocomotionEvent.STOP, "bounded advance"),
        (2.9, ModeEvent.NONE, FencingEvent.NONE,
         LocomotionEvent.STOP, "resume pose"),
        (3.4, ModeEvent.NONE, FencingEvent.RETREAT,
         LocomotionEvent.STOP, "bounded retreat"),
        (3.9, ModeEvent.NONE, FencingEvent.NONE,
         LocomotionEvent.STOP, "resume pose"),
        (4.4, ModeEvent.NONE, FencingEvent.LUNGE,
         LocomotionEvent.STOP, "bounded lunge"),
        (5.1, ModeEvent.NONE, FencingEvent.NONE,
         LocomotionEvent.STOP, "automatic recovery"),
        (6.0, ModeEvent.NONE, FencingEvent.NONE,
         LocomotionEvent.STOP, "return en garde"),
        (6.2, ModeEvent.DISARM, FencingEvent.NONE,
         LocomotionEvent.STOP, "safe/disarmed"),
        (6.4, ModeEvent.SELECT_FIGHTING, FencingEvent.NONE,
         LocomotionEvent.STOP, "fighting rejected"),
    ]
    print(
        "time  mode      armed arms fencing_state   move_v  yaw    lunge "
        "description"
    )
    for now, mode_event, fencing_event, locomotion_event, description in timeline:
        intent = supervisor.update(
            now,
            stream_fresh=True,
            operator_ready=True,
            event=mode_event,
            fencing_event=fencing_event,
            locomotion_event=locomotion_event,
        )
        f = intent.fencing
        movement_v = (
            intent.locomotion.forward_velocity_mps
            if intent.mode.value == "MOVING"
            else f.forward_velocity_mps
        )
        print(
            f"{now:4.1f}  {intent.mode.value:9} {str(intent.armed):5} "
            f"{str(intent.upper_body_tracking_enabled):5} "
            f"{f.state.value:15} {movement_v:6.2f} "
            f"{intent.locomotion.yaw_rate_rad_s:6.2f} "
            f"{f.lunge_phase:5.2f} {description}"
        )


if __name__ == "__main__":
    main()
