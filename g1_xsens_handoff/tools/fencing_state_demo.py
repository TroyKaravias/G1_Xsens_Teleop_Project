#!/usr/bin/env python3
"""Print a robot-output-free fencing supervisor demonstration."""

from xsens_bridge.fencing_control import FencingEvent, FencingSupervisor


def main() -> None:
    supervisor = FencingSupervisor()
    sequence = (
        (0.0, FencingEvent.ENTER_GARDE, "request en garde"),
        (0.6, FencingEvent.ENABLE_POSE, "enable upper-body pose"),
        (1.0, FencingEvent.ADVANCE, "one bounded advance"),
        (1.5, FencingEvent.NONE, "advance completes"),
        (2.0, FencingEvent.RETREAT, "one bounded retreat"),
        (2.5, FencingEvent.NONE, "retreat completes"),
        (3.0, FencingEvent.LUNGE, "begin bounded lunge"),
        (3.7, FencingEvent.NONE, "automatic recovery"),
        (4.6, FencingEvent.NONE, "return to en garde"),
        (5.0, FencingEvent.ESTOP, "emergency stop"),
    )
    print(
        "time  state           pose  vx_mps  lunge  safe_return  event"
    )
    for now, event, label in sequence:
        intent = supervisor.update(
            now,
            stream_fresh=True,
            operator_ready=True,
            event=event,
        )
        print(
            f"{now:4.1f}  {intent.state.value:14} "
            f"{str(intent.pose_tracking_enabled):5} "
            f"{intent.forward_velocity_mps:6.2f} "
            f"{intent.lunge_phase:6.2f} "
            f"{str(intent.request_safe_return):11}  {label}"
        )


if __name__ == "__main__":
    main()
