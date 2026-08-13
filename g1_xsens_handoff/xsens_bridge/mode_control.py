"""Top-level mode selection for live G1 demonstrations.

This module emits intent only. It has no Unitree SDK or motor-command output.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .fencing_control import FencingEvent, FencingIntent, FencingSupervisor
from .locomotion_control import (
    LocomotionEvent,
    LocomotionIntent,
    LocomotionSupervisor,
)


class DemoMode(str, Enum):
    SAFE = "SAFE"
    # General locomotion: walk forward/backward, turn, and stop.
    MOVING = "MOVING"
    FENCING = "FENCING"
    FIGHTING = "FIGHTING"


class ModeEvent(str, Enum):
    NONE = "NONE"
    SELECT_SAFE = "SELECT_SAFE"
    SELECT_MOVING = "SELECT_MOVING"
    SELECT_FENCING = "SELECT_FENCING"
    SELECT_FIGHTING = "SELECT_FIGHTING"
    ARM = "ARM"
    DISARM = "DISARM"
    ESTOP = "ESTOP"
    RESET_ESTOP = "RESET_ESTOP"


@dataclass(frozen=True)
class ModeIntent:
    mode: DemoMode
    armed: bool
    estop_latched: bool
    upper_body_tracking_enabled: bool
    fencing: FencingIntent
    locomotion: LocomotionIntent
    fighting_available: bool


class ModeSupervisor:
    """Safety-gated mode selector wrapping the fencing supervisor."""

    def __init__(self) -> None:
        self.mode = DemoMode.SAFE
        self.armed = False
        self._estop_latched = False
        self.fencing = FencingSupervisor()
        self.locomotion = LocomotionSupervisor()

    def update(
        self,
        now: float,
        *,
        stream_fresh: bool,
        operator_ready: bool,
        event: ModeEvent = ModeEvent.NONE,
        fencing_event: FencingEvent = FencingEvent.NONE,
        locomotion_event: LocomotionEvent = LocomotionEvent.STOP,
    ) -> ModeIntent:
        if event is ModeEvent.ESTOP:
            self._estop_latched = True
            self.mode = DemoMode.SAFE
            self.armed = False
            fencing_event = FencingEvent.ESTOP

        if self._estop_latched:
            if (
                event is ModeEvent.RESET_ESTOP
                and stream_fresh
                and operator_ready
            ):
                self._estop_latched = False
                fencing_event = FencingEvent.RESET_ESTOP
            fencing = self.fencing.update(
                now,
                stream_fresh=stream_fresh,
                operator_ready=operator_ready,
                event=fencing_event,
            )
            return self.intent(fencing, LocomotionIntent())

        if event is ModeEvent.DISARM or not stream_fresh:
            self.mode = DemoMode.SAFE
            self.armed = False
            fencing_event = FencingEvent.DISARM
        elif event is ModeEvent.SELECT_SAFE:
            self.mode = DemoMode.SAFE
            self.armed = False
            fencing_event = FencingEvent.DISARM
        elif event is ModeEvent.SELECT_MOVING:
            self.mode = DemoMode.MOVING
            self.armed = False
            fencing_event = FencingEvent.DISARM
        elif event is ModeEvent.SELECT_FENCING:
            self.mode = DemoMode.FENCING
            self.armed = False
            fencing_event = FencingEvent.DISARM
        elif event is ModeEvent.SELECT_FIGHTING:
            # Fail closed until a non-contact motion library and limits exist.
            self.mode = DemoMode.SAFE
            self.armed = False
            fencing_event = FencingEvent.DISARM
        elif (
            event is ModeEvent.ARM
            and self.mode in {DemoMode.MOVING, DemoMode.FENCING}
            and stream_fresh
            and operator_ready
        ):
            self.armed = True

        if self.mode is not DemoMode.FENCING or not self.armed:
            fencing_event = FencingEvent.DISARM

        fencing = self.fencing.update(
            now,
            stream_fresh=stream_fresh,
            operator_ready=operator_ready,
            event=fencing_event,
        )
        locomotion = self.locomotion.update(
            locomotion_event,
            enabled=self.mode is DemoMode.MOVING and self.armed,
            stream_fresh=stream_fresh,
            operator_ready=operator_ready,
        )
        return self.intent(fencing, locomotion)

    def intent(
        self,
        fencing: FencingIntent | None = None,
        locomotion: LocomotionIntent | None = None,
    ) -> ModeIntent:
        if fencing is None:
            fencing = self.fencing.intent(0.0)
        if locomotion is None:
            locomotion = LocomotionIntent()
        return ModeIntent(
            mode=self.mode,
            armed=self.armed,
            estop_latched=self._estop_latched,
            upper_body_tracking_enabled=(
                self.armed
                and (
                    self.mode is DemoMode.MOVING
                    or (
                        self.mode is DemoMode.FENCING
                        and fencing.pose_tracking_enabled
                    )
                )
            ),
            fencing=fencing,
            locomotion=locomotion,
            fighting_available=False,
        )
