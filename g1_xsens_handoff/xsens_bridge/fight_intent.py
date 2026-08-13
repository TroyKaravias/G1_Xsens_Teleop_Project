"""Conservative Xsens movement and punch-intent extraction."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .body_motion import BodyMotionController
from .fight_control import Punch
from .g1_retarget import _conjugate, _rotate_vectors
from .xudp import PoseFrame


@dataclass(frozen=True)
class XsensFightIntent:
    forward_mps: float
    lateral_mps: float
    yaw_rate_rad_s: float
    target_height_m: float
    punch: Punch
    label: str


class PunchGestureDetector:
    """Detect a debounced forward hand strike in the pelvis frame.

    Detection is intentionally conservative and is only an intent signal.  A
    FightSupervisor still owns arming, cooldown, and output-route selection.
    """

    def __init__(
        self,
        *,
        forward_speed_mps: float = 1.0,
        extension_m: float = 0.08,
        retract_m: float = 0.035,
        cooldown_s: float = 0.60,
    ) -> None:
        if min(forward_speed_mps, extension_m, retract_m, cooldown_s) <= 0.0:
            raise ValueError("punch detector limits must be positive")
        self.forward_speed_mps = forward_speed_mps
        self.extension_m = extension_m
        self.retract_m = retract_m
        self.cooldown_s = cooldown_s
        self._neutral_forward: dict[str, float] = {}
        self._previous_local: dict[str, np.ndarray] = {}
        self._previous_time: float | None = None
        self._armed = {"left": True, "right": True}
        self._cooldown_until = 0.0

    def reset(self) -> None:
        self._neutral_forward.clear()
        self._previous_local.clear()
        self._previous_time = None
        self._armed = {"left": True, "right": True}
        self._cooldown_until = 0.0

    def update(self, frame: PoseFrame, now: float) -> Punch:
        segments = {segment.name: segment for segment in frame.segments}
        required = ("pelvis", "left_hand", "right_hand")
        if any(name not in segments for name in required):
            self.reset()
            return Punch.NONE
        pelvis = segments["pelvis"]
        pelvis_position = np.asarray(pelvis.position_m, dtype=np.float64)
        pelvis_inverse = _conjugate(
            np.asarray(pelvis.quaternion_wxyz, dtype=np.float64)[None]
        )
        local: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            relative = (
                np.asarray(segments[f"{side}_hand"].position_m, dtype=np.float64)
                - pelvis_position
            )
            local[side] = _rotate_vectors(pelvis_inverse, relative[None])[0]
            self._neutral_forward.setdefault(side, float(local[side][0]))

        if self._previous_time is None:
            self._previous_time = now
            self._previous_local = local
            return Punch.NONE
        dt = now - self._previous_time
        self._previous_time = now
        if dt <= 0.0 or dt > 0.20:
            self._previous_local = local
            return Punch.NONE

        best_side: str | None = None
        best_speed = self.forward_speed_mps
        for side in ("left", "right"):
            extension = float(local[side][0] - self._neutral_forward[side])
            if extension <= self.retract_m:
                self._armed[side] = True
            speed = float((local[side][0] - self._previous_local[side][0]) / dt)
            if (
                now >= self._cooldown_until
                and self._armed[side]
                and extension >= self.extension_m
                and speed >= best_speed
            ):
                best_side = side
                best_speed = speed
        self._previous_local = local
        if best_side is None:
            return Punch.NONE
        self._armed[best_side] = False
        self._cooldown_until = now + self.cooldown_s
        return Punch.LEFT_JAB if best_side == "left" else Punch.RIGHT_JAB


class XsensFightIntentDetector:
    def __init__(self) -> None:
        self.body = BodyMotionController()
        self.punch = PunchGestureDetector()

    def reset(self) -> None:
        self.body.reset()
        self.punch.reset()

    def update(self, frame: PoseFrame, now: float) -> XsensFightIntent:
        body = self.body.update(frame, now, fencing=False)
        punch = self.punch.update(frame, now)
        return XsensFightIntent(
            forward_mps=body.forward_velocity_mps,
            lateral_mps=body.lateral_velocity_mps,
            yaw_rate_rad_s=body.yaw_rate_rad_s,
            target_height_m=body.target_height_m,
            punch=punch,
            label=(punch.value.lower() if punch is not Punch.NONE else body.label),
        )
