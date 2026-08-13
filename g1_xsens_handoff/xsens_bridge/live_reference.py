"""Causal 50 Hz reference buffering and watchdog logic for live tracking."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum

import numpy as np

from .g1_retarget import G1_DEFAULT_POSE


class WatchdogState(str, Enum):
    WARMUP = "WARMUP"
    LIVE = "LIVE"
    HOLD = "HOLD"
    SAFE_RETURN = "SAFE_RETURN"
    RECOVERING = "RECOVERING"
    ESTOP = "ESTOP"


@dataclass(frozen=True)
class ReferenceFrame:
    timestamp: float
    dof_pos: np.ndarray
    dof_vel: np.ndarray
    body_rot: np.ndarray


class LiveReferenceBuffer:
    """Build sparse future references without sending robot commands."""

    def __init__(
        self,
        *,
        future_steps: tuple[int, ...] = (1, 2, 4, 8),
        control_dt: float = 0.02,
        hold_timeout: float = 0.10,
        safe_return_timeout: float = 0.30,
        safe_return_duration: float = 1.50,
        recovery_fresh_time: float = 0.50,
        recovery_duration: float = 0.75,
        max_extrapolation: float = 0.16,
        safe_dof_pos: np.ndarray | None = None,
    ) -> None:
        self.future_steps = future_steps
        self.control_dt = control_dt
        self.hold_timeout = hold_timeout
        self.safe_return_timeout = safe_return_timeout
        self.safe_return_duration = safe_return_duration
        self.recovery_fresh_time = recovery_fresh_time
        self.recovery_duration = recovery_duration
        self.max_extrapolation = max_extrapolation
        self.safe_dof_pos = np.asarray(
            G1_DEFAULT_POSE if safe_dof_pos is None else safe_dof_pos,
            dtype=np.float64,
        )
        if self.safe_dof_pos.shape != (29,):
            raise ValueError("safe_dof_pos must have shape (29,)")
        self._frames: deque[ReferenceFrame] = deque(maxlen=16)
        self._estop_latched = False
        self._safe_return_started: float | None = None
        self._safe_return_origin: np.ndarray | None = None
        self._fresh_since: float | None = None
        self._recovery_started: float | None = None

    def push(self, frame: ReferenceFrame) -> None:
        if self._frames and frame.timestamp <= self._frames[-1].timestamp:
            raise ValueError("Reference timestamps must be strictly increasing")
        if frame.dof_pos.shape != (29,) or frame.dof_vel.shape != (29,):
            raise ValueError("G1 reference frames must contain 29 DOFs")
        self._frames.append(frame)
        if self._safe_return_started is not None and self._fresh_since is None:
            self._fresh_since = frame.timestamp

    def state(self, now: float) -> WatchdogState:
        if self._estop_latched:
            return WatchdogState.ESTOP
        if not self._frames:
            return WatchdogState.WARMUP
        age = max(0.0, now - self._frames[-1].timestamp)
        if self._safe_return_started is None and age >= self.safe_return_timeout:
            self._safe_return_started = now
            self._safe_return_origin = self._frames[-1].dof_pos.copy()
            self._fresh_since = None
            self._recovery_started = None
        if self._safe_return_started is not None:
            if age >= self.hold_timeout or self._fresh_since is None:
                return WatchdogState.SAFE_RETURN
            if now - self._fresh_since < self.recovery_fresh_time:
                return WatchdogState.SAFE_RETURN
            if now - self._safe_return_started < self.safe_return_duration:
                return WatchdogState.SAFE_RETURN
            if self._recovery_started is None:
                self._recovery_started = now
            if now - self._recovery_started < self.recovery_duration:
                return WatchdogState.RECOVERING
            self._safe_return_started = None
            self._safe_return_origin = None
            self._fresh_since = None
            self._recovery_started = None
            return WatchdogState.LIVE
        if age >= self.hold_timeout:
            return WatchdogState.HOLD
        return WatchdogState.LIVE

    def trigger_estop(self) -> None:
        """Latch a hard stop for manual activation or a genuine safety fault."""
        self._estop_latched = True

    def reset_estop(self, now: float) -> None:
        """Explicitly clear a latched stop only when a fresh frame is present."""
        if not self._frames:
            raise RuntimeError("Cannot reset ESTOP without a reference frame")
        age = max(0.0, now - self._frames[-1].timestamp)
        if age >= self.hold_timeout:
            raise RuntimeError("Cannot reset ESTOP while the reference is stale")
        self._estop_latched = False

    def future(self, now: float) -> dict[str, np.ndarray]:
        if not self._frames:
            raise RuntimeError("No reference frame is available")
        latest = self._frames[-1]
        state = self.state(now)
        horizons = np.asarray(self.future_steps, dtype=np.float64) * self.control_dt
        if state is WatchdogState.LIVE:
            prediction_time = np.minimum(horizons, self.max_extrapolation)
            dof_pos = latest.dof_pos[None, :] + prediction_time[:, None] * latest.dof_vel
        elif state is WatchdogState.SAFE_RETURN:
            origin = (
                latest.dof_pos
                if self._safe_return_origin is None
                else self._safe_return_origin
            )
            elapsed = max(0.0, now - (self._safe_return_started or now))
            blend = np.clip(elapsed / self.safe_return_duration, 0.0, 1.0)
            target = (1.0 - blend) * origin + blend * self.safe_dof_pos
            dof_pos = np.repeat(target[None, :], len(horizons), axis=0)
        elif state is WatchdogState.RECOVERING:
            elapsed = max(0.0, now - (self._recovery_started or now))
            blend = np.clip(elapsed / self.recovery_duration, 0.0, 1.0)
            target = (1.0 - blend) * self.safe_dof_pos + blend * latest.dof_pos
            dof_pos = np.repeat(target[None, :], len(horizons), axis=0)
        else:
            # HOLD and hard ESTOP freeze the last valid reference. The caller
            # must independently disable physical output in ESTOP.
            dof_pos = np.repeat(latest.dof_pos[None, :], len(horizons), axis=0)
        return {
            "dof_pos": dof_pos.astype(np.float32),
            "dof_vel": np.repeat(
                (latest.dof_vel if state is WatchdogState.LIVE else np.zeros(29))[
                    None, :
                ],
                len(horizons),
                axis=0,
            ).astype(np.float32),
            # Conservative first version: hold measured body orientations.
            "body_rot": np.repeat(
                latest.body_rot[None, :, :], len(horizons), axis=0
            ).astype(np.float32),
        }
