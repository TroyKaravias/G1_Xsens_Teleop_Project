"""Runtime contract for the fused AgiBot X2 SONIC ONNX policy.

This module only constructs and executes policy observations. It does not
connect to robot hardware and does not treat policy inference as validation of
MuJoCo dynamics or balance.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

NUM_DOFS = 31
HISTORY_LENGTH = 10
NUM_FUTURE_FRAMES = 10
PROPRIOCEPTION_SIZE = 990
TOKENIZER_SIZE = 680
OBSERVATION_SIZE = PROPRIOCEPTION_SIZE + TOKENIZER_SIZE
ACTION_SIZE = NUM_DOFS

# Gather indices from the X2 embodiment: result[IL] = source[MJ][IL_TO_MJ].
IL_TO_MJ_DOF = np.asarray(
    [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 29, 15, 22, 4, 10,
     30, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28],
    dtype=np.int64,
)
MJ_TO_IL_DOF = np.asarray(
    [0, 3, 6, 9, 14, 19, 1, 4, 7, 10, 15, 20, 2, 5, 8, 12,
     17, 21, 23, 25, 27, 29, 13, 18, 22, 24, 26, 28, 30, 11, 16],
    dtype=np.int64,
)


class X2SonicProprioceptionBuffer:
    """IsaacLab-compatible, oldest-first, term-major history buffer."""

    _WIDTHS = (3, NUM_DOFS, NUM_DOFS, NUM_DOFS, 3)

    def __init__(self) -> None:
        self._histories = [deque(maxlen=HISTORY_LENGTH) for _ in self._WIDTHS]

    def reset(self) -> None:
        for history in self._histories:
            history.clear()

    def append(
        self,
        base_angular_velocity: Iterable[float],
        joint_position_relative: Iterable[float],
        joint_velocity: Iterable[float],
        last_action: Iterable[float],
        gravity_direction: Iterable[float],
    ) -> None:
        values = (
            base_angular_velocity,
            joint_position_relative,
            joint_velocity,
            last_action,
            gravity_direction,
        )
        arrays = []
        for value, width in zip(values, self._WIDTHS):
            array = np.asarray(value, dtype=np.float32)
            if array.shape != (width,):
                raise ValueError(f"Expected ({width},), got {array.shape}")
            arrays.append(array.copy())

        for history, array in zip(self._histories, arrays):
            if not history:
                history.extend(array.copy() for _ in range(HISTORY_LENGTH))
            else:
                history.append(array)

    def flattened(self) -> np.ndarray:
        if any(len(history) != HISTORY_LENGTH for history in self._histories):
            raise RuntimeError("Proprioception buffer has not been primed")
        result = np.concatenate(
            [frame for history in self._histories for frame in history]
        ).astype(np.float32, copy=False)
        assert result.shape == (PROPRIOCEPTION_SIZE,)
        return result


def assemble_observation(tokenizer: np.ndarray, proprioception: np.ndarray) -> np.ndarray:
    """Assemble the fused graph input: tokenizer (680), then proprio (990)."""
    tokenizer = np.asarray(tokenizer, dtype=np.float32)
    proprioception = np.asarray(proprioception, dtype=np.float32)
    if tokenizer.shape != (TOKENIZER_SIZE,):
        raise ValueError(f"Expected tokenizer ({TOKENIZER_SIZE},), got {tokenizer.shape}")
    if proprioception.shape != (PROPRIOCEPTION_SIZE,):
        raise ValueError(
            f"Expected proprioception ({PROPRIOCEPTION_SIZE},), got {proprioception.shape}"
        )
    return np.concatenate((tokenizer, proprioception))[None, :]


def build_recorded_tokenizer_observation(
    trajectory_mj: np.ndarray,
    frame_dt: float,
    current_time: float,
) -> np.ndarray:
    """Build SONIC's 680-wide future reference from an X2 joint trajectory.

    The recording path currently requests no change in root heading, so every
    relative-orientation channel is the identity 6D rotation. This is suitable
    for the first controlled policy test, not yet global-turn tracking.
    """
    trajectory = np.asarray(trajectory_mj, dtype=np.float32)
    if trajectory.ndim != 2 or trajectory.shape[1] != NUM_DOFS:
        raise ValueError(f"Expected trajectory (frames, {NUM_DOFS}), got {trajectory.shape}")
    if frame_dt <= 0 or len(trajectory) < 1:
        raise ValueError("trajectory and frame_dt must be non-empty and positive")
    future_step = 0.1
    positions = []
    velocities = []
    for future_index in range(1, NUM_FUTURE_FRAMES + 1):
        index = min(int((current_time + future_index * future_step) / frame_dt), len(trajectory) - 1)
        previous = max(0, index - 1)
        positions.append(trajectory[index, IL_TO_MJ_DOF])
        velocities.append((trajectory[index] - trajectory[previous])[IL_TO_MJ_DOF] / frame_dt)
    identity_6d = np.tile(np.asarray([1, 0, 0, 1, 0, 0], dtype=np.float32), (NUM_FUTURE_FRAMES, 1))
    result = _pack_tokenizer_frames(
        np.stack(positions), np.stack(velocities), identity_6d
    )
    assert result.shape == (TOKENIZER_SIZE,)
    return result.astype(np.float32, copy=False)


def build_tokenizer_from_future_frames(
    joint_positions_mj: np.ndarray,
    joint_velocities_mj: np.ndarray,
    heading_6d: np.ndarray | None = None,
) -> np.ndarray:
    """Pack ten already-sampled future frames for the fused policy."""
    positions = np.asarray(joint_positions_mj, dtype=np.float32)
    velocities = np.asarray(joint_velocities_mj, dtype=np.float32)
    expected = (NUM_FUTURE_FRAMES, NUM_DOFS)
    if positions.shape != expected or velocities.shape != expected:
        raise ValueError(f"future positions and velocities must have shape {expected}")
    if heading_6d is None:
        heading = np.tile(
            np.asarray([1, 0, 0, 1, 0, 0], dtype=np.float32),
            (NUM_FUTURE_FRAMES, 1),
        )
    else:
        heading = np.asarray(heading_6d, dtype=np.float32)
        if heading.shape != (NUM_FUTURE_FRAMES, 6):
            raise ValueError("heading_6d must have shape (10, 6)")
    packed = _pack_tokenizer_frames(
        positions[:, IL_TO_MJ_DOF], velocities[:, IL_TO_MJ_DOF], heading
    )
    assert packed.shape == (TOKENIZER_SIZE,)
    return packed


def _pack_tokenizer_frames(
    positions_il: np.ndarray,
    velocities_il: np.ndarray,
    heading_6d: np.ndarray,
) -> np.ndarray:
    """Reproduce the X2 training/export tokenizer layout exactly.

    IsaacLab first concatenates all ten position frames and all ten velocity
    frames into a 620-wide command, then reshapes that flat command to 10x62.
    The six orientation values are appended to each resulting row.  Although
    this is not conventional per-frame position/velocity interleaving, it is
    the contract used by the released X2 policy and its reference evaluator.
    """
    command = np.concatenate((positions_il.reshape(-1), velocities_il.reshape(-1)))
    command_nonflat = command.reshape(NUM_FUTURE_FRAMES, 2 * NUM_DOFS)
    return np.concatenate((command_nonflat, heading_6d), axis=1).reshape(-1).astype(
        np.float32, copy=False
    )


@dataclass(frozen=True)
class X2SonicReferenceFrame:
    timestamp: float
    joint_position_mj: np.ndarray
    joint_velocity_mj: np.ndarray


class X2SonicDelayedReferenceBuffer:
    """Use received history as SONIC's deterministic one-second lookahead."""

    def __init__(self, delay_s: float = 1.0, capacity: int = 256) -> None:
        if delay_s < 1.0:
            raise ValueError("delay_s must be at least the 1.0 s SONIC horizon")
        self.delay_s = float(delay_s)
        self._frames: deque[X2SonicReferenceFrame] = deque(maxlen=capacity)

    def push(self, frame: X2SonicReferenceFrame) -> None:
        position = np.asarray(frame.joint_position_mj, dtype=np.float32)
        velocity = np.asarray(frame.joint_velocity_mj, dtype=np.float32)
        if position.shape != (NUM_DOFS,) or velocity.shape != (NUM_DOFS,):
            raise ValueError("X2 SONIC references must contain 31 joints")
        if self._frames and frame.timestamp <= self._frames[-1].timestamp:
            raise ValueError("reference timestamps must increase")
        self._frames.append(
            X2SonicReferenceFrame(float(frame.timestamp), position.copy(), velocity.copy())
        )

    def ready(self, now: float) -> bool:
        if len(self._frames) < 2:
            return False
        return self._frames[0].timestamp <= now - self.delay_s and self._frames[-1].timestamp >= now

    def _sample(self, timestamp: float) -> tuple[np.ndarray, np.ndarray]:
        if not self._frames or timestamp < self._frames[0].timestamp or timestamp > self._frames[-1].timestamp:
            raise RuntimeError("requested reference time is outside the received buffer")
        frames = list(self._frames)
        for upper_index in range(1, len(frames)):
            upper = frames[upper_index]
            if upper.timestamp >= timestamp:
                lower = frames[upper_index - 1]
                span = upper.timestamp - lower.timestamp
                fraction = 0.0 if span <= 0 else (timestamp - lower.timestamp) / span
                position = (1.0 - fraction) * lower.joint_position_mj + fraction * upper.joint_position_mj
                velocity = (1.0 - fraction) * lower.joint_velocity_mj + fraction * upper.joint_velocity_mj
                return position.astype(np.float32), velocity.astype(np.float32)
        return frames[-1].joint_position_mj.copy(), frames[-1].joint_velocity_mj.copy()

    def tokenizer(self, now: float) -> np.ndarray:
        if not self.ready(now):
            raise RuntimeError("one-second live reference buffer is not ready")
        base_time = now - self.delay_s
        sampled = [self._sample(base_time + 0.1 * index) for index in range(1, 11)]
        return build_tokenizer_from_future_frames(
            np.stack([item[0] for item in sampled]),
            np.stack([item[1] for item in sampled]),
        )

    def delayed_current(self, reference_end_time: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the reference at the policy's delayed current phase."""
        if not self.ready(reference_end_time):
            raise RuntimeError("one-second live reference buffer is not ready")
        return self._sample(reference_end_time - self.delay_s)


class X2SonicOnnxPolicy:
    """Strict wrapper around the fused 1670→31 X2 policy graph."""

    def __init__(self, model_path: str | Path, providers: list[str] | None = None) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("Install requirements-x2-sim.txt to run X2 SONIC") from exc

        self.model_path = Path(model_path)
        if not self.model_path.is_file():
            raise FileNotFoundError(self.model_path)
        self.session = ort.InferenceSession(
            str(self.model_path), providers=providers or ["CPUExecutionProvider"]
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape[-1] != OBSERVATION_SIZE:
            raise ValueError("X2 SONIC graph must have one 1670-wide input")
        if len(outputs) != 1 or outputs[0].shape[-1] != ACTION_SIZE:
            raise ValueError("X2 SONIC graph must have one 31-wide output")
        self.input_name = inputs[0].name

    def infer(self, tokenizer: np.ndarray, proprioception: np.ndarray) -> np.ndarray:
        observation = assemble_observation(tokenizer, proprioception)
        if not np.isfinite(observation).all():
            raise RuntimeError("X2 SONIC observation contains non-finite values")
        action = np.asarray(
            self.session.run(None, {self.input_name: observation})[0], dtype=np.float32
        )
        if action.shape != (1, ACTION_SIZE):
            raise RuntimeError(f"Unexpected X2 SONIC action shape: {action.shape}")
        if not np.isfinite(action).all():
            maximum = float(np.max(np.abs(observation)))
            raise RuntimeError(
                f"X2 SONIC produced a non-finite action (max |observation|={maximum:.3f})"
            )
        return action[0]
