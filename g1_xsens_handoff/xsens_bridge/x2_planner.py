"""Strict runtime contract for the X2 frozen-G1-core kinematic planner."""

from __future__ import annotations

from pathlib import Path

import numpy as np


QPOS_SIZE = 38
CONTEXT_FRAMES = 4
MAX_PREDICTION_FRAMES = 64


def clamp_planner_joints(qpos: np.ndarray, joint_ranges_mj: np.ndarray) -> tuple[np.ndarray, int]:
    """Clamp the 31 joint fields of full planner qpos to the X2 MJCF limits."""
    values = np.asarray(qpos, dtype=np.float32).copy()
    limits = np.asarray(joint_ranges_mj, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != QPOS_SIZE or limits.shape != (31, 2):
        raise ValueError("expected planner qpos (frames,38) and X2 limits (31,2)")
    before = values[:, 7:].copy()
    values[:, 7:] = np.clip(values[:, 7:], limits[:, 0], limits[:, 1])
    return values, int(np.count_nonzero(before != values[:, 7:]))


def pack_mujoco_qpos(
    root_position: np.ndarray,
    root_quaternion_xyzw: np.ndarray,
    joint_position_mj: np.ndarray,
) -> np.ndarray:
    """Pack motion-lib arrays into planner MuJoCo qpos (xyz, wxyz, 31 joints)."""
    root_position = np.asarray(root_position, dtype=np.float32)
    root_quaternion = np.asarray(root_quaternion_xyzw, dtype=np.float32)
    joints = np.asarray(joint_position_mj, dtype=np.float32)
    if root_position.shape[-1] != 3 or root_quaternion.shape[-1] != 4 or joints.shape[-1] != 31:
        raise ValueError("expected root xyz, root quaternion xyzw, and 31 X2 joints")
    if root_position.shape[:-1] != root_quaternion.shape[:-1] or root_position.shape[:-1] != joints.shape[:-1]:
        raise ValueError("planner qpos components must have matching leading dimensions")
    return np.concatenate(
        (root_position, root_quaternion[..., [3, 0, 1, 2]], joints), axis=-1
    ).astype(np.float32, copy=False)


def unpack_mujoco_qpos(qpos: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Unpack planner qpos into motion-lib root xyz, quaternion xyzw, and joints."""
    values = np.asarray(qpos, dtype=np.float32)
    if values.shape[-1] != QPOS_SIZE:
        raise ValueError("planner MuJoCo qpos must have width 38")
    return values[..., :3], values[..., [4, 5, 6, 3]], values[..., 7:]


class X2KinematicPlanner:
    """ONNX wrapper for the matching X2 planner's exact four-input contract."""

    def __init__(self, model_path: str | Path) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("Install requirements-x2-sim.txt to run the X2 planner") from exc
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(path)
        self.session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        contract = {item.name: item.shape for item in self.session.get_inputs()}
        expected = {
            "context_mujoco_qpos": [1, CONTEXT_FRAMES, QPOS_SIZE],
            "velocity_intent": [1, 4],
            "mode": [1],
            "random_seed": [1],
        }
        if contract != expected:
            raise ValueError(f"unexpected X2 planner input contract: {contract}")

    def generate(
        self,
        context_qpos: np.ndarray,
        velocity_intent: np.ndarray | None = None,
        mode: int = 0,
        random_seed: int = 0,
    ) -> np.ndarray:
        context = np.asarray(context_qpos, dtype=np.float32)
        if context.shape != (CONTEXT_FRAMES, QPOS_SIZE):
            raise ValueError("context_qpos must have shape (4, 38)")
        velocity = np.zeros(4, dtype=np.float32) if velocity_intent is None else np.asarray(velocity_intent, dtype=np.float32)
        if velocity.shape != (4,):
            raise ValueError("velocity_intent must have shape (4,)")
        output, count = self.session.run(None, {
            "context_mujoco_qpos": context[None],
            "velocity_intent": velocity[None],
            "mode": np.asarray([mode], dtype=np.int64),
            "random_seed": np.asarray([random_seed], dtype=np.int64),
        })
        valid = int(np.asarray(count).reshape(-1)[0])
        prediction = np.asarray(output, dtype=np.float32)[0]
        if not 1 <= valid <= len(prediction) or prediction.shape != (MAX_PREDICTION_FRAMES, QPOS_SIZE):
            raise RuntimeError("X2 planner returned an invalid prediction shape/count")
        prediction = prediction[:valid]
        if not np.isfinite(prediction).all():
            raise RuntimeError("X2 planner returned non-finite qpos")
        return prediction
