"""SONIC ZMQ protocol-v1 packing for simulation motion streaming."""

from __future__ import annotations

import json
from collections.abc import Mapping

import numpy as np


SONIC_ZMQ_HEADER_SIZE = 1280
# SONIC protocol v1 consumes IsaacLab order. The existing Xsens bridge emits
# MuJoCo/ProtoMotions order, so each IsaacLab slot selects this MuJoCo index.
G1_MUJOCO_TO_ISAACLAB = np.array(
    [
        0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
        16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28,
    ],
    dtype=np.int64,
)
_DTYPE_NAMES = {
    np.dtype(np.uint8): "u8",
    np.dtype(np.float32): "f32",
    np.dtype(np.float64): "f64",
    np.dtype(np.int32): "i32",
    np.dtype(np.int64): "i64",
    np.dtype(np.bool_): "bool",
}


def pack_sonic_pose_message(
    fields: Mapping[str, np.ndarray],
    *,
    topic: str = "pose",
    version: int = 1,
) -> bytes:
    """Pack arrays using SONIC's topic/header/payload ZMQ wire format."""
    if not topic:
        raise ValueError("topic must not be empty")
    descriptions: list[dict[str, object]] = []
    payload: list[bytes] = []
    for name, raw_value in fields.items():
        value = np.asarray(raw_value)
        dtype_name = _DTYPE_NAMES.get(value.dtype)
        if dtype_name is None:
            raise TypeError(f"Unsupported SONIC field dtype for {name}: {value.dtype}")
        value = np.ascontiguousarray(value)
        if value.dtype.byteorder == ">":
            value = value.astype(value.dtype.newbyteorder("<"))
        descriptions.append(
            {"name": name, "dtype": dtype_name, "shape": list(value.shape)}
        )
        payload.append(value.tobytes())

    header = {
        "v": int(version),
        "endian": "le",
        "count": 1,
        "fields": descriptions,
    }
    encoded_header = json.dumps(header, separators=(",", ":")).encode("utf-8")
    if len(encoded_header) > SONIC_ZMQ_HEADER_SIZE:
        raise ValueError(
            f"SONIC header is too large: {len(encoded_header)} > "
            f"{SONIC_ZMQ_HEADER_SIZE}"
        )
    return (
        topic.encode("utf-8")
        + encoded_header.ljust(SONIC_ZMQ_HEADER_SIZE, b"\x00")
        + b"".join(payload)
    )


def protocol_v1_fields(
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    frame_index: np.ndarray,
    *,
    body_quat: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Validate and normalize a SONIC joint-stream protocol-v1 batch."""
    positions = np.asarray(joint_pos, dtype=np.float32)
    velocities = np.asarray(joint_vel, dtype=np.float32)
    indices = np.asarray(frame_index, dtype=np.int64)
    if positions.ndim != 2 or positions.shape[1] != 29:
        raise ValueError("joint_pos must have shape [N, 29]")
    if velocities.shape != positions.shape:
        raise ValueError("joint_vel must match joint_pos shape")
    if indices.shape != (positions.shape[0],):
        raise ValueError("frame_index must have shape [N]")
    if body_quat is None:
        quaternions = np.zeros((positions.shape[0], 4), dtype=np.float32)
        quaternions[:, 0] = 1.0
    else:
        quaternions = np.asarray(body_quat, dtype=np.float32)
        if quaternions.shape != (positions.shape[0], 4):
            raise ValueError("body_quat must have shape [N, 4]")
    return {
        "joint_pos": np.ascontiguousarray(positions),
        "joint_vel": np.ascontiguousarray(velocities),
        "body_quat": np.ascontiguousarray(quaternions),
        "frame_index": np.ascontiguousarray(indices),
    }


def mujoco_to_isaaclab(values: np.ndarray) -> np.ndarray:
    """Reorder final-axis G1 values from bridge/MuJoCo to SONIC IsaacLab order."""
    array = np.asarray(values)
    if array.shape[-1] != 29:
        raise ValueError("G1 joint values must end with 29 elements")
    return np.ascontiguousarray(array[..., G1_MUJOCO_TO_ISAACLAB])
