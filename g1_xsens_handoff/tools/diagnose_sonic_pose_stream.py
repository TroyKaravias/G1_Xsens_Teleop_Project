#!/usr/bin/env python3
"""Measure timing and joint-step continuity of a SONIC pose stream.

Read-only: this subscribes to ZMQ and never publishes robot commands.
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np

from xsens_bridge.g1_retarget import G1_JOINT_NAMES
from xsens_bridge.sonic_zmq import SONIC_ZMQ_HEADER_SIZE


DTYPES = {
    "u8": np.dtype("<u1"),
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"),
    "i64": np.dtype("<i8"),
    "bool": np.dtype("?"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5556")
    parser.add_argument("--topic", default="pose")
    parser.add_argument("--duration", type=float, default=15.0)
    return parser.parse_args()


def unpack_fields(message: bytes, topic: bytes) -> dict[str, np.ndarray]:
    if not message.startswith(topic):
        raise ValueError("unexpected ZMQ topic")
    header_start = len(topic)
    payload_start = header_start + SONIC_ZMQ_HEADER_SIZE
    header = json.loads(
        message[header_start:payload_start].rstrip(b"\x00").decode("utf-8")
    )
    fields: dict[str, np.ndarray] = {}
    offset = payload_start
    for description in header["fields"]:
        dtype = DTYPES[description["dtype"]]
        shape = tuple(description["shape"])
        count = int(np.prod(shape, dtype=np.int64))
        size = count * dtype.itemsize
        fields[description["name"]] = np.frombuffer(
            message, dtype=dtype, count=count, offset=offset
        ).reshape(shape)
        offset += size
    return fields


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(values, q)) if values else 0.0


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")
    import zmq

    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.RCVHWM, 1000)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, args.topic)
    subscriber.connect(args.endpoint)
    poller = zmq.Poller()
    poller.register(subscriber, zmq.POLLIN)

    topic = args.topic.encode("utf-8")
    deadline = time.monotonic() + args.duration
    previous_time: float | None = None
    previous_position: np.ndarray | None = None
    previous_frame_index: int | None = None
    gaps_ms: list[float] = []
    steps: list[np.ndarray] = []
    rates: list[np.ndarray] = []
    messages = 0
    missing_published_frames = 0

    try:
        while time.monotonic() < deadline:
            events = dict(poller.poll(100))
            if subscriber not in events:
                continue
            received_at = time.monotonic()
            fields = unpack_fields(subscriber.recv(), topic)
            position = np.asarray(fields["joint_pos"][-1], dtype=np.float64)
            frame_index = int(fields["frame_index"][-1])
            messages += 1
            if previous_frame_index is not None:
                increment = frame_index - previous_frame_index
                if increment > 1:
                    missing_published_frames += increment - 1
            if previous_time is not None and previous_position is not None:
                dt = received_at - previous_time
                step_vector = np.abs(position - previous_position)
                gaps_ms.append(1000.0 * dt)
                steps.append(step_vector)
                rates.append(step_vector / max(dt, 1e-9))
            previous_time = received_at
            previous_position = position.copy()
            previous_frame_index = frame_index
    finally:
        subscriber.close(linger=0)
        context.term()

    print(
        f"messages={messages} rate={messages / args.duration:.1f}Hz "
        f"index_gaps={missing_published_frames} "
        f"gap_p95={percentile(gaps_ms, 95):.1f}ms "
        f"gap_max={max(gaps_ms, default=0.0):.1f}ms"
    )
    if steps:
        step_array = np.stack(steps)
        rate_array = np.stack(rates)
        overall_sample, overall_joint = np.unravel_index(
            int(np.argmax(step_array)), step_array.shape
        )
        arm_steps = step_array[:, 15:29]
        arm_rates = rate_array[:, 15:29]
        proximal_indices = np.asarray(
            [15, 16, 17, 18, 22, 23, 24, 25], dtype=np.int64
        )
        proximal_steps = step_array[:, proximal_indices]
        proximal_rates = rate_array[:, proximal_indices]
        arm_sample, arm_offset = np.unravel_index(
            int(np.argmax(arm_steps)), arm_steps.shape
        )
        arm_joint = arm_offset + 15
        print(
            f"all_step_p95={np.percentile(step_array, 95):.4f}rad "
            f"all_step_max={step_array[overall_sample, overall_joint]:.4f}rad "
            f"joint={overall_joint}:{G1_JOINT_NAMES[overall_joint]}"
        )
        print(
            f"arm_step_p95={np.percentile(arm_steps, 95):.4f}rad "
            f"arm_step_max={arm_steps[arm_sample, arm_offset]:.4f}rad "
            f"joint={arm_joint}:{G1_JOINT_NAMES[arm_joint]} "
            f"arm_rate_p95={np.percentile(arm_rates, 95):.2f}rad/s "
            f"arm_rate_max={np.max(arm_rates):.2f}rad/s"
        )
        proximal_sample, proximal_offset = np.unravel_index(
            int(np.argmax(proximal_steps)), proximal_steps.shape
        )
        proximal_joint = int(proximal_indices[proximal_offset])
        print(
            f"shoulder_elbow_step_p95={np.percentile(proximal_steps, 95):.4f}rad "
            f"shoulder_elbow_step_max="
            f"{proximal_steps[proximal_sample, proximal_offset]:.4f}rad "
            f"joint={proximal_joint}:{G1_JOINT_NAMES[proximal_joint]} "
            f"shoulder_elbow_rate_p95="
            f"{np.percentile(proximal_rates, 95):.2f}rad/s "
            f"shoulder_elbow_rate_max={np.max(proximal_rates):.2f}rad/s"
        )
    return 0 if messages >= max(1, int(0.8 * 50.0 * args.duration)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
