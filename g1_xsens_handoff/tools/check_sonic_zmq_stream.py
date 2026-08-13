#!/usr/bin/env python3
"""Read-only health check for a live SONIC protocol-v1 ZMQ publisher."""

from __future__ import annotations

import argparse
import json
import time


HEADER_SIZE = 1280


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5556")
    parser.add_argument("--topic", default="pose")
    parser.add_argument("--duration", type=float, default=28.0)
    parser.add_argument("--min-messages", type=int, default=100)
    parser.add_argument("--max-gap-ms", type=float, default=150.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    import zmq

    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.RCVHWM, 1)
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, args.topic)
    subscriber.connect(args.endpoint)

    topic_bytes = args.topic.encode("utf-8")
    deadline = time.monotonic() + args.duration
    received = 0
    previous_received_at = None
    maximum_gap = 0.0
    bad_batches = 0
    bad_versions = 0
    poller = zmq.Poller()
    poller.register(subscriber, zmq.POLLIN)

    try:
        while time.monotonic() < deadline:
            events = dict(poller.poll(100))
            if subscriber not in events:
                continue
            message = subscriber.recv()
            received_at = time.monotonic()
            if previous_received_at is not None:
                maximum_gap = max(
                    maximum_gap, received_at - previous_received_at
                )
            previous_received_at = received_at
            received += 1

            if not message.startswith(topic_bytes):
                bad_versions += 1
                continue
            header_start = len(topic_bytes)
            header_raw = message[
                header_start : header_start + HEADER_SIZE
            ].rstrip(b"\x00")
            header = json.loads(header_raw.decode("utf-8"))
            if header.get("v") != 1:
                bad_versions += 1
            shapes = {
                field["name"]: field["shape"]
                for field in header.get("fields", [])
            }
            if (
                shapes.get("joint_pos") != [1, 29]
                or shapes.get("joint_vel") != [1, 29]
                or shapes.get("frame_index") != [1]
            ):
                bad_batches += 1
    finally:
        subscriber.close(linger=0)
        context.term()

    maximum_gap_ms = maximum_gap * 1000.0
    print(
        f"messages={received} max_gap_ms={maximum_gap_ms:.1f} "
        f"bad_batches={bad_batches} bad_versions={bad_versions}"
    )
    if received < args.min_messages:
        print("FAIL: too few live messages")
        return 1
    if bad_batches:
        print("FAIL: publisher sent a batch other than one frame")
        return 1
    if bad_versions:
        print("FAIL: invalid topic or protocol version")
        return 1
    if maximum_gap_ms > args.max_gap_ms:
        print("FAIL: excessive live-stream gap")
        return 1
    print("PASS: latest-only SONIC stream is healthy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
