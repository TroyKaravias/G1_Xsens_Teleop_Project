"""Command-line inspection tools for Xsens XUDP recordings."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import sys

from .export import export_pose_csv
from .isaac_mapping import map_xsens_to_isaac
from .stream import (
    DEFAULT_XSENS_PORT,
    listen_for_mxtp02,
    record_xudp,
    replay_xudp,
)
from .xudp import iter_mxtp02_frames, iter_xudp_records, parse_mxtp_header


def _summary(path: Path) -> int:
    packet_counts: Counter[str] = Counter()
    packet_lengths: Counter[int] = Counter()
    first_timestamp_ns: int | None = None
    last_timestamp_ns: int | None = None
    first_pose_header = None

    for record in iter_xudp_records(path):
        header = parse_mxtp_header(record.payload)
        packet_counts[header.message_type] += 1
        packet_lengths[len(record.payload)] += 1
        first_timestamp_ns = (
            record.timestamp_ns
            if first_timestamp_ns is None
            else first_timestamp_ns
        )
        last_timestamp_ns = record.timestamp_ns
        if header.message_type == "MXTP02" and first_pose_header is None:
            first_pose_header = header

    if first_timestamp_ns is None or last_timestamp_ns is None:
        print("Recording contains no packets.", file=sys.stderr)
        return 1

    duration_seconds = (last_timestamp_ns - first_timestamp_ns) / 1_000_000_000
    pose_count = packet_counts["MXTP02"]
    pose_rate_hz = pose_count / duration_seconds if duration_seconds else 0.0

    print(f"File: {path}")
    print(f"Duration: {duration_seconds:.3f} s")
    print(f"Total packets: {sum(packet_counts.values())}")
    print(f"MXTP02 pose frames: {pose_count}")
    print(f"Approximate pose rate: {pose_rate_hz:.2f} Hz")
    if first_pose_header is not None:
        print(f"Body segments: {first_pose_header.body_segment_count}")
        print(f"Props: {first_pose_header.prop_count}")
        print(f"Finger segments: {first_pose_header.finger_segment_count}")
    print("Packet types:")
    for message_type, count in sorted(packet_counts.items()):
        print(f"  {message_type}: {count}")
    return 0


def _frame(path: Path, index: int) -> int:
    if index < 0:
        print("--index must be non-negative", file=sys.stderr)
        return 2

    for current_index, frame in enumerate(iter_mxtp02_frames(path)):
        if current_index != index:
            continue
        print(
            f"MXTP02 frame {index}: sample={frame.header.sample_counter}, "
            f"timecode={frame.header.timecode_ms} ms, "
            f"recording_timestamp={frame.recording_timestamp_ns} ns"
        )
        print("segment              position_m (x, y, z)       quaternion (w, x, y, z)")
        for segment in frame.segments:
            position = ", ".join(f"{value: .4f}" for value in segment.position_m)
            quaternion = ", ".join(
                f"{value: .5f}" for value in segment.quaternion_wxyz
            )
            print(f"{segment.name:20} ({position})  ({quaternion})")
        return 0

    print(f"Recording does not contain MXTP02 frame {index}", file=sys.stderr)
    return 1


def _mapped_frame(path: Path, index: int) -> int:
    if index < 0:
        print("--index must be non-negative", file=sys.stderr)
        return 2

    for current_index, frame in enumerate(iter_mxtp02_frames(path)):
        if current_index != index:
            continue
        mapped = map_xsens_to_isaac(frame)
        print(
            f"IsaacTeleop frame {index}: sample={mapped.sample_counter}, "
            f"joints={len(mapped.joints)}"
        )
        print("isaac_joint          xsens_source         position_m (x, y, z)")
        for joint in mapped.joints:
            position = ", ".join(f"{value: .4f}" for value in joint.position_m)
            print(f"{joint.name:20} {joint.source_segment:20} ({position})")
        return 0

    print(f"Recording does not contain MXTP02 frame {index}", file=sys.stderr)
    return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect Xsens XUDP recordings without connecting to a robot."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    summary_parser = subparsers.add_parser("summary", help="summarize a recording")
    summary_parser.add_argument("recording", type=Path)

    frame_parser = subparsers.add_parser(
        "frame", help="print one MXTP02 quaternion-pose frame"
    )
    frame_parser.add_argument("recording", type=Path)
    frame_parser.add_argument("--index", type=int, default=0)

    mapped_parser = subparsers.add_parser(
        "mapped-frame", help="print the provisional IsaacTeleop 24-joint mapping"
    )
    mapped_parser.add_argument("recording", type=Path)
    mapped_parser.add_argument("--index", type=int, default=0)

    listen_parser = subparsers.add_parser(
        "listen", help="monitor live MXTP02 UDP poses without commanding a robot"
    )
    listen_parser.add_argument("--bind", default="0.0.0.0")
    listen_parser.add_argument("--port", type=int, default=DEFAULT_XSENS_PORT)
    listen_parser.add_argument("--duration", type=float)
    listen_parser.add_argument("--stale-ms", type=float, default=250.0)

    record_parser = subparsers.add_parser(
        "record",
        help="record live Xsens UDP packets without commanding a robot",
    )
    record_parser.add_argument("output", type=Path)
    record_parser.add_argument("--bind", default="0.0.0.0")
    record_parser.add_argument("--port", type=int, default=DEFAULT_XSENS_PORT)
    record_parser.add_argument("--duration", type=float)
    record_parser.add_argument(
        "--all-packets",
        action="store_true",
        help="record every valid MXTP type instead of only MXTP02",
    )
    record_parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace an existing output file",
    )

    replay_parser = subparsers.add_parser(
        "replay", help="replay an XUDP recording as a live UDP stream"
    )
    replay_parser.add_argument("recording", type=Path)
    replay_parser.add_argument("--host", default="127.0.0.1")
    replay_parser.add_argument("--port", type=int, default=DEFAULT_XSENS_PORT)
    replay_parser.add_argument("--speed", type=float, default=1.0)
    replay_parser.add_argument(
        "--all-packets",
        action="store_true",
        help="send every recorded MXTP type instead of only MXTP02",
    )

    export_parser = subparsers.add_parser(
        "export-csv",
        help="export portable Xsens positions and quaternions at a fixed rate",
    )
    export_parser.add_argument("recording", type=Path)
    export_parser.add_argument("output", type=Path)
    export_parser.add_argument("--fps", type=float, default=50.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "summary":
        return _summary(args.recording)
    if args.command == "frame":
        return _frame(args.recording, args.index)
    if args.command == "mapped-frame":
        return _mapped_frame(args.recording, args.index)
    if args.command == "listen":
        health = listen_for_mxtp02(
            bind_host=args.bind,
            port=args.port,
            duration_seconds=args.duration,
            stale_timeout_seconds=args.stale_ms / 1000,
        )
        print(
            f"Stopped: frames={health.received_frames}, "
            f"missing={health.missing_frames}, "
            f"malformed={health.malformed_packets}"
        )
        return 0 if health.received_frames else 1
    if args.command == "record":
        try:
            stats = record_xudp(
                output=args.output,
                bind_host=args.bind,
                port=args.port,
                duration_seconds=args.duration,
                message_type=None if args.all_packets else "MXTP02",
                overwrite=args.overwrite,
            )
        except FileExistsError:
            print(
                f"Output already exists: {args.output} "
                "(use --overwrite to replace it)",
                file=sys.stderr,
            )
            return 2
        print(
            f"Recording complete: packets={stats.recorded_packets}, "
            f"missing={stats.missing_frames}, "
            f"ignored={stats.ignored_packets}, "
            f"malformed={stats.malformed_packets}, "
            f"elapsed={stats.elapsed_seconds:.3f}s, file={args.output}"
        )
        return 0 if stats.recorded_packets else 1
    if args.command == "replay":
        sent = replay_xudp(
            recording=args.recording,
            host=args.host,
            port=args.port,
            speed=args.speed,
            message_type=None if args.all_packets else "MXTP02",
        )
        print(f"Replay complete: sent {sent} packets")
        return 0
    if args.command == "export-csv":
        count = export_pose_csv(args.recording, args.output, args.fps)
        print(f"Export complete: wrote {count} frames to {args.output}")
        return 0
    raise AssertionError(f"unhandled command {args.command}")
