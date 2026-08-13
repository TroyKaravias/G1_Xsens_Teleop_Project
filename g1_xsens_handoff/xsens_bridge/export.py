"""Portable exports for decoded Xsens body-pose recordings."""

from __future__ import annotations

from collections.abc import Iterable, Iterator
import csv
from pathlib import Path

from .xudp import PoseFrame, SEGMENT_NAMES, iter_mxtp02_frames


def iter_resampled_frames(
    frames: Iterable[PoseFrame], output_fps: float
) -> Iterator[PoseFrame]:
    """Select frames on a fixed time grid without inventing interpolated poses."""

    if output_fps <= 0:
        raise ValueError("output_fps must be greater than zero")

    first_timestamp_ns: int | None = None
    next_time_ns = 0.0
    period_ns = 1_000_000_000 / output_fps

    for frame in frames:
        if first_timestamp_ns is None:
            first_timestamp_ns = frame.recording_timestamp_ns

        elapsed_ns = frame.recording_timestamp_ns - first_timestamp_ns
        if elapsed_ns < next_time_ns:
            continue

        yield frame
        next_time_ns += period_ns


def export_pose_csv(
    recording: str | Path, output: str | Path, output_fps: float = 50.0
) -> int:
    """Export a portable wide CSV of Xsens positions and quaternions."""

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    segment_names = [SEGMENT_NAMES[index] for index in sorted(SEGMENT_NAMES)]

    header = ["time_s", "sample_counter"]
    for name in segment_names:
        header.extend(
            [
                f"{name}.px_m",
                f"{name}.py_m",
                f"{name}.pz_m",
                f"{name}.qw",
                f"{name}.qx",
                f"{name}.qy",
                f"{name}.qz",
            ]
        )

    frame_count = 0
    first_timestamp_ns: int | None = None
    with output_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for frame in iter_resampled_frames(
            iter_mxtp02_frames(recording), output_fps
        ):
            if first_timestamp_ns is None:
                first_timestamp_ns = frame.recording_timestamp_ns
            segment_by_name = {segment.name: segment for segment in frame.segments}
            row: list[float | int] = [
                round(
                    (frame.recording_timestamp_ns - first_timestamp_ns)
                    / 1_000_000_000,
                    9,
                ),
                frame.header.sample_counter,
            ]
            for name in segment_names:
                segment = segment_by_name[name]
                row.extend(segment.position_m)
                row.extend(segment.quaternion_wxyz)
            writer.writerow(row)
            frame_count += 1

    return frame_count
