from __future__ import annotations

import socket
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path

from xsens_bridge.export import iter_resampled_frames
from xsens_bridge.isaac_mapping import map_xsens_to_isaac
from xsens_bridge.stream import StreamHealth, record_xudp
from xsens_bridge.xudp import (
    XUDPFormatError,
    XUDP_MAGIC,
    XUDPWriter,
    iter_xudp_records,
    parse_mxtp02,
)


def make_mxtp02() -> bytes:
    item = struct.pack(
        ">I7f",
        1,
        1.0,
        2.0,
        3.0,
        1.0,
        0.0,
        0.0,
        0.0,
    )
    header = (
        b"MXTP02"
        + struct.pack(">I", 42)
        + bytes([0x80, 1])
        + struct.pack(">I", 1234)
        + bytes([0, 1, 0, 0])
        + b"\x00\x00"
        + struct.pack(">H", len(item))
    )
    return header + item


class XUDPTests(unittest.TestCase):
    def test_parse_mxtp02_pose(self) -> None:
        frame = parse_mxtp02(make_mxtp02(), recording_timestamp_ns=999)

        self.assertEqual(frame.recording_timestamp_ns, 999)
        self.assertEqual(frame.header.sample_counter, 42)
        self.assertEqual(frame.header.item_count, 1)
        self.assertEqual(frame.segments[0].name, "pelvis")
        self.assertEqual(frame.segments[0].position_m, (1.0, 2.0, 3.0))
        self.assertEqual(frame.segments[0].quaternion_wxyz, (1.0, 0.0, 0.0, 0.0))

    def test_rejects_wrong_message_type(self) -> None:
        datagram = b"MXTP01" + make_mxtp02()[6:]
        with self.assertRaisesRegex(XUDPFormatError, "expected MXTP02"):
            parse_mxtp02(datagram)

    def test_iterates_xudp_wrapper(self) -> None:
        datagram = make_mxtp02()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.xudp"
            path.write_bytes(
                XUDP_MAGIC + struct.pack("<QI", 123, len(datagram)) + datagram
            )
            records = list(iter_xudp_records(path))

        self.assertEqual(len(records), 1)
        self.assertEqual(records[0].timestamp_ns, 123)
        self.assertEqual(records[0].payload, datagram)

    def test_writer_round_trip(self) -> None:
        first = make_mxtp02()
        second = b"MXTP01" + first[6:]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "recorded.xudp"
            with XUDPWriter(path) as writer:
                writer.write(100, first)
                writer.write(200, second)
            records = list(iter_xudp_records(path))

        self.assertEqual(
            [(record.timestamp_ns, record.payload) for record in records],
            [(100, first), (200, second)],
        )

    def test_writer_refuses_to_overwrite_by_default(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "existing.xudp"
            path.write_bytes(b"keep me")
            with self.assertRaises(FileExistsError):
                with XUDPWriter(path):
                    pass
            self.assertEqual(path.read_bytes(), b"keep me")

    def test_records_live_udp_and_round_trips(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]

        datagram = make_mxtp02()
        results = []
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "live.xudp"
            recorder = threading.Thread(
                target=lambda: results.append(
                    record_xudp(
                        path,
                        bind_host="127.0.0.1",
                        port=port,
                        duration_seconds=0.35,
                        status_interval_seconds=1.0,
                    )
                )
            )
            recorder.start()
            time.sleep(0.05)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                for _ in range(3):
                    sender.sendto(datagram, ("127.0.0.1", port))
                    time.sleep(0.01)
            recorder.join(timeout=2.0)

            self.assertFalse(recorder.is_alive())
            records = list(iter_xudp_records(path))

        self.assertEqual(len(records), 3)
        self.assertTrue(all(record.payload == datagram for record in records))
        self.assertEqual(results[0].recorded_packets, 3)
        self.assertEqual(results[0].malformed_packets, 0)

    def test_rejects_bad_magic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.xudp"
            path.write_bytes(b"NOT_XUDP!")
            with self.assertRaisesRegex(XUDPFormatError, "unsupported XUDP"):
                list(iter_xudp_records(path))

    def test_maps_complete_xsens_frame_to_24_isaac_joints(self) -> None:
        items = []
        for segment_id in range(1, 24):
            items.append(
                struct.pack(
                    ">I7f",
                    segment_id,
                    float(segment_id),
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                )
            )
        payload = b"".join(items)
        header = (
            b"MXTP02"
            + struct.pack(">I", 42)
            + bytes([0x80, 23])
            + struct.pack(">I", 1234)
            + bytes([0, 23, 0, 0])
            + b"\x00\x00"
            + struct.pack(">H", len(payload))
        )

        mapped = map_xsens_to_isaac(parse_mxtp02(header + payload))

        self.assertEqual(len(mapped.joints), 24)
        self.assertEqual(mapped.joints[0].name, "pelvis")
        self.assertEqual(mapped.joints[0].source_segment, "pelvis")
        self.assertEqual(mapped.joints[20].source_segment, "left_hand")
        self.assertEqual(mapped.joints[22].source_segment, "left_hand")

    def test_stream_health_detects_missing_and_stale_frames(self) -> None:
        health = StreamHealth()
        health.observe_frame(100, now=1.0)
        health.observe_frame(103, now=1.01)

        self.assertEqual(health.received_frames, 2)
        self.assertEqual(health.missing_frames, 2)
        self.assertFalse(health.is_stale(0.25, now=1.20))
        self.assertTrue(health.is_stale(0.25, now=1.30))

    def test_stream_health_ignores_duplicate_and_out_of_order_frames(self) -> None:
        health = StreamHealth()

        self.assertTrue(health.observe_frame(100, now=1.0))
        self.assertTrue(health.observe_frame(101, now=1.01))
        self.assertFalse(health.observe_frame(101, now=1.02))
        self.assertFalse(health.observe_frame(99, now=1.03))
        self.assertTrue(health.observe_frame(102, now=1.04))

        self.assertEqual(health.missing_frames, 0)
        self.assertEqual(health.duplicate_frames, 1)
        self.assertEqual(health.out_of_order_frames, 1)
        self.assertEqual(health.last_sample_counter, 102)
        self.assertEqual(health.last_packet_monotonic, 1.04)

    def test_stream_health_accepts_uint32_wrap(self) -> None:
        health = StreamHealth()
        health.observe_frame(0xFFFFFFFE, now=1.0)
        health.observe_frame(1, now=1.01)

        self.assertEqual(health.missing_frames, 2)
        self.assertEqual(health.out_of_order_frames, 0)
        self.assertEqual(health.last_sample_counter, 1)

    def test_stream_health_accepts_counter_reset_after_stale_gap(self) -> None:
        health = StreamHealth()
        health.observe_frame(10_000, now=1.0)

        self.assertFalse(
            health.observe_frame(
                5,
                now=1.5,
                counter_reset_after_seconds=0.75,
            )
        )
        self.assertTrue(
            health.observe_frame(
                6,
                now=1.76,
                counter_reset_after_seconds=0.75,
            )
        )
        self.assertTrue(
            health.observe_frame(
                7,
                now=1.77,
                counter_reset_after_seconds=0.75,
            )
        )

        self.assertEqual(health.out_of_order_frames, 1)
        self.assertEqual(health.counter_resets, 1)
        self.assertEqual(health.missing_frames, 0)
        self.assertEqual(health.last_sample_counter, 7)
        self.assertEqual(health.last_packet_monotonic, 1.77)

    def test_resamples_by_timestamp(self) -> None:
        base = parse_mxtp02(make_mxtp02())
        frames = [
            type(base)(
                recording_timestamp_ns=timestamp,
                header=base.header,
                segments=base.segments,
            )
            for timestamp in (0, 4_000_000, 8_000_000, 12_000_000, 20_000_000)
        ]

        selected = list(iter_resampled_frames(frames, output_fps=50.0))

        self.assertEqual(
            [frame.recording_timestamp_ns for frame in selected],
            [0, 20_000_000],
        )


if __name__ == "__main__":
    unittest.main()
