"""Read Xsens XUDP recordings and decode MXTP02 body-pose datagrams."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import os
from pathlib import Path
import struct
from typing import BinaryIO

XUDP_MAGIC = b"XSENSUDP1"
XUDP_RECORD_HEADER = struct.Struct("<QI")
MXTP_HEADER_SIZE = 24
MXTP02_ITEM = struct.Struct(">I7f")

# MXTP segment identifiers are one-based. This order is defined by the Xsens
# MVN real-time network streaming protocol.
SEGMENT_NAMES = {
    1: "pelvis",
    2: "l5",
    3: "l3",
    4: "t12",
    5: "t8",
    6: "neck",
    7: "head",
    8: "right_shoulder",
    9: "right_upper_arm",
    10: "right_forearm",
    11: "right_hand",
    12: "left_shoulder",
    13: "left_upper_arm",
    14: "left_forearm",
    15: "left_hand",
    16: "right_upper_leg",
    17: "right_lower_leg",
    18: "right_foot",
    19: "right_toe",
    20: "left_upper_leg",
    21: "left_lower_leg",
    22: "left_foot",
    23: "left_toe",
}


class XUDPFormatError(ValueError):
    """Raised when an XUDP recording or embedded MXTP datagram is malformed."""


@dataclass(frozen=True)
class XUDPRecord:
    """One timestamped UDP payload stored in an XUDP recording."""

    timestamp_ns: int
    payload: bytes


class XUDPWriter:
    """Incrementally write timestamped UDP payloads in the XUDP format."""

    def __init__(self, path: str | Path, *, overwrite: bool = False) -> None:
        self.path = Path(path)
        self.overwrite = overwrite
        self._stream: BinaryIO | None = None

    def __enter__(self) -> "XUDPWriter":
        mode = "wb" if self.overwrite else "xb"
        self._stream = self.path.open(mode)
        self._stream.write(XUDP_MAGIC)
        return self

    def write(self, timestamp_ns: int, payload: bytes) -> None:
        if self._stream is None:
            raise RuntimeError("XUDP writer is not open")
        if timestamp_ns < 0:
            raise ValueError("XUDP timestamp cannot be negative")
        self._stream.write(XUDP_RECORD_HEADER.pack(timestamp_ns, len(payload)))
        self._stream.write(payload)

    def flush(self) -> None:
        if self._stream is None:
            raise RuntimeError("XUDP writer is not open")
        self._stream.flush()

    def __exit__(self, *_: object) -> None:
        if self._stream is None:
            return
        self._stream.flush()
        os.fsync(self._stream.fileno())
        self._stream.close()
        self._stream = None


@dataclass(frozen=True)
class MXTPHeader:
    """The common 24-byte header at the start of each MXTP datagram."""

    message_type: str
    sample_counter: int
    datagram_counter: int
    item_count: int
    timecode_ms: int
    character_id: int
    body_segment_count: int
    prop_count: int
    finger_segment_count: int
    payload_size: int


@dataclass(frozen=True)
class SegmentPose:
    """Absolute position and quaternion orientation for one Xsens segment."""

    segment_id: int
    name: str
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseFrame:
    """One decoded MXTP02 pose frame."""

    recording_timestamp_ns: int
    header: MXTPHeader
    segments: tuple[SegmentPose, ...]


def _read_exact(stream: BinaryIO, size: int, context: str) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        raise XUDPFormatError(
            f"truncated {context}: expected {size} bytes, received {len(data)}"
        )
    return data


def iter_xudp_records(path: str | Path) -> Iterator[XUDPRecord]:
    """Yield every timestamped UDP payload in an XUDP recording."""

    recording_path = Path(path)
    with recording_path.open("rb") as stream:
        magic = _read_exact(stream, len(XUDP_MAGIC), "XUDP header")
        if magic != XUDP_MAGIC:
            raise XUDPFormatError(
                f"unsupported XUDP header {magic!r}; expected {XUDP_MAGIC!r}"
            )

        while True:
            record_header = stream.read(XUDP_RECORD_HEADER.size)
            if not record_header:
                return
            if len(record_header) != XUDP_RECORD_HEADER.size:
                raise XUDPFormatError("truncated XUDP record header")

            timestamp_ns, payload_size = XUDP_RECORD_HEADER.unpack(record_header)
            payload = _read_exact(stream, payload_size, "XUDP record payload")
            yield XUDPRecord(timestamp_ns=timestamp_ns, payload=payload)


def parse_mxtp_header(datagram: bytes) -> MXTPHeader:
    """Decode and validate the common MXTP datagram header."""

    if len(datagram) < MXTP_HEADER_SIZE:
        raise XUDPFormatError(
            f"MXTP datagram is {len(datagram)} bytes; header requires 24"
        )

    try:
        message_type = datagram[:6].decode("ascii")
    except UnicodeDecodeError as error:
        raise XUDPFormatError("MXTP message type is not ASCII") from error

    if not message_type.startswith("MXTP"):
        raise XUDPFormatError(f"invalid MXTP message type {message_type!r}")

    payload_size = struct.unpack_from(">H", datagram, 22)[0]
    expected_size = MXTP_HEADER_SIZE + payload_size
    if len(datagram) != expected_size:
        raise XUDPFormatError(
            f"{message_type} length mismatch: header declares {expected_size}, "
            f"received {len(datagram)}"
        )

    return MXTPHeader(
        message_type=message_type,
        sample_counter=struct.unpack_from(">I", datagram, 6)[0],
        datagram_counter=datagram[10],
        item_count=datagram[11],
        timecode_ms=struct.unpack_from(">I", datagram, 12)[0],
        character_id=datagram[16],
        body_segment_count=datagram[17],
        prop_count=datagram[18],
        finger_segment_count=datagram[19],
        payload_size=payload_size,
    )


def parse_mxtp02(datagram: bytes, recording_timestamp_ns: int = 0) -> PoseFrame:
    """Decode one MXTP02 absolute-position/quaternion body-pose datagram."""

    header = parse_mxtp_header(datagram)
    if header.message_type != "MXTP02":
        raise XUDPFormatError(
            f"expected MXTP02 pose data, received {header.message_type}"
        )

    expected_payload_size = header.item_count * MXTP02_ITEM.size
    if header.payload_size != expected_payload_size:
        raise XUDPFormatError(
            f"MXTP02 declares {header.item_count} items but payload size "
            f"{header.payload_size} does not equal {expected_payload_size}"
        )

    segments = []
    offset = MXTP_HEADER_SIZE
    for _ in range(header.item_count):
        segment_id, px, py, pz, qw, qx, qy, qz = MXTP02_ITEM.unpack_from(
            datagram, offset
        )
        segments.append(
            SegmentPose(
                segment_id=segment_id,
                name=SEGMENT_NAMES.get(segment_id, f"segment_{segment_id}"),
                position_m=(px, py, pz),
                quaternion_wxyz=(qw, qx, qy, qz),
            )
        )
        offset += MXTP02_ITEM.size

    return PoseFrame(
        recording_timestamp_ns=recording_timestamp_ns,
        header=header,
        segments=tuple(segments),
    )


def iter_mxtp02_frames(path: str | Path) -> Iterator[PoseFrame]:
    """Yield decoded quaternion-pose frames from an XUDP recording."""

    for record in iter_xudp_records(path):
        if record.payload.startswith(b"MXTP02"):
            yield parse_mxtp02(record.payload, record.timestamp_ns)
