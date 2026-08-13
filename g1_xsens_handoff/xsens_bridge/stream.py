"""Live UDP reception and offline replay for Xsens MXTP datagrams."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import socket
import threading
import time

from .xudp import (
    PoseFrame,
    XUDPFormatError,
    XUDPWriter,
    iter_xudp_records,
    parse_mxtp02,
    parse_mxtp_header,
)

DEFAULT_XSENS_PORT = 9763


@dataclass(frozen=True)
class ReceivedPose:
    """One live pose paired with its local monotonic receive time."""

    frame: PoseFrame
    received_at: float
    source: tuple[str, int]


@dataclass
class StreamHealth:
    """Incremental health statistics for an MXTP02 stream."""

    received_frames: int = 0
    missing_frames: int = 0
    duplicate_frames: int = 0
    out_of_order_frames: int = 0
    counter_resets: int = 0
    malformed_packets: int = 0
    last_sample_counter: int | None = None
    last_packet_monotonic: float | None = None

    def observe_frame(
        self,
        sample_counter: int,
        now: float | None = None,
        counter_reset_after_seconds: float | None = None,
    ) -> bool:
        """Record a valid pose frame and return whether it advances the stream."""

        timestamp = time.monotonic() if now is None else now
        self.received_frames += 1

        if self.last_sample_counter is not None:
            increment = (sample_counter - self.last_sample_counter) & 0xFFFFFFFF
            if increment == 0:
                self.duplicate_frames += 1
                return False
            # Forward movement, including a legitimate uint32 wrap, is always
            # less than half the counter range. A larger modular increment is
            # a packet that arrived behind the newest accepted frame.
            if increment >= 0x80000000:
                # MVN may restart its uint32 sample counter at zero after a
                # network interruption. Once the previously accepted stream
                # is stale, treat a lower counter as the beginning of a new
                # epoch instead of rejecting the restarted stream forever.
                if (
                    counter_reset_after_seconds is not None
                    and self.last_packet_monotonic is not None
                    and timestamp - self.last_packet_monotonic
                    >= counter_reset_after_seconds
                ):
                    self.counter_resets += 1
                    self.last_sample_counter = sample_counter
                    self.last_packet_monotonic = timestamp
                    return True
                self.out_of_order_frames += 1
                return False
            if increment > 1:
                self.missing_frames += increment - 1

        self.last_sample_counter = sample_counter
        self.last_packet_monotonic = timestamp
        return True

    def is_stale(
        self, timeout_seconds: float, now: float | None = None
    ) -> bool:
        """Return true when no valid pose has arrived within the timeout."""

        if self.last_packet_monotonic is None:
            return True
        timestamp = time.monotonic() if now is None else now
        return timestamp - self.last_packet_monotonic > timeout_seconds


@dataclass(frozen=True)
class RecordingStats:
    """Final counters from a receive-only XUDP recording session."""

    recorded_packets: int
    ignored_packets: int
    malformed_packets: int
    missing_frames: int
    elapsed_seconds: float


class LatestPoseReceiver:
    """Background UDP receiver that exposes only the newest valid MXTP02 pose."""

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        port: int = DEFAULT_XSENS_PORT,
        counter_reset_after_seconds: float = 0.75,
    ) -> None:
        self.bind_host = bind_host
        self.port = port
        self.counter_reset_after_seconds = counter_reset_after_seconds
        self.health = StreamHealth()
        self._latest: ReceivedPose | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("Receiver has already been started")
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.settimeout(0.1)
        receiver.bind((self.bind_host, self.port))
        self._socket = receiver
        self._thread = threading.Thread(
            target=self._run,
            name="xsens-udp-receiver",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def latest(self) -> ReceivedPose | None:
        with self._lock:
            return self._latest

    def __enter__(self) -> "LatestPoseReceiver":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _run(self) -> None:
        receiver = self._socket
        if receiver is None:
            return
        try:
            while not self._stop.is_set():
                try:
                    datagram, source = receiver.recvfrom(65_535)
                except (TimeoutError, OSError):
                    continue
                now = time.monotonic()
                try:
                    frame = parse_mxtp02(datagram)
                except XUDPFormatError:
                    with self._lock:
                        self.health.malformed_packets += 1
                    continue
                with self._lock:
                    if self.health.observe_frame(
                        frame.header.sample_counter,
                        now,
                        self.counter_reset_after_seconds,
                    ):
                        self._latest = ReceivedPose(frame, now, source)
        finally:
            receiver.close()
            self._socket = None


def listen_for_mxtp02(
    bind_host: str,
    port: int = DEFAULT_XSENS_PORT,
    duration_seconds: float | None = None,
    status_interval_seconds: float = 1.0,
    stale_timeout_seconds: float = 0.25,
) -> StreamHealth:
    """Listen for live Xsens UDP packets and print pose-stream health.

    This monitor only decodes data. It does not publish ROS messages or issue
    robot commands.
    """

    health = StreamHealth()
    start = time.monotonic()
    interval_start = start
    interval_frames = 0
    last_source: tuple[str, int] | None = None

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind((bind_host, port))
        receiver.settimeout(0.1)
        print(f"Listening for Xsens MXTP02 on {bind_host}:{port}")

        while duration_seconds is None or time.monotonic() - start < duration_seconds:
            try:
                datagram, source = receiver.recvfrom(65_535)
            except TimeoutError:
                datagram = None

            now = time.monotonic()
            if datagram is not None:
                try:
                    header = parse_mxtp_header(datagram)
                    if header.message_type == "MXTP02":
                        frame: PoseFrame = parse_mxtp02(datagram)
                        if health.observe_frame(
                            frame.header.sample_counter,
                            now,
                            stale_timeout_seconds,
                        ):
                            interval_frames += 1
                        last_source = source
                except XUDPFormatError:
                    health.malformed_packets += 1

            elapsed = now - interval_start
            if elapsed >= status_interval_seconds:
                rate = interval_frames / elapsed
                state = (
                    "STALE"
                    if health.is_stale(stale_timeout_seconds, now)
                    else "LIVE"
                )
                source_label = (
                    f"{last_source[0]}:{last_source[1]}"
                    if last_source is not None
                    else "waiting"
                )
                print(
                    f"{state}  {rate:6.1f} Hz  frames={health.received_frames}  "
                    f"missing={health.missing_frames}  "
                    f"duplicate={health.duplicate_frames}  "
                    f"reordered={health.out_of_order_frames}  "
                    f"resets={health.counter_resets}  "
                    f"malformed={health.malformed_packets}  source={source_label}"
                )
                interval_start = now
                interval_frames = 0

    return health


def record_xudp(
    output: str | Path,
    bind_host: str = "0.0.0.0",
    port: int = DEFAULT_XSENS_PORT,
    duration_seconds: float | None = None,
    status_interval_seconds: float = 1.0,
    message_type: str | None = "MXTP02",
    overwrite: bool = False,
) -> RecordingStats:
    """Record valid Xsens UDP datagrams without publishing robot commands."""

    if duration_seconds is not None and duration_seconds <= 0:
        raise ValueError("recording duration must be greater than zero")
    if status_interval_seconds <= 0:
        raise ValueError("status interval must be greater than zero")

    health = StreamHealth()
    recorded = 0
    ignored = 0
    malformed = 0
    started = time.monotonic()
    interval_started = started
    interval_packets = 0
    last_source: tuple[str, int] | None = None

    with XUDPWriter(output, overwrite=overwrite) as writer:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind((bind_host, port))
            receiver.settimeout(0.1)
            print(
                f"Recording Xsens UDP on {bind_host}:{port} to {output} "
                f"({message_type or 'all MXTP packets'})"
            )
            print("Press Ctrl+C to stop and finalize the recording.")

            try:
                while (
                    duration_seconds is None
                    or time.monotonic() - started < duration_seconds
                ):
                    try:
                        datagram, source = receiver.recvfrom(65_535)
                    except TimeoutError:
                        datagram = None

                    now = time.monotonic()
                    if datagram is not None:
                        try:
                            header = parse_mxtp_header(datagram)
                        except XUDPFormatError:
                            malformed += 1
                        else:
                            if (
                                message_type is not None
                                and header.message_type != message_type
                            ):
                                ignored += 1
                            else:
                                timestamp_ns = time.monotonic_ns()
                                writer.write(timestamp_ns, datagram)
                                recorded += 1
                                interval_packets += 1
                                last_source = source
                                if header.message_type == "MXTP02":
                                    health.observe_frame(
                                        header.sample_counter, now
                                    )

                    interval_elapsed = now - interval_started
                    if interval_elapsed >= status_interval_seconds:
                        writer.flush()
                        rate = interval_packets / interval_elapsed
                        source_label = (
                            f"{last_source[0]}:{last_source[1]}"
                            if last_source is not None
                            else "waiting"
                        )
                        print(
                            f"RECORDING  {rate:6.1f} Hz  "
                            f"packets={recorded}  missing={health.missing_frames}  "
                            f"duplicate={health.duplicate_frames}  "
                            f"reordered={health.out_of_order_frames}  "
                            f"ignored={ignored}  malformed={malformed}  "
                            f"source={source_label}"
                        )
                        interval_started = now
                        interval_packets = 0
            except KeyboardInterrupt:
                print("\nStopping and finalizing recording...")

    return RecordingStats(
        recorded_packets=recorded,
        ignored_packets=ignored,
        malformed_packets=malformed,
        missing_frames=health.missing_frames,
        elapsed_seconds=time.monotonic() - started,
    )


def replay_xudp(
    recording: str | Path,
    host: str,
    port: int = DEFAULT_XSENS_PORT,
    speed: float = 1.0,
    message_type: str | None = "MXTP02",
) -> int:
    """Replay recorded Xsens datagrams over UDP using recorded timing."""

    if speed <= 0:
        raise ValueError("replay speed must be greater than zero")

    sent = 0
    first_recording_timestamp_ns: int | None = None
    replay_start = time.monotonic()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        for record in iter_xudp_records(recording):
            header = parse_mxtp_header(record.payload)
            if message_type is not None and header.message_type != message_type:
                continue

            if first_recording_timestamp_ns is None:
                first_recording_timestamp_ns = record.timestamp_ns
                replay_start = time.monotonic()

            target_elapsed = (
                record.timestamp_ns - first_recording_timestamp_ns
            ) / 1_000_000_000 / speed
            remaining = target_elapsed - (time.monotonic() - replay_start)
            if remaining > 0:
                time.sleep(remaining)

            sender.sendto(record.payload, (host, port))
            sent += 1

    return sent
