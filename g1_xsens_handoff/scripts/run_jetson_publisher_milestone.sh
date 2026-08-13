#!/usr/bin/env bash
set -euo pipefail

# Jetson-local equivalent of the known-working 2026-07-29 tunnel publisher.
# The Xsens-to-G1 reference behavior is unchanged; only the transport changes:
# MVN sends UDP directly to this Jetson, and SONIC consumes ZMQ on loopback.
# This publisher contains no Unitree SDK, DDS, ROS, or motor output.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-$HOME/.venvs/g1_xsens/bin/python}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Publisher Python not found or not executable: $PYTHON_BIN" >&2
  echo "Set PYTHON_BIN to the Python containing numpy and pyzmq." >&2
  exit 1
fi

cd "$ROOT_DIR"

echo "JETSON-LOCAL MILESTONE POSE PUBLISHER"
echo "MVN UDP: 0.0.0.0:9763 -> local SONIC ZMQ: 127.0.0.1:5556"
echo "Calibration: N-pose for 40%, then T-pose and hold"
echo "No Mac, WSL, or SSH forwarding is used."

exec env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_sonic.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind tcp://127.0.0.1:5556 \
  --calibration-seconds "${CALIBRATION_SECONDS:-12}" \
  --calibration-sequence nt \
  --stale-ms "${STALE_MS:-250}" \
  --fps 50 \
  --batch-frames 1
