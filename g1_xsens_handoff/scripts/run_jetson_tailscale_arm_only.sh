#!/usr/bin/env bash
set -euo pipefail

# Restrained first-stage physical validation over the Tailscale Xsens path.
# This publisher contains no Unitree SDK, DDS, ROS, or motor output. The G1
# controller is a separate process and must remain under a dedicated e-stop
# operator. Legs and waist are held at the validated neutral reference.

HANDOFF_ROOT="${HANDOFF_ROOT:-$HOME/g1_xsens_direct/g1_xsens_handoff}"
PYTHON_BIN="${PYTHON_BIN:-$HOME/.venvs/g1_xsens/bin/python}"

if [[ ! -d "$HANDOFF_ROOT/xsens_bridge" ]]; then
  echo "Xsens handoff not found: $HANDOFF_ROOT" >&2
  exit 1
fi
if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Publisher Python not found or not executable: $PYTHON_BIN" >&2
  exit 1
fi

cd "$HANDOFF_ROOT"

echo "RESTRAINED TAILSCALE ARM-ONLY PUBLISHER"
echo "MVN UDP: 0.0.0.0:9763 -> local SONIC ZMQ: 127.0.0.1:5556"
echo "Legs/waist: locked to validated neutral | wrists: neutral"
echo "Arm speed cap: ${ARM_MAX_SPEED:-4.0} rad/s"
echo "Requires gantry support and a dedicated physical e-stop operator."

exec env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_sonic.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind tcp://127.0.0.1:5556 \
  --calibration-seconds "${CALIBRATION_SECONDS:-15}" \
  --calibration-sequence ntf \
  --stale-ms "${STALE_MS:-750}" \
  --fps 50 \
  --batch-frames 1 \
  --physical-arm-only \
  --arm-max-speed "${ARM_MAX_SPEED:-4.0}" \
  --max-joint-speed 12.0
