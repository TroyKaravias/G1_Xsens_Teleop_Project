#!/usr/bin/env bash
set -euo pipefail

HANDOFF_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/home/hugh/.venvs/g1_xsens/bin/python}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Bridge Python not found: ${PYTHON_BIN}" >&2
  exit 1
fi

cd "${HANDOFF_ROOT}"

echo "EXISTING PHYSICAL G1 POSE PUBLISHER (WSL)"
echo "Calibration: N-pose -> T-pose -> arms straight forward"
echo "Publisher only: this does not start SONIC or motor output."
echo "Requires the reverse SSH tunnel to remain open."

exec env PYTHONPATH=. "${PYTHON_BIN}" tools/live_xsens_sonic.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind tcp://127.0.0.1:5556 \
  --calibration-seconds "${CALIBRATION_SECONDS:-15}" \
  --calibration-sequence ntf \
  --stale-ms 750 \
  --batch-frames 1 \
  --physical-full-body \
  --physical-leg-blend "${LEG_BLEND:-1.00}" \
  --arm-max-speed "${ARM_MAX_SPEED:-17.0}" \
  --max-joint-speed 12.0 \
  --torso-gain 0.75 \
  --track-root-heading \
  --root-heading-gain 1.00 \
  --root-heading-sign 1 \
  --root-heading-limit-deg 360.0 \
  --root-heading-deadband-deg 2.0 \
  --root-heading-max-rate-deg-s 180.0 \
  --track-relative-waist \
  --track-foot-heading \
  --track-lifted-hip-roll \
  --track-wrist-pitch \
  --wrist-pitch-sign 1 \
  --left-wrist-pitch-sign -1 \
  --track-wrists
