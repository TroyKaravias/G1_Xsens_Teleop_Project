#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

cd "$ROOT_DIR"

exec env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_sonic.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind tcp://127.0.0.1:5556 \
  --calibration-seconds "${CALIBRATION_SECONDS:-15}" \
  --calibration-sequence ntf \
  --stale-ms 750 \
  --fps 50 \
  --batch-frames 1 \
  --physical-full-body \
  --physical-leg-blend "${LEG_BLEND:-1.00}" \
  --physical-hip-limit-deg "${HIP_LIMIT_DEG:-20.0}" \
  --physical-foot-limit-deg "${FOOT_LIMIT_DEG:-15.0}" \
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
  --track-lifted-hip-roll
