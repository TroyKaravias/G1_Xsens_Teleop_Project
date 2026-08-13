#!/usr/bin/env bash
set -euo pipefail

HANDOFF_ROOT="${HOME}/g1_xsens_handoff"
CONDA_SETUP="${HOME}/miniconda3/etc/profile.d/conda.sh"

if [[ ! -f "${CONDA_SETUP}" ]]; then
  echo "Conda setup not found: ${CONDA_SETUP}" >&2
  exit 1
fi

cd "${HANDOFF_ROOT}"
source "${CONDA_SETUP}"
conda activate protomotions_mujoco

echo "PHYSICAL G1 POSE MODE"
echo "Arms: ${ARM_MAX_SPEED:-17.0} rad/s | internal cap: 12.0 rad/s | Legs: ${LEG_BLEND:-1.00} | Waist: relative"
echo "Requires support/harness, E-stop operator, and ZMQ tunnel."

PYTHONPATH=. python tools/live_xsens_sonic.py \
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
