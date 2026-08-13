#!/usr/bin/env bash
set -euo pipefail

# Jetson-local adaptation of run_existing_wsl_publisher.sh, the latest
# physical teleoperation profile that previously published through the SSH
# tunnel. The retargeting options are intentionally preserved; only the
# machine paths and transport topology change.

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

echo "LATEST TUNNEL PROFILE — JETSON LOCAL, NO TUNNEL"
echo "MVN Ethernet UDP: 0.0.0.0:9763"
echo "SONIC loopback ZMQ: tcp://127.0.0.1:5556"
echo "Calibration: N-pose -> T-pose -> arms straight forward"
echo "Arms: ${ARM_MAX_SPEED:-17.0} rad/s | internal cap: 12.0 rad/s"
echo "Leg blend: ${LEG_BLEND:-1.00} | waist: relative tracking"
echo "Requires support/harness and a dedicated physical E-stop operator."

exec env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_sonic.py \
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
