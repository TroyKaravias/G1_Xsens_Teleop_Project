#!/usr/bin/env bash
set -euo pipefail

# EXPERIMENTAL sidecar profile. The proven publisher and controller are not
# modified. Stop this script and run run_jetson_current_proven_arms.sh to
# return immediately to the proven path.

HANDOFF_ROOT="${HANDOFF_ROOT:-$HOME/g1_xsens_direct/g1_xsens_handoff}"
PYTHON_BIN="${PYTHON_BIN:-$HOME/.venvs/g1_xsens/bin/python}"

cd "$HANDOFF_ROOT"

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "${PUBLISHER_PID:-}" ]]; then
    kill "$PUBLISHER_PID" 2>/dev/null || true
    wait "$PUBLISHER_PID" 2>/dev/null || true
  fi
  if [[ -n "${SIDECAR_PID:-}" ]]; then
    kill "$SIDECAR_PID" 2>/dev/null || true
    wait "$SIDECAR_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

echo "EXPERIMENTAL KICK-STABILITY SIDECAR"
echo "MVN UDP: 0.0.0.0:9763"
echo "Internal publisher: tcp://127.0.0.1:5567"
echo "Stabilized SONIC output: tcp://127.0.0.1:5556"
echo "Rollback: stop this script, then run ~/run_jetson_current_proven_arms.sh"

env PYTHONPATH=. "$PYTHON_BIN" tools/kick_stability_sidecar.py \
  --input tcp://127.0.0.1:5567 \
  --output tcp://127.0.0.1:5556 \
  --stale-ms 750 &
SIDECAR_PID=$!

sleep 0.5

env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_sonic.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind tcp://127.0.0.1:5567 \
  --calibration-seconds "${CALIBRATION_SECONDS:-15}" \
  --calibration-sequence ntf \
  --stale-ms 750 \
  --batch-frames 1 \
  --physical-full-body \
  --physical-leg-blend "${LEG_BLEND:-1.00}" \
  --arm-max-speed "${ARM_MAX_SPEED:-4.0}" \
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
  --track-lifted-hip-roll &
PUBLISHER_PID=$!

wait "$PUBLISHER_PID"
