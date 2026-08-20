#!/usr/bin/env bash
set -euo pipefail

HANDOFF_ROOT="/home/unitree/g1_xsens_development/g1_xsens_handoff"
PYTHON_BIN="${PYTHON_BIN:-/home/unitree/.venvs/g1_xsens/bin/python}"

cd "$HANDOFF_ROOT"

echo "DEVELOPMENT XSENS TELEOP"
echo "Original arm tracking; elbow-plane gain: ${DEV_ELBOW_PLANE_YAW_GAIN:-1.00}"
echo "Wrist roll tracking: axis=${WRIST_ROLL_AXIS:-y} gain=${WRIST_ROLL_GAIN:-2.60} limit=${WRIST_ROLL_LIMIT_DEG:-45}deg"
echo "Frozen best-working launcher is unchanged."

exec env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_global_position.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind tcp://127.0.0.1:5556 \
  --calibration-seconds "${CALIBRATION_SECONDS:-15}" \
  --velocity-deadzone-mps "${GLOBAL_VELOCITY_DEADZONE_MPS:-0.008}" \
  --translation-gain "${GLOBAL_TRANSLATION_GAIN:-1.0}" \
  --forward-gain "${GLOBAL_FORWARD_GAIN:-1.50}" \
  --backward-gain "${GLOBAL_BACKWARD_GAIN:-1.98}" \
  --left-gain "${GLOBAL_LEFT_GAIN:-1.60}" \
  --right-gain "${GLOBAL_RIGHT_GAIN:-1.60}" \
  --max-forward-mps "${GLOBAL_MAX_FORWARD_MPS:-0.60}" \
  --max-backward-mps "${GLOBAL_MAX_BACKWARD_MPS:-0.50}" \
  --max-lateral-mps "${GLOBAL_MAX_LATERAL_MPS:-0.40}" \
  --max-left-mps "${GLOBAL_MAX_LEFT_MPS:-0.38}" \
  --max-right-mps "${GLOBAL_MAX_RIGHT_MPS:-0.50}" \
  --position-filter-alpha "${GLOBAL_POSITION_FILTER_ALPHA:-0.85}" \
  --command-hold-s "${GLOBAL_COMMAND_HOLD_S:-0.10}" \
  --enable-squat "${XSENS_SQUAT:-1}" \
  --squat-enter-drop-m "${SQUAT_ENTER_DROP_M:-0.08}" \
  --squat-exit-drop-m "${SQUAT_EXIT_DROP_M:-0.04}" \
  --squat-full-drop-m "${SQUAT_FULL_DROP_M:-0.24}" \
  --squat-min-height-m "${SQUAT_MIN_HEIGHT_M:-0.52}" \
  --squat-horizontal-limit-mps "${SQUAT_HORIZONTAL_LIMIT_MPS:-0.55}" \
  --arm-max-speed "${ARM_MAX_SPEED:-4.0}" \
  --elbow-plane-yaw-gain "${DEV_ELBOW_PLANE_YAW_GAIN:-1.00}" \
  --wrist-roll-axis "${WRIST_ROLL_AXIS:-y}" \
  --wrist-roll-gain "${WRIST_ROLL_GAIN:-2.60}" \
  --wrist-roll-limit-deg "${WRIST_ROLL_LIMIT_DEG:-45}"
