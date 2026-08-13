#!/usr/bin/env bash
set -euo pipefail

# Hybrid physical teleoperation profile:
# - Current Jetson-local full-body, waist, root-heading, and lifted-foot path.
# - Earlier proven arm behavior: shoulder/elbow tracking at a conservative
#   4 rad/s default, with wrist joints held at their neutral references.
# - Direct MVN Ethernet UDP and Jetson-local SONIC ZMQ; no SSH tunnel.
# - Optional integrated kick-stability sidecar on the same entrypoint.

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

KICK_STABILITY="${KICK_STABILITY:-1}"
PUBLISHER_ZMQ_BIND="tcp://127.0.0.1:5556"
SIDECAR_INPUT_ZMQ="${SIDECAR_INPUT_ZMQ:-tcp://127.0.0.1:5567}"
SIDECAR_OUTPUT_ZMQ="${SIDECAR_OUTPUT_ZMQ:-tcp://127.0.0.1:5556}"
EFFECTIVE_LEG_BLEND="${LEG_BLEND:-1.00}"

echo "CURRENT JETSON TELEOP WITH EARLIER PROVEN ARMS"
echo "MVN Ethernet UDP: 0.0.0.0:9763"
echo "Calibration: N-pose -> T-pose -> arms straight forward"
echo "Arms: shoulder/elbow tracking at ${ARM_MAX_SPEED:-4.0} rad/s"
echo "Wrists: neutral | leg blend: ${LEG_BLEND:-1.00} | waist: relative"
if [[ "$KICK_STABILITY" == "1" ]]; then
  PUBLISHER_ZMQ_BIND="$SIDECAR_INPUT_ZMQ"
  EFFECTIVE_LEG_BLEND="${LEG_BLEND:-0.60}"
  echo "Kick stability: ON"
  echo "Publisher internal ZMQ: $SIDECAR_INPUT_ZMQ"
  echo "SONIC stabilized ZMQ: $SIDECAR_OUTPUT_ZMQ"
  echo "Kick tuning: leg_blend=$EFFECTIVE_LEG_BLEND stance=${KICK_STANCE_PROTECTION:-0.86} waist=${KICK_WAIST_PROTECTION:-0.50} adaptive_stance=${KICK_ADAPTIVE_STANCE_PROTECTION:-0.97} adaptive_waist=${KICK_ADAPTIVE_WAIST_PROTECTION:-0.85} support_pitch=${KICK_SUPPORT_HIP_PITCH_LIMIT_DEG:-10.0}deg support_knee=${KICK_SUPPORT_KNEE_LIMIT_DEG:-12.0}deg adaptive_support_pitch=${KICK_ADAPTIVE_SUPPORT_HIP_PITCH_LIMIT_DEG:-5.7}deg adaptive_forward_pitch=${KICK_ADAPTIVE_SUPPORT_HIP_PITCH_FORWARD_LIMIT_DEG:-3.4}deg adaptive_waist_pitch=${KICK_ADAPTIVE_WAIST_PITCH_LIMIT_DEG:-2.9}deg support_counter_pitch=${KICK_SUPPORT_HIP_PITCH_COUNTER_BIAS_DEG:-4.6}deg support_counter_knee=${KICK_SUPPORT_KNEE_COUNTER_BIAS_DEG:-5.7}deg support_counter_ankle=${KICK_SUPPORT_ANKLE_PITCH_COUNTER_BIAS_DEG:-2.9}deg waist_counter_pitch=${KICK_WAIST_PITCH_COUNTER_BIAS_DEG:-2.3}deg hip_pitch=${KICK_SWING_HIP_PITCH_LIMIT_DEG:-32.0}deg hip_roll=${KICK_SWING_HIP_ROLL_LIMIT_DEG:-28.0}deg hip_yaw=${KICK_SWING_HIP_YAW_LIMIT_DEG:-40.0}deg"
else
  echo "Kick stability: OFF"
  echo "SONIC loopback ZMQ: $PUBLISHER_ZMQ_BIND"
fi
echo "Requires support/harness and a dedicated physical E-stop operator."

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

if [[ "$KICK_STABILITY" == "1" ]]; then
  env PYTHONPATH=. "$PYTHON_BIN" tools/kick_stability_sidecar.py \
    --input "$SIDECAR_INPUT_ZMQ" \
    --output "$SIDECAR_OUTPUT_ZMQ" \
    --stale-ms 750 \
    --lift-on-rad "${KICK_LIFT_ON_RAD:-0.22}" \
    --lift-off-rad "${KICK_LIFT_OFF_RAD:-0.12}" \
    --side-margin-rad "${KICK_SIDE_MARGIN_RAD:-0.07}" \
    --phase-dwell-s "${KICK_PHASE_DWELL_S:-0.06}" \
    --stance-protection "${KICK_STANCE_PROTECTION:-0.86}" \
    --waist-protection "${KICK_WAIST_PROTECTION:-0.50}" \
    --support-hip-pitch-limit-deg "${KICK_SUPPORT_HIP_PITCH_LIMIT_DEG:-10.0}" \
    --support-hip-roll-limit-deg "${KICK_SUPPORT_HIP_ROLL_LIMIT_DEG:-8.0}" \
    --support-hip-yaw-limit-deg "${KICK_SUPPORT_HIP_YAW_LIMIT_DEG:-10.0}" \
    --support-knee-limit-deg "${KICK_SUPPORT_KNEE_LIMIT_DEG:-12.0}" \
    --support-ankle-pitch-limit-deg "${KICK_SUPPORT_ANKLE_PITCH_LIMIT_DEG:-7.0}" \
    --support-ankle-roll-limit-deg "${KICK_SUPPORT_ANKLE_ROLL_LIMIT_DEG:-5.0}" \
    --balance-activation-rad "${KICK_BALANCE_ACTIVATION_RAD:-0.18}" \
    --balance-full-rad "${KICK_BALANCE_FULL_RAD:-0.45}" \
    --adaptive-stance-protection "${KICK_ADAPTIVE_STANCE_PROTECTION:-0.97}" \
    --adaptive-waist-protection "${KICK_ADAPTIVE_WAIST_PROTECTION:-0.85}" \
    --adaptive-support-hip-pitch-limit-deg "${KICK_ADAPTIVE_SUPPORT_HIP_PITCH_LIMIT_DEG:-5.7}" \
    --adaptive-support-hip-roll-limit-deg "${KICK_ADAPTIVE_SUPPORT_HIP_ROLL_LIMIT_DEG:-6.9}" \
    --adaptive-support-hip-yaw-limit-deg "${KICK_ADAPTIVE_SUPPORT_HIP_YAW_LIMIT_DEG:-8.0}" \
    --adaptive-support-knee-limit-deg "${KICK_ADAPTIVE_SUPPORT_KNEE_LIMIT_DEG:-6.9}" \
    --adaptive-support-ankle-pitch-limit-deg "${KICK_ADAPTIVE_SUPPORT_ANKLE_PITCH_LIMIT_DEG:-4.6}" \
    --adaptive-support-ankle-roll-limit-deg "${KICK_ADAPTIVE_SUPPORT_ANKLE_ROLL_LIMIT_DEG:-4.6}" \
    --adaptive-support-hip-pitch-forward-limit-deg "${KICK_ADAPTIVE_SUPPORT_HIP_PITCH_FORWARD_LIMIT_DEG:-3.4}" \
    --adaptive-waist-roll-limit-deg "${KICK_ADAPTIVE_WAIST_ROLL_LIMIT_DEG:-5.7}" \
    --adaptive-waist-pitch-limit-deg "${KICK_ADAPTIVE_WAIST_PITCH_LIMIT_DEG:-2.9}" \
    --support-hip-pitch-counter-bias-deg "${KICK_SUPPORT_HIP_PITCH_COUNTER_BIAS_DEG:-4.6}" \
    --support-knee-counter-bias-deg "${KICK_SUPPORT_KNEE_COUNTER_BIAS_DEG:-5.7}" \
    --support-ankle-pitch-counter-bias-deg "${KICK_SUPPORT_ANKLE_PITCH_COUNTER_BIAS_DEG:-2.9}" \
    --waist-pitch-counter-bias-deg "${KICK_WAIST_PITCH_COUNTER_BIAS_DEG:-2.3}" \
    --swing-hip-pitch-limit-deg "${KICK_SWING_HIP_PITCH_LIMIT_DEG:-32.0}" \
    --swing-hip-roll-limit-deg "${KICK_SWING_HIP_ROLL_LIMIT_DEG:-28.0}" \
    --swing-hip-yaw-limit-deg "${KICK_SWING_HIP_YAW_LIMIT_DEG:-40.0}" \
    --swing-ankle-pitch-limit-deg "${KICK_SWING_ANKLE_PITCH_LIMIT_DEG:-15.0}" \
    --swing-ankle-roll-limit-deg "${KICK_SWING_ANKLE_ROLL_LIMIT_DEG:-9.0}" \
    --waist-roll-limit-deg "${KICK_WAIST_ROLL_LIMIT_DEG:-9.0}" \
    --waist-pitch-limit-deg "${KICK_WAIST_PITCH_LIMIT_DEG:-8.0}" \
    --lower-body-slew-rad-s "${KICK_LOWER_BODY_SLEW_RAD_S:-2.5}" \
    --maximum-dt-s "${KICK_MAXIMUM_DT_S:-0.04}" &
  SIDECAR_PID=$!
  sleep 0.5
fi

env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_sonic.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind "$PUBLISHER_ZMQ_BIND" \
  --calibration-seconds "${CALIBRATION_SECONDS:-15}" \
  --calibration-sequence ntf \
  --stale-ms 750 \
  --batch-frames 1 \
  --physical-full-body \
  --physical-leg-blend "$EFFECTIVE_LEG_BLEND" \
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
