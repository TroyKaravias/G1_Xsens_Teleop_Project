#!/usr/bin/env bash
set -Eeuo pipefail

# One-command, simulation-only launcher for:
# MuJoCo -> SONIC zmq_manager -> calibrated Xsens arms-only publisher.
# This script never starts Unitree hardware output.

GROOT_ROOT="${HOME}/GR00T-WholeBodyControl"
HANDOFF_ROOT="${HOME}/g1_xsens_handoff"
SIM_PYTHON="${GROOT_ROOT}/.venv_sim/bin/python"
XSENS_PYTHON="${HOME}/miniconda3/envs/protomotions_mujoco/bin/python"
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
SIM_LOG="/tmp/g1_combined_sim_${RUN_TAG}.log"
SONIC_LOG="/tmp/g1_combined_sonic_${RUN_TAG}.log"
SIM_PID=""
SONIC_PID=""

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "Stopping combined simulation..."
  if [[ -n "${SONIC_PID}" ]] && kill -0 "${SONIC_PID}" 2>/dev/null; then
    kill "${SONIC_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    kill "${SIM_PID}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
  echo "Stopped. Logs:"
  echo "  ${SIM_LOG}"
  echo "  ${SONIC_LOG}"
}
trap cleanup EXIT INT TERM

for required in \
  "${SIM_PYTHON}" \
  "${XSENS_PYTHON}" \
  "${GROOT_ROOT}/gear_sonic/scripts/run_sim_loop.py" \
  "${GROOT_ROOT}/gear_sonic_deploy/deploy.sh" \
  "${HANDOFF_ROOT}/tools/live_xsens_locomotion_sim.py"; do
  if [[ ! -e "${required}" ]]; then
    echo "Missing required file: ${required}" >&2
    exit 1
  fi
done

echo "SIMULATION ONLY — keep the physical G1 controller off."
echo "Starting MuJoCo..."
(
  cd "${GROOT_ROOT}"
  export DISPLAY="${DISPLAY:-:0}"
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
  exec stdbuf -oL -eL "${SIM_PYTHON}" gear_sonic/scripts/run_sim_loop.py
) >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

sleep 5
if ! kill -0 "${SIM_PID}" 2>/dev/null; then
  echo "MuJoCo failed to start:" >&2
  tail -n 30 "${SIM_LOG}" >&2
  exit 1
fi
echo "MuJoCo started. Leave the robot on virtual support; do not press 9."

echo "Starting SONIC..."
(
  cd "${GROOT_ROOT}/gear_sonic_deploy"
  # NVIDIA's setup script probes optional variables before defining them,
  # which is incompatible with the launcher's otherwise useful nounset mode.
  set +u
  source scripts/setup_env.sh
  set -u
  # deploy.sh always asks for confirmation. An empty line selects its
  # documented default [Y] and is safe here because this launcher hard-codes
  # the final target as MuJoCo simulation.
  exec stdbuf -oL -eL ./deploy.sh \
    --cp policy/low_latency/model \
    --obs-config policy/low_latency/observation_config.yaml \
    --input-type zmq_manager \
    sim <<< ""
) >"${SONIC_LOG}" 2>&1 &
SONIC_PID=$!

ready=0
for _ in $(seq 1 60); do
  if ! kill -0 "${SONIC_PID}" 2>/dev/null; then
    echo "SONIC exited before becoming ready:" >&2
    tail -n 40 "${SONIC_LOG}" >&2
    exit 1
  fi
  if grep -q "Init Done" "${SONIC_LOG}"; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "${ready}" -ne 1 ]]; then
  echo "SONIC did not report Init Done within 60 seconds." >&2
  tail -n 40 "${SONIC_LOG}" >&2
  exit 1
fi

echo "SONIC is ready."
echo "Starting Xsens direct-pose calibration."
echo "Sequence: relaxed N-pose, T-pose, then arms straight forward."
cd "${HANDOFF_ROOT}"
export PYTHONPATH="${HANDOFF_ROOT}"
"${XSENS_PYTHON}" tools/live_xsens_locomotion_sim.py \
  --manual-locomotion \
  --body-motion \
  --track-wrists \
  --stale-ms 750 \
  --upper-max-speed 2.5
