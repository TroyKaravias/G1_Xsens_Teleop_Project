#!/usr/bin/env bash
set -Eeuo pipefail

# Simulation-only direct 29-joint Xsens pose teleoperation. This uses SONIC's
# raw ZMQ pose input, not the locomotion planner, so hips/knees/ankles remain
# under the calibrated pose reference. It never starts Unitree hardware output.

GROOT_ROOT="${HOME}/GR00T-WholeBodyControl"
HANDOFF_ROOT="${HOME}/g1_xsens_handoff"
SIM_PYTHON="${GROOT_ROOT}/.venv_sim/bin/python"
XSENS_PYTHON="${HOME}/miniconda3/envs/protomotions_mujoco/bin/python"
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
SIM_LOG="/tmp/g1_direct_pose_sim_${RUN_TAG}.log"
SONIC_LOG="/tmp/g1_direct_pose_sonic_${RUN_TAG}.log"
SIM_PID=""
SONIC_PID=""

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "Stopping direct-pose simulation..."
  if [[ -n "${SONIC_PID}" ]] && kill -0 "${SONIC_PID}" 2>/dev/null; then
    kill "${SONIC_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    kill "${SIM_PID}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
  echo "Logs: ${SIM_LOG} ${SONIC_LOG}"
}
trap cleanup EXIT INT TERM

for required in \
  "${SIM_PYTHON}" \
  "${XSENS_PYTHON}" \
  "${GROOT_ROOT}/gear_sonic/scripts/run_sim_loop.py" \
  "${GROOT_ROOT}/gear_sonic_deploy/deploy.sh" \
  "${HANDOFF_ROOT}/tools/live_xsens_sonic.py"; do
  if [[ ! -e "${required}" ]]; then
    echo "Missing required file: ${required}" >&2
    exit 1
  fi
done

echo "SIMULATION ONLY — keep the physical G1 controller off."
echo "Starting MuJoCo for direct-pose teleoperation..."
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

echo "Starting SONIC raw-pose simulation input..."
(
  cd "${GROOT_ROOT}/gear_sonic_deploy"
  set +u
  source scripts/setup_env.sh
  set -u
  exec stdbuf -oL -eL ./deploy.sh \
    --cp policy/low_latency/model \
    --obs-config policy/low_latency/observation_config.yaml \
    --input-type zmq \
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

echo "SONIC is ready. Calibration: N-pose, T-pose, arms forward."
echo "After LIVE, press 9 once in MuJoCo and test small motions first."
cd "${HANDOFF_ROOT}"
export PYTHONPATH="${HANDOFF_ROOT}"
"${XSENS_PYTHON}" tools/live_xsens_sonic.py \
  --calibration-seconds 12 \
  --calibration-sequence ntf \
  --stale-ms 750 \
  --batch-frames 1 \
  --arm-max-speed 4.0 \
  --track-root-heading \
  --track-wrists
