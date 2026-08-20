#!/usr/bin/env bash
set -euo pipefail

SONIC_ROOT="${SONIC_ROOT:-$HOME/GR00T-WholeBodyControl/gear_sonic_deploy}"
SONIC_INPUT_TYPE="${SONIC_INPUT_TYPE:-zmq_manager}"

cd "$SONIC_ROOT"
# NVIDIA's setup script probes optional environment variables such as
# CMAKE_PREFIX_PATH before defining them. Temporarily disable nounset while
# sourcing it, then restore strict checking for the controller launch.
set +u
source scripts/setup_env.sh
set -u

if [[ "$SONIC_INPUT_TYPE" == "zmq_manager" ]]; then
  echo "SONIC physical planner mode (zmq_manager)"
  echo "Planner: global pelvis XY/yaw; Xsens upper body override"
  exec ./target/release/g1_deploy_onnx_ref \
    enP8p1s0 \
    policy/low_latency/model_decoder.onnx \
    reference/example/ \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --obs-config policy/low_latency/observation_config.yaml \
    --encoder-file policy/low_latency/model_encoder.onnx \
    --input-type zmq_manager \
    --output-type zmq \
    --zmq-host localhost \
    --zmq-port 5556 \
    --zmq-topic pose \
    --zmq-conflate
elif [[ "$SONIC_INPUT_TYPE" == "zmq" ]]; then
  echo "SONIC legacy raw-pose mode"
  exec ./target/release/g1_deploy_onnx_ref \
    enP8p1s0 \
    policy/low_latency/model_decoder.onnx \
    reference/example/ \
    --obs-config policy/low_latency/observation_config.yaml \
    --encoder-file policy/low_latency/model_encoder.onnx \
    --input-type zmq \
    --output-type zmq \
    --zmq-host localhost \
    --zmq-port 5556 \
    --zmq-topic pose \
    --zmq-conflate
else
  echo "Unsupported SONIC_INPUT_TYPE: $SONIC_INPUT_TYPE" >&2
  exit 1
fi
