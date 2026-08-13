#!/usr/bin/env bash
set -euo pipefail

SONIC_ROOT="${SONIC_ROOT:-$HOME/GR00T-WholeBodyControl/gear_sonic_deploy}"

cd "$SONIC_ROOT"
# NVIDIA's setup script probes optional environment variables such as
# CMAKE_PREFIX_PATH before defining them. Temporarily disable nounset while
# sourcing it, then restore strict checking for the controller launch.
set +u
source scripts/setup_env.sh
set -u

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
