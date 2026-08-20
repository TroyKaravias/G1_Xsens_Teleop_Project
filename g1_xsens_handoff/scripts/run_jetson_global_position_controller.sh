#!/usr/bin/env bash
set -euo pipefail

SONIC_ROOT="${SONIC_ROOT:-$HOME/GR00T-WholeBodyControl/gear_sonic_deploy}"
cd "$SONIC_ROOT"
set +u
source scripts/setup_env.sh
set -u

# Keep CycloneDDS C and C++ libraries from the same Unitree SDK build. Mixing
# ROS Humble libddsc with Unitree's libddscxx aborts during G1Deploy startup.
UNITREE_DDS_LIB="$SONIC_ROOT/thirdparty/unitree_sdk2/thirdparty/lib/aarch64"
export LD_LIBRARY_PATH="$UNITREE_DDS_LIB:${LD_LIBRARY_PATH:-}"

exec ./target/release/g1_deploy_onnx_ref \
  enP8p1s0 \
  policy/release/model_decoder.onnx \
  reference/example/ \
  --obs-config policy/release/observation_config.yaml \
  --encoder-file policy/release/model_encoder.onnx \
  --planner-file planner/target_vel/V2/planner_sonic.onnx \
  --input-type zmq_manager \
  --output-type zmq \
  --zmq-host localhost \
  --zmq-port 5556 \
  --zmq-topic pose \
  --zmq-conflate
