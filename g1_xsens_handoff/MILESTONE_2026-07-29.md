# Milestone: Live Xsens to Unitree G1

Date: 2026-07-29

## Outcome

Live Xsens motion was successfully retargeted to the 29-DOF Unitree G1,
validated in MuJoCo, and then validated on the supported physical G1 for a
small arm-motion test. The final test had responsive tracking without the
previously increasing delay.

This is a restrained-test milestone, not approval for unsupported walking,
jumping, lunges, or operation without an emergency-stop operator.

## Verified data path

```text
Xsens MVN on Windows
  -> MXTP02 UDP position + quaternion stream
  -> WSL2 Xsens retargeter at 50 Hz
  -> SONIC protocol-v1 ZMQ publisher
  -> two SSH tunnels through the Mac
  -> SONIC controller on the G1 Jetson Orin NX
  -> Unitree LowState and restrained physical G1
```

## Important machines and addresses

- Windows/WSL SSH endpoint: `192.168.30.75:2222`
- WSL user: `griff`
- G1 Jetson internal Ethernet: `192.168.123.164`
- G1 Jetson lab Wi-Fi: `192.168.30.172`
- G1 Jetson user: `unitree`
- G1 internal controller interface on Jetson: `enP8p1s0`
- Previously assumed `192.168.30.199` was not reachable and was not used.
- No passwords are stored in this milestone.

## Installed software confirmed

### Windows/WSL

- Ubuntu 24.04 under WSL2
- ProtoMotions at `/home/griff/protomotions`
- Conda environment `protomotions_mujoco`
- GR00T-WholeBodyControl at `/home/griff/GR00T-WholeBodyControl`
- Xsens handoff at `/home/griff/g1_xsens_workspace/g1_xsens_handoff`

### G1 Jetson

- NVIDIA Jetson Orin NX, ARM64
- Ubuntu 22.04 / L4T 36.4.3
- CUDA 12.6
- TensorRT 10.7
- ROS 2 Humble
- GR00T-WholeBodyControl at `/home/unitree/GR00T-WholeBodyControl`
- Existing customized repository commit: `3d3e593`
- Existing customized working tree must not be reset or overwritten.
- Existing ARM64 controller:
  `gear_sonic_deploy/target/release/g1_deploy_onnx_ref`
- Existing low-latency SONIC encoder, decoder, and observation configuration

## Corrections required for the working milestone

### Joint order

The Xsens bridge produces G1 joints in MuJoCo/ProtoMotions order, while SONIC
protocol v1 requires IsaacLab order. The working reorder selector is:

```text
[0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
```

### Live batch size

Live ZMQ messages must contain one frame per 50 Hz message:

```text
--batch-frames 1
```

The earlier four-frame overlapping batches caused SONIC's reference queue to
grow faster than playback, producing steadily increasing latency.

## Start the WSL publisher

Run only at a prompt containing `griff@DESKTOP-CHQ88EN`:

```bash
cd ~/g1_xsens_workspace/g1_xsens_handoff
conda activate protomotions_mujoco

PYTHONPATH=. python tools/live_xsens_sonic.py \
  --bind 0.0.0.0 \
  --xsens-port 9763 \
  --zmq-bind tcp://0.0.0.0:5556 \
  --calibration-seconds 12 \
  --batch-frames 1
```

Calibration sequence: hold N-pose, move smoothly to T-pose, then hold T-pose.
Proceed only after `Calibration complete` and `Stream state: LIVE`.

## Start the SSH tunnels from the Mac

Tunnel 1, Mac to WSL:

```bash
ssh -o ExitOnForwardFailure=yes \
  -p 2222 -N \
  -L 15556:127.0.0.1:5556 \
  griff@192.168.30.75
```

Tunnel 2, Mac to Jetson:

```bash
ssh -o ExitOnForwardFailure=yes \
  -N -R 5556:127.0.0.1:15556 \
  unitree@192.168.123.164
```

Both tunnel terminals remain blank while working and must remain open.

Verify from the Jetson:

```bash
timeout 3 bash -c '</dev/tcp/127.0.0.1/5556' \
  && echo REACHABLE || echo BLOCKED
```

## Start the Jetson SONIC controller

Run only at a prompt containing `unitree@unitree-g1-nx`:

```bash
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh

./target/release/g1_deploy_onnx_ref \
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
```

Controls:

- `]`: engage neutral control
- `Enter`: toggle live ZMQ tracking
- `O`: stop controller and enter damping shutdown

## Safety requirements

- Validate every code or mapping change in MuJoCo first.
- For physical testing, use the support harness and a dedicated physical
  emergency-stop operator.
- Clear people and equipment from the robot.
- Match the operator pose to the robot before enabling ZMQ.
- First test neutral engagement, then one small arm motion.
- Pause ZMQ with `Enter` and stop with `O`.
- Expect the robot to become limp after stopping; the harness must support it.
- Use the physical emergency stop for abrupt motion, oscillation, crossed legs,
  incorrect joint direction, aggressive torso pitch, or delayed response.
- Do not attempt walking, jumping, lunges, dancing, or unsupported operation at
  this milestone.

## Verification results

- Xsens UDP: live, approximately 240-260 Hz, zero malformed packets observed.
- Publisher: 50 Hz SONIC protocol v1.
- Protocol fields: joint positions, joint velocities, body quaternion, frame
  index.
- Local test suite: 18 tests passed.
- SONIC simulation: arms, squat, knee lifts, stepping, and forward movement
  validated.
- Jetson ZMQ tunnel: reachable.
- Jetson controller: `Init Done`, proving Xsens receipt plus G1 LowState.
- Restrained physical test: neutral engagement and small live arm tracking
  succeeded.
- Increasing-delay defect resolved by changing live batch size from 4 to 1.

## Known limitations and next phase

- The current protocol-v1 publisher uses a fixed identity body quaternion.
  Live pelvis orientation/root intent still needs a separate, simulation-first
  implementation and validation.
- The G1 has no hands in this project; hand-related SONIC log messages are
  irrelevant.
- Add a source timestamp/end-to-end latency monitor and a hard stale-data
  watchdog before broader physical motion.
- Next physical tests should remain restrained: longer arm test, small squat,
  individual knee lift, then carefully bounded stepping.
- Unsupported walking and dynamic motion require additional validation and
  safety review.

