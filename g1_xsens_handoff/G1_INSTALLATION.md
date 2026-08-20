# Install the known-working G1 Xsens stack

This guide reconstructs the source-controlled part of the setup captured from
the working Jetson on 2026-08-20. The working layout uses:

```text
Xsens MVN on Windows
  -> MXTP02 quaternion UDP to the G1 Jetson on port 9763
  -> global-position publisher on the Jetson
  -> ZMQ manager messages on 127.0.0.1:5556
  -> existing SONIC controller on the Jetson
  -> G1 interface enP8p1s0
```

## What GitHub does and does not provide

GitHub provides this project's bridge, launchers, tests, and replay data. It
does not provide the customized `GR00T-WholeBodyControl` checkout, compiled
`g1_deploy_onnx_ref`, SONIC ONNX policies, Unitree SDK libraries, firmware, or
licensed vendor software. See `EXTERNAL_FILES.md` before installing.

The global-position launcher deliberately imports the dated
`*_working_20260820.py` compatibility modules captured from the Jetson. This
keeps the physically observed configuration separate from newer experimental
PC-side retargeting modules.

Do not replace a working vendor checkout with files from another robot. Back it
up before making changes.

## 1. Prerequisites

- Unitree G1 EDU, 29 DOF, without hands.
- Jetson running Ubuntu 22.04 with interface `enP8p1s0` connected to the G1.
- Existing compatible SONIC checkout at
  `/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy`.
- Windows computer running Xsens MVN.
- Python 3.10 with NumPy and pyzmq.

The known working controller launcher expects these vendor-relative paths:

```text
target/release/g1_deploy_onnx_ref
policy/release/model_decoder.onnx
policy/release/model_encoder.onnx
policy/release/observation_config.yaml
planner/target_vel/V2/planner_sonic.onnx
reference/example/
thirdparty/unitree_sdk2/thirdparty/lib/aarch64/
```

## 2. Clone into the expected Jetson path

On the Jetson:

```bash
cd /home/unitree
git clone https://github.com/TroyKaravias/G1_Xsens_Teleop_Project.git g1_xsens_development
cd /home/unitree/g1_xsens_development/g1_xsens_handoff
```

For an existing clone, back it up and then update it using your normal Git
workflow. Do not overwrite uncommitted changes.

## 3. Create the isolated Python environment

```bash
python3 -m venv /home/unitree/.venvs/g1_xsens
/home/unitree/.venvs/g1_xsens/bin/python -m pip install --upgrade pip
/home/unitree/.venvs/g1_xsens/bin/python -m pip install -r requirements-jetson.txt
```

Install the Terminal A convenience launcher:

```bash
cp scripts/run_development_jetson.sh /home/unitree/run_development_jetson.sh
chmod +x /home/unitree/run_development_jetson.sh \
  scripts/run_jetson_global_position_controller.sh
```

## 4. Verify without robot output

```bash
cd /home/unitree/g1_xsens_development/g1_xsens_handoff
bash scripts/verify_package.sh
/home/unitree/.venvs/g1_xsens/bin/python -m unittest discover -s tests -v
```

Passing tests establish software behavior only. They do not validate physical
robot motion. Run the replay/no-controller preflight before live input:

```bash
PYTHON_BIN=/home/unitree/.venvs/g1_xsens/bin/python \
  bash scripts/preflight_jetson_replay.sh
```

## 5. Configure Xsens MVN

Configure one enabled MXTP02 network streamer:

```text
Protocol:      UDP / MXTP02
Data:          Position + Orientation (Quaternion)
Destination:   the current G1 Jetson IP
Port:          9763
Stream rate:   Max
Avatar offset: 1
Send paused:   Off
```

Confirm live packets without starting the controller. Do not proceed until
calibration succeeds and the publisher reports a fresh live stream.

## 6. Known-working launch commands

Use two SSH terminals on the Jetson. The robot must already be supported, the
area clear, and a separate operator ready at the physical E-stop.

Terminal A — Xsens publisher and global-position command generator:

```bash
cd /home/unitree

GLOBAL_FORWARD_GAIN=1.00 \
GLOBAL_MAX_FORWARD_MPS=0.35 \
GLOBAL_LEFT_GAIN=1.00 \
GLOBAL_RIGHT_GAIN=1.00 \
GLOBAL_MAX_LATERAL_MPS=0.25 \
GLOBAL_MAX_LEFT_MPS=0.25 \
GLOBAL_MAX_RIGHT_MPS=0.25 \
GLOBAL_COMMAND_HOLD_S=0.08 \
PYTHON_BIN=/home/unitree/.venvs/g1_xsens/bin/python \
./run_development_jetson.sh
```

Terminal B — SONIC global-position controller:

```bash
cd /home/unitree/g1_xsens_development/g1_xsens_handoff

bash ./scripts/run_jetson_global_position_controller.sh
```

Wait for publisher calibration and controller `Init Done`. Follow the current
on-screen arming instructions; do not assume historical Enter-key controls
apply to the manager-mode launcher.

## 7. Commissioning order

1. Replay with no controller or robot output.
2. Live Xsens stream with no controller.
3. Neutral/arm-only check under support.
4. Only then test one small, slow, restrained translation.

Stop immediately for wrong joint direction, abrupt motion, crossed legs,
oscillation, growing delay, stale input, or unexpected torso motion. Expect the
robot to become limp after controller shutdown; the support must carry it.

## Updating

After pulling a newer revision, read `AI_HANDOFF.md`, rerun package verification
and all tests, and compare the documented Terminal A/B commands. Never infer
physical validation from a successful update or test run.
