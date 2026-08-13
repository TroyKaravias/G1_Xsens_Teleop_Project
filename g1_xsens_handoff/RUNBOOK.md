# Simulation-Only Runbook

All commands below run inside Linux/WSL.

## A. Verify the handoff package

```bash
cd ~/g1_xsens_handoff/g1_xsens_handoff
bash scripts/verify_package.sh
python -m unittest discover -s tests -v
```

## B. Recorded UDP transport test

Terminal 1:

```bash
cd ~/g1_xsens_handoff/g1_xsens_handoff
PYTHONPATH=. python -m xsens_bridge listen \
  --bind 127.0.0.1 \
  --port 9763 \
  --stale-ms 250
```

Terminal 2:

```bash
cd ~/g1_xsens_handoff/g1_xsens_handoff
PYTHONPATH=. python -m xsens_bridge replay \
  data/xsens_20260723_114126.xudp \
  --host 127.0.0.1 \
  --port 9763 \
  --speed 1.0
```

Pass condition: receiver changes `STALE → LIVE → STALE`, receives 5,605 frames at
approximately 240 Hz, and reports `missing=0 malformed=0`.

## C. Verify the official tracker

From `~/protomotions`:

```bash
python deployment/test_tracker_mujoco.py \
  --onnx data/pretrained_models/motion_tracker/g1-bones-deploy/compiled_models/unified_pipeline.onnx \
  --motion data/motion_for_trackers/g1_bones_seed_mini.pt \
  --no-realtime \
  --loops 1
```

## D. Convert the validated Xsens reference

```bash
cd ~/protomotions
PYTHONPATH=~/g1_xsens_handoff/g1_xsens_handoff \
python ~/g1_xsens_handoff/g1_xsens_handoff/tools/export_protomotions_cache.py \
  ~/g1_xsens_handoff/g1_xsens_handoff/data/xsens_20260723_114126_g1_reference.npz \
  protomotions/data/assets/mjcf/g1_holo_compat.xml \
  data/motion_for_trackers/xsens_20260723_114126.50fps.pt
```

Expected: 1,167 frames, 50 Hz, 29 DOF, 33 bodies, 23.34 seconds.

## E. Run the custom reference

```bash
python deployment/test_tracker_mujoco.py \
  --onnx data/pretrained_models/motion_tracker/g1-bones-deploy/compiled_models/unified_pipeline.onnx \
  --motion data/motion_for_trackers/xsens_20260723_114126.50fps.pt \
  --no-realtime \
  --loops 1
```

## F. Test live Windows Xsens → WSL transport

WSL:

```bash
cd ~/g1_xsens_handoff/g1_xsens_handoff
PYTHONPATH=. python -m xsens_bridge listen \
  --bind 0.0.0.0 \
  --port 9763 \
  --stale-ms 250
```

Configure Windows Xsens MVN for MXTP02 UDP port 9763 as described in `WSL2_SETUP.md`.
Pass condition: live rate near 240 Hz with zero malformed packets.

## G. End-to-end live simulation test

This remains simulation-only. It does not import Unitree ROS/DDS code and does not
connect to `192.168.30.199`.

First stop any old listener on port 9763 with `Ctrl+C`. In WSL Terminal 1:

```bash
cd ~/protomotions
conda activate protomotions_mujoco
PYTHONPATH=~/g1_xsens_workspace/g1_xsens_handoff \
python ~/g1_xsens_workspace/g1_xsens_handoff/tools/live_xsens_mujoco.py \
  --protomotions-root ~/protomotions \
  --onnx ~/protomotions/data/pretrained_models/motion_tracker/g1-bones-deploy/compiled_models/unified_pipeline.onnx \
  --bind 127.0.0.1 \
  --port 9763 \
  --calibration-seconds 9 \
  --runtime-seconds 12
```

Once Terminal 1 says it is waiting for MXTP02, run in WSL Terminal 2:

```bash
cd ~/g1_xsens_workspace/g1_xsens_handoff
conda activate protomotions_mujoco
PYTHONPATH=. python -m xsens_bridge replay \
  data/xsens_20260723_114126.xudp \
  --host 127.0.0.1 \
  --port 9763 \
  --speed 1.0
```

Pass conditions:

- calibration captures over 100 frames and selects a T-pose;
- state changes to `LIVE`;
- root height remains between 0.45 m and 1.10 m;
- packets remain free of malformed/missing-frame errors;
- the process completes 12 seconds without an exception.

After recorded replay passes, repeat Terminal 1 with `--bind 0.0.0.0` and point
Windows MVN at the current WSL IP on UDP port 9763. During its nine-second
calibration window, hold N-pose first, then move smoothly to T-pose and hold.

## H. Record a reusable live UDP take

The recorder stores raw, timestamped Xsens packets only. It has no SONIC,
Unitree SDK, DDS, ROS, or motor-command output.

Use a separate MVN destination and UDP port when recording alongside another
live consumer. For example, keep the control or simulation destination on
port 9763 and add a recorder destination on port 9764. For both destinations,
select only `Position + Orientation (Quaternion)`.

On the recorder machine:

```bash
mkdir -p ~/xsens_recordings
cd ~/g1_xsens_handoff
PYTHONPATH=. python -m xsens_bridge record \
  ~/xsens_recordings/take_001.xudp \
  --bind 0.0.0.0 \
  --port 9764
```

Begin the take with N-pose, T-pose, and arms-forward calibration, followed by
a neutral hold and the desired motions. Press `Ctrl+C` to finalize the file.
The recorder refuses to replace an existing file unless `--overwrite` is
explicitly supplied.

Verify the capture before using it:

```bash
PYTHONPATH=. python -m xsens_bridge summary \
  ~/xsens_recordings/take_001.xudp
```

Then replay it to a receive-only listener or simulation:

```bash
PYTHONPATH=. python -m xsens_bridge replay \
  ~/xsens_recordings/take_001.xudp \
  --host 127.0.0.1 \
  --port 9763 \
  --speed 1.0
```

Do not bind the recorder and an Xsens publisher/listener to the same local UDP
port. Duplicating the destination in MVN keeps recording out of the live
control path.

## Stop condition

If any command mentions Unitree DDS, ROS command topics, motor commands, or attempts to
contact `192.168.30.199`, stop. Those are outside the validated scope of this handoff.
