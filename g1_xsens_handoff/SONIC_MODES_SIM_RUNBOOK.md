# SONIC Modes Simulation Demo

This milestone uses the NVIDIA SONIC ZMQ-manager protocol at repository commit
`4141c34`. It is for MuJoCo simulation only.

The scripted demo performs:

```text
IDLE -> FORWARD -> IDLE -> BACKWARD -> IDLE -> TURN LEFT -> IDLE -> STOP
```

Speeds are intentionally conservative: 0.20 m/s forward, 0.12 m/s backward,
and 0.35 rad/s turning.

## 1. Copy the package from the Mac

Run in a Mac terminal:

```bash
scp -P 2222 \
  "$HOME/Desktop/G1_Xsens_SONIC_Sim_Demo_2026-07-29.zip" \
  griff@192.168.30.75:/home/griff/
```

## 2. Extract it in WSL

SSH into WSL from the Mac:

```bash
ssh -p 2222 griff@192.168.30.75
```

Then:

```bash
cd /home/griff
unzip -o G1_Xsens_SONIC_Sim_Demo_2026-07-29.zip
cd /home/griff/g1_xsens_handoff
PYTHONPATH=. python3 -m unittest discover -s tests -q
```

Expected result: `Ran 38 tests` and `OK`.

## 3. Start the known-working MuJoCo simulator

In WSL terminal 1, use the same WSLg simulator command that previously showed
the G1 window:

```bash
cd /home/griff/GR00T-WholeBodyControl
source .venv_sim/bin/activate
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
python gear_sonic/scripts/run_sim_loop.py
```

Do not continue unless the simulated G1 is visible and standing normally.

## 4. Start SONIC in simulation

In WSL terminal 2:

```bash
cd /home/griff/GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh
./deploy.sh \
  --cp policy/low_latency/model \
  --obs-config policy/low_latency/observation_config.yaml \
  --input-type zmq_manager \
  sim
```

This must say that the input type is `zmq_manager`, the planner file is loaded,
and the output is MuJoCo simulation. Do not select `real`.

## 5. Run the scripted movement publisher

In WSL terminal 3:

```bash
cd /home/griff/g1_xsens_handoff
PYTHONPATH=. python3 tools/sonic_modes_sim_demo.py
```

It prints each segment and sends a STOP command when complete.

## Stop procedure

1. Press `Ctrl+C` in the publisher terminal.
2. Press `O` in the SONIC deployment terminal if it is still controlling.
3. Press `Ctrl+C` in the SONIC and simulator terminals.

## What this test does not prove

- It does not command the physical G1.
- It does not yet use a true fencing lunge.
- It does not yet blend live Xsens upper-body targets into the planner loop.

After this passes visually, the next integration sends the already-calibrated
Xsens 29-DOF reference as SONIC's optional `upper_body_position` while the
planner supplies the legs.
