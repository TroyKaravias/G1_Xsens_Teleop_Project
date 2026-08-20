# AgiBot X2 simulation bring-up

This is the first simulation-only X2 milestone. It reuses the recorded Xsens
parser and calibrated G1 retargeter, then transfers semantic joint deltas into
the official 31-joint AgiBot X2 Ultra MuJoCo models. It does **not** contain an
X2 motor interface, balance controller, locomotion policy, or hardware output.

## 1. Install the simulator dependency

From `g1_xsens_handoff`:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-x2-sim.txt
```

On Ubuntu, install `python3-venv` first if creation of the virtual environment
reports that `ensurepip` is unavailable.

## 2. Fetch the official robot models

```bash
./scripts/fetch_agibot_x2_models.sh
```

The script clones AgiBotTech's `agibot_x2_urdf` repository into the ignored
`../external/agibot_x2_urdf` directory and checks out commit
`77f43eb0904dae4c48ccd9154fee824f8ffd4d38`. The vendor assets remain under
their Mulan PSL v2 license and are not copied into this repository.

## 3. Run the known recording through the model

First validate without opening a window:

```bash
python tools/preview_xsens_x2.py \
  data/xsens_20260723_114126_50hz.csv \
  --models ../external/agibot_x2_urdf \
  --variant v1.3 \
  --summary-only
```

Then open the passive MuJoCo viewer by removing `--summary-only`:

```bash
python tools/preview_xsens_x2.py \
  data/xsens_20260723_114126_50hz.csv \
  --models ../external/agibot_x2_urdf \
  --variant v1.3
```

Use `--variant v1.4` to test the other official model revision. Select the
revision matching the eventual robot before any hardware integration.

The viewer writes joint positions and calls `mj_forward`; it never calls
`mj_step`, never drives model actuators, and never opens a robot connection.
This makes it useful for checking joint names, axes, signs, poses, and limits,
but it does not demonstrate balance or dynamic feasibility.

## Current validation and limitations

On 2026-08-20, MuJoCo 3.12 loaded both pinned model revisions and replayed all
1,167 frames of the repository capture. Both models passed the 31-joint name
and ordering contract. At the current conservative gains, v1.3 clamped 4,224
of 36,177 joint values and v1.4 clamped 4,119; peak kinematic joint speed was
10.36 rad/s. These results prove the data/model path runs, but also show that
the mapping needs visual inspection, filtering, and X2-specific tuning before
physics, policy training, or hardware work.

The next acceptance sequence is:

1. Inspect both revisions in the passive viewer and record every incorrect
   joint direction or neutral pose.
2. Add X2-specific filtering and retarget calibration until clamping and
   discontinuities are acceptable.
3. Add a dynamics-capable controller or X2-specific SONIC embodiment and test
   it in simulation on a larger GPU system.
4. Only after simulation acceptance, design a separate fail-closed X2 hardware
   adapter and supported commissioning runbook.
