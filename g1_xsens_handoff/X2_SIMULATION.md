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

For reproducible visual review on a headless Linux host, render front and
right-side snapshots with EGL:

```bash
MUJOCO_GL=egl python tools/preview_xsens_x2.py \
  data/xsens_20260723_114126_50hz.csv \
  --models ../external/agibot_x2_urdf \
  --variant v1.3 \
  --render-dir /tmp/x2-v1.3-snapshots
```

The default X2-only 4 Hz joint filter can be changed with
`--filter-cutoff-hz`; zero disables it for before/after comparisons. Neither
the filter nor snapshot rendering changes the preserved G1 retargeting path.

## 4. Run fixed-base dynamics tracking

The first controlled simulation numerically pins the floating base and applies
bounded PD torque through the 31 motors already present in the official model:

```bash
python tools/simulate_xsens_x2.py \
  data/xsens_20260723_114126_50hz.csv \
  --models ../external/agibot_x2_urdf \
  --variant v1.3
```

Add `--viewer --realtime` for visual playback. This runner calls `mj_step` and
therefore exercises model inertia, collisions, and torque limits, but its base
fixture prevents it from testing balance, ground support, or locomotion.
Use `--playback-speed 2` for twice-real-time reference playback. Replace the
CSV positional argument with another 50 Hz bridge CSV to replay another Xsens
capture; it must use the same segment position/quaternion column schema.

## 5. Drive X2 directly from live MVN UDP

The live runner accepts the same MXTP02 packets from either a person wearing
the suit or MVN Analyze playing a saved `.mvn` take:

```bash
python tools/live_xsens_x2.py \
  --models ../external/agibot_x2_urdf \
  --variant v1.3 \
  --bind 0.0.0.0 \
  --port 9764 \
  --viewer
```

Start the runner first, then begin the MVN stream. The original N/T timing is
unchanged: during the first nine seconds, hold N-pose for roughly 3.6 seconds,
then move to T-pose and hold. After that phase, hold both arms straight forward
at shoulder height and shoulder width for three additional seconds. For a saved
take, use `Send Paused` to hold each requested calibration pose if its opening
does not already contain that sequence. After calibration, new packets immediately
update the X2 reference. Paused or lost input changes the state to
`STALE-HOLD`, zeroes target velocity, and holds the last bounded pose. This is
still fixed-base simulation; the fail-safe does not provide balance control.
The live viewer adds a non-colliding checkered floor, light gradient sky, and
broad ambient/head lighting at runtime. These scene aids do not provide
physical ground support or modify the pinned vendor asset.

Global root-yaw tracking is enabled by default. Calibrated Xsens pelvis heading
rotates the whole X2 around its fixed world position, at a default maximum of
180 degrees per second, so the world-fixed viewer can show the robot's front,
sides, and back. Use `--no-track-global-yaw` to restore the earlier fixed-facing
behavior.

Global X/Y root tracking is also enabled by default. Bounded, filtered pelvis
displacement moves the X2 across the world-fixed checkered floor while Z and
base roll/pitch remain pinned. The default maximum visual root speed is 0.75
m/s, and discontinuous pelvis jumps above 0.25 m per sample are rejected. Use
`--no-track-global-position` for the earlier centered behavior. Root X/Y/yaw
placement visualizes the performer's global motion; it is not foot-force-driven
locomotion or free-base balance validation.

An end-to-end loopback replay test on 2026-08-20 calibrated from 1,002 streamed
frames and processed 253 live reference frames during its two-second control
window with zero missing packets, malformed packets, stale transitions, or
torque saturation events.

The first viewer loop synchronized graphics at every 0.001-second physics
step, allowing display overhead to make simulation lag behind live packets.
The corrected loop catches physics up to wall time in substeps and renders
independently at approximately 60 Hz. A headless timing replay measured a
0.998x real-time factor with zero torque saturation.

The viewer writes joint positions and calls `mj_forward`; it never calls
`mj_step`, never drives model actuators, and never opens a robot connection.
This makes it useful for checking joint names, axes, signs, poses, and limits,
but it does not demonstrate balance or dynamic feasibility.

## Current validation and limitations

On 2026-08-20, MuJoCo 3.12 loaded both pinned model revisions and replayed all
1,167 frames of the repository capture. Both models passed the 31-joint name
and ordering contract. The initial baseline clamped 4,224 of 36,177 joint
values on v1.3 and 4,119 on v1.4, with a 10.36 rad/s peak kinematic joint
speed.

Subsequent joint-level review found that the initial X2 adapter incorrectly
inverted elbow flexion, placing every elbow sample outside the official X2
range. After correcting that X2-only sign, counting true model-limit
violations without an artificial neutral-pose margin, and applying the default
4 Hz X2 filter, v1.3 clamps 1,054 values and v1.4 clamps 951. Peak kinematic
speed is 4.35 rad/s. The remaining violations are confined to knee
hyperextension references (865 samples), right shoulder roll (86), and v1.3
waist pitch (103). Headless front/right snapshots at six representative frames
showed coherent neutral, T-pose, arm elevation, elbow flexion, and return for
both revisions. This is sampled visual and kinematic software validation only;
it is not continuous-motion, collision, balance, dynamics, policy, or hardware
validation.

The first fixed-base dynamics run completed the full 23.691-second reference
on v1.3 at a 0.001-second simulation timestep. Overall tracking error was
0.0132 rad RMS, peak error was 0.1870 rad, and none of 734,421 joint-step
commands reached a torque limit. The corresponding v1.4 run reached 0.0864 rad
RMS and 0.5352 rad peak error, with 16,253 head-pitch torque saturations. Direct
contact inspection identified persistent official-model self-contact between
`head_pitch_link` and `torso_link` plus `torso_link` and `waist_yaw_link`,
including at the initial T-pose. Do not tune around this by increasing head
torque. Use v1.3 for current dynamics work unless
the eventual X2 hardware revision requires v1.4, in which case the v1.4 model
geometry/contact definition must be resolved first. Fixed-base tracking is
simulation dynamics evidence only, not balance or hardware validation.

The next acceptance sequence is:

1. Inspect both revisions continuously in the passive viewer, focusing on the
   remaining knee, right-shoulder-roll, and v1.3 waist-pitch limit events.
2. Add X2-specific lower-body calibration so human knee hyperextension noise
   is handled intentionally, then decide acceptable clamp and speed thresholds.
3. Continue fixed-base v1.3 visual tracking and establish tracking/contact
   thresholds; resolve the official v1.4 head/torso self-contact if v1.4 is the
   required embodiment.
4. Add a free-base balance controller or X2-specific SONIC embodiment and test
   it in simulation on a larger GPU system.
5. Only after simulation acceptance, design a separate fail-closed X2 hardware
   adapter and supported commissioning runbook.
