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

## 6. X2 SONIC integration (primary development path)

The direct live runner above remains a calibration and kinematic-reference
diagnostic. It must not be presented as SONIC. The primary controller path is
now the X2 Ultra embodiment developed in NVlabs/GR00T-WholeBodyControl PR #112.
Fetch its source at the reviewed commit into the ignored `external/` tree:

```bash
./scripts/fetch_x2_sonic_source.sh
```

That source contains the 31-DoF embodiment, training configuration, joint-order
converters, and `gear_sonic/scripts/eval_x2_mujoco.py`. The selected fused
policy is `tinkerbuggy/sonic-x2` file
`sonic_policy/x2_sonic_policy.onnx`. It is intentionally kept under the ignored
`external/sonic_x2/` directory because its weights license is pending review.
Check the local policy, and optionally a known X2 motion, before evaluation:

```bash
./scripts/check_x2_sonic_assets.sh \
  ../external/sonic_x2/x2_sonic_policy.onnx \
  /path/to/x2_motion.pkl  # optional until known-motion evaluation
```

The evaluator contract is 50 Hz control, 31 policy actions, ten frames of
proprioceptive history, and ten future reference frames. Live Xsens integration
therefore belongs upstream of the SONIC encoder as a buffered X2 motion
reference. It must not publish Xsens-derived joint targets directly to the
actuators. First validate the supplied checkpoint and known X2 motion in
IsaacLab, then in free-base MuJoCo. Only after those two baselines pass should
the saved-MVN and live-UDP reference providers be connected.

The fused X2 ONNX is present locally and its SHA-256 matches the Hugging Face
manifest (`e7ccd6522010ea660facfb7265fb129dac2b580dd1483cc1dbada73c205309d7`).
This establishes file identity only—not inference, dynamic simulation, or
hardware validation. Never substitute the released G1 checkpoint: its
observation/action embodiment is incompatible.

`tools/simulate_xsens_x2_sonic.py` is the first free-base recorded-reference
integration. It constructs the fused 1670-float observation, runs the actual
ONNX policy at 50 Hz, converts its 31 IsaacLab-order actions to MuJoCo order,
and applies the embodiment PD/action scales. It also adds a physical plane at
runtime because the pinned official v1.3 MJCF contains no floor; the vendor
asset itself remains unchanged.

The first controlled runs are **failures**, not dynamic validation. The generic
fused policy fell at 0.50 s after the floor and per-frame tokenizer-layout fixes;
the versioned step-14,000 policy (MD5
`ec745672cfefd9507e9d5e9834bf4537`) fell at 0.36 s. These runs exposed and
corrected two false leads: initially the robot fell through the missing floor,
and an older grouped tokenizer layout permuted future positions/velocities.
The remaining blocker is reproducing the publisher's known X2 motion/RSI
baseline exactly before evaluating an Xsens-derived reference. Do not tune
contacts or gains to the Xsens clip until that baseline is available.

### Live Xsens through the actual X2 SONIC policy

The new X2-only live runner is:

```bash
PYTHONPATH=../external/x2_python_deps:. \
python tools/live_xsens_x2_sonic.py \
  --models ../external/agibot_x2_urdf \
  --policy ../external/sonic_x2/x2_sonic_14000_g1.onnx \
  --bind 0.0.0.0 --port 9764 --viewer
```

It preserves the 9-second N/T plus 3-second arms-forward calibration, converts
the resulting motion to an X2 reference centered on SONIC's reset pose, and
buffers one second of received frames. The policy's ten future frames are thus
real past-received Xsens samples rather than guessed extrapolations. After the
buffer fills, the runner builds the 680-value tokenizer reference and 990-value
MuJoCo proprioception history, runs the fused X2 policy at 50 Hz, and drives the
free-base simulated X2. Stale input terminates the experimental policy test.

A localhost replay on 2026-08-21 exercised the complete path with the real
9+3-second calibration: 1,371 calibration frames, 117 post-calibration Xsens
frames, zero missing or malformed packets, and 21 actual policy steps. The
robot then terminated on a 0.392 m pelvis-height fall with 51 torque saturation
events. This validates live dataflow and policy execution only. It is a failed
dynamic test and does not validate balance, tracking quality, or hardware.

The first live runner revision contained two policy-input contract errors that
explained that immediate fall. The released X2 training code concatenates all
ten future position frames and all ten velocity frames *before* reshaping the
command to 10x62; it does not use conventional per-frame position/velocity
interleaving. Its 6D identity orientation is the row-major 3x2 value
`[1, 0, 0, 1, 0, 0]`. Both paths now reproduce those layouts. Because live
Xsens has no X2 root height, startup also places the model using its actual foot
collision spheres instead of dropping it roughly four centimetres from the
vendor MJCF default free-joint height.

After those fixes, the saved recorded-reference runner completed 3.02 seconds
(151 policy steps) with no fall, minimum pelvis height 0.601 m, and maximum tilt
13.0 degrees. A longer request fell at 3.40 seconds after transitioning from
the policy startup pose into the Xsens-derived motion. A localhost UDP replay
through the live runner executed 28 policy steps without a fall and stopped on
stale input when that recording ended. These are simulation observations only:
startup is repaired, but sustained Xsens-motion tracking and X2 hardware remain
unvalidated.

A longer zero-motion control subsequently established that the step-14000
policy falls from the static default command at 3.82 seconds even when Xsens
deltas are zero. Thus “stands up, then falls” is currently a policy/reference
initialization failure, not evidence that the suit retargeting caused the fall.
The policy requires a training-distribution X2 motion anchor and matching RSI
root pose/linear velocity/angular velocity; those artifacts are not present in
the supplied Hugging Face files. Automatic resets, fixed-base support, or
contact/gain tuning must not be represented as successful dynamic tracking.

### Publisher quick-play baseline (v2)

The current `tinkerbuggy/sonic-x2` quickstart identifies the frozen-G1-core
LoRA v2 tracker and matching frozen-G1-core v1 planner as the latest pair. Its
standalone `meetsitaram/sonic-x2` bundle includes known X2 motion PKLs and the
exact RSI-aware MuJoCo evaluator. It is cloned under the ignored
`external/sonic_x2_quickplay` directory; large policy/planner assets remain
ignored and must not be committed.

On 2026-08-21, the unmodified evaluator and required v2 runtime settings
(parity gains, action clip 20, frozen wrists) produced:

- idle: full 9.62-second clip, `motion_end`, no fall, pelvis-z MAE 0.003 m;
- relaxed walk: 10.02-second headless rollout with no fall.

This validates that v2 SONIC works dynamically with its intended X2 reference
and RSI state. It does not validate Xsens substitution or hardware. Xsens must
now drive the X2 planner's context/intention, while SONIC receives the
planner-generated full 38-value future root+joint trajectory rather than a
raw ligament mapping.

`xsens_bridge/x2_planner.py` implements the planner's verified four-input ONNX
contract, and `tools/generate_x2_planner_reference.py` creates chained 30 Hz
root+joint references. Two 10.02-second v2 SONIC/free-base tests passed without
falls: planner-only idle, then the same pipeline with recorded Xsens
waist/arm/head deltas injected into each four-frame planner context. The Xsens
case differed from planner-only idle by mean 0.257 rad and max 2.377 rad across
upper-body joints (lower-body mean difference 0.033 rad), confirming that the
planner responded while retaining locomotion ownership. This validates an
offline Xsens-to-planner dynamic simulation only; live UDP remains next and no
X2 hardware behavior was tested.

### MVN playback/live capture through planner and v2 SONIC

Both MVN recording playback and the live suit emit the same MXTP02 stream, so
the capture path is shared:

```bash
PYTHONPATH=../external/x2_python_deps:. \
python tools/capture_xsens_x2_planner_sonic.py \
  --bind 0.0.0.0 --port 9764 --capture-seconds 0 --viewer
```

Start MVN playback (or begin live streaming) to this workstation while the
command waits. It performs the unchanged 9-second N/T and 3-second
arms-forward calibration. `--capture-seconds 0` captures until playback ends;
use `--motion-start-delay N` to skip a post-calibration lead-in. It then
injects waist/arm/head deltas into the matching X2 planner, writes
`data/x2_planner_xsens_live.pkl`, and launches the validated v2 SONIC/free-base
viewer. By default it opens a kinematic reference window first; confirm the
expected motion there and close it to start the SONIC physics window. On
2026-08-21 a localhost `.xudp` replay captured 401 motion frames,
generated 4.93 seconds of planned X2 reference, and completed its 4.92-second
dynamic rollout without a fall. This is capture-then-plan simulation, not yet
continuous real-time replanning or hardware validation.

The initial version allowed the planner's mode-0 gesture template to overwrite
most Xsens intent, producing an unrelated salute. The corrected ownership is:
planner root + 12 lower-body joints, Xsens waist/arms/head on every reference
frame, and v2 SONIC for physics control. A corrected 10.02-second CSV rollout
and a fresh 4.92-second UDP-capture rollout both completed without a fall.
These were headless dynamic tests; visually confirm motion correspondence in
the viewer. The six wrist actions remain frozen as required by the published
v2 runtime, so wrist-specific Xsens motion is not expected to track yet.
### Stationary Xsens response through v2 SONIC

The operator-selected default is stationary response, not generated planner
behavior. `capture_xsens_x2_planner_sonic.py --stationary` (the default) holds
world XY and uses the validated idle root/leg reference, places absolute
calibrated Xsens waist/arm/head poses into every frame, and uses v2 SONIC for
dynamic control. It does not let the planner invent salutes, walking, or other
gestures. A 10.04-second stationary composite completed without falling with
joint MAE 0.0986 rad and pelvis-z MAE 0.005 m. Use `--no-stationary` only for
explicit experimental planner locomotion. This is simulation validation only.

### Continuous MVN playback through v2 SONIC

For playback that must move MuJoCo while MVN is still streaming, use
`tools/live_xsens_x2_sonic.py`, not the capture tool. It performs calibration,
buffers one second of already-received frames for SONIC's future horizon, and
runs v2 SONIC/free-base MuJoCo continuously at 50 Hz. The validated idle RSI
owns the stable baseline; streamed waist/arms/head always update. Add
`--track-xsens-legs` for a walking recording so calibrated leg deltas are
applied around the stable idle legs.

Localhost continuous tests completed 427 upper-body policy steps and 400
full-body policy steps without a fall, torque saturation, missing frame, or
malformed packet. They validate headless continuous dynamics only; visually
confirm the new walking/sign clip and do not infer X2 hardware behavior.

The continuous runner passes calibrated, unwrapped pelvis yaw to SONIC as a
root-orientation target relative to the simulated robot's current heading. Its
viewer uses a fixed world camera, so translation and turning remain visible on
the plane rather than being hidden by a robot-following camera. This remains a
policy-controlled simulation: Xsens supplies the future reference, the runner
builds SONIC's 680-value tokenizer and 990-value proprioceptive history, the v2
ONNX model produces 31 actions at 50 Hz, and PD torques drive the free-base X2.
It is not direct MuJoCo `qpos` replay. A focused unit test verifies the yaw
target's row-major 6D rotation layout; visually validating the operator's MVN
turns is still required.

### Xsens pelvis intent through the X2 planner

The experimental `--no-stationary` capture path now converts Xsens world XY
into the calibrated pelvis frame and unwraps pelvis yaw. Per planner chunk it
derives bounded planar velocity and yaw rate, selects idle/slow-walk/walk, and
feeds the planner's empirically verified velocity contract:
`[yaw_rate, velocity_x, velocity_y, velocity_z]`. The prior experimental order
mistook yaw rate for facing-X and therefore commanded continuous spinning; do
not restore it.

A localhost replay captured 943 post-calibration frames and generated an
11.30-second reference. The corrected reference moved 0.229 m / -0.200 m in
XY, had -2.3 degrees net heading change, and no longer accumulated the false
612-degree turn produced by the incorrect contract. Under v2 SONIC it remained
upright for 10.60 seconds, then failed the pelvis-height threshold at 0.395 m.
This validates intent extraction, planner invocation, and removal of the spin
bug, but it is a **failed full-duration dynamic test**. Inspect the final
planner transition and contacts before calling planner locomotion stable.
