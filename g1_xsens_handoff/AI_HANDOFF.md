# Durable AI and Workstation Handoff

Last reviewed: 2026-08-21

This file is the compact, durable context for a person or AI agent opening the
project on another computer. It deliberately lives in Git so it does not depend
on a particular Codex chat being synchronized.

## Read this first

1. Read this file completely.
2. Read `MILESTONE_2026-08-11.md` for the latest documented deployment milestone.
3. Read `JETSON_DIRECT_RUNBOOK.md` before touching the Jetson path.
4. Read `KICK_STABILITY_EXPERIMENT.md` before changing the experimental sidecar.
5. Run `bash scripts/verify_package.sh` from `g1_xsens_handoff/` before editing.

## Project objective

Retarget Xsens MVN motion to a 29-DOF Unitree G1 EDU without hands. Human motion
is a reference rather than direct motor mirroring. The project includes an
offline/simulation path and a Jetson-local physical teleoperation path using the
existing SONIC controller.

## Current physical topology

```text
Xsens suit
  -> Xsens MVN on Windows
  -> MXTP02 quaternion UDP to the active G1 Jetson on port 9763
  -> retargeter on the G1 Jetson
  -> ZMQ on Jetson loopback
  -> existing SONIC controller on the G1 Jetson
  -> Unitree interface enP8p1s0
```

- Old working G1 Jetson Ethernet: `192.168.123.164`
- New target G1 Jetson: `192.168.30.133`
- G1 Jetson current Wi-Fi (verified SSH 2026-08-13): `192.168.8.151`
- Earlier documented lab Wi-Fi: `192.168.30.172`
- MVN is Windows software; WSL2 is used for Windows-side Linux tooling where needed.
- The direct physical path does not require a Mac, WSL tunnel, or SSH tunnel.
- Required vendor/large Jetson state is intentionally not stored here; see
  `EXTERNAL_FILES.md` and `MILESTONE_2026-08-11.md`.

## Current launcher and recent work

The active launcher is `scripts/run_jetson_current_proven_arms.sh`. Its default
mode is now the uncommissioned global-pelvis planner path:

- calibrated Xsens pelvis X/Y displacement becomes bounded forward, backward,
  and lateral SONIC planner locomotion;
- calibrated global pelvis yaw becomes planner facing;
- SONIC owns the balance-critical leg gait while Xsens supplies bounded waist
  and arm targets; and
- network `start` replaces the earlier `]`/`Enter` raw-pose activation sequence.

Read `GLOBAL_PELVIS_LOCOMOTION.md` before using it. The initial speed is 0.20
m/s with a hard software ceiling of 0.35 m/s. The position follower integrates
commanded velocity because robot odometry is not yet fed back, so it is
open-loop and not verified global robot position tracking.

Set `TELEOP_MODE=raw_pose` to restore the earlier behavior, which combines:

- direct MVN-to-Jetson UDP;
- full-body, relative-waist, root-heading, and lifted-foot reference handling;
- the earlier conservative arm behavior (4 rad/s default, neutral wrists); and
- an integrated, opt-out kick-stability sidecar (`KICK_STABILITY=1` by default).

The latest raw-pose kick work adds progressively stronger support-leg and waist protection
as the swing leg lifts, including support hip/knee/ankle and waist counterpose
biases. The launcher exposes those values as environment variables and passes
them to `tools/kick_stability_sidecar.py`. The shaping logic is in
`xsens_bridge/kick_stability.py`, with regression coverage in
`tests/test_kick_stability.py`.

Important limitation: the sidecar shapes references only. It does not consume
robot IMU or foot-contact feedback and is not a balance, fall-recovery, or get-up
controller.

## Verified evidence versus work still requiring verification

Documented as verified by the 2026-08-11 milestone:

- live Windows MVN packets reached the Jetson directly over Ethernet;
- recorded replay sent 5,605 MXTP02 frames;
- the Jetson-local path observed 719 latest-only SONIC messages with a 34.8 ms
  maximum gap and no bad batches/protocol versions;
- the publisher reported no dropped, missing, or malformed packets; and
- the regression suite at that milestone reported 70 passing tests.

Do **not** infer from that evidence that unsupported walking, kicking, fighting,
or other dynamic physical motion is validated. The newest adaptive kick tuning
must be rechecked with the current test suite and commissioned in the documented
order: replay without controller, live stream without controller, then a restrained
gantry test with support and a dedicated physical E-stop operator.

## Safety boundary

- Physical tests require a harness/support, exclusion zone, and independent E-stop operator.
- Begin with reduced leg blend and small, slow movements.
- Stop on wrong joint direction, abrupt motion, oscillation, crossed legs,
  increasing latency, stale input, or unexpected torso movement.
- Expect the robot to become limp after controller shutdown; support must carry it.
- Never describe unit tests or simulation as physical-robot validation.
- Never overwrite the customized `/home/unitree/GR00T-WholeBodyControl` checkout.

## Jetson deployment ownership

- The operator owns Jetson uploads and installation. By default, make and verify
  repository changes locally, then provide exact commands for the operator to
  back up, upload, install, verify, and roll back Jetson files.
- Do not upload or replace Jetson files directly unless the operator explicitly
  overrides this rule for that deployment.
- Never include a password in a command, script, repository file, or handoff;
  SSH/SCP authentication must remain interactive or use operator-managed keys.

## Useful commands

From the repository root:

```bash
cd g1_xsens_handoff
bash scripts/verify_package.sh
python -m unittest discover -s tests -v
```

Jetson launcher (only under the runbook's physical safety conditions):

```bash
bash ~/run_jetson_current_proven_arms.sh
```

Disable the experimental sidecar for comparison:

```bash
KICK_STABILITY=0 bash ~/run_jetson_current_proven_arms.sh
```

### Operator shorthand: Terminal A

When the operator asks for **Terminal A**, provide the current updated version
of this development launcher command. As recorded on 2026-08-19, it is:

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

Update this block whenever code or deployment changes require a different
Terminal A command.

### Operator shorthand: Terminal B

When the operator asks for **Terminal B**, provide the current updated version
of this controller launcher command. As recorded on 2026-08-19, it is:

```bash
cd /home/unitree/g1_xsens_development/g1_xsens_handoff

bash ./scripts/run_jetson_global_position_controller.sh
```

Update this block whenever code or deployment changes require a different
Terminal B command.

## Windows continuation

Follow `WINDOWS_CONTINUATION.md`. On a new Codex thread, use this prompt:

> Read `AGENTS.md` and `g1_xsens_handoff/AI_HANDOFF.md`, then inspect the current
> branch and recent Git history. Summarize the verified state, safety boundary,
> and next task before changing code. Do not treat tests as hardware validation.

## Immediate handoff state

- Checkpoint boundary requested on 2026-08-20: the current X2 path is live
  Xsens semantic retargeting into bounded PD-controlled, pinned-base MuJoCo
  dynamics. It is **not** an X2 SONIC policy, balance controller, or
  foot-force-driven locomotion implementation. The next development task is
  to identify and obtain a genuine X2-specific SONIC policy/runtime contract
  (encoder, decoder/policy, observations, action/joint ordering, configuration,
  and compatible simulator integration), then validate its provenance and
  model compatibility before connecting the existing Xsens reference layer.
  Do not run the G1 SONIC policy against the X2 embodiment or describe the
  current PD simulation as SONIC.
- On 2026-08-20, the operator selected X2 SONIC as the primary X2 development
  architecture. The published Sonic AgiBot X2 Port demonstrates that trained
  X2 Ultra policies exist and reports IsaacLab, MuJoCo, and gantry-supported
  hardware runs, but its project page and paper do not publish the canonical
  checkpoint. NVlabs/GR00T-WholeBodyControl PR #112 supplies the experimental
  31-DoF embodiment, training configuration, order converters, and X2 MuJoCo
  evaluator. `scripts/fetch_x2_sonic_source.sh` pins that external source at
  commit `489d6dffc6e327b2e28f995d21e26ab300227ada`; it remains ignored and
  separate from the preserved G1 code. `scripts/check_x2_sonic_assets.sh`
  validates the selected fused `.onnx` policy and optional X2-retargeted `.pkl`
  reference. The operator supplied `tinkerbuggy/sonic-x2`; its 58,505,722-byte
  `sonic_policy/x2_sonic_policy.onnx` is stored locally under ignored
  `external/sonic_x2/` and matches the Hugging Face manifest SHA-256
  `e7ccd6522010ea660facfb7265fb129dac2b580dd1483cc1dbada73c205309d7`.
  The model card declares a 1670-float actor observation and 31-float X2 action,
  but its weights license is still pending review/all-rights-reserved. File
  identity and CPU ONNX inference are verified: ONNX Runtime 1.23.2 reports
  one `obs[batch,1670]` float input and one `action[batch,31]` float output;
  a synthetic finite observation produced a finite 31-action result.
  `xsens_bridge/x2_sonic.py` now enforces the 680-tokenizer + 990-proprioception
  ordering and IsaacLab-compatible ten-frame, term-major history; four focused
  tests pass. This is graph-contract validation only, not meaningful motion,
  dynamic simulation, or hardware validation. Immediate next task: reproduce
  a known-motion baseline in free-base MuJoCo, then replace the
  motion-file provider with a buffered live-Xsens reference provider. Do not
  substitute the public G1 checkpoint or call the existing PD runner SONIC.
  The first free-base ONNX integration now exists as
  `tools/simulate_xsens_x2_sonic.py`. It identified that the official v1.3
  MJCF has no floor and adds a colliding plane at runtime without editing the
  vendor asset. It also uses per-future-frame tokenizer layout
  `[31 q, 31 qd, 6D heading]`; the older grouped reshape was rejected because
  it permutes policy semantics. Neither available fused policy currently
  survives the synthetic SONIC-default/Xsens-delta startup: the generic graph
  fell at 0.50 s and the checksum-verified step-14,000 graph fell at 0.36 s.
  Treat these as failed dynamic tests. Immediate blocker: obtain or reconstruct
  one known training-distribution X2 motion PKL plus its RSI frame and reproduce
  the publisher baseline before diagnosing the live Xsens reference layer.
- On 2026-08-21, `tools/live_xsens_x2_sonic.py` connected the live MXTP02
  receiver and unchanged 9-second N/T + 3-second arms-forward calibration to
  the actual fused X2 SONIC policy. `X2SonicDelayedReferenceBuffer` introduces
  a deterministic one-second delay, using ten already-received Xsens frames as
  the policy's 0.1...1.0 s future horizon instead of prediction. A localhost
  replay captured 1,371 calibration frames, processed 117 live reference
  frames with zero missing/malformed packets, and executed 21 real policy
  steps before terminating on pelvis height 0.392 m; 51 torque saturation
  events occurred. This verifies the UDP-to-policy dataflow, not balance or
  successful dynamic tracking. The live integration is no longer missing;
  the immediate X2 blocker remains exact known-motion/RSI reproduction for the
  supplied policy, followed by fixing the 0.42-second free-base fall without
  tuning around an unverified observation or initialization mismatch.
- Later on 2026-08-21, the immediate startup fall was fixed by matching two
  subtle contracts in the X2 training code: the tokenizer command is packed as
  all ten position frames followed by all ten velocity frames before its 10x62
  reshape, and identity orientation in the stored 3x2 rotation layout is
  `[1, 0, 0, 1, 0, 0]`. The runner also reconstructs the missing live root
  height from the X2 foot collision spheres, equivalent to the contact-height
  component that known-motion RSI would normally provide. A headless replay
  then ran 3.02 seconds / 151 policy steps with zero falls, minimum pelvis
  height 0.601 m, and maximum tilt 13.0 degrees. An 8-second request entered
  the Xsens motion and fell at 3.40 seconds, so sustained Xsens-derived dynamic
  tracking is **not** validated. A localhost live-path replay reached 28 policy
  steps without falling and ended because the saved UDP stream became stale;
  this verifies the repaired startup and data path, not sustained balance.
  Immediate next task: visually test a longer live/playback stream, then make
  the Xsens-to-policy reference transition remain inside the policy's motion
  distribution instead of tuning MuJoCo contacts or bypassing fall detection.
- A subsequent zero-Xsens-motion control on 2026-08-21 showed that the supplied
  step-14000 policy also falls from the static SONIC default reference at 3.82
  seconds. Therefore the observed stand-then-fall is not attributable to suit
  motion or leg retargeting. The earlier 3.02-second no-fall run verified the
  repaired startup contract only, not stable standing. Do not add auto-reset,
  fixed-base support, gain reduction, or leg masking and call that balance.
  The blocking input is still a publisher-matching X2 motion anchor plus its
  full RSI root pose and velocity state; reproduce that baseline dynamically
  before evaluating Xsens substitution.
- The publisher's current Hugging Face quickstart was then followed directly.
  The standalone `meetsitaram/sonic-x2` quick-play bundle is cloned (ignored)
  at `external/sonic_x2_quickplay`; it supplies the missing X2 motion PKLs,
  full RSI state, exact evaluator, and recommended frozen-G1-core LoRA v2
  tracker. With parity gains, action clip 20, and frozen wrists, idle completed
  all 9.62 seconds and ended on `motion_end` with pelvis-z MAE 0.003 m; relaxed
  walk completed a 10.02-second headless rollout without a fall. These are
  genuine free-base MuJoCo dynamic baselines, not hardware validation. The
  matching frozen-G1-core v1 planner is downloaded under ignored
  `external/sonic_x2/`. Immediate task: feed Xsens into planner context/intent
  while preserving its full root+joint reference and RSI, then repeat these
  dynamic gates before calling Xsens tracking successful.
- `xsens_bridge/x2_planner.py` now wraps the matching planner's strict
  `(4,38)` context contract, and `tools/generate_x2_planner_reference.py`
  chains its 64-frame predictions. Planner-only idle survived 10.02 seconds
  under v2 SONIC/free-base MuJoCo with no fall. Recorded Xsens upper-body
  deltas were then inserted into the planner context while retaining generated
  root and locomotion state; that trajectory also survived 10.02 seconds with
  no fall. Relative to planner-only idle it changed upper-body joints by mean
  0.257 rad / max 2.377 rad, so this was not a no-op. This is the first
  successful recorded-Xsens -> X2 planner -> v2 SONIC dynamic simulation, not
  yet live UDP and not hardware validation. Immediate task: convert this exact
  planner-context path to the calibrated live MXTP02 stream without reverting
  to direct SONIC joint-reference mapping.
- `tools/capture_xsens_x2_planner_sonic.py` now accepts either MVN recording
  playback or a live suit as MXTP02 UDP on port 9764, runs the unchanged 9 s
  N/T + 3 s arms-forward calibration, captures post-calibration motion, plans
  a full X2 root+joint reference, and optionally opens the v2 SONIC evaluator.
  Localhost replay captured 401 post-calibration frames, generated 4.93 s of
  X2 motion, and completed the 4.92-second free-base SONIC rollout without a
  fall. This validates capture-then-plan playback dynamics, not continuous
  real-time replanning and not hardware. Immediate task: make replanning
  continuous while retaining the same planner context and safety contracts.
- The first capture tool only perturbed four planner context frames, allowing
  mode 0 to resume its learned template (observed by the operator as a salute)
  instead of following MVN. This is fixed: planner output owns root and the 12
  lower-body joints, while calibrated Xsens waist/arm/head values replace all
  19 upper-body reference joints at every 30 Hz frame before v2 SONIC. The
  corrected CSV path survived 10.02 s, and a fresh localhost UDP capture (402
  Xsens frames, 4.90 s reference) survived its complete 4.92 s free-base
  rollout without a fall. Headless dynamics verify survival, not visual
  correspondence; operator visual review is next. Wrists remain frozen because
  that is a required v2 runtime safety setting.
- Operator visual review rejected that result: the generated reference showed
  non-punch arm poses. Diagnosis separated kinematics from dynamics. The saved
  10-second reference exceeded X2 elbow limits (down to -3.25 rad versus the
  -2.356 rad MJCF limit), used planner-idle-relative deltas instead of absolute
  calibrated X2 poses, and truncated capture at exactly 10 seconds. The capture
  tool now uses absolute retargeted upper-body poses on every frame, clamps all
  31 joints to the quick-play MJCF, defaults to capture-until-stream-end, and
  supports `--motion-start-delay`. SONIC tracked a corrected 10.04-second
  reference at joint MAE 0.101 rad without falling, proving tracker response;
  this does **not** prove the operator's punch clip was captured. Immediate
  acceptance gate: kinematically inspect the full newly captured reference and
  confirm the punch sequence before judging SONIC dynamics.
- Operator clarified that the desired mode is stationary X2 response, matching
  the earlier direct preview but with SONIC as the physics controller. The
  capture tool now defaults to `--stationary`: it bypasses generative planner
  motion, holds world XY and the validated idle root/12 leg reference, replaces
  all 19 waist/arm/head reference values with absolute calibrated Xsens poses,
  then lets v2 SONIC produce actions. A 10.04-second stationary composite
  completed on `motion_end` without a fall (joint MAE 0.0986 rad, pelvis-z MAE
  0.005 m). This validates stationary dynamic tracking of that generated
  reference, not visual punch correspondence or hardware. `--no-stationary`
  retains experimental planner locomotion but is not the desired default.
- User clarified the required execution is continuous MVN network playback,
  not capture-then-generate-then-view. `tools/live_xsens_x2_sonic.py` now uses
  the validated v2 tracker and idle RSI directly after the unchanged 9+3 s
  calibration, consumes each arriving MXTP02 frame, maintains a one-second
  received-frame lookahead, and updates free-base MuJoCo at 50 Hz while MVN is
  still playing. A localhost stream processed 766 frames / 427 policy steps
  with zero loss, malformed packets, torque saturation, or fall, stopping only
  on stale playback. `--track-xsens-legs` applies calibrated leg deltas around
  stable idle legs for walking clips; an 8-second full-body stream completed
  400 policy steps with no fall or saturation. These are headless continuous
  simulation tests, not visual correspondence or hardware validation.
- On 2026-08-20, `tools/live_xsens_x2.py` added the direct simulation path
  from MXTP02 UDP to fixed-base X2 v1.3 dynamics. It accepts live-suit and MVN
  playback packets identically, performs streamed N-to-T calibration, reuses
  the bounded online human reference logic, maps semantic G1-layout references
  into 31 X2 joints, and holds the last bounded pose with zero target velocity
  on stale input. An end-to-end loopback replay calibrated from 1,002 frames
  and processed 253 live references with zero missing/malformed packets, stale
  transitions, or torque saturation. This is fixed-base simulation validation,
  not balance, locomotion, policy, or hardware validation. The immediate task
  is operator visual review using MVN playback on UDP port 9764, then tuning
  X2-specific live mapping based on observed motion.
  The initial per-physics-step viewer synchronization caused slow response on
  ordinary displays. It now advances 1 ms physics substeps to wall time,
  renders at about 60 Hz, and refreshes transforms after restoring the pinned
  base. A timing replay measured a 0.998x real-time factor with zero
  saturation. The viewer also adds a non-colliding checkered floor, light
  gradient sky, and broad ambient/head lighting; these are scene aids, not
  ground support.
  Live X2 calibration now appends a three-second arms-forward hold after the
  original nine-second N/T phase. The N-to-T prompt remains unchanged at 40%
  of that original phase (3.6 seconds with defaults). The measured forward
  pose is used by the existing online arm-offset calibration.
  Calibrated global pelvis yaw now rotates the entire X2 base by default while
  its world position and base roll/pitch remain pinned. Heading is continuous,
  gain-adjustable, and limited to 180 deg/s by default; use
  `--no-track-global-yaw` for the previous fixed-facing view. This is global
  orientation visualization, not free-base balance or locomotion validation.
  Calibrated pelvis displacement also moves the pinned X2 root across world X/Y
  by default, with a 0.75 m/s speed bound, filtering, and rejection of position
  jumps above 0.25 m per sample. The checkered floor and viewer remain in world
  coordinates. Use `--no-track-global-position` for centered playback. This is
  kinematic root placement, not foot-force-driven locomotion.
- On 2026-08-20, the first X2 fixed-base MuJoCo dynamics controller was added.
  It uses the official model's 31 torque motors, conservative joint-group PD
  gains, vendor torque limits, interpolated recorded references, and a
  numerically pinned floating base. The full v1.3 recording completed at
  0.0132 rad RMS tracking error, 0.1870 rad peak error, and zero torque
  saturation events across 734,421 joint-steps. The v1.4 run exposed persistent
  `head_pitch_link`/`torso_link` and `torso_link`/`waist_yaw_link`
  self-contact, 0.5352 rad peak head error, and 16,253 head-pitch torque
  saturations; do not hide this with higher gains.
  Current dynamics development should use v1.3 unless actual X2 hardware is
  confirmed as v1.4, in which case the official model contact geometry must be
  resolved. This fixed-base result is dynamics tracking evidence, not balance,
  locomotion, policy, or hardware validation. The immediate task is v1.3
  continuous visual dynamics review and explicit tracking/contact acceptance
  thresholds before any free-base controller work.
- On 2026-08-20, X2-specific visual/retarget validation corrected an erroneous
  elbow sign in the simulation-only adapter, added a default 4 Hz X2-only
  joint filter, per-joint clamp diagnostics, and headless front/right snapshot
  rendering. Both pinned variants replay all 1,167 frames. Representative
  snapshots show coherent neutral, T-pose, arm elevation, elbow flexion, and
  return. Clamps decreased to 1,054/36,177 on v1.3 and 951/36,177 on v1.4;
  peak kinematic speed decreased from 10.36 to 4.35 rad/s. This is sampled
  visual and kinematic software validation only, not continuous motion,
  dynamics, balance, policy, or hardware validation. The immediate X2 task is
  continuous passive-viewer inspection of remaining knee, right-shoulder-roll,
  and v1.3 waist-pitch limits, followed by X2-specific lower-body calibration
  and explicit acceptance thresholds.
- On 2026-08-20, branch `agibot-x2-sim` added the first AgiBot X2 simulation
  path. It pins the official `AgibotTech/agibot_x2_urdf` assets at commit
  `77f43eb0904dae4c48ccd9154fee824f8ffd4d38`, maps calibrated Xsens/G1-layout
  joint deltas to the X2's 31 named joints, and provides an actuator-free
  MuJoCo replay tool. MuJoCo 3.12 loaded both official v1.3 and v1.4 models and
  replayed the 1,167-frame recording. This is kinematic software validation,
  not balance, policy, or hardware validation. The immediate X2 task is visual
  joint-axis review and X2-specific filtering/tuning before dynamics or SONIC
  embodiment work; `X2_SIMULATION.md` is the runbook.
- On 2026-08-20, the working Jetson global-position implementation was captured
  for GitHub: `xsens_bridge/global_position.py`,
  `tools/live_xsens_global_position.py`, its regression tests, and the exact
  Terminal A/B launchers. `G1_INSTALLATION.md` documents reconstruction from a
  clone while keeping vendor SONIC binaries/models external. Local tests cover
  this source path but are not physical-motion verification.
- On 2026-08-17, the August 12 restored baseline was installed by the operator
  on the new G1 Jetson, now identified as `192.168.30.133`. This G1 uses the installed SONIC
  `policy/release` decoder, encoder, and observation configuration rather than
  the old G1's `policy/low_latency` paths. System Python 3.10 has NumPy 1.26.4
  and user-installed ARM64 pyzmq 26.4.0. The restored package passed all 83
  tests on the new Jetson. No physical behavior is claimed from those tests.
- The old working G1 is `192.168.123.164`. Its verified 8.8 GB milestone
  archive was downloaded to the workstation and passed `sha256sum -c`. The
  old working runtime uses controller SHA-256 `3028dbaf...0953` plus the
  matching `policy/low_latency` model set. The new G1 controller was observed
  as `454fa530...18c6`, has no `policy/low_latency` directory, and its launcher
  selects `policy/release`; do not confuse the two robots during restoration.
- On 2026-08-18, the operator installed the old working SONIC runtime subset
  on the new G1 via `192.168.30.133`. The controller and all three
  `policy/low_latency` files match the archived SHA-256 values, and the launcher
  selects the low-latency decoder, encoder, and observation configuration.
  The displaced new-G1 state is recoverable at
  `/home/unitree/before_working_sonic_restore_20260818_222533`; physical robot
  behavior has not yet been tested. The immediate next step is a loopback-only
  startup check before any supported hardware engagement.
- The complete archived user-space stack was subsequently extracted on the new
  G1 at `/home/unitree/old_g1_full_restore_20260811`. With `LD_LIBRARY_PATH`
  selecting that tree's ONNX Runtime 1.16.3 and Unitree SDK libraries, the
  controller passed a 15-second loopback-only startup check and exited with the
  expected timeout status 124, without heap corruption. This establishes a
  runtime-library mismatch in the new installation; it is not physical-motion
  validation and does not by itself distinguish ONNX Runtime from Unitree SDK
  as the mismatched library.
- A first live connection had upper-body behavior but no locomotion because the
  isolated controller was launched with legacy `--input-type zmq` while the
  global-pelvis publisher emits `zmq_manager` command/planner messages. For
  global pelvis locomotion, launch the isolated controller with
  `--input-type zmq_manager` and
  `--planner-file planner/target_vel/V2/planner_sonic.onnx`; arm by typing
  `ARM` in the publisher after controller `Init Done`, not by pressing Enter in
  the controller. SONIC owns the legs in this mode, so `LEG_BLEND` is not the
  locomotion control.
- First commissioning on the new G1 must use `KICK_STABILITY=0 LEG_BLEND=0`
  for a restrained arm-only check before introducing any leg blend. Global
  pelvis translation is not present in this restored baseline.

- Git branch at handoff creation: `work`.
- Repository initially had one local commit and no configured Git remote.
- The original cloud workspace had no configured Git remote. The owner's Linux
  checkout may already have `origin`; inspect it rather than adding it again. Complete
  the branch-independent authenticated push described in `WINDOWS_CONTINUATION.md`.
- The immediate engineering task is to validate the adaptive kick-stability changes
  with replay/no-controller checks before any restrained hardware commissioning.
- On 2026-08-13, the current `scripts/`, `tools/`, `xsens_bridge/`, and matching
  `tests/` trees were uploaded over Wi-Fi to
  `/home/unitree/g1_xsens_direct/g1_xsens_handoff` on the Jetson. Convenience
  copies of `run_jetson_current_proven_arms.sh`, `run_jetson_controller.sh`, and
  `live_xsens_sonic.py` were also refreshed in `/home/unitree`. SHA-256 checks
  matched the Linux source, imports succeeded, and all 86 Jetson tests passed.
  No publisher, sidecar, or SONIC controller process was started.
- The pre-upload Jetson state is recoverable at
  `/home/unitree/g1_xsens_upload_backup_20260813_DOvBbh`.
- The immediate debugging task is to diagnose why `Enter` does not toggle live
  Xsens reference tracking after SONIC reaches `Init Done`. The new planner
  publisher bypasses that input path using repeated network `start` messages,
  but this has not yet been verified on hardware.
- The current engineering task is to validate global-pelvis planner locomotion
  in MuJoCo, then with live Xsens/no controller, then neutral under support,
  before one restrained forward test. Forward/backward/lateral code and tests
  are not physical validation.

Update this section whenever that next task or verification status changes.

On 2026-08-21, the continuous X2 MXTP02 runner was extended to pass calibrated,
unwrapped Xsens pelvis yaw into the published v2 SONIC policy as a root-heading
target relative to the simulated robot heading. Its MuJoCo viewer now uses a
fixed world camera so world translation and turns remain visible. The path is
genuine SONIC inference (680 reference values + 990 proprioception values, 31
policy actions at 50 Hz, applied through PD torques), not direct `qpos`
animation. The 6D yaw representation has a focused unit test; 129 unrestricted
tests plus the separately permitted UDP test pass. Actual visual turning with
the operator's MVN playback is the immediate next validation. This is
simulation software validation only, not X2 hardware or completed
motion-quality validation.
