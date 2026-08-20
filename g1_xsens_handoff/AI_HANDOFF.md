# Durable AI and Workstation Handoff

Last reviewed: 2026-08-13

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
