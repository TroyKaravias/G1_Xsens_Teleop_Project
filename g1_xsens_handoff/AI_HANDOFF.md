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
  -> MXTP02 quaternion UDP to 192.168.123.164:9763
  -> retargeter on the G1 Jetson
  -> ZMQ on Jetson loopback
  -> existing SONIC controller on the G1 Jetson
  -> Unitree interface enP8p1s0
```

- G1 Jetson internal Ethernet: `192.168.123.164`
- G1 Jetson lab Wi-Fi: `192.168.30.172`
- MVN is Windows software; WSL2 is used for Windows-side Linux tooling where needed.
- The direct physical path does not require a Mac, WSL tunnel, or SSH tunnel.
- Required vendor/large Jetson state is intentionally not stored here; see
  `EXTERNAL_FILES.md` and `MILESTONE_2026-08-11.md`.

## Current launcher and recent work

The active launcher is `scripts/run_jetson_current_proven_arms.sh`. It combines:

- direct MVN-to-Jetson UDP;
- full-body, relative-waist, root-heading, and lifted-foot reference handling;
- the earlier conservative arm behavior (4 rad/s default, neutral wrists); and
- an integrated, opt-out kick-stability sidecar (`KICK_STABILITY=1` by default).

The latest kick work adds progressively stronger support-leg and waist protection
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

## Windows continuation

Follow `WINDOWS_CONTINUATION.md`. On a new Codex thread, use this prompt:

> Read `AGENTS.md` and `g1_xsens_handoff/AI_HANDOFF.md`, then inspect the current
> branch and recent Git history. Summarize the verified state, safety boundary,
> and next task before changing code. Do not treat tests as hardware validation.

## Immediate handoff state

- Git branch at handoff creation: `work`.
- Repository initially had one local commit and no configured Git remote.
- The original cloud workspace had no configured Git remote. The owner's Linux
  checkout may already have `origin`; inspect it rather than adding it again. Complete
  the branch-independent authenticated push described in `WINDOWS_CONTINUATION.md`.
- The immediate engineering task is to validate the adaptive kick-stability changes
  with the regression suite and replay/no-controller checks before any restrained
  hardware commissioning.

Update this section whenever that next task or verification status changes.
