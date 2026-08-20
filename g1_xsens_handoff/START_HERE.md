# Xsens → Unitree G1 Handoff

This package contains the custom and validated parts of a simulation-first pipeline for
retargeting Xsens MVN motion to a Unitree G1 EDU (29 DOF, no hands).

## Current milestone

The restrained physical arm-tracking milestone was completed on 2026-07-29.
Read `MILESTONE_2026-07-29.md` for the verified state and
`JETSON_DIRECT_RUNBOOK.md` for the preferred low-latency deployment without
Mac or WSL SSH tunnels.

The working Jetson global-position path captured on 2026-08-20 is preserved in
`xsens_bridge/global_position.py`, `tools/live_xsens_global_position.py`, and
the Terminal A/B launchers. New G1 installations should begin with
`G1_INSTALLATION.md`. This capture preserves observed working software but does
not expand the physical safety boundary.

## Safety boundary

Physical use is limited to the supported, restrained test sequence documented
in the milestone. Unsupported walking and dynamic motion are not validated.

## Hardware and topology

- G1: Unitree G1 EDU, 29 DOF, no hands, Jetson onboard
- G1 Jetson internal Ethernet: `192.168.123.164`
- G1 Jetson lab Wi-Fi: `192.168.30.172`
- Lab workstation: RTX 5070, 12 GB
- Windows is required for Xsens MVN
- The workstation is dual boot, so native Ubuntu and Windows cannot run simultaneously
- Intended single-PC live topology:
  - Windows: Xsens MVN
  - WSL2 Ubuntu: bridge, retargeting, ONNX, MuJoCo

## Begin here

1. Read `AI_HANDOFF.md` for the current cross-machine and AI-agent context.
2. For a new G1, follow `G1_INSTALLATION.md`.
3. Read `MILESTONE_2026-08-11.md` for the latest documented deployment milestone.
4. Read `STATUS_AND_PLAN.md` and `PROJECT_FRAMEWORK.md` for the architecture and plan.
5. Read `GLOBAL_PELVIS_LOCOMOTION.md` before using the current experimental
   planner-based physical launchers.
6. Follow `WINDOWS_CONTINUATION.md` when moving development to Windows.
7. Follow `WSL2_SETUP.md` on the Windows workstation.
8. Run `scripts/verify_package.sh`.
9. Run the recorded-UDP test in `RUNBOOK.md` before any live-input test.

For the separate AgiBot X2 simulation track, follow `X2_SIMULATION.md`. That
path is kinematic simulation only and does not reuse the G1 hardware output.

## Package contents

- `xsens_bridge/`: MXTP02 parser, mapping, retargeting and watchdog code
- `tools/`: converters, preview and watchdog validation tools
- `tests/`: parser, retargeter and watchdog tests
- `data/`: original XUDP recording and validated intermediate references
- `scripts/`: verification and setup helpers
- `EXTERNAL_FILES.md`: official dependencies not redistributed in this package
- `X2_SIMULATION.md`: pinned official X2 models and MuJoCo replay instructions
