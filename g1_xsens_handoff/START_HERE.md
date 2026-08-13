# Xsens → Unitree G1 Handoff

This package contains the custom and validated parts of a simulation-first pipeline for
retargeting Xsens MVN motion to a Unitree G1 EDU (29 DOF, no hands).

## Current milestone

The restrained physical arm-tracking milestone was completed on 2026-07-29.
Read `MILESTONE_2026-07-29.md` for the verified state and
`JETSON_DIRECT_RUNBOOK.md` for the preferred low-latency deployment without
Mac or WSL SSH tunnels.

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

1. Read `STATUS_AND_PLAN.md`.
2. Read `PROJECT_FRAMEWORK.md` for the architecture and phased plan.
3. Follow `WSL2_SETUP.md` on the Windows workstation.
4. Run `scripts/verify_package.sh`.
5. Run the recorded-UDP test in `RUNBOOK.md`.
6. Only after the recorded test passes, test Windows Xsens MVN → WSL2 UDP.

## Package contents

- `xsens_bridge/`: MXTP02 parser, mapping, retargeting and watchdog code
- `tools/`: converters, preview and watchdog validation tools
- `tests/`: parser, retargeter and watchdog tests
- `data/`: original XUDP recording and validated intermediate references
- `scripts/`: verification and setup helpers
- `EXTERNAL_FILES.md`: official dependencies not redistributed in this package
