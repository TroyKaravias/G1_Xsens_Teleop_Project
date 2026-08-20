# G1 Xsens Teleoperation

Xsens MVN motion retargeting and bounded global-position locomotion for a
29-DOF Unitree G1 EDU without hands.

The repository contains the custom Python bridge, launchers, recorded replay
data, tests, and deployment documentation. Unitree/NVIDIA controller binaries,
SONIC policy models, and other vendor assets are intentionally not redistributed.

Start here:

- [G1 installation and operation](g1_xsens_handoff/G1_INSTALLATION.md)
- [Current durable handoff](g1_xsens_handoff/AI_HANDOFF.md)
- [External/vendor dependencies](g1_xsens_handoff/EXTERNAL_FILES.md)
- [Direct Jetson runbook](g1_xsens_handoff/JETSON_DIRECT_RUNBOOK.md)

## Safety

This is experimental robotics software. Unit tests and replay checks do not
validate physical behavior. Use a support harness, exclusion zone, and an
independent physical E-stop operator. Begin with neutral/arm-only commissioning
and stop on unexpected motion, oscillation, latency, or stale input. Dynamic
unsupported motion is not validated.
