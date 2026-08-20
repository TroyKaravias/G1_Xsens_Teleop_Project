# Milestone: Direct-Ethernet Xsens Teleoperation with Proven Arms

Date: 2026-08-11

> Historical milestone: the default launchers changed after this milestone.
> For the current uncommissioned global-pelvis planner path and its required
> test order, read `AI_HANDOFF.md` and `GLOBAL_PELVIS_LOCOMOTION.md`.

## Outcome

This milestone preserves the current no-tunnel Xsens-to-G1 implementation and
the hybrid physical publisher requested for commissioning:

- Xsens MVN sends MXTP02 quaternion poses directly over Ethernet.
- The Xsens bridge and SONIC controller both run on the G1 Jetson.
- Publisher-to-controller ZMQ stays on Jetson loopback, so Mac, WSL, and SSH
  forwarding are not part of the real-time path.
- The current full-body, relative-waist, root-heading, and lifted-foot path is
  combined with the earlier restrained arm behavior: 4 rad/s default arm slew
  and neutral wrist references.

```text
Xsens MVN on Windows
  -> UDP 192.168.123.164:9763
  -> Xsens retargeter on G1 Jetson
  -> ZMQ 127.0.0.1:5556, one latest frame at 50 Hz
  -> SONIC controller on G1 Jetson
  -> Unitree interface enP8p1s0
```

## Primary launchers

- `scripts/run_jetson_current_proven_arms.sh`
  - Current full-body Jetson pipeline.
  - Earlier shoulder/elbow behavior at 4 rad/s by default.
  - Wrist tracking deliberately disabled; wrist references remain neutral.
  - `LEG_BLEND=1.00` by default and adjustable at launch.
- `scripts/run_jetson_controller.sh`
  - Loads the existing SONIC encoder/decoder and observation configuration.
  - Consumes local ZMQ pose messages on port 5556.
  - Temporarily disables Bash nounset while sourcing NVIDIA's environment
    script, fixing the optional `CMAKE_PREFIX_PATH` startup failure.

The exact latest tunnel profile is retained separately as
`scripts/run_jetson_tunnel_profile.sh` for comparison. The earlier July
milestone-equivalent profile is retained as
`scripts/run_jetson_publisher_milestone.sh`.

## Verified in this milestone preparation

- Live Windows MVN reached the Jetson receiver over Ethernet and changed the
  stream state from `STALE` to `LIVE`.
- Direct Jetson replay preflight completed without robot output:
  - 5,605 replayed MXTP02 frames.
  - 719 observed latest-only SONIC messages.
  - Maximum observed message gap: 34.8 ms.
  - Zero bad batches and protocol versions.
  - Publisher reported zero dropped, missing, and malformed packets.
- Local regression suite: 70 tests passed.
- The hybrid launcher's final restrained physical motion envelope must still
  be confirmed on the Jetson under support; the above evidence does not claim
  unsupported walking or dynamic-motion validation.

## Required external Jetson state

These large/vendor components are not included in this milestone folder:

- `/home/unitree/GR00T-WholeBodyControl`
- Existing customized Jetson checkout and compiled
  `gear_sonic_deploy/target/release/g1_deploy_onnx_ref`
- SONIC encoder, decoder, reference data, and observation configuration
- `/home/unitree/.venvs/g1_xsens` containing `numpy` and `pyzmq`
- Unitree firmware, DDS/LowState access, and interface `enP8p1s0`

Do not reset or overwrite the customized GR00T checkout.

## MVN network configuration

Use one enabled robot destination:

```text
Host:          192.168.123.164
Port:          9763
Protocol:      UDP
Stream rate:   Max
Avatar offset: 1
Send paused:   Off
Datagram:      Position + Orientation (Quaternion) only
```

## Upload from Windows

From PowerShell, with this milestone folder on the Desktop:

```powershell
scp `
  "$HOME\OneDrive\Desktop\G1_Xsens_Milestone_2026-08-11\g1_xsens_handoff\scripts\run_jetson_current_proven_arms.sh" `
  "$HOME\OneDrive\Desktop\G1_Xsens_Milestone_2026-08-11\g1_xsens_handoff\scripts\run_jetson_controller.sh" `
  unitree@192.168.123.164:/home/unitree/
```

Then on the Jetson:

```bash
chmod +x ~/run_jetson_current_proven_arms.sh ~/run_jetson_controller.sh
```

## Restrained startup

Terminal 1, with the operator already holding N-pose:

```bash
LEG_BLEND=0.35 bash ~/run_jetson_current_proven_arms.sh
```

Perform N-pose, T-pose, and arms-forward calibration. Continue only after
`Calibration complete` and `Stream state: LIVE`.

Terminal 2:

```bash
bash ~/run_jetson_controller.sh
```

Continue only after `Init Done`. With the robot supported and the physical
E-stop operator ready:

- `]`: engage neutral control.
- `Enter`: toggle live Xsens tracking.
- `O`: stop and enter damping shutdown.

## Safety boundary

- Begin with reduced leg blend and small, slow arm movements.
- Use a support harness, exclusion zone, and independent E-stop operator.
- Stop on incorrect joint direction, abrupt motion, oscillation, crossed legs,
  increasing delay, stale input, or unexpected torso movement.
- Unsupported walking, punches, lunges, jumps, and dynamic fighting are not
  authorized by this milestone.
- Expect the G1 to become limp after controller shutdown; the harness must
  carry it.
