# Global Pelvis Locomotion Commissioning

This experimental physical mode follows calibrated Xsens global horizontal
pelvis motion through SONIC's locomotion planner:

- forward/backward pelvis displacement requests forward/backward walking;
- left/right pelvis displacement requests lateral walking;
- calibrated global pelvis yaw controls the facing direction; and
- Xsens waist and arms remain bounded upper-body targets.

SONIC owns the hip, knee, and ankle gait trajectory while walking. The installed
`zmq_manager` exposes a 17-DOF waist/arms override, not a simultaneous raw-leg
override. Sending raw Xsens leg joints while SONIC generates a support-leg gait
would give two controllers ownership of the balance-critical joints.

## Important limitation

The current bridge receives no robot odometry. It estimates robot horizontal
travel by integrating the bounded commands it sent. This is open-loop global
following, not verified closed-loop position tracking. Slip, harness loading, or
blocked motion can make the estimate differ from actual robot position.

## Defaults and safety gates

- Engage distance: 0.08 m from the calibrated pelvis origin.
- Release distance: 0.04 m.
- Initial maximum speed: 0.20 m/s.
- Hard software maximum: 0.35 m/s.
- Xsens stale timeout: 300 ms, then planner IDLE.
- A horizontal sample jump over 0.20 m latches a stop requiring restart.
- An excursion over 2.0 m latches a stop requiring restart.
- Arming requires the exact terminal confirmation `ARM`.
- Publisher startup sends SONIC's network `start` command repeatedly, avoiding
  the old raw-ZMQ `]` and `Enter` engagement sequence.

These are software guards, not proof of physical stability.

## Required commissioning order

1. Run the full unit suite.
2. Validate the planner path in MuJoCo.
3. Run live Xsens with the publisher while SONIC is off and inspect labels,
   desired displacement, estimated displacement, speed, and heading.
4. Run a harness-supported neutral-only test with the operator inside the
   0.08 m dead zone.
5. With an independent E-stop operator, test one small forward displacement.
6. Only after forward stop/recovery succeeds, separately test backward and
   lateral motion.

Do not begin with combined diagonal motion or large heading changes. Stop on
wrong direction, foot crossing, oscillation, increasing lag, unexpected trunk
motion, or harness loading.

## Two terminals on the Jetson

Terminal 1—publisher and Xsens calibration:

```bash
bash ~/run_jetson_current_proven_arms.sh
```

After calibration, keep the operator centered and still. Do not type `ARM`
until Terminal 2 reports `Init Done` and the physical safety team is ready.

Terminal 2—SONIC planner controller:

```bash
bash ~/run_jetson_controller.sh
```

After `Init Done`, return to Terminal 1 and type exactly:

```text
ARM
```

The publisher sends SONIC START in planner mode. Do not press `]` or Enter in
this mode. Use `O` in Terminal 2 for controller stop/damping shutdown.

## Conservative tuning

Keep the first supported test at the defaults. After recorded evidence shows
correct direction and clean stops, the maximum can be adjusted up to the hard
0.35 m/s ceiling:

```bash
PELVIS_MAX_SPEED_MPS=0.25 bash ~/run_jetson_current_proven_arms.sh
```

Widening raw hip/knee/ankle tolerances is not part of this mode. Restore the
earlier raw-pose launcher only for the already documented restrained pose tests:

```bash
TELEOP_MODE=raw_pose LEG_BLEND=0.35 bash ~/run_jetson_current_proven_arms.sh
SONIC_INPUT_TYPE=zmq bash ~/run_jetson_controller.sh
```

The publisher and controller modes must match.
