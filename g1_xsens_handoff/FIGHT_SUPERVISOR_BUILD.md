# Fight supervisor build

Status: simulation-only foundation. It has no Unitree SDK, DDS, ROS, or motor
output. The existing physical workflow remains unchanged.

## Built control path

```text
Xsens frame
  -> XsensFightIntentDetector
       pelvis-relative bounded footwork
       debounced left/right jab intent
  -> FightSupervisor
       exactly one route: SAFE | POSE | PLANNER | REFERENCE
  -> SonicFightAdapter
       continuous heading integration
       pinned SONIC manager fields
```

`FightSupervisor` implements these states:

```text
DISARMED -> GUARD/POSE -> LOCOMOTION or BOXING
     |                         |
     +---- ESTOP (latched) ----+
     +---- FALLEN -> GET_UP -> RECOVERING -> GUARD
```

Current commissioning limits are 0.30 m/s combined translation, 0.25 m/s
lateral motion, and 0.50 rad/s yaw. Punches are 0.65-second planner pulses
with a 0.45-second cooldown. These are software intent limits, not proof that
a checkpoint is safe on hardware.

## Run the deterministic replay

From the repository root, using the project Python environment:

```bash
python tools/fight_supervisor_sim.py
```

The replay prints one JSON record per transition. It opens no sockets and
cannot command the G1. Expected route sequence includes guarded pose,
sidestep, turn, jab, safe fall, explicit get-up reference, recovery dwell,
and stale-stream safe-stop.

Run all tests with:

```bash
python -m unittest discover -s tests -v
```

## Ground-truth mapping

- [SONIC `zmq_manager`](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/manager.html)
  is the intended owner-switching mechanism between live pose and planner
  modes.
- [SONIC planner boxing modes](https://nvlabs.github.io/GR00T-WholeBodyControl/references/planner_onnx.html)
  are used for jabs/hooks; Xsens is not converted directly to leg torques or
  a hand-authored punch trajectory.
- Get-up is represented as a named reference route and remains deliberately
  unconnected until the installed SONIC checkpoint's supported recovery
  path is confirmed.
- Future kicks should enter through a validated motion-tracking reference
  policy ([ASAP](https://github.com/LeCAR-Lab/ASAP) /
  [TWIST](https://github.com/YanjieZe/TWIST)-style separation), not raw Xsens
  leg mirroring.

## Physical commissioning gate

Do not connect this supervisor to the physical publisher until all items are
confirmed on the Jetson:

1. Record the installed GR00T-WholeBodyControl git commit and checkpoint.
2. Inspect its `zmq_manager` decoder and verify the command/planner field
   schema against `xsens_bridge/sonic_manager.py`.
3. Verify planner mode IDs 9-16 on that exact checkout. The local serializer
   is pinned to NVIDIA commit `4141c34`; mode IDs added for this build must be
   treated as unverified until that comparison is complete.
4. Confirm which checkpoint supports idle boxing, walk boxing, jabs, hooks,
   lateral movement, and recovery.
5. Connect robot estimator tilt, root height, and angular speed to
   `FallDetector`; never infer robot fall state from the operator's Xsens.
6. Supply an independently tested E-stop and keep the arena human-clear.
7. Commission in simulation, then supported/no-contact physical trials at
   reduced limits, one capability at a time.

The old `ModeSupervisor` continues to report `FIGHTING` unavailable. That is
the deliberate physical-output lock. Enabling it is a later, explicit gate
after the checks above pass.
