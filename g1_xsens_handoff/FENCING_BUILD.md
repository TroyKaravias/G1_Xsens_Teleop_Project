# Fencing Demonstration Build

## Target demonstration

```text
SAFE -> EN_GARDE -> POSE_TRACKING
                        |-> ADVANCE -> POSE_TRACKING
                        |-> RETREAT -> POSE_TRACKING
                        |-> LUNGE -> RECOVER -> EN_GARDE
```

The first implementation is a robot-output-free supervisor in
`xsens_bridge/fencing_control.py`.

The top-level selector in `xsens_bridge/mode_control.py` adds:

- `MOVING` means general SONIC locomotion: walk forward/backward, turn, and
  stop, while Xsens continues tracking the operator's arms and torso. It does
  not mean unconstrained lower-body imitation.
- `FENCING` for the bounded fencing sequence.
- `FIGHTING` as a visible but fail-closed placeholder until non-contact
  motions and a separate safety envelope are validated.
- Explicit selection and arming for every dynamic mode.
- Automatic disarming and return to `SAFE` when the Xsens stream is lost.

## Initial limits

- Advance speed: 0.15 m/s
- Retreat speed: 0.12 m/s
- Step command: 0.45 s
- Lunge command: 0.65 s
- Recovery: 0.80 s
- Minimum en-garde dwell before live pose tracking: 0.50 s

These are intent limits for simulation integration, not authorization for
physical execution.

## Design

- Live Xsens controls expressive torso and weapon-arm pose only in
  `POSE_TRACKING`.
- Advance and retreat become bounded SONIC planner commands.
- Lunge is a separately validated primitive and always transitions through
  `RECOVER`.
- Loss of a fresh Xsens stream requests recovery and then SAFE.
- Emergency stop is latched and requires an explicit reset with a fresh stream
  and a ready operator.

## Build sequence

1. Unit-test the supervisor.
2. Run the console simulator for deliberate operator events:

   ```bash
   PYTHONPATH=. python3 tools/fencing_state_demo.py
   PYTHONPATH=. python3 tools/mode_demo.py
   ```

3. Connect supervisor output to a MuJoCo-only SONIC planner adapter.
4. Validate en garde, advance, retreat, shallow lunge, and recover.
5. Add pose and joint limits specific to fencing.
6. Add a soft practice weapon model and payload estimate in simulation.
7. Repeat restrained physical neutral/arm tests.
8. Restrained advance/retreat tests.
9. Restrained shallow lunge and recovery.

No person may be used as a fencing opponent during development. Use a target
dummy, a lightweight foam weapon, a clear exclusion zone, a support harness,
and a dedicated physical emergency-stop operator.
