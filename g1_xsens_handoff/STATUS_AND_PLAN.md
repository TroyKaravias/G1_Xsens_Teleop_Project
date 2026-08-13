# Current Status and Plan

## Verified

- Original XUDP parses correctly.
- UDP replay/listen: 5,605 frames at approximately 240 Hz, zero missing and malformed.
- Validated G1 reference: 1,167 frames at 50 Hz, 29 DOF, 23.34 seconds.
- Motion order: initial forward lean → N-pose → T-pose → arms forward.
- Shoulder/elbow/hip mapping was visually corrected.
- ProtoMotions official G1 tracker runs in MuJoCo.
- Custom Xsens reference runs through the tracker with a stable root height around 0.79 m.
- ONNX inference measured about 0.16–0.20 ms per 50 Hz control step on RTX 5070.
- Watchdog behavior was verified:
  - `LIVE → HOLD → SAFE_RETURN → RECOVERING → LIVE`
  - HOLD after 100 ms
  - SAFE_RETURN after 300 ms
  - 1.5 second safe return
  - 0.5 second fresh-data requirement
  - 0.75 second recovery blend
  - separate latched hard ESTOP
- Corrective balance steps are intentionally allowed.
- Live Windows Xsens → WSL2 UDP transport is verified at approximately 260 Hz with
  zero missing and malformed frames.
- The integrated simulation-only live runner and automatic N/T calibration have
  been implemented and unit-tested against the recorded XUDP.

## Not yet implemented

- Persistent live N/T calibration profile.
- End-to-end execution of the new integrated runner in WSL, first by recorded replay
  and then from live MVN.
- Any output to Unitree ROS/SDK or the physical G1.
- Jetson deployment and real-robot safety validation.

## Next engineering milestone

Validate the implemented simulation-only process:

`UDP receiver → online N/T calibration → 50 Hz retargeter → watchdog/predictor →`
`ProtoMotions ONNX → MuJoCo`

Start with recorded XUDP replay, then live Xsens MVN. Keep robot output impossible by
construction until the simulation chain survives deliberate stream dropouts and joint/
velocity limit tests.

## Calibration design

Reuse the fixed axes, signs and offsets established by the validated recording.

Normal full calibration:

1. N-pose, still for 2–3 seconds
2. T-pose, still for 2–3 seconds
3. Optional shallow squat
4. Return to N-pose
5. Enable tracking

Separate left/right knee bends are commissioning checks, not normal startup calibration.
After saving a profile, subsequent sessions should usually require only an N-pose check.
