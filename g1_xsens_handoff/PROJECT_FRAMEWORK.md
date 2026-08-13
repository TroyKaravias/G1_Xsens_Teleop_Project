# Project Framework

## Objective

Use an Xsens suit to command the whole-body motion of a 29-DOF Unitree G1 EDU in near
real time, including arms, legs, squats, lunges and eventually dynamic motions, while a
learned controller maintains balance. The G1 has no dexterous hands in this project.

This is not direct joint mirroring. Human motion provides a reference; a G1 tracking
policy produces dynamically feasible robot commands and may take corrective steps.

## System architecture

```text
Xsens suit
    ↓
Windows Xsens MVN
    ↓  MXTP02 UDP, approximately 240 Hz
Packet parser and stream-health monitor
    ↓
Live calibration and coordinate normalization
    ↓
Human-to-G1 retargeter, 29 joint references at 50 Hz
    ↓
Reference predictor + dropout watchdog
    ↓
ProtoMotions G1 tracking policy (ONNX)
    ↓
Simulation or robot-state feedback loop
    ├── Current stage: MuJoCo
    └── Future stage: Jetson + Unitree ROS/SDK
```

## Why a policy is between Xsens and the robot

Human and G1 proportions, joint limits and balance dynamics differ. Sending retargeted
angles directly to motors would not guarantee balance, contact consistency or safe
torques. The tracking policy receives the desired future motion plus the robot's current
state and produces joint targets, stiffness and damping. Corrective foot placement is
allowed so that matching the upper body does not take priority over staying upright.

## Work phases

### Phase 1 — Capture and parsing: complete

- Decode recorded XUDP/MXTP02.
- Export segment poses and resample to 50 Hz.
- Verify UDP replay and stream health.

### Phase 2 — Offline retargeting: complete for the sample

- Map Xsens segments to all 29 G1 joints.
- Correct coordinate axes, signs, neutral offsets, elbows, hips and shoulders.
- Visually validate lean → N-pose → T-pose → arms-forward sequence.
- Export the validated G1 reference.

### Phase 3 — General tracking policy: complete as separate components

- Install the official ProtoMotions G1 tracker.
- Run its pretrained ONNX policy in MuJoCo.
- Convert and play the custom Xsens reference.
- Confirm stable root height and fast ONNX inference.

### Phase 4 — Safety state machine: complete as separate components

- Detect stale input.
- Briefly hold the last valid reference.
- Smoothly return to a safe pose.
- Require sustained fresh input.
- Blend back into live tracking.
- Preserve a separate, latched hard ESTOP.
- Validate the behavior through ONNX and MuJoCo.

### Phase 5 — Integrated live simulation: current phase

Build one process that connects:

```text
live UDP → startup calibration → framewise retargeting → 50 Hz scheduling →
watchdog/prediction → ONNX tracker → MuJoCo
```

Verification order:

1. Recorded XUDP replay into the integrated process.
2. Intentional UDP dropouts and recovery.
3. Live Windows Xsens MVN into WSL2.
4. N-pose and T-pose startup calibration.
5. Slow arm, squat and stepping trials.
6. Faster whole-body motion in simulation.

### Phase 6 — Jetson integration: not started

- Reproduce the ONNX runtime on the Jetson.
- Read actual G1 joint, IMU and base state.
- Verify joint ordering, units and coordinate frames.
- Add a command adapter behind a disabled-by-default hardware gate.
- Measure end-to-end latency and control-loop timing.

### Phase 7 — Physical robot commissioning: not authorized

Required before dynamic motion:

- Robot suspended or supported for initial low-gain tests.
- Independent physical emergency stop and trained spotters.
- Joint-position, velocity, torque and workspace limits.
- Fall detection, network timeout and operator-enable control.
- Start with standing and arms only.
- Progress through shallow squat and slow stepping.
- Lunges, dancing and jumps only after progressively harder simulation and supported
  hardware validation. A pretrained tracker is not proof that jumping is safe on this
  specific robot.

## Current handoff point

The project is between Phases 4 and 5. All major components work independently, but the
single live simulation process does not exist yet. The immediate task for the next
engineer is integration—not policy training and not robot command output.

The existing recording and general tracker are sufficient to build that integration.
Training a specialized policy may be useful later for motions the general tracker cannot
perform robustly, but it is not the next step.

## Success criteria for the current phase

- Live packets received continuously with no malformed frames.
- Output scheduled at 50 Hz with bounded queueing and no accumulating latency.
- Saved calibration produces the same directions and neutral pose as the validated
  recording.
- Stream interruption follows `HOLD → SAFE_RETURN → RECOVERING → LIVE`.
- Simulation remains upright during slow whole-body movements.
- Logs include packet age, dropped frames, state-machine state, ONNX latency, joint
  errors, root height and limit violations.
- There is no code path capable of contacting the physical G1.

