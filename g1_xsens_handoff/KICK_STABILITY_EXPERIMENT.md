# Experimental kick stability sidecar

This is an opt-in reference-shaping experiment. It does not replace or edit
the proven publisher or SONIC controller.

## Paths

Proven path:

```text
Xsens UDP -> live_xsens_sonic.py -> tcp://127.0.0.1:5556 -> SONIC
```

Experimental path:

```text
Xsens UDP -> live_xsens_sonic.py -> tcp://127.0.0.1:5567
          -> kick_stability_sidecar.py -> tcp://127.0.0.1:5556 -> SONIC
```

The sidecar detects a likely left/right swing leg from hip and knee reference
motion. During single support it:

- anchors the likely stance leg's hip roll/yaw and ankle pitch/roll toward the
  last double-support reference;
- bounds swing-leg hip roll/yaw and ankle pitch/roll;
- reduces and bounds waist roll/pitch;
- slew-limits lower-body transitions;
- pauses output and resets on stale input.

It does **not** yet use robot IMU or foot contact. Therefore it is not push,
fall, or get-up recovery and must not be represented as such.

## Rollback

Stop the experimental launcher with `Ctrl+C`. The original path is unchanged:

```bash
bash ~/run_jetson_current_proven_arms.sh
```

## Commissioning order

1. Replay Xsens recordings with no controller.
2. Confirm 50 Hz output, phase transitions, joint bounds, and no index gaps.
3. Run publisher/sidecar with live Xsens but no SONIC controller.
4. Only then conduct a restrained gantry test with an E-stop operator.
