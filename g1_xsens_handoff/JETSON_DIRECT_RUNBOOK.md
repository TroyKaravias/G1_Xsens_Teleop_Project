# Direct Jetson Deployment (No Mac or WSL Tunnel)

> The raw-pose commands and keyboard controls below document the earlier
> restrained milestone path. The launcher now defaults to the experimental
> global-pelvis SONIC planner mode. Read `GLOBAL_PELVIS_LOCOMOTION.md` for the
> current two-terminal commands and commissioning sequence. Use
> `TELEOP_MODE=raw_pose` together with `SONIC_INPUT_TYPE=zmq` only when
> deliberately reproducing this older raw-pose path.

This is the preferred low-latency deployment after the restrained milestone
test succeeded.

## Architecture

```text
Xsens MVN on Windows
  -> MXTP02 UDP to 192.168.30.172:9763
  -> Xsens retargeter on G1 Jetson
  -> latest-only ZMQ on 127.0.0.1:5556
  -> SONIC controller on G1 Jetson
  -> Unitree DDS on enP8p1s0
```

The Mac and WSL are not part of the real-time data path. They remain optional
administration and development machines.

## One-time installation on the Jetson

Copy the milestone ZIP to `/home/unitree`, unpack it in a new directory, and
verify dependencies:

```bash
mkdir -p ~/g1_xsens_direct
cd ~/g1_xsens_direct
unzip ~/G1_Xsens_Milestone_2026-07-29.zip
cd g1_xsens_handoff

python3 -c "import numpy, zmq; print(numpy.__version__, zmq.__version__)"
chmod +x scripts/run_jetson_publisher.sh scripts/run_jetson_controller.sh
python3 -m unittest discover -s tests -v
```

If `numpy` or `zmq` is unavailable, create an isolated environment rather than
altering the vendor Python installation:

```bash
cd ~/g1_xsens_direct/g1_xsens_handoff
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install numpy pyzmq
PYTHON_BIN="$PWD/.venv/bin/python" scripts/run_jetson_publisher.sh
```

## Configure Xsens MVN

Set the MVN network streamer to:

- Protocol: MXTP02
- Data: Position + Orientation (Quaternion)
- Destination: `192.168.30.172`
- UDP port: `9763`

Do not send to the old WSL address or `192.168.30.199`.

## No-suit preflight using the recorded XUDP

Before wearing the suit, exercise the exact Jetson-local UDP, retargeting, and
ZMQ path using the included recording:

```bash
cd ~/g1_xsens_direct/g1_xsens_handoff
PYTHON_BIN="${PYTHON_BIN:-python3}" ./scripts/preflight_jetson_replay.sh
```

This does not start SONIC or create robot motor output. It must finish with:

```text
PASS: latest-only SONIC stream is healthy
PASS: direct Jetson replay preflight completed without robot output
```

## Terminal 1: publisher on the Jetson

```bash
cd ~/g1_xsens_direct/g1_xsens_handoff
./scripts/run_jetson_publisher.sh
```

If using the isolated environment:

```bash
cd ~/g1_xsens_direct/g1_xsens_handoff
PYTHON_BIN="$PWD/.venv/bin/python" ./scripts/run_jetson_publisher.sh
```

Calibrate N-pose to T-pose and proceed only when the publisher reports
`Stream state: LIVE`. Healthy status has `dropped=0`, low packet age, and zero
malformed packets.

### Milestone-equivalent publisher without tunnels

To reproduce the known-working 2026-07-29 tunnel publisher behavior while
keeping the complete real-time path on the Jetson, use:

```bash
cd ~/g1_xsens_direct/g1_xsens_handoff
PYTHON_BIN="$HOME/.venvs/g1_xsens/bin/python" \
  bash scripts/run_jetson_publisher_milestone.sh
```

This preserves the milestone's raw 29-joint reference, 12-second N-to-T
calibration, 50 Hz scheduling, and one-frame SONIC protocol-v1 messages. The
only transport change is that MVN sends MXTP02 directly to the Jetson on UDP
9763 and SONIC consumes ZMQ on Jetson loopback port 5556. No SSH forwarding is
used.

The milestone's restrained physical validation covered neutral engagement and
small arm tracking. Matching its software path does not extend that validation
to unsupported lower-body motion, walking, punches, or dynamic actions.

### Latest tunnel teleop profile, adapted to Jetson loopback

`scripts/run_jetson_tunnel_profile.sh` preserves the options from
`run_existing_wsl_publisher.sh`: full-body leg blending, relative waist,
root-heading, lifted-foot heading and hip-roll refinements, and wrist tracking.
It replaces only the WSL/tunnel topology with direct MVN UDP to the Jetson and
Jetson-local ZMQ:

```bash
bash ~/run_jetson_tunnel_profile.sh
```

This is a higher-authority experimental physical profile. Its presence does
not expand the restrained milestone's validated motion envelope. Commission
neutral and individual capabilities under support before combined motion.

### Current full-body path with earlier proven arms

`scripts/run_jetson_current_proven_arms.sh` keeps the current Jetson-local
full-body, relative-waist, root-heading, and lifted-foot behavior while using
the earlier restrained arm profile: 4 rad/s default arm slew and neutral wrist
references. It deliberately omits wrist roll/pitch tracking from the latest
tunnel profile.

```bash
bash ~/run_jetson_current_proven_arms.sh
```

## Terminal 2: SONIC controller on the Jetson

```bash
cd ~/g1_xsens_direct/g1_xsens_handoff
./scripts/run_jetson_controller.sh
```

Controls:

- `]`: engage neutral control
- `Enter`: toggle live Xsens reference
- `O`: stop controller and enter damping shutdown

## Latency controls included

- One reference frame per 50 Hz message.
- ZMQ publisher high-water mark of one.
- Nonblocking publisher send: stale poses are dropped instead of queued.
- SONIC `--zmq-conflate`: consume the newest available reference.
- A 4 rad/s joint-position slew limit prevents a held arm target or resumed
  stream from snapping directly to the newest pose.
- The slew limiter caps outage catch-up time at 40 ms, so a long stall cannot
  authorize one oversized position jump.
- Publisher and controller communicate over Jetson loopback.
- WSL NAT, Windows TCP forwarding, Mac relays, and SSH tunnel buffers removed.

## Safety

- Always validate mapping changes in MuJoCo.
- Use the support harness and a dedicated physical e-stop operator.
- Match the human pose to the robot before enabling live ZMQ.
- Start with neutral engagement and one small arm motion.
- Stop if `dropped` rises continuously, packet age increases, or physical
  response falls behind.
- Dynamic motion and unsupported walking remain outside this milestone.
