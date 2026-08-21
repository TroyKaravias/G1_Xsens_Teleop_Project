#!/usr/bin/env python3
"""Run a recorded Xsens reference through X2 SONIC in free-base MuJoCo."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xsens_bridge.g1_retarget import detect_t_pose_frame, load_segment_positions, load_xsens_csv
from xsens_bridge.x2_retarget import (
    X2_JOINT_NAMES, clamp_x2_trajectory, lowpass_x2_trajectory, model_xml,
    retarget_xsens_to_x2,
)
from xsens_bridge.x2_sonic import (
    IL_TO_MJ_DOF, MJ_TO_IL_DOF, X2SonicOnnxPolicy,
    X2SonicProprioceptionBuffer, build_recorded_tokenizer_observation,
)

CONTROL_DT = 0.02
SIM_DT = 0.005
DECIMATION = 4


def quat_rotate_inverse(q, v):
    w, x, y, z = q
    u = np.asarray([x, y, z])
    t = 2.0 * np.cross(u, v)
    return v - w * t + np.cross(u, t)


def policy_control_constants():
    natural_frequency = 20.0 * math.pi
    damping_ratio = 2.0
    armatures = {
        "hip": .025101925, "knee": .025101925, "waist_yaw": .010177520,
        "waist_pitch": .003609725, "waist_roll": .003609725, "ankle": .003609725,
        "shoulder": .003609725, "elbow": .003609725, "wrist_yaw": .003609725,
        "wrist_pitch": .00425, "wrist_roll": .00425, "head": .00425,
    }
    efforts = {
        "hip_yaw": 120., "hip_roll": 120., "hip_pitch": 120., "knee": 120.,
        "ankle_pitch": 36., "ankle_roll": 24., "waist_yaw": 120.,
        "waist_pitch": 48., "waist_roll": 48., "shoulder_pitch": 36.,
        "shoulder_roll": 36., "shoulder_yaw": 24., "elbow": 24.,
        "wrist_yaw": 24., "wrist_pitch": 4.8, "wrist_roll": 4.8,
        "head_yaw": 2.6, "head_pitch": .6,
    }
    kp = np.zeros(31); kd = np.zeros(31); scale = np.ones(31); default = np.zeros(31)
    for i, name in enumerate(X2_JOINT_NAMES):
        for key, armature in armatures.items():
            if key in name:
                kp[i] = armature * natural_frequency**2
                kd[i] = 2 * damping_ratio * armature * natural_frequency
                break
        for key, effort in efforts.items():
            if key in name:
                scale[i] = .25 * effort / kp[i]
                break
        if "hip_pitch" in name: default[i] = -.312
        elif "knee" in name: default[i] = .669
        elif "ankle_pitch" in name: default[i] = -.363
        elif "elbow" in name: default[i] = -.6
        elif name == "left_shoulder_roll_joint": default[i] = .2
        elif name == "right_shoulder_roll_joint": default[i] = -.2
        elif "shoulder_pitch" in name: default[i] = .2
    return kp, kd, scale, default


def prepend_policy_startup(trajectory, default, frame_dt, hold_s=2.0, blend_s=1.0):
    """Start inside the policy reset distribution before entering Xsens motion."""
    hold_frames = max(1, int(round(hold_s / frame_dt)))
    blend_frames = max(1, int(round(blend_s / frame_dt)))
    hold = np.broadcast_to(default, (hold_frames, len(default))).copy()
    fractions = np.linspace(0.0, 1.0, blend_frames, endpoint=False)[:, None]
    blend = default[None, :] + fractions * (trajectory[0][None, :] - default[None, :])
    return np.concatenate((hold, blend, trajectory), axis=0)


def load_model_with_floor(mujoco, path):
    """Load the vendor MJCF with a physical plane, without editing the asset."""
    xml = Path(path).read_text()
    mesh_dir = (Path(path).parent / "meshes").resolve()
    xml = xml.replace('meshdir="./meshes"', f'meshdir="{mesh_dir}"')
    marker = "<worldbody>"
    if marker not in xml:
        raise RuntimeError("X2 MJCF has no worldbody")
    floor = '<geom name="sonic_floor" type="plane" size="0 0 0.05" friction="1 0.005 0.0001" rgba="0.72 0.72 0.72 1"/>'
    xml = xml.replace(marker, marker + floor, 1)
    return mujoco.MjModel.from_xml_string(xml)


def place_x2_feet_on_floor(mujoco, model, data, clearance=0.001):
    """Set free-base height so the X2 foot collision spheres contact the plane.

    The vendor MJCF's free-joint default leaves the crouched SONIC reset pose
    about four centimetres above the floor.  Dropping from that pose injects a
    large unmodelled startup transient.  RSI normally supplies the matching
    root height from the reference motion; live Xsens has no robot root state,
    so reconstruct the missing height from the actual collision geometry.
    """
    mujoco.mj_forward(model, data)
    foot_geoms = [
        index for index in range(model.ngeom)
        if model.geom(index).type == mujoco.mjtGeom.mjGEOM_SPHERE
        and "ankle_roll_link" in model.body(model.geom_bodyid[index]).name
    ]
    if not foot_geoms:
        raise RuntimeError("X2 model has no foot collision spheres")
    lowest = min(float(data.geom_xpos[index, 2] - model.geom_size[index, 0])
                 for index in foot_geoms)
    data.qpos[2] += float(clearance) - lowest
    mujoco.mj_forward(model, data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--variant", choices=("v1.3",), default="v1.3")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0: parser.error("--duration must be positive")

    import mujoco
    times, quaternions = load_xsens_csv(args.csv)
    positions = load_segment_positions(args.csv, (
        "left_upper_arm", "left_forearm", "left_hand",
        "right_upper_arm", "right_forearm", "right_hand",
    ))
    frame_dt = float(np.median(np.diff(times)))
    calibration_frame = detect_t_pose_frame(args.csv)
    trajectory = retarget_xsens_to_x2(
        quaternions, calibration_frame=calibration_frame, positions=positions,
    )
    trajectory = lowpass_x2_trajectory(trajectory, frame_dt, 4.0)

    model_path = model_xml(args.models, args.variant)
    model = load_model_with_floor(mujoco, model_path); model.opt.timestep = SIM_DT
    data = mujoco.MjData(model)
    joint_ids = np.asarray([model.joint(name).id for name in X2_JOINT_NAMES])
    qpos_adr = model.jnt_qposadr[joint_ids]; dof_adr = model.jnt_dofadr[joint_ids]
    actuator_ids = np.asarray([model.actuator(f"motor_{name}").id for name in X2_JOINT_NAMES])
    policy = X2SonicOnnxPolicy(args.policy)
    history = X2SonicProprioceptionBuffer()
    kp, kd, action_scale, default = policy_control_constants()
    # Direct-retarget coordinates are centered on the visual T-pose. SONIC was
    # trained around its crouched X2 reset pose, so retain calibrated motion
    # deltas while changing the absolute reference center.
    # Calibration frames establish the reference origin; they are not motion
    # commands and must not be replayed to the policy after initialization.
    trajectory = default + (
        trajectory[calibration_frame:] - trajectory[calibration_frame]
    )
    trajectory, clamp_count = clamp_x2_trajectory(trajectory, model.jnt_range[joint_ids])
    trajectory = prepend_policy_startup(trajectory, default, frame_dt)

    mujoco.mj_resetData(model, data)
    data.qpos[qpos_adr] = default
    place_x2_feet_on_floor(mujoco, model, data)
    last_action_mj = np.zeros(31, dtype=np.float32)
    falls = 0; saturation_events = 0; steps = 0
    minimum_height = float(data.qpos[2]); maximum_tilt = 0.0

    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    try:
        while data.time < args.duration:
            started = time.monotonic()
            qpos_mj = data.qpos[qpos_adr].copy(); qvel_mj = data.qvel[dof_adr].copy()
            base_quat = data.qpos[3:7].copy()
            gravity = quat_rotate_inverse(base_quat, np.asarray([0., 0., -1.]))
            history.append(
                data.qvel[3:6], qpos_mj[IL_TO_MJ_DOF] - default[IL_TO_MJ_DOF],
                qvel_mj[IL_TO_MJ_DOF], last_action_mj[IL_TO_MJ_DOF], gravity,
            )
            tokenizer = build_recorded_tokenizer_observation(trajectory, frame_dt, data.time)
            action_il = policy.infer(tokenizer, history.flattened())
            action_mj = action_il[MJ_TO_IL_DOF]
            target = default + action_mj * action_scale
            for _ in range(DECIMATION):
                raw_torque = kp * (target - data.qpos[qpos_adr]) - kd * data.qvel[dof_adr]
                limited = np.clip(raw_torque, model.actuator_ctrlrange[actuator_ids, 0],
                                  model.actuator_ctrlrange[actuator_ids, 1])
                saturation_events += int(np.count_nonzero(raw_torque != limited))
                data.ctrl[:] = 0.; data.ctrl[actuator_ids] = limited
                mujoco.mj_step(model, data)
            last_action_mj = action_mj
            steps += 1
            minimum_height = min(minimum_height, float(data.qpos[2]))
            tilt = math.degrees(math.acos(float(np.clip(-gravity[2], -1., 1.))))
            maximum_tilt = max(maximum_tilt, tilt)
            if data.qpos[2] < .4 or gravity[2] > -.3:
                falls += 1
                break
            if not np.isfinite(data.qpos).all(): raise RuntimeError("non-finite MuJoCo state")
            if viewer is not None:
                if not viewer.is_running(): break
                viewer.sync()
            if args.realtime: time.sleep(max(0., CONTROL_DT - (time.monotonic() - started)))
    finally:
        if viewer is not None: viewer.close()

    print(f"Mode: X2 SONIC ONNX, FREE-BASE MUJOCO")
    print(f"Policy: {args.policy}")
    print(f"Model: {model_path}")
    print(f"Simulated: {data.time:.3f}s ({steps} policy steps at 50 Hz)")
    print(f"Reference clamps: {clamp_count}/{trajectory.size}")
    print(f"Falls: {falls}; minimum pelvis height: {minimum_height:.3f}m; maximum tilt: {maximum_tilt:.1f}deg")
    print(f"Torque saturation events: {saturation_events}")
    print("SIMULATION ONLY: this is not hardware validation.")


if __name__ == "__main__":
    main()
