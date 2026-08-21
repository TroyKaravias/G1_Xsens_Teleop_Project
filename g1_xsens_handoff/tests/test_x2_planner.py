import numpy as np
import pytest

from xsens_bridge.x2_planner import (
    calibrated_planar_pelvis, clamp_planner_joints, pack_mujoco_qpos,
    pack_velocity_intent, unpack_mujoco_qpos,
)


def test_planner_qpos_round_trip_converts_quaternion_order():
    root = np.arange(12, dtype=np.float32).reshape(4, 3)
    quaternion = np.tile([1, 2, 3, 4], (4, 1)).astype(np.float32)
    joints = np.arange(124, dtype=np.float32).reshape(4, 31)
    qpos = pack_mujoco_qpos(root, quaternion, joints)
    assert qpos.shape == (4, 38)
    np.testing.assert_array_equal(qpos[:, 3:7], np.tile([4, 1, 2, 3], (4, 1)))
    unpacked = unpack_mujoco_qpos(qpos)
    np.testing.assert_array_equal(unpacked[0], root)
    np.testing.assert_array_equal(unpacked[1], quaternion)
    np.testing.assert_array_equal(unpacked[2], joints)


def test_planner_qpos_rejects_wrong_joint_width():
    with pytest.raises(ValueError, match="31 X2 joints"):
        pack_mujoco_qpos(np.zeros((4, 3)), np.zeros((4, 4)), np.zeros((4, 30)))


def test_clamp_planner_joints_preserves_root_and_limits_joints():
    qpos = np.zeros((2, 38), dtype=np.float32)
    qpos[:, :7] = np.arange(7)
    qpos[:, 7:] = 2.0
    clamped, count = clamp_planner_joints(qpos, np.tile([-1.0, 1.0], (31, 1)))
    np.testing.assert_array_equal(clamped[:, :7], qpos[:, :7])
    np.testing.assert_array_equal(clamped[:, 7:], 1.0)
    assert count == 62


def test_calibrated_planar_pelvis_rotates_world_motion_and_unwraps_yaw():
    half = np.sqrt(0.5)
    positions = np.asarray([[2, 3, 1], [2, 4, 1], [1, 4, 1]], dtype=float)
    quaternions = np.asarray([
        [half, 0, 0, half],
        [np.cos(3 * np.pi / 8), 0, 0, np.sin(3 * np.pi / 8)],
        [np.cos(5 * np.pi / 8), 0, 0, np.sin(5 * np.pi / 8)],
    ])
    xy, heading = calibrated_planar_pelvis(positions, quaternions)
    np.testing.assert_allclose(xy, [[0, 0], [1, 0], [1, 1]], atol=1e-6)
    np.testing.assert_allclose(heading, [0, np.pi / 4, 3 * np.pi / 4], atol=1e-6)


def test_pack_velocity_intent_uses_verified_field_order_and_limits():
    intent = pack_velocity_intent(
        [3, 4], 2.0, maximum_speed_mps=1.0, maximum_yaw_rate_rad_s=0.5,
    )
    np.testing.assert_allclose(intent, [0.5, 0.6, 0.8, 0.0], atol=1e-6)
