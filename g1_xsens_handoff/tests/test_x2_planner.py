import numpy as np
import pytest

from xsens_bridge.x2_planner import clamp_planner_joints, pack_mujoco_qpos, unpack_mujoco_qpos


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
