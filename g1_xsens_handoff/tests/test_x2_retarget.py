import unittest

import numpy as np

from xsens_bridge.g1_retarget import G1_JOINT_NAMES, G1_T_POSE
from xsens_bridge.x2_retarget import (
    X2_JOINT_NAMES,
    X2_T_POSE,
    clamp_x2_trajectory,
    g1_deltas_to_x2,
    validate_model_joint_names,
)


class X2RetargetTests(unittest.TestCase):
    def test_calibration_pose_maps_to_x2_t_pose(self):
        result = g1_deltas_to_x2(G1_T_POSE[None])
        np.testing.assert_allclose(result[0], X2_T_POSE)

    def test_joint_layout_has_31_unique_entries(self):
        self.assertEqual(len(X2_JOINT_NAMES), 31)
        self.assertEqual(len(set(X2_JOINT_NAMES)), 31)

    def test_semantic_mapping_reorders_waist_and_wrists(self):
        source = np.broadcast_to(G1_T_POSE, (1, len(G1_JOINT_NAMES))).copy()
        probes = {
            "waist_roll_joint": 0.12,
            "waist_pitch_joint": -0.08,
            "left_wrist_roll_joint": 0.21,
            "left_wrist_yaw_joint": -0.19,
        }
        for name, delta in probes.items():
            source[0, G1_JOINT_NAMES.index(name)] += delta
        result = g1_deltas_to_x2(source)[0]
        for name, delta in probes.items():
            self.assertAlmostEqual(result[X2_JOINT_NAMES.index(name)], delta)

    def test_elbow_flexion_sign_is_inverted(self):
        source = np.broadcast_to(G1_T_POSE, (1, len(G1_JOINT_NAMES))).copy()
        source[0, G1_JOINT_NAMES.index("left_elbow_joint")] += 0.4
        result = g1_deltas_to_x2(source)
        self.assertAlmostEqual(
            result[0, X2_JOINT_NAMES.index("left_elbow_joint")], -0.4
        )

    def test_clamp_respects_margin(self):
        values = np.asarray([[-2.0, 0.0, 2.0]])
        ranges = np.asarray([[-1.0, 1.0]] * 3)
        result, count = clamp_x2_trajectory(values, ranges, margin_rad=0.1)
        np.testing.assert_allclose(result, [[-0.9, 0.0, 0.9]])
        self.assertEqual(count, 2)

    def test_model_contract_rejects_missing_and_reordered_joints(self):
        validate_model_joint_names(X2_JOINT_NAMES)
        with self.assertRaises(ValueError):
            validate_model_joint_names(X2_JOINT_NAMES[:-1])
        with self.assertRaises(ValueError):
            validate_model_joint_names(tuple(reversed(X2_JOINT_NAMES)))


if __name__ == "__main__":
    unittest.main()
