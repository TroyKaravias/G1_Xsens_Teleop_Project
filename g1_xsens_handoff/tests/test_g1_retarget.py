import unittest

import numpy as np

from xsens_bridge.g1_retarget import (
    G1_DEFAULT_POSE,
    G1_T_POSE,
    clamp_to_model_limits,
    retarget_quaternions,
)


class G1RetargetTests(unittest.TestCase):
    def test_calibration_frame_maps_to_default_pose(self):
        identity = np.array([[1.0, 0.0, 0.0, 0.0]])
        segments = {
            name: identity.copy()
            for name in (
                "pelvis", "t8",
                "left_upper_leg", "left_lower_leg", "left_foot",
                "right_upper_leg", "right_lower_leg", "right_foot",
                "left_upper_arm", "left_forearm",
                "right_upper_arm", "right_forearm",
            )
        }
        result = retarget_quaternions(segments)
        np.testing.assert_allclose(result[0], G1_DEFAULT_POSE)

    def test_clamps_to_limits(self):
        trajectory = np.array([[-2.0, 0.0, 2.0]])
        ranges = np.array([[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]])
        result, count = clamp_to_model_limits(trajectory, ranges, margin_rad=0.1)
        np.testing.assert_allclose(result, [[-0.9, 0.0, 0.9]])
        self.assertEqual(count, 2)

    def test_selected_calibration_frame_maps_to_selected_base_pose(self):
        identity = np.array(
            [[1.0, 0.0, 0.0, 0.0], [0.98, 0.0, 0.0, 0.199]]
        )
        segments = {
            name: identity.copy()
            for name in (
                "pelvis", "t8",
                "left_upper_leg", "left_lower_leg", "left_foot",
                "right_upper_leg", "right_lower_leg", "right_foot",
                "left_upper_arm", "left_forearm",
                "right_upper_arm", "right_forearm",
            )
        }
        result = retarget_quaternions(
            segments,
            calibration_frame=1,
            base_pose=G1_T_POSE,
        )
        np.testing.assert_allclose(result[1], G1_T_POSE, atol=1e-7)


if __name__ == "__main__":
    unittest.main()
