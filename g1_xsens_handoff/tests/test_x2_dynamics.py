import unittest

import numpy as np

from xsens_bridge.x2_dynamics import (
    bounded_pd_torque,
    default_x2_pd_gains,
    interpolate_x2_reference,
    X2GlobalRootTracker,
)
from xsens_bridge.x2_retarget import X2_JOINT_NAMES


class X2DynamicsTests(unittest.TestCase):
    def test_default_gains_are_positive_and_ordered(self):
        kp, kd = default_x2_pd_gains()
        self.assertEqual(kp.shape, (len(X2_JOINT_NAMES),))
        self.assertEqual(kd.shape, (len(X2_JOINT_NAMES),))
        self.assertTrue(np.all(kp > 0.0))
        self.assertTrue(np.all(kd > 0.0))

    def test_pd_torque_is_clipped_and_reports_saturation(self):
        size = len(X2_JOINT_NAMES)
        zeros = np.zeros(size)
        target = np.ones(size)
        limits = np.broadcast_to([-2.0, 2.0], (size, 2))
        torque, saturated = bounded_pd_torque(
            zeros, zeros, target, zeros, np.full(size, 10.0), zeros, limits
        )
        np.testing.assert_allclose(torque, 2.0)
        self.assertTrue(np.all(saturated))

    def test_reference_interpolation_and_end_hold(self):
        trajectory = np.zeros((2, len(X2_JOINT_NAMES)))
        trajectory[1] = 1.0
        position, velocity = interpolate_x2_reference(trajectory, 0.02, 0.01)
        np.testing.assert_allclose(position, 0.5)
        np.testing.assert_allclose(velocity, 50.0)
        position, velocity = interpolate_x2_reference(trajectory, 0.02, 1.0)
        np.testing.assert_allclose(position, 1.0)
        np.testing.assert_allclose(velocity, 0.0)

    def test_global_root_tracks_bounded_displacement(self):
        tracker = X2GlobalRootTracker(
            np.asarray([1.0, 0.0, 0.0, 0.0]),
            np.asarray([2.0, -1.0]),
            filter_alpha=1.0,
            deadband_mps=0.0,
        )
        np.testing.assert_allclose(tracker.update(np.zeros(3), 1.0), [2.0, -1.0])
        np.testing.assert_allclose(
            tracker.update(np.asarray([0.05, -0.02, 0.0]), 1.1),
            [2.05, -1.02],
        )

    def test_global_root_rejects_position_jump(self):
        tracker = X2GlobalRootTracker(
            np.asarray([1.0, 0.0, 0.0, 0.0]), np.zeros(2)
        )
        tracker.update(np.zeros(3), 1.0)
        np.testing.assert_allclose(
            tracker.update(np.asarray([2.0, 0.0, 0.0]), 1.1), np.zeros(2)
        )
        self.assertEqual(tracker.rejected_jumps, 1)


if __name__ == "__main__":
    unittest.main()
