from __future__ import annotations

import math
import unittest

import numpy as np

from xsens_bridge.global_pelvis import GlobalPelvisFollower, GlobalPelvisLimits


def yaw_quaternion(angle: float) -> np.ndarray:
    return np.asarray((math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)))


class GlobalPelvisFollowerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.follower = GlobalPelvisFollower(GlobalPelvisLimits(
            engage_distance_m=0.08,
            release_distance_m=0.04,
            position_gain_s=1.2,
            minimum_speed_mps=0.20,
            maximum_speed_mps=0.20,
            maximum_sample_jump_m=0.20,
            maximum_excursion_m=1.0,
            heading_max_rate_rad_s=0.5,
        ))
        self.follower.arm(np.zeros(3), yaw_quaternion(0.0), 1.0)

    def test_forward_backward_and_lateral_displacement(self) -> None:
        forward = self.follower.update(
            np.asarray((0.10, 0.0, 0.0)), yaw_quaternion(0.0), 1.02
        )
        self.assertAlmostEqual(forward.velocity_x_mps, 0.20)
        self.assertAlmostEqual(forward.velocity_y_mps, 0.0)
        self.assertEqual(forward.label, "pelvis-forward")

        self.follower.arm(np.zeros(3), yaw_quaternion(0.0), 2.0)
        backward = self.follower.update(
            np.asarray((-0.10, 0.0, 0.0)), yaw_quaternion(0.0), 2.02
        )
        self.assertAlmostEqual(backward.velocity_x_mps, -0.20)

        self.follower.arm(np.zeros(3), yaw_quaternion(0.0), 3.0)
        left = self.follower.update(
            np.asarray((0.0, 0.10, 0.0)), yaw_quaternion(0.0), 3.02
        )
        self.assertAlmostEqual(left.velocity_y_mps, 0.20)
        self.assertEqual(left.label, "pelvis-left")

    def test_translation_is_relative_to_calibrated_global_heading(self) -> None:
        self.follower.arm(
            np.zeros(3), yaw_quaternion(math.pi / 2.0), 4.0
        )
        command = self.follower.update(
            np.asarray((0.0, 0.10, 0.0)),
            yaw_quaternion(math.pi / 2.0),
            4.02,
        )
        self.assertAlmostEqual(command.velocity_x_mps, 0.20)
        self.assertAlmostEqual(command.velocity_y_mps, 0.0, places=6)

    def test_holding_displacement_stops_after_estimated_catchup(self) -> None:
        position = np.asarray((0.10, 0.0, 0.0))
        command = self.follower.update(position, yaw_quaternion(0.0), 1.02)
        timestamp = 1.02
        for _ in range(40):
            timestamp += 0.02
            command = self.follower.update(
                position, yaw_quaternion(0.0), timestamp
            )
        self.assertFalse(command.active)
        self.assertEqual(command.velocity_x_mps, 0.0)

    def test_deadband_prevents_drift(self) -> None:
        command = self.follower.update(
            np.asarray((0.03, 0.02, 0.0)), yaw_quaternion(0.0), 1.02
        )
        self.assertFalse(command.active)
        self.assertEqual(command.velocity_x_mps, 0.0)
        self.assertEqual(command.velocity_y_mps, 0.0)

    def test_heading_is_calibrated_and_rate_limited(self) -> None:
        command = self.follower.update(
            np.zeros(3), yaw_quaternion(math.radians(30.0)), 1.02
        )
        self.assertAlmostEqual(command.heading_rad, 0.01)
        self.assertFalse(command.active)

    def test_position_jump_latches_fault_and_requires_rearm(self) -> None:
        command = self.follower.update(
            np.asarray((0.30, 0.0, 0.0)), yaw_quaternion(0.0), 1.02
        )
        self.assertTrue(command.faulted)
        self.assertEqual(command.velocity_x_mps, 0.0)
        held = self.follower.update(
            np.asarray((0.31, 0.0, 0.0)), yaw_quaternion(0.0), 1.04
        )
        self.assertTrue(held.faulted)

    def test_invalid_speed_ceiling_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GlobalPelvisLimits(maximum_speed_mps=0.36)


if __name__ == "__main__":
    unittest.main()
