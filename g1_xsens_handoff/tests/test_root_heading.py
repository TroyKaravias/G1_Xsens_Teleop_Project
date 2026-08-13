from __future__ import annotations

import math
import unittest

import numpy as np

from xsens_bridge.g1_retarget import G1_DEFAULT_POSE
from tools.live_xsens_sonic import (
    bounded_root_heading,
    clamp_physical_leg_angles,
    relative_pelvis_yaw,
    unwrap_yaw,
    wrap_full_turn,
    yaw_quaternion,
)


class RootHeadingTests(unittest.TestCase):
    def test_physical_hip_and_foot_limits_lock_outward_velocity(self) -> None:
        position = np.zeros(29, dtype=np.float32)
        position[:] = 0.0
        position[0] = 1.0
        position[1] = -1.0
        position[4] = 1.0
        position[10] = -1.0
        position[15] = 1.0
        velocity = np.zeros(29, dtype=np.float32)
        velocity[[0, 4]] = 2.0
        velocity[[1, 10]] = -2.0

        limited_pos, limited_vel = clamp_physical_leg_angles(
            position,
            velocity,
            hip_limit_rad=0.20,
            foot_limit_rad=0.10,
        )

        self.assertAlmostEqual(limited_pos[0], G1_DEFAULT_POSE[0] + 0.20)
        self.assertAlmostEqual(limited_pos[1], G1_DEFAULT_POSE[1] - 0.20)
        self.assertAlmostEqual(limited_pos[4], G1_DEFAULT_POSE[4] + 0.10)
        self.assertAlmostEqual(limited_pos[10], G1_DEFAULT_POSE[10] - 0.10)
        np.testing.assert_array_equal(limited_vel[[0, 1, 4, 10]], 0.0)
        self.assertEqual(limited_pos[15], 1.0)

    def test_relative_pelvis_yaw(self) -> None:
        measured = relative_pelvis_yaw(
            yaw_quaternion(math.radians(20.0)),
            yaw_quaternion(math.radians(80.0)),
        )
        self.assertAlmostEqual(measured, math.radians(60.0), places=5)

    def test_heading_is_deadbanded_scaled_bounded_and_slew_limited(self) -> None:
        kwargs = dict(
            gain=0.20,
            deadband_rad=math.radians(5.0),
            limit_rad=math.radians(15.0),
            max_rate_rad_s=math.radians(10.0),
        )
        self.assertEqual(bounded_root_heading(0.01, 0.0, 0.02, **kwargs), 0.0)
        first = bounded_root_heading(math.pi, 0.0, 0.02, **kwargs)
        self.assertAlmostEqual(first, math.radians(0.2), places=6)
        value = 0.0
        for _ in range(200):
            value = bounded_root_heading(math.pi, value, 0.02, **kwargs)
        self.assertLessEqual(value, math.radians(15.0) + 1e-9)

    def test_yaw_quaternion_is_normalized(self) -> None:
        self.assertAlmostEqual(float(np.linalg.norm(yaw_quaternion(1.0))), 1.0)

    def test_full_positive_turn_wraps_to_zero_and_keeps_direction(self) -> None:
        kwargs = dict(
            gain=1.0,
            deadband_rad=0.0,
            limit_rad=math.radians(360.0),
            max_rate_rad_s=math.radians(1000.0),
        )
        at_reset = bounded_root_heading(
            math.radians(360.0), math.radians(359.0), 0.02, **kwargs
        )
        after_reset = bounded_root_heading(
            math.radians(361.0), at_reset, 0.02, **kwargs
        )

        self.assertEqual(at_reset, 0.0)
        self.assertAlmostEqual(math.degrees(after_reset), 1.0, places=6)

    def test_full_negative_turn_wraps_to_zero_and_keeps_direction(self) -> None:
        kwargs = dict(
            gain=1.0,
            deadband_rad=0.0,
            limit_rad=math.radians(360.0),
            max_rate_rad_s=math.radians(1000.0),
        )
        at_reset = bounded_root_heading(
            math.radians(-360.0), math.radians(-359.0), 0.02, **kwargs
        )
        after_reset = bounded_root_heading(
            math.radians(-361.0), at_reset, 0.02, **kwargs
        )

        self.assertEqual(at_reset, 0.0)
        self.assertAlmostEqual(math.degrees(after_reset), -1.0, places=6)

    def test_full_turn_wrap_preserves_overshoot_and_quaternion_sign(self) -> None:
        self.assertAlmostEqual(
            math.degrees(wrap_full_turn(math.radians(721.5))),
            1.5,
            places=6,
        )
        self.assertAlmostEqual(
            math.degrees(wrap_full_turn(math.radians(-721.5))),
            -1.5,
            places=6,
        )
        before = yaw_quaternion(math.radians(359.0))
        reset = yaw_quaternion(0.0, before)
        self.assertGreater(float(np.dot(before, reset)), 0.99)

    def test_yaw_unwrap_preserves_direction_across_pi(self) -> None:
        previous_wrapped = None
        unwrapped = 0.0
        results = []
        for degrees in (0.0, 90.0, 170.0, -170.0, -90.0, 0.0):
            previous_wrapped, unwrapped = unwrap_yaw(
                math.radians(degrees), previous_wrapped, unwrapped
            )
            results.append(math.degrees(unwrapped))
        np.testing.assert_allclose(
            results, (0.0, 90.0, 170.0, 190.0, 270.0, 360.0), atol=1e-6
        )

    def test_yaw_unwrap_preserves_negative_direction(self) -> None:
        previous_wrapped = None
        unwrapped = 0.0
        for degrees in (0.0, -90.0, -170.0, 170.0):
            previous_wrapped, unwrapped = unwrap_yaw(
                math.radians(degrees), previous_wrapped, unwrapped
            )
        self.assertAlmostEqual(math.degrees(unwrapped), -190.0)

if __name__ == "__main__":
    unittest.main()
