from __future__ import annotations

import unittest
import numpy as np

from xsens_bridge.global_position import GlobalPositionController, SquatController


class GlobalPositionTests(unittest.TestCase):
    def controller(self, **kwargs):
        return GlobalPositionController(
            filter_alpha=1.0,
            velocity_deadzone_mps=0.0,
            forward_gain=1.0,
            backward_gain=1.0,
            left_gain=1.0,
            right_gain=1.0,
            **kwargs,
        )

    def test_first_frame_is_only_baseline(self):
        result = self.controller().update(
            np.asarray([4.0, -3.0, 1.0]),
            np.asarray([1.0, 0.0, 0.0, 0.0]),
            10.0,
        )
        self.assertEqual(result.forward_mps, 0.0)
        self.assertEqual(result.lateral_mps, 0.0)

    def test_uses_previous_frame_not_calibration_origin(self):
        controller = self.controller()
        identity = np.asarray([1.0, 0.0, 0.0, 0.0])
        controller.update(np.asarray([10.0, 10.0, 0.0]), identity, 1.0)
        result = controller.update(np.asarray([10.01, 10.0, 0.0]), identity, 1.1)
        self.assertAlmostEqual(result.forward_mps, 0.10)
        held = controller.update(np.asarray([10.01, 10.0, 0.0]), identity, 1.2)
        self.assertAlmostEqual(held.forward_mps, 0.10)
        stopped = controller.update(np.asarray([10.01, 10.0, 0.0]), identity, 1.5)
        self.assertEqual(stopped.forward_mps, 0.0)

    def test_axes_and_speed_limits(self):
        controller = self.controller()
        identity = np.asarray([1.0, 0.0, 0.0, 0.0])
        controller.update(np.zeros(3), identity, 1.0)
        result = controller.update(np.asarray([1.0, -1.0, 0.0]), identity, 1.1)
        self.assertAlmostEqual(result.forward_mps, 0.50)
        self.assertAlmostEqual(result.lateral_mps, -0.45)

    def test_calibration_heading_rotates_world_delta(self):
        half = np.sqrt(0.5)
        controller = GlobalPositionController(
            filter_alpha=1.0,
            velocity_deadzone_mps=0.0,
        )
        facing_left = np.asarray([half, 0.0, 0.0, half])
        controller.update(np.zeros(3), facing_left, 1.0)
        result = controller.update(
            np.asarray([0.0, 0.01, 0.0]), facing_left, 1.1
        )
        self.assertGreater(result.forward_mps, 0.0)
        self.assertAlmostEqual(result.lateral_mps, 0.0, places=6)

    def test_gap_rebaselines_without_jump(self):
        controller = self.controller()
        identity = np.asarray([1.0, 0.0, 0.0, 0.0])
        controller.update(np.zeros(3), identity, 1.0)
        result = controller.update(np.asarray([5.0, 0.0, 0.0]), identity, 2.0)
        self.assertEqual(result.forward_mps, 0.0)

    def test_pelvis_pitch_does_not_mix_vertical_into_forward(self):
        controller = self.controller()
        half = np.sqrt(0.5)
        pitched = np.asarray([half, 0.0, half, 0.0])
        controller.update(np.zeros(3), pitched, 1.0)
        result = controller.update(np.asarray([0.0, 0.0, 0.1]), pitched, 1.1)
        self.assertEqual(result.forward_mps, 0.0)
        self.assertEqual(result.lateral_mps, 0.0)

    def test_short_motion_command_is_held_for_planner_commit(self):
        controller = self.controller(command_hold_s=0.30)
        identity = np.asarray([1.0, 0.0, 0.0, 0.0])
        controller.update(np.zeros(3), identity, 1.0)
        moving = controller.update(np.asarray([-0.02, 0.0, 0.0]), identity, 1.1)
        held = controller.update(np.asarray([-0.02, 0.0, 0.0]), identity, 1.2)
        self.assertLess(moving.forward_mps, 0.0)
        self.assertEqual(held.forward_mps, moving.forward_mps)

    def test_directional_gains_are_independent(self):
        controller = GlobalPositionController(
            filter_alpha=1.0,
            velocity_deadzone_mps=0.0,
            forward_gain=1.0,
            backward_gain=1.5,
            left_gain=1.4,
            right_gain=1.0,
            max_forward_mps=2.0,
            max_backward_mps=2.0,
            max_lateral_mps=2.0,
            command_hold_s=0.0,
        )
        identity = np.asarray([1.0, 0.0, 0.0, 0.0])
        controller.update(np.zeros(3), identity, 1.0)
        backward_left = controller.update(
            np.asarray([-0.01, 0.01, 0.0]), identity, 1.1
        )
        self.assertAlmostEqual(backward_left.forward_mps, -0.15)
        self.assertAlmostEqual(backward_left.lateral_mps, 0.14)


class SquatTests(unittest.TestCase):
    def controller(self):
        return SquatController(1.0, 0.05, 0.05)

    def test_planted_pelvis_drop_enters_and_maps_depth(self):
        controller = self.controller()
        result = controller.update(0.84, 0.05, 0.05, 0.0)
        self.assertTrue(result.active)
        self.assertLess(result.height_m, 0.80)
        self.assertGreaterEqual(result.height_m, 0.55)

    def test_lifted_foot_blocks_squat(self):
        result = self.controller().update(0.84, 0.10, 0.05, 0.0)
        self.assertFalse(result.active)

    def test_horizontal_motion_blocks_squat_entry(self):
        result = self.controller().update(0.84, 0.05, 0.05, 0.50)
        self.assertFalse(result.active)

    def test_natural_horizontal_drift_during_squat_is_allowed(self):
        result = self.controller().update(0.84, 0.05, 0.05, 0.16)
        self.assertTrue(result.active)

    def test_hysteresis_exits_near_standing(self):
        controller = self.controller()
        self.assertTrue(controller.update(0.84, 0.05, 0.05, 0.0).active)
        self.assertTrue(controller.update(0.92, 0.05, 0.05, 0.0).active)
        self.assertFalse(controller.update(0.97, 0.05, 0.05, 0.0).active)


if __name__ == "__main__":
    unittest.main()
