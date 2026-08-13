from __future__ import annotations

import unittest

import numpy as np

from tools.kick_stability_sidecar import (
    isaaclab_to_mujoco,
    mujoco_to_isaaclab,
    stabilize_message,
    unpack_fields,
)
from xsens_bridge.g1_retarget import G1_DEFAULT_POSE
from xsens_bridge.kick_stability import (
    KickPhase,
    KickStabilityGovernor,
    KickStabilityLimits,
)
from xsens_bridge.sonic_zmq import pack_sonic_pose_message, protocol_v1_fields


class KickStabilityTests(unittest.TestCase):
    def make_governor(self) -> KickStabilityGovernor:
        return KickStabilityGovernor(KickStabilityLimits(phase_dwell_s=0.04))

    def test_double_support_passes_reference_through(self) -> None:
        governor = self.make_governor()
        position = G1_DEFAULT_POSE.copy()
        velocity = np.linspace(-1.0, 1.0, 29)

        result = governor.update(0.0, position, velocity)

        self.assertEqual(result.phase, KickPhase.DOUBLE_SUPPORT)
        np.testing.assert_allclose(result.joint_pos, position)
        np.testing.assert_allclose(result.joint_vel, velocity)

    def test_left_swing_anchors_right_stance_ankle_and_yaw(self) -> None:
        governor = self.make_governor()
        neutral = G1_DEFAULT_POSE.copy()
        governor.update(0.0, neutral, np.zeros(29))
        lifted = neutral.copy()
        lifted[0] -= 0.50
        lifted[3] += 0.45
        lifted[6] -= 0.35
        lifted[7] = 0.30
        lifted[8] = 0.35
        lifted[9] += 0.25
        lifted[10] = 0.20
        lifted[11] = 0.25
        governor.update(0.02, lifted, np.zeros(29))
        governor.update(0.04, lifted, np.zeros(29))
        result = governor.update(0.07, lifted, np.zeros(29))

        self.assertEqual(result.phase, KickPhase.LEFT_SWING)
        self.assertEqual(result.support_side, "right")
        for index in (6, 7, 8, 9, 10, 11):
            self.assertLess(
                abs(float(result.joint_pos[index] - neutral[index])),
                abs(float(lifted[index] - neutral[index])),
            )

    def test_support_leg_is_clamped_around_stance_reference(self) -> None:
        limits = KickStabilityLimits(
            phase_dwell_s=0.04,
            stance_protection=0.0,
            support_hip_pitch_limit_rad=0.05,
            support_hip_roll_limit_rad=0.05,
            support_hip_yaw_limit_rad=0.05,
            support_knee_limit_rad=0.05,
            support_ankle_pitch_limit_rad=0.05,
            support_ankle_roll_limit_rad=0.05,
        )
        governor = KickStabilityGovernor(limits)
        neutral = G1_DEFAULT_POSE.copy()
        governor.update(0.0, neutral, np.zeros(29))
        lifted = neutral.copy()
        lifted[0] -= 0.90
        lifted[3] += 0.70
        lifted[6:12] += np.array((-0.18, 0.16, -0.14, 0.12, 0.10, -0.10))
        governor.update(0.02, lifted, np.zeros(29))
        governor.update(0.04, lifted, np.zeros(29))
        result = governor.update(0.07, lifted, np.zeros(29))

        self.assertEqual(result.phase, KickPhase.LEFT_SWING)
        self.assertTrue(
            np.all(np.abs(result.joint_pos[6:12] - neutral[6:12]) <= 0.05 + 1e-6)
        )

    def test_adaptive_balance_assist_limits_forward_support_hip_flexion(self) -> None:
        limits = KickStabilityLimits(
            phase_dwell_s=0.04,
            stance_protection=0.0,
            support_hip_pitch_limit_rad=np.pi,
            balance_activation_score_rad=0.10,
            balance_full_score_rad=0.20,
            adaptive_support_hip_pitch_forward_limit_rad=0.04,
        )
        governor = KickStabilityGovernor(limits)
        neutral = G1_DEFAULT_POSE.copy()
        governor.update(0.0, neutral, np.zeros(29))
        lifted = neutral.copy()
        lifted[0] -= 0.90
        lifted[3] += 0.80
        lifted[6] -= 0.50
        governor.update(0.02, lifted, np.zeros(29))
        governor.update(0.04, lifted, np.zeros(29))
        result = governor.update(0.07, lifted, np.zeros(29))

        self.assertEqual(result.phase, KickPhase.LEFT_SWING)
        self.assertGreaterEqual(
            float(result.joint_pos[6]),
            float(neutral[6] - 0.04 - 1e-6),
        )

    def test_balance_counterpose_biases_support_knee_ankle_and_waist(self) -> None:
        limits = KickStabilityLimits(
            phase_dwell_s=0.04,
            stance_protection=0.0,
            waist_protection=0.0,
            support_hip_pitch_limit_rad=np.pi,
            support_knee_limit_rad=np.pi,
            support_ankle_pitch_limit_rad=np.pi,
            waist_pitch_limit_rad=np.pi,
            balance_activation_score_rad=0.10,
            balance_full_score_rad=0.20,
            support_hip_pitch_counter_bias_rad=0.06,
            support_knee_counter_bias_rad=0.08,
            support_ankle_pitch_counter_bias_rad=0.05,
            waist_pitch_counter_bias_rad=0.04,
        )
        governor = KickStabilityGovernor(limits)
        neutral = G1_DEFAULT_POSE.copy()
        governor.update(0.0, neutral, np.zeros(29))
        lifted = neutral.copy()
        lifted[0] -= 0.90
        lifted[3] += 0.80
        governor.update(0.02, lifted, np.zeros(29))
        governor.update(0.04, lifted, np.zeros(29))
        result = governor.update(0.07, lifted, np.zeros(29))

        self.assertEqual(result.phase, KickPhase.LEFT_SWING)
        self.assertGreaterEqual(
            float(result.joint_pos[6]),
            float(neutral[6] + 0.06 - 1e-6),
        )
        self.assertGreaterEqual(
            float(result.joint_pos[9]),
            float(neutral[9] + 0.08 - 1e-6),
        )
        self.assertLessEqual(
            float(result.joint_pos[10]),
            float(neutral[10] - 0.05 + 1e-6),
        )
        self.assertGreaterEqual(
            float(result.joint_pos[14]),
            0.04 - 1e-6,
        )

    def test_adaptive_balance_assist_reengages_support_limits(self) -> None:
        limits = KickStabilityLimits(
            phase_dwell_s=0.04,
            stance_protection=0.0,
            waist_protection=0.0,
            support_hip_pitch_limit_rad=np.pi,
            support_hip_roll_limit_rad=np.pi,
            support_hip_yaw_limit_rad=np.pi,
            support_knee_limit_rad=np.pi,
            support_ankle_pitch_limit_rad=np.pi,
            support_ankle_roll_limit_rad=np.pi,
            waist_roll_limit_rad=np.pi,
            waist_pitch_limit_rad=np.pi,
            balance_activation_score_rad=0.10,
            balance_full_score_rad=0.20,
            adaptive_support_hip_pitch_limit_rad=0.08,
            adaptive_support_knee_limit_rad=0.09,
            adaptive_support_ankle_pitch_limit_rad=0.07,
            adaptive_waist_pitch_limit_rad=0.06,
        )
        governor = KickStabilityGovernor(limits)
        neutral = G1_DEFAULT_POSE.copy()
        governor.update(0.0, neutral, np.zeros(29))
        lifted = neutral.copy()
        lifted[0] -= 0.90
        lifted[3] += 0.80
        lifted[6] -= 0.50
        lifted[9] += 0.50
        lifted[10] += 0.30
        lifted[14] = 0.40
        governor.update(0.02, lifted, np.zeros(29))
        governor.update(0.04, lifted, np.zeros(29))
        result = governor.update(0.07, lifted, np.zeros(29))

        self.assertEqual(result.phase, KickPhase.LEFT_SWING)
        self.assertLessEqual(
            abs(float(result.joint_pos[6] - neutral[6])),
            0.08 + 1e-6,
        )
        self.assertLessEqual(
            abs(float(result.joint_pos[9] - neutral[9])),
            0.09 + 1e-6,
        )
        self.assertLessEqual(
            abs(float(result.joint_pos[10] - neutral[10])),
            0.07 + 1e-6,
        )
        self.assertLessEqual(
            abs(float(result.joint_pos[14])),
            0.06 + 1e-6,
        )

    def test_swing_hip_and_ankle_angles_are_bounded(self) -> None:
        governor = self.make_governor()
        neutral = G1_DEFAULT_POSE.copy()
        governor.update(0.0, neutral, np.zeros(29))
        extreme = neutral.copy()
        extreme[0] -= 2.00
        extreme[3] += 0.60
        extreme[1] = 2.0
        extreme[2] = -2.0
        extreme[4] = 2.0
        extreme[5] = -2.0
        governor.update(0.02, extreme, np.zeros(29))
        governor.update(0.04, extreme, np.zeros(29))
        result = governor.update(0.07, extreme, np.zeros(29))

        self.assertEqual(result.phase, KickPhase.LEFT_SWING)
        limits = governor.limits
        self.assertLessEqual(
            abs(float(result.joint_pos[0] - neutral[0])),
            limits.swing_hip_pitch_limit_rad + 1e-6,
        )
        self.assertLessEqual(
            abs(float(result.joint_pos[1] - neutral[1])),
            limits.swing_hip_roll_limit_rad + 1e-6,
        )
        self.assertLessEqual(
            abs(float(result.joint_pos[2] - neutral[2])),
            limits.swing_hip_yaw_limit_rad + 1e-6,
        )
        self.assertLessEqual(
            abs(float(result.joint_pos[4] - neutral[4])),
            limits.swing_ankle_pitch_limit_rad + 1e-6,
        )
        self.assertLessEqual(
            abs(float(result.joint_pos[5] - neutral[5])),
            limits.swing_ankle_roll_limit_rad + 1e-6,
        )

    def test_swing_hip_pitch_stays_chambered_while_knee_extends(self) -> None:
        governor = self.make_governor()
        neutral = G1_DEFAULT_POSE.copy()
        governor.update(0.0, neutral, np.zeros(29))

        chamber = neutral.copy()
        chamber[0] -= 0.70
        chamber[3] += 0.75
        governor.update(0.02, chamber, np.zeros(29))
        governor.update(0.04, chamber, np.zeros(29))
        chambered = governor.update(0.07, chamber, np.zeros(29))

        extend = chamber.copy()
        extend[0] = neutral[0] - 0.20
        extend[3] = neutral[3] + 0.10
        extended = governor.update(0.10, extend, np.zeros(29))

        self.assertEqual(chambered.phase, KickPhase.LEFT_SWING)
        self.assertEqual(extended.phase, KickPhase.LEFT_SWING)
        self.assertLessEqual(
            float(extended.joint_pos[0]),
            float(chambered.joint_pos[0]) + 1e-6,
        )
        self.assertGreater(
            float(extended.joint_pos[3]),
            float(neutral[3]),
        )

    def test_lower_body_transition_obeys_slew_limit(self) -> None:
        governor = self.make_governor()
        neutral = G1_DEFAULT_POSE.copy()
        first = governor.update(0.0, neutral, np.zeros(29))
        changed = neutral.copy()
        changed[:15] += 2.0
        second = governor.update(0.02, changed, np.zeros(29))

        maximum_step = governor.limits.lower_body_slew_rad_s * 0.02
        self.assertLessEqual(
            float(np.max(np.abs(second.joint_pos[:15] - first.joint_pos[:15]))),
            maximum_step + 1e-6,
        )

    def test_isaaclab_round_trip_and_sidecar_preserve_protocol(self) -> None:
        mujoco = np.arange(29, dtype=np.float32)
        np.testing.assert_array_equal(
            isaaclab_to_mujoco(mujoco_to_isaaclab(mujoco)), mujoco
        )
        fields = protocol_v1_fields(
            mujoco_to_isaaclab(G1_DEFAULT_POSE)[None],
            np.zeros((1, 29), dtype=np.float32),
            np.asarray([42], dtype=np.int64),
        )
        message = pack_sonic_pose_message(fields)
        output, _ = stabilize_message(
            message, b"pose", KickStabilityGovernor(), 1.0
        )
        version, decoded = unpack_fields(output, b"pose")

        self.assertEqual(version, 1)
        self.assertEqual(int(decoded["frame_index"][0]), 42)
        self.assertEqual(decoded["joint_pos"].shape, (1, 29))


if __name__ == "__main__":
    unittest.main()
