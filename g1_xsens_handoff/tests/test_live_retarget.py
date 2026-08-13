import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from xsens_bridge.live_retarget import OnlineG1Retargeter, calibrate_nt_sequence
from xsens_bridge.xudp import iter_mxtp02_frames


ROOT = Path(__file__).resolve().parents[1]
RECORDING = ROOT / "data" / "xsens_20260723_114126.xudp"
if not RECORDING.exists():
    RECORDING = (
        ROOT
        / "g1_xsens_handoff"
        / "data"
        / "xsens_20260723_114126.xudp"
    )


class LiveRetargetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        frames = list(iter_mxtp02_frames(RECORDING))
        cls.frames = frames
        start_ns = frames[0].recording_timestamp_ns
        cls.calibration_frames = [
            frame
            for frame in frames
            if frame.recording_timestamp_ns - start_ns <= 9_000_000_000
        ]

    def test_nt_calibration_selects_extended_arm_pose(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        positions = calibration.positions
        hand_span = np.linalg.norm(
            positions["left_hand"] - positions["right_hand"]
        )
        shoulder_span = np.linalg.norm(
            positions["left_shoulder"] - positions["right_shoulder"]
        )
        self.assertGreater(hand_span, shoulder_span * 2.5)
        self.assertGreater(calibration.captured_frames, 100)

    def test_online_retarget_is_finite_and_timestamped(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        retargeter = OnlineG1Retargeter(calibration)
        first = retargeter.retarget(self.frames[0], 10.0)
        second = retargeter.retarget(self.frames[1], 10.004)
        self.assertEqual(first.dof_pos.shape, (29,))
        self.assertEqual(second.dof_vel.shape, (29,))
        self.assertTrue(np.isfinite(first.dof_pos).all())
        self.assertTrue(np.isfinite(second.dof_vel).all())
        self.assertTrue((np.abs(second.dof_vel) <= 12.0).all())

    def test_position_reference_cannot_snap_after_long_stall(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        retargeter = OnlineG1Retargeter(
            calibration,
            max_position_speed=4.0,
            max_position_dt=0.04,
        )
        first = retargeter.retarget(self.frames[0], 10.0)
        # A very different pose arrives after a long outage. The elapsed wall
        # time must not permit one large catch-up command.
        resumed = retargeter.retarget(self.frames[-1], 20.0)
        self.assertLessEqual(
            float(np.max(np.abs(resumed.dof_pos - first.dof_pos))),
            4.0 * 0.04 + 1e-6,
        )

    def test_wrist_tracking_is_opt_in_and_bounded(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        source = self.frames[0]
        angle = np.deg2rad(90.0)

        def multiply(a, b):
            aw, ax, ay, az = a
            bw, bx, by, bz = b
            return np.array((
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ))

        forearm = next(
            segment for segment in source.segments
            if segment.name == "left_forearm"
        )
        hand = next(
            segment for segment in source.segments
            if segment.name == "left_hand"
        )
        forearm_axis_world = (
            np.asarray(hand.position_m) - np.asarray(forearm.position_m)
        )
        q = np.asarray(forearm.quaternion_wxyz)
        pure = np.r_[0.0, forearm_axis_world]
        conjugate = q * np.array([1.0, -1.0, -1.0, -1.0])
        forearm_axis_local = multiply(multiply(conjugate, pure), q)[1:]
        forearm_axis_local /= np.linalg.norm(forearm_axis_local)
        local_roll = np.r_[
            np.cos(angle / 2.0),
            np.sin(angle / 2.0) * forearm_axis_local,
        ]

        segments = []
        for segment in source.segments:
            if segment.name == "left_hand":
                rotated = multiply(
                    np.asarray(segment.quaternion_wxyz), local_roll
                )
                segment = replace(
                    segment, quaternion_wxyz=tuple(rotated.tolist())
                )
            segments.append(segment)
        rotated_frame = replace(source, segments=tuple(segments))

        disabled = OnlineG1Retargeter(calibration, track_wrists=False)
        enabled = OnlineG1Retargeter(calibration, track_wrists=True)
        disabled_pose = disabled.retarget(rotated_frame, 10.0)
        enabled_pose = enabled.retarget(rotated_frame, 10.0)

        self.assertAlmostEqual(float(disabled_pose.dof_pos[19]), 0.0)
        self.assertGreater(abs(float(enabled_pose.dof_pos[19])), 0.5)
        self.assertLessEqual(abs(float(enabled_pose.dof_pos[19])), 1.4)
        self.assertAlmostEqual(float(enabled_pose.dof_pos[20]), 0.0)
        self.assertAlmostEqual(float(enabled_pose.dof_pos[21]), 0.0)

    def test_wrist_pitch_is_opt_in_bounded_and_yaw_stays_neutral(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        source = self.frames[0]
        probe = OnlineG1Retargeter(
            calibration, track_wrists=True, track_wrist_pitch=True
        )
        axis = probe._wrist_pitch_axes[0]
        angle = np.deg2rad(70.0)
        local_pitch = np.r_[
            np.cos(angle / 2.0), np.sin(angle / 2.0) * axis
        ]

        def multiply(a, b):
            aw, ax, ay, az = a
            bw, bx, by, bz = b
            return np.array((
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ))

        segments = tuple(
            replace(
                segment,
                quaternion_wxyz=tuple(
                    multiply(
                        np.asarray(segment.quaternion_wxyz), local_pitch
                    ).tolist()
                ),
            )
            if segment.name == "left_hand"
            else segment
            for segment in source.segments
        )
        changed = replace(source, segments=segments)
        disabled = OnlineG1Retargeter(
            calibration, track_wrists=True, track_wrist_pitch=False
        ).retarget(changed, 10.0)
        enabled = probe.retarget(changed, 10.0)

        self.assertAlmostEqual(float(disabled.dof_pos[20]), 0.0)
        self.assertAlmostEqual(float(enabled.dof_pos[20]), 0.0)
        self.assertGreater(abs(float(enabled.dof_pos[21])), 0.2)
        self.assertLessEqual(abs(float(enabled.dof_pos[21])), 0.35)

        inverted = OnlineG1Retargeter(
            calibration,
            track_wrists=True,
            track_wrist_pitch=True,
            wrist_pitch_sign=-1.0,
        ).retarget(changed, 10.0)
        self.assertAlmostEqual(
            float(inverted.dof_pos[21]),
            -float(enabled.dof_pos[21]),
            places=5,
        )
        left_only = OnlineG1Retargeter(
            calibration,
            track_wrists=True,
            track_wrist_pitch=True,
            left_wrist_pitch_sign=-1.0,
        ).retarget(changed, 10.0)
        self.assertAlmostEqual(
            float(left_only.dof_pos[21]),
            -float(enabled.dof_pos[21]),
            places=5,
        )

    def test_pelvis_heading_maps_to_bounded_waist_yaw_only_when_enabled(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        source = self.frames[0]
        angle = np.deg2rad(90.0)
        yaw = np.array(
            [np.cos(angle / 2.0), 0.0, 0.0, np.sin(angle / 2.0)]
        )
        segments = tuple(
            replace(segment, quaternion_wxyz=tuple(yaw.tolist()))
            if segment.name == "pelvis"
            else segment
            for segment in source.segments
        )
        turned = replace(source, segments=segments)

        disabled = OnlineG1Retargeter(calibration, track_pelvis_yaw=False)
        enabled = OnlineG1Retargeter(
            calibration,
            track_pelvis_yaw=True,
            pelvis_yaw_gain=1.0,
            pelvis_yaw_limit_rad=0.5,
        )
        disabled_pose = disabled.retarget(turned, 10.0)
        enabled_pose = enabled.retarget(turned, 10.0)

        self.assertNotAlmostEqual(
            float(disabled_pose.dof_pos[12]),
            float(enabled_pose.dof_pos[12]),
        )
        self.assertLessEqual(abs(float(enabled_pose.dof_pos[12])), 0.5)

    def test_foot_heading_only_overrides_hip_yaw_while_lifted(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        source = self.frames[0]
        angle = np.deg2rad(70.0)
        yaw = np.array(
            [np.cos(angle / 2.0), 0.0, 0.0, np.sin(angle / 2.0)]
        )

        def changed_frame(*, lift: float):
            segments = []
            for segment in source.segments:
                if segment.name == "left_foot":
                    position = list(segment.position_m)
                    position[2] += lift
                    segment = replace(
                        segment,
                        position_m=tuple(position),
                        quaternion_wxyz=tuple(yaw.tolist()),
                    )
                segments.append(segment)
            return replace(source, segments=tuple(segments))

        grounded_controller = OnlineG1Retargeter(
            calibration, track_foot_heading=True
        )
        lifted_controller = OnlineG1Retargeter(
            calibration, track_foot_heading=True
        )
        grounded = grounded_controller.retarget(
            changed_frame(lift=0.0), 10.0
        )
        lifted = lifted_controller.retarget(
            changed_frame(lift=0.12), 10.0
        )

        self.assertNotAlmostEqual(
            float(grounded.dof_pos[2]), float(lifted.dof_pos[2])
        )
        self.assertLessEqual(abs(float(lifted.dof_pos[2])), 0.7)

    def test_arm_pose_is_invariant_to_global_body_heading(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        source = self.frames[-1]
        angle = np.deg2rad(80.0)
        yaw = np.array(
            [np.cos(angle / 2.0), 0.0, 0.0, np.sin(angle / 2.0)]
        )
        rotation = np.array((
            (np.cos(angle), -np.sin(angle), 0.0),
            (np.sin(angle), np.cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        ))

        def multiply(a, b):
            aw, ax, ay, az = a
            bw, bx, by, bz = b
            return np.array((
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ))

        rotated_segments = tuple(
            replace(
                segment,
                position_m=tuple(
                    (rotation @ np.asarray(segment.position_m)).tolist()
                ),
                quaternion_wxyz=tuple(
                    multiply(yaw, np.asarray(segment.quaternion_wxyz)).tolist()
                ),
            )
            for segment in source.segments
        )
        rotated = replace(source, segments=rotated_segments)
        original_pose = OnlineG1Retargeter(calibration).retarget(source, 10.0)
        rotated_pose = OnlineG1Retargeter(calibration).retarget(rotated, 10.0)

        np.testing.assert_allclose(
            rotated_pose.dof_pos[15:29],
            original_pose.dof_pos[15:29],
            atol=1e-5,
        )
        np.testing.assert_allclose(
            rotated_pose.dof_pos[12:15],
            original_pose.dof_pos[12:15],
            atol=1e-5,
        )

    def test_lateral_knee_position_refines_only_lifted_hip_roll(self):
        calibration = calibrate_nt_sequence(self.calibration_frames)
        source = self.frames[0]

        def changed_frame(*, lift: float, lateral: float):
            segments = []
            for segment in source.segments:
                if segment.name == "left_lower_leg":
                    position = list(segment.position_m)
                    position[1] += lateral
                    segment = replace(segment, position_m=tuple(position))
                if segment.name == "left_foot":
                    position = list(segment.position_m)
                    position[1] += lateral
                    position[2] += lift
                    segment = replace(segment, position_m=tuple(position))
                segments.append(segment)
            return replace(source, segments=tuple(segments))

        grounded_controller = OnlineG1Retargeter(
            calibration, track_lifted_hip_roll=True
        )
        lifted_controller = OnlineG1Retargeter(
            calibration, track_lifted_hip_roll=True
        )
        grounded = grounded_controller.retarget(
            changed_frame(lift=0.0, lateral=0.20), 10.0
        )
        lifted = lifted_controller.retarget(
            changed_frame(lift=0.12, lateral=0.20), 10.0
        )

        self.assertNotAlmostEqual(
            float(grounded.dof_pos[1]), float(lifted.dof_pos[1])
        )
        self.assertLessEqual(abs(float(lifted.dof_pos[1])), 0.35)


if __name__ == "__main__":
    unittest.main()
