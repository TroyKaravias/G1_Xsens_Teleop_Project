from __future__ import annotations

import math
import unittest

from xsens_bridge.body_motion import BodyMotionController
from xsens_bridge.xudp import MXTPHeader, PoseFrame, SegmentPose


def yaw_quaternion(angle: float) -> tuple[float, float, float, float]:
    return (math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0))


def frame(pelvis_yaw: float, head_yaw: float) -> PoseFrame:
    header = MXTPHeader("MXTP02", 0, 0, 2, 0, 0, 2, 0, 0, 0)
    return PoseFrame(
        recording_timestamp_ns=0,
        header=header,
        segments=(
            SegmentPose(1, "pelvis", (0.0, 0.0, 1.0), yaw_quaternion(pelvis_yaw)),
            SegmentPose(7, "head", (0.0, 0.0, 1.7), yaw_quaternion(head_yaw)),
        ),
    )


class BodyMotionHeadTurnTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = BodyMotionController(
            head_turn_rate_deadband_rad_s=0.1,
            head_turn_gain=0.5,
            head_turn_max_rate_rad_s=0.6,
        )
        self.controller.update(frame(0.0, 0.0), 1.00, fencing=False)
        self.controller.update(frame(0.0, 0.0), 1.02, fencing=False)

    def test_centered_head_commands_no_turn(self) -> None:
        intent = self.controller.update(frame(0.0, 0.0), 1.04, fencing=False)
        self.assertEqual(intent.yaw_rate_rad_s, 0.0)

    def test_head_yaw_relative_to_pelvis_commands_turn(self) -> None:
        left = self.controller.update(
            frame(0.0, math.radians(30.0)), 1.04, fencing=False
        )
        right = self.controller.update(
            frame(0.0, 0.0), 1.06, fencing=False
        )
        self.assertGreater(left.yaw_rate_rad_s, 0.0)
        self.assertLess(right.yaw_rate_rad_s, 0.0)

    def test_whole_body_yaw_does_not_command_turn(self) -> None:
        intent = self.controller.update(
            frame(math.radians(35.0), math.radians(35.0)),
            1.04,
            fencing=False,
        )
        self.assertEqual(intent.yaw_rate_rad_s, 0.0)

    def test_turn_rate_is_bounded(self) -> None:
        intent = self.controller.update(
            frame(0.0, math.radians(90.0)), 1.04, fencing=False
        )
        self.assertAlmostEqual(intent.yaw_rate_rad_s, 0.6)

    def test_holding_turned_head_stops_turn_command(self) -> None:
        self.controller.update(
            frame(0.0, math.radians(45.0)), 1.04, fencing=False
        )
        held = self.controller.update(
            frame(0.0, math.radians(45.0)), 1.06, fencing=False
        )
        self.assertEqual(held.yaw_rate_rad_s, 0.0)
if __name__ == "__main__":
    unittest.main()
