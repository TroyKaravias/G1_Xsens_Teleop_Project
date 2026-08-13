import unittest

from xsens_bridge.fight_control import Punch
from xsens_bridge.fight_intent import PunchGestureDetector
from xsens_bridge.xudp import MXTPHeader, PoseFrame, SegmentPose


def frame(left_x=0.35, right_x=0.35):
    header = MXTPHeader("MXTP02", 0, 0, 3, 0, 0, 3, 0, 0, 0)
    return PoseFrame(
        recording_timestamp_ns=0,
        header=header,
        segments=(
            SegmentPose(1, "pelvis", (0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0)),
            SegmentPose(15, "left_hand", (left_x, 0.3, 1.3), (1.0, 0.0, 0.0, 0.0)),
            SegmentPose(11, "right_hand", (right_x, -0.3, 1.3), (1.0, 0.0, 0.0, 0.0)),
        ),
    )


class PunchGestureDetectorTests(unittest.TestCase):
    def test_fast_extension_emits_one_jab_until_retracted(self):
        detector = PunchGestureDetector(
            forward_speed_mps=0.8, extension_m=0.08, cooldown_s=0.2
        )
        self.assertEqual(detector.update(frame(), 0.0), Punch.NONE)
        self.assertEqual(detector.update(frame(left_x=0.45), 0.05), Punch.LEFT_JAB)
        self.assertEqual(detector.update(frame(left_x=0.56), 0.10), Punch.NONE)
        self.assertEqual(detector.update(frame(left_x=0.35), 0.30), Punch.NONE)
        self.assertEqual(detector.update(frame(left_x=0.45), 0.35), Punch.LEFT_JAB)

    def test_slow_extension_does_not_trigger(self):
        detector = PunchGestureDetector(forward_speed_mps=1.0, extension_m=0.08)
        detector.update(frame(), 0.0)
        self.assertEqual(detector.update(frame(right_x=0.39), 0.1), Punch.NONE)
        self.assertEqual(detector.update(frame(right_x=0.44), 0.2), Punch.NONE)
