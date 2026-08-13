import json
import unittest

import numpy as np

from xsens_bridge.sonic_zmq import (
    G1_MUJOCO_TO_ISAACLAB,
    SONIC_ZMQ_HEADER_SIZE,
    mujoco_to_isaaclab,
    pack_sonic_pose_message,
    protocol_v1_fields,
)


class SonicZMQTests(unittest.TestCase):
    def test_protocol_v1_shapes_and_identity_body_quaternion(self):
        fields = protocol_v1_fields(
            np.zeros((4, 29)),
            np.ones((4, 29)),
            np.arange(4),
        )
        self.assertEqual(fields["joint_pos"].shape, (4, 29))
        self.assertEqual(fields["joint_pos"].dtype, np.float32)
        np.testing.assert_array_equal(fields["body_quat"][:, 0], 1.0)
        np.testing.assert_array_equal(fields["body_quat"][:, 1:], 0.0)

    def test_packed_header_matches_sonic_protocol(self):
        fields = protocol_v1_fields(
            np.zeros((4, 29)),
            np.ones((4, 29)),
            np.arange(4),
        )
        message = pack_sonic_pose_message(fields)
        self.assertTrue(message.startswith(b"pose"))
        header_start = len(b"pose")
        header = json.loads(
            message[
                header_start : header_start + SONIC_ZMQ_HEADER_SIZE
            ].rstrip(b"\x00")
        )
        self.assertEqual(header["v"], 1)
        self.assertEqual(header["endian"], "le")
        self.assertEqual(
            [field["name"] for field in header["fields"]],
            ["joint_pos", "joint_vel", "body_quat", "frame_index"],
        )

    def test_rejects_wrong_joint_shape(self):
        with self.assertRaisesRegex(ValueError, "joint_pos"):
            protocol_v1_fields(
                np.zeros((4, 28)),
                np.zeros((4, 28)),
                np.arange(4),
            )

    def test_reorders_mujoco_values_to_isaaclab_slots(self):
        mujoco = np.arange(29, dtype=np.float32)
        isaaclab = mujoco_to_isaaclab(mujoco)
        np.testing.assert_array_equal(
            isaaclab, mujoco[G1_MUJOCO_TO_ISAACLAB]
        )
        self.assertEqual(isaaclab[0], 0)   # left hip pitch
        self.assertEqual(isaaclab[1], 6)   # right hip pitch
        self.assertEqual(isaaclab[2], 12)  # waist yaw
        self.assertEqual(isaaclab[11], 15) # left shoulder pitch
        self.assertEqual(isaaclab[12], 22) # right shoulder pitch


if __name__ == "__main__":
    unittest.main()
