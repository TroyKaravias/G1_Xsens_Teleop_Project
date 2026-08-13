import unittest

from xsens_bridge.locomotion_control import (
    LocomotionEvent,
    LocomotionSupervisor,
)


class LocomotionSupervisorTests(unittest.TestCase):
    def test_disabled_is_stopped(self):
        intent = LocomotionSupervisor().update(
            LocomotionEvent.FORWARD,
            enabled=False,
            stream_fresh=True,
            operator_ready=True,
        )
        self.assertTrue(intent.stopped)
        self.assertEqual(intent.forward_velocity_mps, 0.0)

    def test_forward_backward_and_turn_are_bounded(self):
        supervisor = LocomotionSupervisor()
        forward = supervisor.update(
            LocomotionEvent.FORWARD,
            enabled=True,
            stream_fresh=True,
            operator_ready=True,
        )
        backward = supervisor.update(
            LocomotionEvent.BACKWARD,
            enabled=True,
            stream_fresh=True,
            operator_ready=True,
        )
        left = supervisor.update(
            LocomotionEvent.TURN_LEFT,
            enabled=True,
            stream_fresh=True,
            operator_ready=True,
        )
        self.assertGreater(forward.forward_velocity_mps, 0.0)
        self.assertLess(backward.forward_velocity_mps, 0.0)
        self.assertGreater(left.yaw_rate_rad_s, 0.0)

    def test_stale_stream_stops(self):
        intent = LocomotionSupervisor().update(
            LocomotionEvent.TURN_RIGHT,
            enabled=True,
            stream_fresh=False,
            operator_ready=True,
        )
        self.assertTrue(intent.stopped)

