import unittest

from xsens_bridge.fencing_control import FencingEvent, FencingState
from xsens_bridge.mode_control import DemoMode, ModeEvent, ModeSupervisor
from xsens_bridge.locomotion_control import LocomotionEvent


class ModeSupervisorTests(unittest.TestCase):
    def test_fencing_requires_selection_then_arm(self):
        supervisor = ModeSupervisor()
        selected = supervisor.update(
            0.0, stream_fresh=True, operator_ready=True,
            event=ModeEvent.SELECT_FENCING,
        )
        self.assertEqual(selected.mode, DemoMode.FENCING)
        self.assertFalse(selected.armed)
        ignored = supervisor.update(
            0.1, stream_fresh=True, operator_ready=True,
            fencing_event=FencingEvent.ENTER_GARDE,
        )
        self.assertEqual(ignored.fencing.state, FencingState.SAFE)
        armed = supervisor.update(
            0.2, stream_fresh=True, operator_ready=True,
            event=ModeEvent.ARM,
        )
        self.assertTrue(armed.armed)

    def test_fencing_sequence_reaches_advance(self):
        supervisor = ModeSupervisor()
        supervisor.update(
            0.0, stream_fresh=True, operator_ready=True,
            event=ModeEvent.SELECT_FENCING,
        )
        supervisor.update(
            0.1, stream_fresh=True, operator_ready=True,
            event=ModeEvent.ARM,
        )
        supervisor.update(
            0.2, stream_fresh=True, operator_ready=True,
            fencing_event=FencingEvent.ENTER_GARDE,
        )
        supervisor.update(
            0.8, stream_fresh=True, operator_ready=True,
            fencing_event=FencingEvent.ENABLE_POSE,
        )
        advancing = supervisor.update(
            1.0, stream_fresh=True, operator_ready=True,
            fencing_event=FencingEvent.ADVANCE,
        )
        self.assertEqual(advancing.fencing.state, FencingState.ADVANCE)
        self.assertGreater(advancing.fencing.forward_velocity_mps, 0.0)

    def test_stream_loss_fails_closed(self):
        supervisor = ModeSupervisor()
        supervisor.update(
            0.0, stream_fresh=True, operator_ready=True,
            event=ModeEvent.SELECT_MOVING,
        )
        supervisor.update(
            0.1, stream_fresh=True, operator_ready=True,
            event=ModeEvent.ARM,
        )
        intent = supervisor.update(
            0.2, stream_fresh=False, operator_ready=True
        )
        self.assertEqual(intent.mode, DemoMode.SAFE)
        self.assertFalse(intent.armed)

    def test_moving_mode_emits_bounded_locomotion(self):
        supervisor = ModeSupervisor()
        supervisor.update(
            0.0, stream_fresh=True, operator_ready=True,
            event=ModeEvent.SELECT_MOVING,
        )
        supervisor.update(
            0.1, stream_fresh=True, operator_ready=True,
            event=ModeEvent.ARM,
        )
        intent = supervisor.update(
            0.2, stream_fresh=True, operator_ready=True,
            locomotion_event=LocomotionEvent.FORWARD,
        )
        self.assertEqual(intent.mode, DemoMode.MOVING)
        self.assertGreater(intent.locomotion.forward_velocity_mps, 0.0)
        self.assertFalse(intent.locomotion.stopped)
        self.assertTrue(intent.upper_body_tracking_enabled)

    def test_fighting_is_unavailable_and_fails_closed(self):
        supervisor = ModeSupervisor()
        intent = supervisor.update(
            0.0, stream_fresh=True, operator_ready=True,
            event=ModeEvent.SELECT_FIGHTING,
        )
        self.assertEqual(intent.mode, DemoMode.SAFE)
        self.assertFalse(intent.fighting_available)
        self.assertFalse(intent.armed)

    def test_estop_is_latched(self):
        supervisor = ModeSupervisor()
        stopped = supervisor.update(
            0.0, stream_fresh=True, operator_ready=True,
            event=ModeEvent.ESTOP,
        )
        self.assertTrue(stopped.estop_latched)
        still_stopped = supervisor.update(
            0.1, stream_fresh=True, operator_ready=True,
            event=ModeEvent.SELECT_FENCING,
        )
        self.assertTrue(still_stopped.estop_latched)
