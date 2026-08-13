import unittest

from xsens_bridge.fencing_control import (
    FencingEvent,
    FencingLimits,
    FencingState,
    FencingSupervisor,
)


class FencingSupervisorTests(unittest.TestCase):
    def test_guard_requires_fresh_stream_and_ready_operator(self):
        supervisor = FencingSupervisor()
        intent = supervisor.update(
            0.0,
            stream_fresh=False,
            operator_ready=True,
            event=FencingEvent.ENTER_GARDE,
        )
        self.assertEqual(intent.state, FencingState.SAFE)
        intent = supervisor.update(
            0.1,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENTER_GARDE,
        )
        self.assertEqual(intent.state, FencingState.EN_GARDE)

    def test_pose_requires_guard_dwell(self):
        supervisor = FencingSupervisor()
        supervisor.update(
            0.0,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENTER_GARDE,
        )
        early = supervisor.update(
            0.2,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENABLE_POSE,
        )
        self.assertEqual(early.state, FencingState.EN_GARDE)
        ready = supervisor.update(
            0.6,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENABLE_POSE,
        )
        self.assertEqual(ready.state, FencingState.POSE_TRACKING)

    def test_advance_is_bounded_and_returns_to_pose(self):
        limits = FencingLimits(
            advance_speed_mps=0.15,
            step_duration_s=0.45,
        )
        supervisor = FencingSupervisor(limits)
        supervisor.update(
            0.0,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENTER_GARDE,
        )
        supervisor.update(
            0.6,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENABLE_POSE,
        )
        advancing = supervisor.update(
            0.7,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ADVANCE,
        )
        self.assertEqual(advancing.state, FencingState.ADVANCE)
        self.assertEqual(advancing.forward_velocity_mps, 0.15)
        done = supervisor.update(
            1.2,
            stream_fresh=True,
            operator_ready=True,
        )
        self.assertEqual(done.state, FencingState.POSE_TRACKING)
        self.assertEqual(done.forward_velocity_mps, 0.0)

    def test_lunge_always_enters_recovery(self):
        supervisor = FencingSupervisor()
        supervisor.update(
            0.0,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENTER_GARDE,
        )
        supervisor.update(
            0.6,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENABLE_POSE,
        )
        supervisor.update(
            0.7,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.LUNGE,
        )
        recovering = supervisor.update(
            1.4,
            stream_fresh=True,
            operator_ready=True,
        )
        self.assertEqual(recovering.state, FencingState.RECOVER)
        guarded = supervisor.update(
            2.3,
            stream_fresh=True,
            operator_ready=True,
        )
        self.assertEqual(guarded.state, FencingState.EN_GARDE)

    def test_stale_stream_recovers_then_safes(self):
        supervisor = FencingSupervisor()
        supervisor.update(
            0.0,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ENTER_GARDE,
        )
        recovering = supervisor.update(
            0.1,
            stream_fresh=False,
            operator_ready=True,
        )
        self.assertEqual(recovering.state, FencingState.RECOVER)
        safe = supervisor.update(
            1.0,
            stream_fresh=False,
            operator_ready=True,
        )
        self.assertEqual(safe.state, FencingState.SAFE)

    def test_estop_is_latched_until_explicit_safe_reset(self):
        supervisor = FencingSupervisor()
        stopped = supervisor.update(
            0.0,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.ESTOP,
        )
        self.assertEqual(stopped.state, FencingState.ESTOP)
        still_stopped = supervisor.update(
            1.0,
            stream_fresh=False,
            operator_ready=True,
            event=FencingEvent.RESET_ESTOP,
        )
        self.assertEqual(still_stopped.state, FencingState.ESTOP)
        reset = supervisor.update(
            2.0,
            stream_fresh=True,
            operator_ready=True,
            event=FencingEvent.RESET_ESTOP,
        )
        self.assertEqual(reset.state, FencingState.SAFE)
        self.assertFalse(reset.estop_latched)


if __name__ == "__main__":
    unittest.main()
