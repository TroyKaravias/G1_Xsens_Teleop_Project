import unittest

import numpy as np

from xsens_bridge.live_reference import (
    LiveReferenceBuffer,
    ReferenceFrame,
    WatchdogState,
)


def frame(timestamp: float) -> ReferenceFrame:
    return ReferenceFrame(
        timestamp=timestamp,
        dof_pos=np.zeros(29),
        dof_vel=np.ones(29),
        body_rot=np.tile(np.array((0.0, 0.0, 0.0, 1.0)), (33, 1)),
    )


class LiveReferenceTests(unittest.TestCase):
    def test_watchdog_transitions(self) -> None:
        buffer = LiveReferenceBuffer()
        self.assertIs(buffer.state(0.0), WatchdogState.WARMUP)
        buffer.push(frame(1.0))
        self.assertIs(buffer.state(1.05), WatchdogState.LIVE)
        self.assertIs(buffer.state(1.10), WatchdogState.HOLD)
        self.assertIs(buffer.state(1.30), WatchdogState.SAFE_RETURN)
        buffer.trigger_estop()
        self.assertIs(buffer.state(1.31), WatchdogState.ESTOP)
        buffer.push(frame(1.51))
        self.assertIs(buffer.state(1.51), WatchdogState.ESTOP)
        buffer.reset_estop(1.51)
        self.assertIs(buffer.state(1.51), WatchdogState.SAFE_RETURN)

    def test_future_extrapolation_and_hold(self) -> None:
        buffer = LiveReferenceBuffer()
        buffer.push(frame(1.0))
        live = buffer.future(1.01)
        np.testing.assert_allclose(
            live["dof_pos"][:, 0], (0.02, 0.04, 0.08, 0.16)
        )
        held = buffer.future(1.20)
        np.testing.assert_allclose(held["dof_pos"], 0.0)
        np.testing.assert_allclose(held["dof_vel"], 0.0)


if __name__ == "__main__":
    unittest.main()
