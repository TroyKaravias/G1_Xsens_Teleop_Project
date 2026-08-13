import math
import unittest

from xsens_bridge.fight_control import (
    FallDetector,
    FightManeuver,
    FightRequest,
    FightRoute,
    FightState,
    FightSupervisor,
    Punch,
    RobotBalanceSample,
)


def ready(**kwargs):
    values = dict(armed=True, stream_fresh=True, operator_ready=True)
    values.update(kwargs)
    return FightRequest(**values)


class FightSupervisorTests(unittest.TestCase):
    def test_disarmed_and_stale_fail_closed(self):
        supervisor = FightSupervisor()
        self.assertEqual(supervisor.update(0.0, FightRequest()).route, FightRoute.SAFE)
        output = supervisor.update(0.1, ready(stream_fresh=False))
        self.assertEqual(output.state, FightState.DISARMED)
        self.assertEqual(output.route, FightRoute.SAFE)

    def test_pose_and_locomotion_are_exclusive_and_bounded(self):
        supervisor = FightSupervisor()
        pose = supervisor.update(0.0, ready())
        self.assertEqual(pose.route, FightRoute.POSE)
        moving = supervisor.update(
            0.1,
            ready(forward_mps=9.0, lateral_mps=-9.0, yaw_rate_rad_s=9.0),
        )
        self.assertEqual(moving.route, FightRoute.PLANNER)
        self.assertEqual(moving.maneuver, FightManeuver.WALK_BOXING)
        self.assertLessEqual(math.hypot(moving.forward_mps, moving.lateral_mps), 0.30)
        self.assertGreater(moving.forward_mps, 0.0)
        self.assertLess(moving.lateral_mps, 0.0)
        self.assertAlmostEqual(moving.yaw_rate_rad_s, 0.50)

    def test_punch_is_pulsed_then_cooldown_returns_to_pose(self):
        supervisor = FightSupervisor(punch_duration_s=0.5, punch_cooldown_s=0.5)
        punch = supervisor.update(0.0, ready(punch=Punch.LEFT_JAB))
        self.assertEqual(punch.maneuver, FightManeuver.LEFT_JAB)
        self.assertEqual(supervisor.update(0.4, ready()).state, FightState.BOXING)
        finished = supervisor.update(0.6, ready(punch=Punch.RIGHT_JAB))
        self.assertEqual(finished.route, FightRoute.POSE)

    def test_fall_requires_settle_and_explicit_get_up(self):
        supervisor = FightSupervisor(fallen_settle_s=0.5)
        fallen = supervisor.update(0.0, ready(fall_detected=True))
        self.assertEqual(fallen.state, FightState.FALLEN)
        early = supervisor.update(0.1, ready(robot_stable=True, request_get_up=True))
        self.assertEqual(early.route, FightRoute.SAFE)
        start = supervisor.update(0.7, ready(robot_stable=True, request_get_up=True))
        self.assertEqual(start.route, FightRoute.REFERENCE)
        self.assertEqual(start.reference_name, "get_up")
        recovering = supervisor.update(
            1.0,
            ready(
                robot_stable=True,
                fall_detected=True,
                get_up_complete=True,
            ),
        )
        self.assertEqual(recovering.state, FightState.RECOVERING)
        self.assertEqual(recovering.route, FightRoute.SAFE)
        restored = supervisor.update(2.1, ready(robot_stable=True))
        self.assertEqual(restored.route, FightRoute.POSE)

    def test_estop_reset_requires_disarmed_stable_robot(self):
        supervisor = FightSupervisor()
        self.assertTrue(supervisor.update(0.0, ready(estop=True)).estop_latched)
        self.assertTrue(
            supervisor.update(0.1, ready(reset_estop=True, robot_stable=True)).estop_latched
        )
        reset = supervisor.update(
            0.2, FightRequest(reset_estop=True, robot_stable=True)
        )
        self.assertFalse(reset.estop_latched)
        self.assertEqual(reset.state, FightState.DISARMED)

    def test_non_finite_request_fails_closed(self):
        output = FightSupervisor().update(0.0, ready(forward_mps=math.nan))
        self.assertEqual(output.route, FightRoute.SAFE)


class FallDetectorTests(unittest.TestCase):
    def test_debounces_fall_and_settle(self):
        detector = FallDetector(fall_dwell_s=0.1, stable_dwell_s=0.2)
        falling = RobotBalanceSample(math.radians(70), 0.8, 1.0)
        self.assertFalse(detector.update(0.0, falling).fallen)
        self.assertTrue(detector.update(0.11, falling).fallen)
        settled = RobotBalanceSample(math.radians(70), 0.3, 0.1)
        self.assertFalse(detector.update(0.2, settled).stable)
        self.assertTrue(detector.update(0.41, settled).stable)

    def test_non_finite_robot_state_fails_as_fallen(self):
        status = FallDetector().update(
            0.0, RobotBalanceSample(math.nan, 0.8, 0.0)
        )
        self.assertTrue(status.fallen)
        self.assertFalse(status.stable)
