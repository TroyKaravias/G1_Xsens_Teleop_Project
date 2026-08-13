import json
import math
import unittest

import numpy as np

from xsens_bridge.locomotion_control import LocomotionEvent
from xsens_bridge.mode_control import ModeEvent, ModeSupervisor
from xsens_bridge.sonic_manager import (
    SonicFightAdapter,
    SonicLocomotionMode,
    mode_intent_to_planner,
    pack_command_message,
    pack_planner_message,
)
from xsens_bridge.fight_control import (
    FightManeuver,
    FightOutput,
    FightRoute,
    FightState,
)
from xsens_bridge.sonic_zmq import SONIC_ZMQ_HEADER_SIZE


def decode_header(message: bytes, topic: str):
    start = len(topic)
    raw = message[start : start + SONIC_ZMQ_HEADER_SIZE].rstrip(b"\x00")
    return json.loads(raw)


class SonicManagerTests(unittest.TestCase):
    def test_command_schema(self):
        message = pack_command_message(start=True, stop=False, planner=True)
        self.assertTrue(message.startswith(b"command"))
        header = decode_header(message, "command")
        self.assertEqual(
            [field["name"] for field in header["fields"]],
            ["start", "stop", "planner"],
        )

    def test_planner_schema(self):
        message = pack_planner_message(
            mode=SonicLocomotionMode.SLOW_WALK,
            movement=(1.0, 0.0, 0.0),
            facing=(1.0, 0.0, 0.0),
            speed=0.2,
        )
        self.assertTrue(message.startswith(b"planner"))
        header = decode_header(message, "planner")
        self.assertEqual(
            [field["name"] for field in header["fields"]],
            ["mode", "movement", "facing", "speed", "height"],
        )

    def test_moving_keeps_upper_body_reference(self):
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
        fields = mode_intent_to_planner(
            intent, upper_body_position=np.zeros(29, dtype=np.float32)
        )
        self.assertEqual(int(fields["mode"][0]), SonicLocomotionMode.SLOW_WALK)
        self.assertAlmostEqual(float(fields["speed"][0]), 0.2)
        self.assertIn("upper_body_position", fields)

    def test_safe_intent_is_idle_without_upper_body(self):
        intent = ModeSupervisor().update(
            0.0, stream_fresh=True, operator_ready=True
        )
        fields = mode_intent_to_planner(
            intent, upper_body_position=np.ones(29, dtype=np.float32)
        )
        self.assertEqual(int(fields["mode"][0]), SonicLocomotionMode.IDLE)
        self.assertNotIn("upper_body_position", fields)

    def test_fight_adapter_integrates_heading_across_full_turns(self):
        adapter = SonicFightAdapter()
        output = FightOutput(
            state=FightState.LOCOMOTION,
            route=FightRoute.PLANNER,
            maneuver=FightManeuver.WALK_BOXING,
            yaw_rate_rad_s=0.5,
        )
        adapter.planner_fields(output, 0.0)
        for index in range(1, 201):
            fields = adapter.planner_fields(output, index * 0.1)
        self.assertGreater(adapter.heading_rad, 2.0 * math.pi)
        self.assertTrue(np.all(np.isfinite(fields["facing"])))
        self.assertAlmostEqual(
            float(np.linalg.norm(fields["facing"][:2])), 1.0, places=6
        )

    def test_fight_adapter_maps_jab_and_rejects_nonplanner_route(self):
        adapter = SonicFightAdapter()
        jab = FightOutput(
            state=FightState.BOXING,
            route=FightRoute.PLANNER,
            maneuver=FightManeuver.RIGHT_JAB,
        )
        fields = adapter.planner_fields(jab, 0.0)
        self.assertEqual(int(fields["mode"][0]), SonicLocomotionMode.RIGHT_JAB)
        hook = FightOutput(
            state=FightState.BOXING,
            route=FightRoute.PLANNER,
            maneuver=FightManeuver.LEFT_HOOK,
        )
        fields = adapter.planner_fields(hook, 0.05)
        self.assertEqual(int(fields["mode"][0]), 15)
        pose = FightOutput(
            state=FightState.POSE,
            route=FightRoute.POSE,
            maneuver=FightManeuver.IDLE_BOXING,
        )
        with self.assertRaises(ValueError):
            adapter.planner_fields(pose, 0.1)
