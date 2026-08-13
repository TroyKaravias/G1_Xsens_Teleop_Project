#!/usr/bin/env python3
"""Replay the fight supervisor without networking or robot output."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xsens_bridge.fight_control import FightRequest, FightSupervisor, Punch
from xsens_bridge.sonic_manager import SonicFightAdapter


def main() -> None:
    supervisor = FightSupervisor()
    adapter = SonicFightAdapter()
    timeline = [
        (0.00, FightRequest(), "disarmed"),
        (0.10, FightRequest(armed=True, stream_fresh=True, operator_ready=True), "guard pose"),
        (0.20, FightRequest(armed=True, stream_fresh=True, operator_ready=True, lateral_mps=0.20), "sidestep"),
        (0.30, FightRequest(armed=True, stream_fresh=True, operator_ready=True, yaw_rate_rad_s=0.40), "turn"),
        (0.40, FightRequest(armed=True, stream_fresh=True, operator_ready=True, punch=Punch.LEFT_JAB), "left jab"),
        (1.10, FightRequest(armed=True, stream_fresh=True, operator_ready=True), "punch completes"),
        (1.20, FightRequest(armed=True, stream_fresh=True, operator_ready=True, fall_detected=True), "fall"),
        (1.80, FightRequest(armed=True, stream_fresh=True, operator_ready=True, robot_stable=True, request_get_up=True), "settle begins"),
        (2.40, FightRequest(armed=True, stream_fresh=True, operator_ready=True, robot_stable=True, request_get_up=True), "get up"),
        (4.00, FightRequest(armed=True, stream_fresh=True, operator_ready=True, robot_stable=True, get_up_complete=True), "recovery"),
        (5.10, FightRequest(armed=True, stream_fresh=True, operator_ready=True, robot_stable=True), "guard restored"),
        (5.20, FightRequest(armed=True, stream_fresh=False, operator_ready=True), "stale fail closed"),
    ]
    for now, request, label in timeline:
        output = supervisor.update(now, request)
        row: dict[str, object] = {"time": now, "step": label, **asdict(output)}
        if output.route.value == "PLANNER":
            fields = adapter.planner_fields(output, now)
            row["sonic"] = {
                "mode": int(fields["mode"][0]),
                "movement": fields["movement"].tolist(),
                "facing": fields["facing"].tolist(),
                "speed": float(fields["speed"][0]),
            }
        print(json.dumps(row, default=lambda value: value.value))


if __name__ == "__main__":
    main()
