"""Artificial map and kinematics for offline checks; never a platform map."""

from __future__ import annotations

import math
import copy
import json
import sys
from dataclasses import dataclass
from pathlib import Path


_SOURCE = Path(__file__).resolve().parents[1] / "src" / "inspection_mission"
if str(_SOURCE) not in sys.path:
    sys.path.insert(0, str(_SOURCE))


def demonstration_plan(*, protocol="legacy") -> dict:
    """Return invented coordinates for a local controller demonstration only."""
    if protocol not in ("legacy", "inspection_v1"):
        raise ValueError("unsupported demonstration protocol")
    plan = {
        "protocol": protocol,
        "home_ground": [0.0, 0.0, 0.0],
        "home_uav": [0.0, 0.0, 0.0, 0.0],
        "robot_radius_m": 0.08,
        "clearance_m": 0.03,
        "ground_points": [
            {"id": "g1", "target_id": "ground-one", "pose": [0.4, 0.0, 0.0]},
            {"id": "g2", "target_id": "ground-two", "pose": [0.4, 0.4, math.pi / 2]},
            {"id": "g3", "target_id": "ground-three", "pose": [0.0, 0.4, math.pi]},
        ],
        "aerial_points": [
            {"id": "a1", "target_id": "air-one", "pose": [1.3, 0.2, 1.0, 0.0]},
            {"id": "a2", "target_id": "air-two", "pose": [1.3, 0.8, 1.1, math.pi / 2]},
        ],
        "windows": [
            {"id": "w1", "target_id": "window-one", "approach": [0.6, 0.0, 1.2, 0.0],
             "exit": [1.0, 0.0, 0.45, 0.0], "crossing_z": 0.45, "bottom": 0.15, "top": 0.75},
            {"id": "w2", "target_id": "window-two", "approach": [1.0, 0.9, 1.2, math.pi],
             "exit": [0.6, 0.9, 0.45, math.pi], "crossing_z": 0.45, "bottom": 0.15, "top": 0.75},
        ],
    }
    if protocol == "inspection_v1":
        for point in plan["ground_points"]:
            x, y, yaw = point["pose"]
            point["approach"] = [x - 0.24 * math.cos(yaw), y - 0.24 * math.sin(yaw), yaw]
    return plan


@dataclass
class ScenarioResult:
    completed: bool
    fault: str
    steps: int
    ground: object
    uav: object
    states: list[str]
    reports: list[dict]
    trace: list[dict]


def run_scenario(plan=None, settings=None, *, dt=0.1, max_steps=10000,
                 ground_order=None, light_feedback=True, acknowledge=True,
                 feedback_hook=None) -> ScenarioResult:
    """Integrate commanded velocities against artificial observations.

    This is a transparent kinematic plant, not the contest simulator/referee.
    Synthetic camera labels come from nearby demonstration waypoints only.
    ``feedback_hook(context)`` may mutate input observations for fault tests.
    """
    from inspection_mission.control import Pose2D
    from inspection_mission.mission import MissionController

    if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive and finite")
    plan = copy.deepcopy(plan if plan is not None else demonstration_plan())
    mission = MissionController(plan, settings)
    gh, uh = plan["home_ground"], plan["home_uav"]
    ground = Pose2D(x=gh[0], y=gh[1], yaw=gh[2])
    uav = Pose2D(x=uh[0], y=uh[1], z=uh[2], yaw=uh[3])
    order = ground_order if ground_order is not None else [point["id"] for point in plan["ground_points"]]
    states, reports, trace, accepted, seen = [mission.state], [], [], set(), set()
    light_command = False
    first_lit = None
    interrupted = False
    for index in range(max_steps):
        now = index * dt
        ground.stamp = uav.stamp = now
        detections = []
        for vehicle, pose, points, pose_key in (
            ("ground", ground, plan["ground_points"], "approach" if plan.get("protocol") == "inspection_v1" else "pose"),
            ("uav", uav, plan["aerial_points"], "pose"),
            ("uav", uav, plan["windows"], "approach"),
        ):
            for point in points:
                target = point[pose_key]
                near = math.hypot(pose.x - target[0], pose.y - target[1]) < 0.14
                if vehicle == "uav":
                    near = near and abs(pose.z - target[2]) < 0.14
                if near:
                    detections.append({"target_id": point["target_id"], "source": vehicle,
                                       "stamp": now, "center": [0.5, 0.5], "confidence": 0.99})
        ready = bool(light_feedback) and light_command
        if light_feedback == "interrupt_once" and mission.state == "AIR_INSPECT_1":
            if light_command and first_lit is None:
                first_lit = now
            if first_lit is not None and now - first_lit >= 0.6 and not interrupted:
                ready, interrupted = False, True
        context = {"now": now, "ground": ground, "uav": uav, "detections": detections,
                   "light_ready": ready, "controls_ready": True, "ground_clear": True,
                   "uav_clear": True, "recognition_ack": set(accepted), "state": mission.state}
        if feedback_hook:
            feedback_hook(context)
        command = mission.step(now, context["ground"], context["uav"], context["detections"],
                               ground_clear=context["ground_clear"], uav_clear=context["uav_clear"],
                               light_ready=context["light_ready"], controls_ready=context["controls_ready"],
                               recognition_ack=context["recognition_ack"], ground_order=order if index == 0 else None)
        if states[-1] != command.state:
            states.append(command.state)
        trace.append({"now": now, "state": command.state, "ground": copy.copy(ground),
                      "uav": copy.copy(uav), "command": command, "light_ready": context["light_ready"]})
        if command.recognition:
            event = command.recognition
            if event["event_id"] not in seen:
                reports.append(copy.deepcopy(event))
                seen.add(event["event_id"])
            if acknowledge:
                accepted.add(event["event_id"])
        if command.done or command.fault:
            return ScenarioResult(command.done, command.fault, index + 1, ground, uav, states, reports, trace)
        linear, angular = command.ground
        ground.x += linear * math.cos(ground.yaw) * dt
        ground.y += linear * math.sin(ground.yaw) * dt
        ground.yaw += angular * dt
        vx, vy, vz, yawrate = command.uav
        uav.x += vx * dt
        uav.y += vy * dt
        uav.z += vz * dt
        uav.yaw += yawrate * dt
        light_command = command.light
    return ScenarioResult(False, "scenario_step_limit", max_steps, ground, uav, states, reports, trace)


if __name__ == "__main__":
    outcome = run_scenario()
    print(json.dumps({"mode": "offline_kinematics", "completed": outcome.completed,
                      "steps": outcome.steps, "recognition_reports": len(outcome.reports),
                      "states": outcome.states, "fault": outcome.fault}, ensure_ascii=False))
    raise SystemExit(0 if outcome.completed else 1)
