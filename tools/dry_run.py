"""Deterministic offline mission simulation for checks on a Windows host."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path


# Allow ``python tools/dry_run.py`` from a source checkout without requiring
# ROS2 or an editable package install.
_SRC_ROOT = Path(__file__).resolve().parents[1] / "src" / "inspection_mission"
if str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

from inspection_mission.control import Pose2D, aerial_command, aerial_reached, ground_command, ground_reached
from inspection_mission.fsm import InspectionFSM, MissionState


@dataclass
class DryRunResult:
    completed: bool
    steps: int
    final_ground: Pose2D
    final_aerial: Pose2D
    states: list[MissionState]
    fault_reason: str = ""


def run_dry_run(
    ground_waypoints,
    aerial_waypoints,
    dt: float = 0.1,
    max_steps: int = 10000,
    ground_return_waypoint=None,
    aerial_return_waypoint=None,
) -> DryRunResult:
    fsm = InspectionFSM(phase_timeout_s=90.0)
    ground = Pose2D(stamp=0.0)
    aerial = Pose2D(stamp=0.0)
    ground_index = aerial_index = 0
    states = [fsm.state]
    now = 0.0
    fsm.tick(now, ready=True, pose_valid=True, control_ok=True)
    states.append(fsm.state)

    for step in range(max_steps):
        now += dt
        if fsm.state is MissionState.GROUND_SCAN:
            vx, wz = ground_command(ground, ground_waypoints[ground_index], 0.18, 0.28, 0.55)
            ground.x += vx * math.cos(ground.yaw) * dt
            ground.y += vx * math.sin(ground.yaw) * dt
            ground.yaw += wz * dt
            if ground_reached(ground, ground_waypoints[ground_index], 0.18):
                ground_index += 1
                if ground_index >= len(ground_waypoints):
                    fsm.mark_ground_complete(now)
        elif fsm.state is MissionState.AERIAL_SCAN:
            vx, vy, vz = aerial_command(aerial, aerial_waypoints[aerial_index], 0.18, 0.35)
            aerial.x += vx * dt
            aerial.y += vy * dt
            aerial.z += vz * dt
            if aerial_reached(aerial, aerial_waypoints[aerial_index], 0.18):
                aerial_index += 1
                if aerial_index >= len(aerial_waypoints):
                    fsm.mark_aerial_complete(now)
        elif fsm.state is MissionState.RETURN:
            ground_home = ground_return_waypoint or ground_waypoints[-1]
            aerial_home = aerial_return_waypoint or (aerial_waypoints[-1][0], aerial_waypoints[-1][1], 0.0)
            ground_done = ground_reached(ground, ground_home, 0.18)
            if not ground_done:
                vx, wz = ground_command(ground, ground_home, 0.18, 0.28, 0.55)
                ground.x += vx * math.cos(ground.yaw) * dt
                ground.y += vx * math.sin(ground.yaw) * dt
                ground.yaw += wz * dt
                ground_done = ground_reached(ground, ground_home, 0.18)
            aerial_done = aerial_reached(aerial, aerial_home, 0.18)
            if not aerial_done:
                vx, vy, vz = aerial_command(aerial, aerial_home, 0.18, 0.35)
                aerial.x += vx * dt
                aerial.y += vy * dt
                aerial.z += vz * dt
                aerial_done = aerial_reached(aerial, aerial_home, 0.18)
            if ground_done and aerial_done:
                fsm.mark_return_complete(now)
        fsm.tick(now, ready=True, pose_valid=True, control_ok=True)
        if not states or states[-1] is not fsm.state:
            states.append(fsm.state)
        if fsm.state in (MissionState.COMPLETE, MissionState.FAILSAFE):
            return DryRunResult(fsm.state is MissionState.COMPLETE, step + 1, ground, aerial, states, fsm.fault_reason)
    return DryRunResult(False, max_steps, ground, aerial, states, "dry_run_step_limit")


if __name__ == "__main__":
    result = run_dry_run(
        [(0.8, 0.0, 0.0), (0.8, 1.0, 1.57), (0.0, 1.0, 3.14)],
        [(0.8, 0.0, 1.05), (0.8, 1.0, 1.05), (0.0, 1.0, 1.05), (0.0, 0.0, 1.05)],
    )
    print(f"completed={result.completed} steps={result.steps} state={result.states[-1].value}")
    raise SystemExit(0 if result.completed else 1)

