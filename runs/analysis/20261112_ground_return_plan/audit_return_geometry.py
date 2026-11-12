"""Read-only public-candidate enumeration; does not send ROS or read referee data."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/inspection_mission"))
from inspection_mission.ground_three_point_axis_plan import _AxisBuilder, plan_ground_leg
from inspection_mission.ground_three_point_plan import _POINTS, CANDIDATE_WORLD_SHA256


def return_operations(actual_pose):
    builder = _AxisBuilder(actual_pose)
    builder.drive((.6, actual_pose[1]), "return_exit_to_spine")
    builder.drive((.6, -.2), "return_north_to_turn_anchor")
    builder.reverse_axis((.6, 0.), "return_reverse_north")
    builder.drive((0., 0.), "return_west_to_home")
    builder.turn(0., "return_final_alignment")
    return tuple(builder.operations), builder.initial_clearance


def main():
    returns = {}
    for point_id, (_, _, xy, yaw) in _POINTS.items():
        operations, initial = return_operations((*xy, yaw))
        returns[point_id] = {
            "path_m": sum(op.length_m for op in operations),
            "rotation_rad": sum(op.rotation_rad for op in operations),
            "minimum_laser_distance_m": min([initial] + [op.minimum_laser_distance_m for op in operations]),
            "operations": [asdict(op) for op in operations],
        }
    rows = []
    for draw in itertools.permutations(_POINTS, 3):
        pose, previous = (0., 0., 0.), None
        path = rotation = 0.
        clearance = math.inf
        operations = []
        for point_id in draw:
            plan = plan_ground_leg(previous, point_id, pose)
            path += plan.path_m
            rotation += plan.rotation_rad
            clearance = min(clearance, plan.minimum_laser_distance_m)
            operations.extend(plan.to_observation_operations)
            operations.extend(plan.entry_operations)
            pose, previous = (*plan.endpoint_xy_m, plan.endpoint_yaw_rad), point_id
        back, initial = return_operations(pose)
        operations.extend(back)
        path += sum(op.length_m for op in back)
        rotation += sum(op.rotation_rad for op in back)
        clearance = min(clearance, initial, *(op.minimum_laser_distance_m for op in back))
        drives = sum(op.kind in ("DRIVE_TO_XY", "DRIVE_REVERSE_TO_XY") for op in operations)
        rows.append({"draw": list(draw), "path_m": path, "rotation_rad": rotation,
                     "minimum_laser_distance_m": clearance, "operation_count": len(operations),
                     "drive_count": drives, "turn_count": len(operations) - drives,
                     "v25_extrapolated_wall_s": 36.0058 * path + 22.9369 * rotation})
    perturbations = []
    for point_id, (_, _, xy, yaw) in _POINTS.items():
        accepted, rejected, minimum = 0, [], math.inf
        for dx, dy, da in itertools.product((-.02, 0., .02), (-.02, 0., .02), (-.025, 0., .025)):
            if math.hypot(dx, dy) > .02 + 1e-12:
                continue
            pose = (xy[0] + dx, xy[1] + dy, yaw + da)
            try:
                ops, initial = return_operations(pose)
                minimum = min(minimum, initial, *(op.minimum_laser_distance_m for op in ops))
                accepted += 1
            except ValueError as exc:
                rejected.append({"offset": [dx, dy, da], "reason": str(exc)})
        perturbations.append({"last_point": point_id, "accepted": accepted, "rejected": rejected,
                              "accepted_minimum_laser_distance_m": minimum})
    sources = {}
    for name in ("ground_three_point_axis_plan.py", "ground_three_point_plan.py",
                 "ground_three_point_axis.py", "ground_three_point_recovery_budget.py"):
        file = ROOT / "src/inspection_mission/inspection_mission" / name
        sources[str(file.relative_to(ROOT))] = hashlib.sha256(file.read_bytes()).hexdigest()
    result = {
        "scope": "public candidate only; nominal geometry, no platform execution or score",
        "world_sha256": CANDIDATE_WORLD_SHA256, "source_sha256": sources,
        "summary": {"draw_count": len(rows), "max_path": max(rows, key=lambda row: row["path_m"]),
                    "max_rotation": max(rows, key=lambda row: row["rotation_rad"]),
                    "max_v25_extrapolated_wall": max(rows, key=lambda row: row["v25_extrapolated_wall_s"]),
                    "minimum_laser_distance_m": min(row["minimum_laser_distance_m"] for row in rows),
                    "maximum_operation_count": max(row["operation_count"] for row in rows),
                    "maximum_drive_count": max(row["drive_count"] for row in rows),
                    "maximum_turn_count": max(row["turn_count"] for row in rows)},
        "return_candidates": returns, "all_draws": rows,
        "sampled_terminal_pose_perturbations_not_a_continuous_proof": perturbations,
    }
    output = Path(__file__).with_name("geometry_audit.json")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"summary": result["summary"], "perturbation_counts": [
        {"last_point": row["last_point"], "accepted": row["accepted"], "rejected": len(row["rejected"])}
        for row in perturbations]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
