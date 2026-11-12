"""Read-only v26 public trajectory/command replay; no simulation or control."""
from pathlib import Path
import hashlib
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[3]
BUNDLE = Path(__file__).with_name("bundle.json")


def diagnose():
    b = json.loads(BUNDLE.read_text(encoding="utf-8"))
    survey, a = b["survey"], b["survey"]["approach"]
    origin, yaw = a["drive_origin"], a["drive_yaw_rad"]
    rows = []
    for sample in survey["trajectory_10hz"]:
        dx, dy = (sample["position"][i]-origin[i] for i in (0, 1))
        rows.append({"stamp_s": sample["stamp_s"], "wall_s": sample["wall_elapsed_s"],
            "xy": sample["position"][:2], "yaw_rad": sample["yaw_rad"],
            "forward_m": dx*math.cos(yaw)+dy*math.sin(yaw),
            "cross_m": -dx*math.sin(yaw)+dy*math.cos(yaw)})
    windows = {}
    for name, start, end in (("slow_tier", 13.6, 14.4), ("fast_before_calibration", 14.6, 15.24),
                             ("fast_after_calibration", 16.46, 17.16)):
        samples = [row for row in rows if start-1e-9 <= row["stamp_s"] <= end+1e-9]
        first, last = samples[0], samples[-1]
        dx, dy = (last["xy"][i]-first["xy"][i] for i in (0, 1))
        bearing = math.atan2(dy, dx)
        body_yaw = statistics.mean(row["yaw_rad"] for row in samples)
        windows[name] = {"start_stamp_s": first["stamp_s"], "end_stamp_s": last["stamp_s"],
            "dx_m": dx, "dy_m": dy, "displacement_m": math.hypot(dx, dy),
            "displacement_bearing_rad": bearing, "mean_pose_yaw_rad": body_yaw,
            "bearing_minus_pose_yaw_rad": math.atan2(math.sin(bearing-body_yaw), math.cos(bearing-body_yaw)),
            "mean_world_velocity_mps": [dx/(last["stamp_s"]-first["stamp_s"]),
                                         dy/(last["stamp_s"]-first["stamp_s"])]}
    at_drive = a["three_point"]["operation_records"][1]
    target = at_drive["operation"]["target_xy_m"]
    calculated_bearing = math.atan2(target[1]-origin[1], target[0]-origin[0])
    crossings = [row for row in rows if row["wall_s"] >= at_drive["started_wall_s"]-a["first_motion_wall_s"]
                 and row["cross_m"] > .02]
    return {"kind": "v26_actual_cross_track_read_only_diagnosis", "bundle_sha256": hashlib.sha256(BUNDLE.read_bytes()).hexdigest(),
        "session_id": b["session_id"], "reason": a["reason"], "draw": a["three_point"]["mission_draw"],
        "drive_origin": origin, "stored_drive_yaw_rad": yaw, "target_xy": target,
        "independently_calculated_drive_bearing_rad": calculated_bearing,
        "drive_bearing_difference_rad": yaw-calculated_bearing,
        "first_sample_over_2cm": crossings[0], "windows": windows,
        "trajectory_rows": len(rows), "command_rows": len(survey["actual_commands"]),
        "trajectory_dropped": survey["trajectory_dropped"], "commands_dropped": survey["actual_commands_dropped"],
        "full_moving_twist_xyz_available": False,
        "confirmed": "real positional deviation crossed the unchanged guard; anchors are current first-drive anchors",
        "unresolved": "cannot distinguish contact slip, public bridge transform, or odom semantics from pose and speed norms alone"}


if __name__ == "__main__":
    result = diagnose()
    Path(__file__).with_name("cross_track_diagnosis.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=True, indent=2))
