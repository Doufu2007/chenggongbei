"""Read-only v25 clock/command/odometry arithmetic; no ROS imports."""
from collections import defaultdict
from datetime import datetime
from pathlib import Path
import bisect
import csv
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent


def identity(path):
    return {"path": path.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    source = ROOT / "runs/analysis/20261112_single_point_v25_independent/summary.json"
    s = json.loads(source.read_text(encoding="utf-8"))
    c = s["series"]["actual_commands"]["records"]
    t = s["series"]["trajectory_10hz"]["records"]
    a = s["survey"]["approach"]
    phase = defaultdict(float)
    for x, y in zip(c, c[1:]):
        phase[x["state"]] += y["wall_elapsed_s"]-x["wall_elapsed_s"]
    stamps = [x["wall_elapsed_s"] for x in c]
    response = defaultdict(lambda: defaultdict(float))
    for x, y in zip(t, t[1:]):
        wall = y["wall_elapsed_s"]-x["wall_elapsed_s"]
        sim = y["stamp_s"]-x["stamp_s"]
        if wall <= 0 or sim <= 0:
            continue
        command = c[max(0, bisect.bisect_right(stamps, (x["wall_elapsed_s"]+y["wall_elapsed_s"])/2)-1)]
        linear, angular = command["linear_x"], command["angular_z"]
        if angular:
            value = abs(angular)
            tier = "turn_0.35" if abs(value-.35)<1e-6 else "turn_0.50" if abs(value-.5)<1e-6 else "turn_between_0.35_0.50"
        elif linear:
            tier = "drive_0.10" if abs(linear-.1)<1e-6 else "drive_0.03" if abs(linear-.03)<1e-6 else "drive_variable"
        else:
            tier = "zero"
        r = response[tier]
        r["sample_intervals"] += 1
        r["wall_s"] += wall
        r["sim_s"] += sim
        r["path_m"] += math.hypot(y["x"]-x["x"], y["y"]-x["y"])
        r["absolute_rotation_rad"] += abs(math.atan2(math.sin(y["yaw_rad"]-x["yaw_rad"]), math.cos(y["yaw_rad"]-x["yaw_rad"])))
    for r in response.values():
        for quantity in ("path_m", "absolute_rotation_rad"):
            r[quantity+"_per_wall_s"] = r[quantity]/r["wall_s"]
            r[quantity+"_per_sim_s"] = r[quantity]/r["sim_s"]
    geometry_path = ROOT/"runs/analysis/20261112_ground_three_point_corridor/report.json"
    geometry = json.loads(geometry_path.read_text(encoding="utf-8"))
    scheme = geometry["schemes"]["preserve_ground0_axis"]
    drive_rate = a["route"]["drive_elapsed_wall_s"]/a["total_path_m"]
    turn_rate = a["route"]["turn_elapsed_wall_s"]/a["accumulated_rotation_rad"]
    sequences = scheme["sequences"]
    predictions = []
    for row in sequences:
        predictions.append({"order": row["order"], "path_m": row["path_m"], "rotation_rad": row["rotation_rad"],
            "turn_wall_s": row["rotation_rad"]*turn_rate, "drive_wall_s": row["path_m"]*drive_rate,
            "scaled_motion_wall_s": row["rotation_rad"]*turn_rate+row["path_m"]*drive_rate})
    worst = max(predictions, key=lambda x:x["scaled_motion_wall_s"])
    lifecycle = s["lifecycle"]
    launcher_wall = (datetime.fromisoformat(lifecycle[-1]["payload_timestamp"])-datetime.fromisoformat(lifecycle[0]["payload_timestamp"])).total_seconds()
    result = {"kind":"v25_observation_based_time_scenario_not_deadline_guarantee",
        "sources":[identity(source), identity(geometry_path),identity(ROOT/"docs/referee_rules_v1.txt")],
        "complete_series":{k: {"complete":v["complete"],"rows":len(v["records"])} for k,v in s["series"].items()},
        "clock": {"guard_wall_s":s["snapshots"]["assembled"]["elapsed_s"],"trajectory_wall_s":t[-1]["wall_elapsed_s"]-t[0]["wall_elapsed_s"],
            "trajectory_sim_s":t[-1]["stamp_s"]-t[0]["stamp_s"],"sim_per_wall":(t[-1]["stamp_s"]-t[0]["stamp_s"])/(t[-1]["wall_elapsed_s"]-t[0]["wall_elapsed_s"]),
            "platform_duration_s":275.179,"launcher_payload_wall_s":launcher_wall,
            "platform_minus_guard_s":275.179-s["snapshots"]["assembled"]["elapsed_s"],
            "launcher_minus_guard_s":launcher_wall-s["snapshots"]["assembled"]["elapsed_s"]},
        "command_state_wall_s":dict(phase),"midpoint_wall_aligned_response":dict(response),
        "v25_accounting":{"path_m":a["total_path_m"],"rotation_rad":a["accumulated_rotation_rad"],
            "turn_elapsed_wall_s":a["route"]["turn_elapsed_wall_s"],"drive_elapsed_wall_s":a["route"]["drive_elapsed_wall_s"],
            "turn_wall_s_per_rad":turn_rate,"drive_wall_s_per_m":drive_rate},
        "sixty_order_scenario":predictions,"worst_scaled_motion":worst,
        "geometry_cap_scenario":{"path_cap_m":12.5,"rotation_cap_rad":25.,"drive_wall_s":12.5*drive_rate,"turn_wall_s":25*turn_rate,
            "motion_wall_s":12.5*drive_rate+25*turn_rate,"extra_per_operation_settle_reserve_s":32.,
            "three_visual_report_reserve_s":45.,"startup_stop_pause_reserve_s":30.},
        "recommended_engineering_caps":{"global_wall_s":1200.,"turn_total_wall_s":700.,"drive_total_wall_s":500.,"single_turn_wall_s":45.,
            "single_drive_wall_s":190.,"capture_recognition_report_per_point_wall_s":15.,
            "not_official_limits":True,"shared_global_ledger":True},
        "official_deadline":"not_found_in_cached_public_rules; platform duration is not a deadline",
        "limitations":["Single v25 run; midpoint time association is descriptive, not actuator latency or causal gain identification.",
            "Integrated odometry and header time are distinct from requested velocity and wall time.",
            "Scaled rates include v25 state overhead but not a proof for other headings, path lengths, load or tracking errors.",
            "Extra 32s settle reserve is conservative double-counting allowance, not a measured three-point value.",
            "No official total deadline found; cannot state calibration is mandatory to meet it.",
            "Do not extrapolate turn command beyond tested .35-.50 range or relax current scan/freshness/stop gates."]}
    (OUT/"time_budget.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    with (OUT/"sixty_orders.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=list(predictions[0]));writer.writeheader();writer.writerows(predictions)
    print(json.dumps({"clock":result["clock"],"v25":result["v25_accounting"],"response":result["midpoint_wall_aligned_response"],"worst":worst,"caps":result["geometry_cap_scenario"]},indent=1))


if __name__ == "__main__":
    main()
