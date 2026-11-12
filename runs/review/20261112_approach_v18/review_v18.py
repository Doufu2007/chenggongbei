"""Independent offline arithmetic from the immutable v18 analysis and raw identity.

Does not reconstruct a different raw message, run ROS, fit geometry or publish.
Only compare_scan's pose-only argument is composed from each original prior pose.
"""
from __future__ import annotations

import hashlib
import json
import math
import bisect
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
from validate_ground_scan_geometry import compare_scan, load_candidate_world


def ident(path):
    raw = path.read_bytes()
    return {"path": path.relative_to(ROOT).as_posix(), "bytes": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest()}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def xyz(row):
    return tuple(row[k] for k in ("x", "y", "z"))


def point_summary(rows):
    if not rows:
        return None
    origin = xyz(rows[0])
    return {"count": len(rows), "first": rows[0], "last": rows[-1],
            "wall_span_s": rows[-1]["wall_elapsed_s"] - rows[0]["wall_elapsed_s"],
            "stamp_span_s": rows[-1]["stamp_s"] - rows[0]["stamp_s"],
            "max_wall_gap_s": max((b["wall_elapsed_s"] - a["wall_elapsed_s"] for a, b in zip(rows, rows[1:])), default=0),
            "max_stamp_gap_s": max((b["stamp_s"] - a["stamp_s"] for a, b in zip(rows, rows[1:])), default=0),
            "max_linear_speed_mps": max(r["linear_speed"] for r in rows),
            "max_angular_speed_radps": max(r["angular_speed"] for r in rows),
            "max_distance_from_first_m": max(math.dist(xyz(r), origin) for r in rows),
            "max_yaw_difference_from_first_rad": max(abs(wrap(r["yaw_rad"] - rows[0]["yaw_rad"])) for r in rows),
            "sampled_path_m": sum(math.dist(xyz(a), xyz(b)) for a, b in zip(rows, rows[1:]))}


def main():
    analysis = ROOT / "runs/analysis/20261112_approach_v18/summary.json"
    raw = ROOT / "runs/evidence/20261112/callback_approach_v18.jsonl"
    world = ROOT / "external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
    before = {"analysis": ident(analysis), "raw": ident(raw), "world": ident(world)}
    summary = json.loads(analysis.read_text("utf-8"))
    assembled = summary["snapshots"]["assembled"]
    approach = summary["survey"]["approach"]
    commands = summary["series"]["actual_commands"]["records"]
    trajectory = summary["series"]["trajectory_10hz"]["records"]
    for series in summary["series"].values():
        assert series["complete"] and not series["missing_rows"] and not series["conflicted"]
    assert all(b["wall_elapsed_s"] >= a["wall_elapsed_s"] for a,b in zip(commands, commands[1:]))
    assert all(b["stamp_s"] > a["stamp_s"] for a,b in zip(trajectory, trajectory[1:]))
    segments = []
    for index, command in enumerate(commands):
        if not segments or command["state"] != segments[-1]["state"]:
            segments.append({"state": command["state"], "first": command, "count": 0,
                             "linear_min": command["linear_x"], "linear_max": command["linear_x"],
                             "angular_min": command["angular_z"], "angular_max": command["angular_z"]})
        seg = segments[-1]
        seg["last"] = command
        seg["count"] += 1
        seg["linear_min"] = min(seg["linear_min"], command["linear_x"])
        seg["linear_max"] = max(seg["linear_max"], command["linear_x"])
        seg["angular_min"] = min(seg["angular_min"], command["angular_z"])
        seg["angular_max"] = max(seg["angular_max"], command["angular_z"])
    for a,b in zip(segments,segments[1:]):
        a["until_next_state_wall_s"] = b["first"]["wall_elapsed_s"] - a["first"]["wall_elapsed_s"]
        a["until_next_state_clock_s"] = b["first"]["clock_stamp_s"] - a["first"]["clock_stamp_s"]
    by_state = {s["state"]: s for s in segments}
    first_stop = by_state["DRIVE_SETTLE"]["first"]["wall_elapsed_s"]
    capture_start = by_state["CAPTURE"]["first"]["wall_elapsed_s"]
    stop_start = by_state["STOP"]["first"]["wall_elapsed_s"]
    final_start = by_state["FINISHED"]["first"]["wall_elapsed_s"]
    pre_stop = max((r for r in trajectory if r["wall_elapsed_s"] <= first_stop), key=lambda r:r["wall_elapsed_s"])
    post_stop = min((r for r in trajectory if r["wall_elapsed_s"] > first_stop), key=lambda r:r["wall_elapsed_s"])
    after_zero = [r for r in trajectory if r["wall_elapsed_s"] >= first_stop]
    capture_rows = [r for r in trajectory if capture_start <= r["wall_elapsed_s"] <= stop_start]
    stable_rows = [r for r in trajectory if first_stop <= r["wall_elapsed_s"] <= capture_start]
    after_stop_rows = [r for r in trajectory if stop_start <= r["wall_elapsed_s"]]
    stationary = {
        "first_drive_zero_command": by_state["DRIVE_SETTLE"]["first"],
        "all_commands_zero_after_drive_settle": all(c["linear_x"] == 0 and c["angular_z"] == 0 for c in commands if c["wall_elapsed_s"] >= first_stop),
        "bracketing_pre_zero_trajectory": pre_stop, "bracketing_post_zero_trajectory": post_stop,
        "zero_command_to_next_trajectory_wall_s": post_stop["wall_elapsed_s"] - first_stop,
        "bracket_sample_distance_m": math.dist(xyz(pre_stop), xyz(post_stop)),
        "max_sample_distance_from_pre_zero_m": max(math.dist(xyz(pre_stop),xyz(r)) for r in after_zero),
        "max_sample_distance_from_post_zero_m": max(math.dist(xyz(post_stop),xyz(r)) for r in after_zero),
        "settle_before_capture_samples": point_summary(stable_rows),
        "capture_samples": point_summary(capture_rows),
        "stop_to_final_samples": point_summary(after_stop_rows),
        "scope": "10Hz sampled trajectory; not a continuous stopping-distance bound or all odom callbacks"
    }
    captures = [r for r in summary["survey"]["exposure_records"] if r["phase"].startswith("capture_")]
    capture_review = []
    for r in captures:
        pose = r["odom"]; q = pose["quaternion"]
        q_yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        matching_images = [n for n in summary["native_images"] if n["phase"] == r["phase"] and n["stamp_s"] == r["stamp_s"] and n["topic"] == r["topic"]]
        assert len(matching_images) == 1
        image = matching_images[0]
        image_file = analysis.parent / image["file_name"]
        assert ident(image_file)["sha256"] == image["sha256"]
        recent_command = max((c for c in commands if c["wall_elapsed_s"] <= r["wall_elapsed_s"]), key=lambda c:c["wall_elapsed_s"])
        capture_review.append({"exposure_original": r, "native_image": ident(image_file),
                               "native_prior_validation": image["exposure_validation"],
                               "quaternion_norm": math.hypot(*q), "yaw_recomputed_rad": q_yaw,
                               "yaw_recompute_difference_rad": wrap(q_yaw-pose["yaw_rad"]),
                               "target_yaw_error_rad": wrap(q_yaw-approach["target_yaw_rad"]),
                               "drive_yaw_error_rad": wrap(q_yaw-approach["drive_yaw_rad"]),
                               "image_minus_prior_stamp_recomputed_s": r["stamp_s"]-pose["stamp_s"],
                               "capture_after_first_zero_wall_s": r["wall_elapsed_s"]-first_stop,
                               "latest_pre_image_command": recent_command,
                               "image_minus_command_wall_s":r["wall_elapsed_s"]-recent_command["wall_elapsed_s"]})
    pairs = [{"phases": [a["phase"],b["phase"]], "translation_m": math.dist(a["odom"]["position"],b["odom"]["position"]),
              "yaw_difference_rad":abs(wrap(a["odom"]["yaw_rad"]-b["odom"]["yaw_rad"]))}
             for i,a in enumerate(captures) for b in captures[i+1:]]
    drive_origin, drive_yaw = approach["drive_origin"], approach["drive_yaw_rad"]
    drive_samples = [r for r in trajectory if by_state["DRIVE"]["first"]["wall_elapsed_s"] <= r["wall_elapsed_s"] <= capture_start]
    def projected(r):
        dx,dy = r["x"]-drive_origin[0],r["y"]-drive_origin[1]
        return dx*math.cos(drive_yaw)+dy*math.sin(drive_yaw), -dx*math.sin(drive_yaw)+dy*math.cos(drive_yaw)
    motion_review = {"sampled_global_path_m": sum(math.dist(xyz(a),xyz(b)) for a,b in zip(trajectory,trajectory[1:])),
                     "sampled_max_origin_displacement_m": max(math.dist(xyz(r),approach["global_origin"]) for r in trajectory),
                     "sampled_drive_min_forward_m": min(projected(r)[0] for r in drive_samples),
                     "sampled_drive_max_forward_m": max(projected(r)[0] for r in drive_samples),
                     "sampled_drive_max_abs_cross_track_m": max(abs(projected(r)[1]) for r in drive_samples),
                     "sampled_drive_max_heading_error_rad": max(abs(wrap(r["yaw_rad"]-drive_yaw)) for r in drive_samples),
                     "controller_reported_approach": approach,
                     "actual_command_count": len(commands),
                     "actual_command_nonzero_count": sum(c["linear_x"] != 0 or c["angular_z"] != 0 for c in commands),
                     "actual_command_zero_count": sum(c["linear_x"] == 0 and c["angular_z"] == 0 for c in commands),
                     "core_published_nonzero_count": assembled["published_nonzero_count"],
                     "core_published_zero_count": assembled["published_zero_count"],
                     "core_elapsed_s": assembled["elapsed_s"],
                     "last_command_after_core_elapsed_s": commands[-1]["wall_elapsed_s"]-assembled["elapsed_s"]}
    command_times = [c["wall_elapsed_s"] for c in commands]
    intervals = []
    for a,b in zip(trajectory, trajectory[1:]):
        dt = b["wall_elapsed_s"] - a["wall_elapsed_s"]
        sim_dt = b["stamp_s"] - a["stamp_s"]
        index = bisect.bisect_right(command_times, a["wall_elapsed_s"]) - 1
        end_index = bisect.bisect_right(command_times, b["wall_elapsed_s"]) - 1
        if index < 0 or not 0 < dt <= .35 or sim_dt <= 0:
            continue
        if a["wall_elapsed_s"]-command_times[index] > .1:
            continue
        active = commands[index:end_index+1]
        if not all(c["state"] == "DRIVE" and c["linear_x"] == .03 and c["angular_z"] == 0 for c in active):
            continue
        intervals.append({"wall_s":dt,"sim_s":sim_dt,"forward_m":projected(b)[0]-projected(a)[0]})
    motion_review["effective_drive_intervals"] = {"count":len(intervals),
        "wall_s":sum(x["wall_s"] for x in intervals),"sim_s":sum(x["sim_s"] for x in intervals),
        "forward_m":sum(x["forward_m"] for x in intervals),
        "forward_per_wall_s":sum(x["forward_m"] for x in intervals)/sum(x["wall_s"] for x in intervals),
        "forward_per_sim_s":sum(x["forward_m"] for x in intervals)/sum(x["sim_s"] for x in intervals),
        "selection":"adjacent actual 10Hz rows; 0<wall dt<=.35; increasing sim stamp; latest pre-start command age<=.1; all commands throughout interval DRIVE/.03/0; no future command as prior"}
    geometry = load_candidate_world(world)
    scan_review=[]
    for record in summary["survey"]["scan_records"]:
        # compare_scan only needs pose fields. These are the original prior's
        # exact position/quaternion; no stamp changes or interpolation occur.
        prior = record["odom"]
        argument = {"pose": {"pose": {"position": dict(zip(("x","y","z"),prior["position"])),
                                       "orientation": dict(zip(("x","y","z","w"),prior["quaternion"]))}}}
        scan = record["payload"]
        primary = compare_scan(scan, argument, geometry, tolerance_m=.001)
        alternative_x = compare_scan(scan, argument, geometry, tolerance_m=.001,translation=(.1,0,0))
        alternative_yaw = compare_scan(scan, argument, geometry, tolerance_m=.001,yaw=.05)
        scan_stamp=scan["header"]["stamp"]["sec"]+scan["header"]["stamp"]["nanosec"]*1e-9
        finite=[r for r in scan["ranges"] if isinstance(r,(int,float)) and math.isfinite(r) and scan["range_min"] <= r <= scan["range_max"]]
        scan_review.append({"stage":record["stage"],"recorded_wall_elapsed_s":record["recorded_wall_elapsed_s"],
                            "scan_header_original":scan["header"],"prior_odom_original":prior,
                            "scan_minus_prior_recomputed_s":scan_stamp-prior["stamp_s"],
                            "pairing_original_s":record["scan_minus_odom_s"],
                            "quaternion_norm":math.hypot(*prior["quaternion"]),
                            "observed_min_range_m":min(finite), "observed_max_range_m":max(finite),
                            "ring_gate_threshold_m":.447+.114+.06+.02,
                            "ring_gate_observed_margin_m":min(finite)-(.447+.114+.06+.02),
                            "angle_endpoint_difference_rad":scan["angle_min"]+scan["angle_increment"]*(len(scan["ranges"])-1)-scan["angle_max"],
                            "identity_hypothesis":primary,
                            "alternative_x_plus_0_1m_metrics":alternative_x["metrics"],
                            "alternative_yaw_plus_0_05rad_metrics":alternative_yaw["metrics"]})
    timing={}
    for name,sp in assembled["callback_timing"]["spans"].items():
        timing[name]={key:{**v,"mean":v["total"]/v["count"] if v["count"] else None} for key,v in sp.items()}
    result={"schema_version":1,"kind":"v18_independent_offline_review","source_identity":before,
            "mode_original":summary["mode"],"official_scoring":"unverified","task_completion":"unverified","score":None,
            "command_state_segments":segments,"stationary_review":stationary,"capture_review":capture_review,
            "capture_pair_drift":pairs,"motion_review":motion_review,"timing_arithmetic":timing,
            "scan_geometry_hypothesis":{"candidate_source_commit":"82b041043849a5d86e085390bf0df787366d98cd",
              "world_from_odom":{"translation_m":[0,0,0],"yaw_rad":0},
              "fitting":False,"interpolation":False,"pairing":"exact recorded prior odom, original scan header",
              "scope":"static collision boxes and public candidate rplidar pose; no official-map, class-ID, calibration or whole-route proof",
              "stages":scan_review}}
    after={"analysis":ident(analysis),"raw":ident(raw),"world":ident(world)}
    assert before==after
    out=Path(__file__).with_name("independent_metrics.json")
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({"output":str(out.relative_to(ROOT)),"segments":segments,"stationary_review":stationary,
                      "capture_pair_drift":pairs,"motion_review":{k:v for k,v in motion_review.items() if k!='controller_reported_approach'},
                      "scan_metrics":[{k:v for k,v in x.items() if k not in ['identity_hypothesis','prior_odom_original','alternative_x_plus_0_1m_metrics','alternative_yaw_plus_0_05rad_metrics']}|{"primary":x['identity_hypothesis']['metrics'],"consistent":x['identity_hypothesis']['candidate_consistency']} for x in scan_review]},ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
