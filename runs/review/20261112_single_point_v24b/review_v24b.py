"""Recompute observations from v24b raw without ROS or site interaction."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def identity(path):
    raw = path.read_bytes()
    return {"path": path.relative_to(ROOT).as_posix(), "bytes": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest()}


def main():
    summary_path = ROOT / "runs/analysis/20261112_single_point_v24b_independent/summary.json"
    source_path = ROOT / "runs/evidence/20261112/callback_single_point_v24b.jsonl"
    s = json.loads(summary_path.read_text(encoding="utf-8"))
    c = s["snapshots"]["assembled"]
    sp = s["single_point"]
    approach = s["survey"]["approach"]
    commands = s["series"]["actual_commands"]["records"]
    first_stop = next(row for row in commands if row["state"] == "STOP")
    images = []
    for row in s["native_images"]:
        if not row["phase"].startswith("capture_"):
            continue
        image_path = summary_path.parent / row["file_name"]
        pose = row["exposure"]["odom"]
        assert hashlib.sha256(image_path.read_bytes()).hexdigest() == row["sha256"]
        assert row["topic"] == "/uav/camera/image" and row["frame_id"] == "uav/camera_link"
        assert pose["source_topic"] == "/uav/odom" and pose["stamp_s"] <= row["stamp_s"]
        assert row["exposure_validation"] == "valid_prior_pose_observation"
        images.append({"phase": row["phase"], "file": row["file_name"], "sha256": row["sha256"],
            "image_stamp_s": row["stamp_s"], "image_wall_elapsed_s": row["exposure"]["wall_elapsed_s"],
            "prior_odom_stamp_s": pose["stamp_s"], "prior_pose": pose,
            "skew_s": row["stamp_s"] - pose["stamp_s"],
            "waypoint_error_m": math.dist(pose["position"][:2], [.35, 0.])})
    assert len(images) == 3 and len({r["sha256"] for r in images}) == 3
    span = c["callback_timing"]["spans"]["receive:/uav/camera/image"]
    stop_log = next(row for row in c["motion_log"] if row["state"] == "STOP")
    platform = json.loads((ROOT / "runs/evidence/20261112/single_point_v24b_list_after_run.json").read_text(encoding="utf-8"))
    official = next(r for r in platform["data"]["list"] if r["id"] == "e51c52ce-2605-4a4b-acb2-ea9c4a8bbfee")
    result = {"raw": identity(source_path), "analysis": identity(summary_path),
        "platform": {"record_id": official["id"], "runId": official["runId"],
            "submissionNo": official["submissionNo"], "status": official["status"], "score": official["score"],
            "duration_s": int(official["durationMs"]) / 1000., "failedReason": official["failedReason"]},
        "parser_status": s["status"], "parser_partial_reason": "no_ground_motion_full_event",
        "unknown_sections": s["unknown_sections"], "survey_fallback_fragment_count": len(s["survey_fragment_observations"]),
        "record_count": s["record_count"], "series": {key: {"rows": len(value["records"]), "complete": value["complete"]}
            for key, value in s["series"].items()},
        "capture_images": images, "header_capture_deltas_s": [b["image_stamp_s"]-a["image_stamp_s"] for a,b in zip(images,images[1:])],
        "camera_header_delta_s": span["header_delta_s"], "camera_wall_gap_s": span["entry_gap_wall_s"],
        "camera_rate_from_header_hz": span["header_delta_s"]["count"] / span["header_delta_s"]["total"],
        "camera_rate_from_wall_hz": span["entry_gap_wall_s"]["count"] / span["entry_gap_wall_s"]["total"],
        "stop": {"first_actual_zero_command_in_stop": first_stop, "motion_transition": stop_log,
            "last_capture_to_first_stop_command_s": first_stop["wall_elapsed_s"]-images[-1]["image_wall_elapsed_s"],
            "clock_minus_last_capture_s": first_stop["clock_stamp_s"]-images[-1]["image_stamp_s"]},
        "guard_final": {key: c[key] for key in ("state", "reason", "elapsed_s", "errors")},
        "draw": approach["draw"], "in_entry": sp["in_entry"], "match_results": sp["match_results"],
        "attempts": sp["attempts"], "matcher_failures": sp["matcher_failures"], "recognition_status": sp["recognition_status"],
        "route_legs": approach["route"]["legs"], "route_origin": approach["global_origin"],
        "final_odom": c["odom_last"], "command_endpoints": c["command_endpoints"],
        "all_commands_from_last_capture_are_zero": all(row["linear_x"] == row["angular_z"] == 0.
            for row in commands if row["wall_elapsed_s"] >= images[-1]["image_wall_elapsed_s"]),
        "maximum_recorded_x_m": max(row["x"] for row in s["series"]["trajectory_10hz"]["records"]),
        "minimum_recorded_y_m": min(row["y"] for row in s["series"]["trajectory_10hz"]["records"]),
        "official_acceptance": False, "full_mission_completed": False}
    output = Path(__file__).with_name("independent_metrics.json")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(output.relative_to(ROOT)), "raw_sha256": result["raw"]["sha256"],
        "camera_hz": result["camera_rate_from_header_hz"], "last_capture_to_stop_s": result["stop"]["last_capture_to_first_stop_command_s"],
        "official": result["platform"], "capture_count": len(images)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
