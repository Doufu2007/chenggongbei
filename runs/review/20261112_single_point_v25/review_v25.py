"""Independent source/transport arithmetic for the first officially scored point."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def identity(path):
    raw = path.read_bytes()
    return {"path": path.relative_to(ROOT).as_posix(), "bytes": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest()}


def stamp(header):
    return header["stamp"]["sec"] + header["stamp"]["nanosec"] / 1e9


def main():
    summary_path = ROOT / "runs/analysis/20261112_single_point_v25_independent/summary.json"
    s = json.loads(summary_path.read_text(encoding="utf-8"))
    c, sp = s["snapshots"]["assembled"], s["single_point"]
    approach = s["survey"]["approach"]
    platform = json.loads((ROOT / "runs/evidence/20261112/single_point_v25_list_final.json").read_text(encoding="utf-8"))
    official = next(row for row in platform["data"]["list"] if row["id"] == "c4202874-407a-4361-844a-0f95ba7b1d7f")
    assert official["status"] == "success" and float(official["score"]) == 10.
    assert sp["recognition"]["first_id"] == approach["draw"][0] == "ground_0"
    assert len(sp["match_results"]) == 3
    captures = {row["stamp_s"]: row for row in s["native_images"] if row["phase"].startswith("capture_")}
    assert len(captures) == 3
    manifest_path = ROOT / "src/inspection_mission/inspection_mission/assets/ground_templates/manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    template = next(row for row in manifest["templates"] if row["file"] == "trapped_people_01.png")
    assert hashlib.sha256((manifest_path.parent / template["file"]).read_bytes()).hexdigest() == template["sha256"]
    checks = []
    for result in sp["match_results"]:
        job = result["job"]
        image = captures[result["image_stamp_s"]]
        image_path = summary_path.parent / image["file_name"]
        assert hashlib.sha256(image_path.read_bytes()).hexdigest() == image["sha256"] == job["source_sha256"] == result["source_sha256"]
        assert job["image_stamp_s"] == image["stamp_s"]
        assert job["pose"] == image["exposure"]["odom"] == result["pose"]
        assert job["topic"] == image["topic"] == "/uav/camera/image"
        assert job["frame_id"] == image["frame_id"] == "uav/camera_link"
        assert job["session_id"] == sp["session_id"] and job["frozen_first_ID"] == "ground_0"
        assert job["stationary_epoch"] == sp["stationary_epoch"] == 1
        assert result["template_sha256"] == template["sha256"] and result["class_id"] == "trapped_people"
        assert job["source_ros_encoding"] == "rgb8" and job["source_ros_stride"] == 2400
        assert len(job["source_ros_sha256"]) == 64
        assert [value for row in job["K"] for value in row] == job["camera_info"]["k"]
        assert stamp(job["camera_info"]["header"]) <= image["stamp_s"]
        assert job["camera_info"]["header"]["frame_id"] == job["frame_id"]
        assert job["camera_info"]["width"] == job["camera_info"]["height"] == image["width"] == image["height"] == 800
        assert image["exposure_validation"] == "valid_prior_pose_observation"
        checks.append({"job_seq": job["job_seq"], "phase": image["phase"], "image_stamp_s": image["stamp_s"],
            "prior_odom_stamp_s": job["pose"]["stamp_s"], "image_minus_prior_s": image["stamp_s"]-job["pose"]["stamp_s"],
            "source_jpeg": identity(image_path), "source_ros_sha256_observed": job["source_ros_sha256"],
            "raw_ros_hash_independent_recompute": "unavailable_original_raw_ros_bytes_not_transported",
            "camera_info_stamp_s": stamp(job["camera_info"]["header"]), "camera_info": job["camera_info"],
            "K": job["K"], "pose": job["pose"], "source_ros_encoding": job["source_ros_encoding"],
            "source_ros_stride": job["source_ros_stride"], "source_jpeg_derivation": job["source_jpeg_derivation"],
            "class_id": result["class_id"], "template_sha256": result["template_sha256"],
            "world_center_xy_m": result["world_center_xy_m"],
            "candidate_center_error_m": math.dist(result["world_center_xy_m"], [1.125, -1.35]),
            "worker": result["worker"]})
    for name, topic in (("recognition", "/inspection/detections"), ("declaration", "/referee/current_target")):
        assert sp["attempts"][name]["attempted"] is True and sp["attempts"][name]["sent"] is True
        assert c["command_endpoints"][topic]["actual_publish_attempt_count"] == 1
        assert c["command_endpoints"][topic]["actual_publish_return_count"] == 1
    assert sp["attempts"]["declaration"]["data"] == "ground_0"
    feedback = sp["recognition_status"]
    assert len(feedback) == 1 and feedback[0]["data"] == "ground_0"
    assert hashlib.sha256(feedback[0]["data"].encode()).hexdigest() == feedback[0]["original_sha256"]
    commands = s["series"]["actual_commands"]["records"]
    trajectories = s["series"]["trajectory_10hz"]["records"]
    legs = approach["route"]["legs"]
    assert len(legs) == 4
    result = {"raw": identity(ROOT / "runs/evidence/20261112/callback_single_point_v25.jsonl"),
        "analysis": identity(summary_path),
        "official_list": identity(ROOT / "runs/evidence/20261112/single_point_v25_list_final.json"),
        "official_screenshot": identity(ROOT / "runs/evidence/20261112/single_point_v25_official_success.png"),
        "official_result": {"record_id": official["id"], "runId": official["runId"], "submissionNo": official["submissionNo"],
            "status": official["status"], "total": float(official["score"]), "duration_s": int(official["durationMs"])/1000.,
            "ground_0_score": 10., "item_evidence": "independently viewed official screenshot: ground_0 到达并计分 +10"},
        "parser_status": s["status"], "parser_missing_snapshot_fields": s["missing_snapshot_fields"],
        "parser_full_snapshot_count": s["snapshots"]["full_count"],
        "parser_score_remains_null": s["score"] is None,
        "series": {key: {"rows": len(value["records"]), "complete": value["complete"]} for key,value in s["series"].items()},
        "draw": approach["draw"], "captures_and_match_jobs": checks,
        "maximum_matched_center_disagreement_m": max(math.dist(a["world_center_xy_m"], b["world_center_xy_m"])
            for a in sp["match_results"] for b in sp["match_results"]),
        "attempts": sp["attempts"], "recognition_status_original": feedback,
        "feedback_after_recognition_publish_record_s": feedback[0]["wall_s"]-sp["attempts"]["recognition"]["wall_s"],
        "declaration_after_recognition_publish_record_s": sp["attempts"]["declaration"]["wall_s"]-sp["attempts"]["recognition"]["wall_s"],
        "body_fully_inside_source_claim": sp["body_fully_inside"], "route_legs": legs,
        "final_path_m": approach["total_path_m"], "path_budget_m": sp["plan"]["budgets"]["max_path_m"],
        "final_rotation_rad": approach["accumulated_rotation_rad"], "rotation_budget_rad": sp["plan"]["budgets"]["max_rotation_rad"],
        "turn_elapsed_wall_s": approach["route"]["turn_elapsed_wall_s"],
        "drive_elapsed_wall_s": approach["route"]["drive_elapsed_wall_s"],
        "resume_count": approach["resume_count"], "guard_elapsed_s": c["elapsed_s"],
        "guard_state": c["state"], "guard_reason": c["reason"], "guard_errors": c["errors"],
        "declaration_position": sp["attempts"]["declaration"]["position"],
        "declaration_to_candidate_center_m": math.dist(sp["attempts"]["declaration"]["position"][:2], [1.125,-1.35]),
        "final_odom": c["odom_last"],
        "last_trajectory": trajectories[-1], "last_command": commands[-1],
        "command_endpoints": c["command_endpoints"],
        "official_full_mission_completed": False,
        "settlement_conclusion": "this finite first-point run settled successfully without a full-mission completion publish; no universal exit guarantee"}
    output = Path(__file__).with_name("independent_metrics.json")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(output.relative_to(ROOT)), "official": result["official_result"],
        "raw_sha256": result["raw"]["sha256"], "capture_job_pairs_verified": len(checks),
        "center_spread_m": result["maximum_matched_center_disagreement_m"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
