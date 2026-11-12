"""Native survey images and motion series must be reassembled as evidence only."""
from __future__ import annotations

import base64
import copy
import hashlib
import importlib
import importlib.util
import io
import json

from PIL import Image
import pytest

from test_analyze_ground_motion_evidence import event, write_events, core_snapshot


MODE = "ground_observation_survey"
COMMON = {"mode": MODE, "read_only": False, "official_scoring": "unverified", "partial": True}
TCOLS = ["wall_elapsed_s", "stamp_s", "x", "y", "z", "qx", "qy", "qz", "qw", "yaw_rad", "linear_speed", "angular_speed"]
CCOLS = ["wall_elapsed_s", "clock_stamp_s", "linear_x", "angular_z", "state"]


def api():
    name = "tools.analyze_ground_survey_evidence"
    assert importlib.util.find_spec(name) is not None, "ground survey evidence analyzer is missing"
    return importlib.import_module(name)


def core():
    value = core_snapshot(); value["mode"] = MODE; value["startup"]["mode"] = MODE
    value["state"], value["reason"] = "FINISHED", "target_yaw_reached"
    return value


def metadata():
    pose = {"stamp_s": 10., "wall_elapsed_s": 2., "frame_id": "odom", "child_frame_id": "uav/base_link",
            "position": [0., 0., .055], "quaternion": [0., 0., 0., 1.], "yaw_rad": 0.,
            "linear_speed": 0., "angular_speed": 0., "association": "prior_uav_odom", "image_minus_odom_s": .02}
    return {**COMMON, "section": "survey_metadata", "survey": {
        "draw": ["ground_1", "ground_4", "ground_0"], "target_id": "ground_1", "target_xy": [0., -1.35],
        "candidate_mapping": {"status": "candidate_unverified", "official_id_mapping_confirmed": False},
        "trajectory_dropped": 0, "actual_commands_dropped": 0, "task_completion": "unverified", "image_errors": [],
        "exposure_records": [{"topic": "/uav/camera/image", "phase": "last", "stamp_s": 10.02,
            "frame_id": "uav/camera_link", "wall_elapsed_s": 2.02, "association_error": None, "odom": pose}]}}


def image_parts():
    buffer = io.BytesIO(); Image.new("RGB", (800, 640), (50, 90, 140)).save(buffer, format="JPEG", quality=75)
    raw = buffer.getvalue(); encoded = base64.b64encode(raw).decode("ascii")
    chunks = [encoded[:4000], encoded[4000:]]
    parts = [{**COMMON, "section": "survey_image_part", "topic": "/uav/camera/image", "phase": "last",
        "stamp_s": 10.02, "frame_id": "uav/camera_link", "width": 800, "height": 640, "encoding": "jpeg",
        "jpeg_sha256": hashlib.sha256(raw).hexdigest(), "part_index": index, "part_count": len(chunks),
        "jpeg_base64_part": chunk} for index, chunk in enumerate(chunks)]
    return parts, raw


def series(name, rows, offset=0, total=None):
    return {**COMMON, "section": "survey_motion_series", "series": name,
        "columns": TCOLS if name == "trajectory_10hz" else CCOLS, "offset": offset,
        "total_rows": len(rows) if total is None else total, "rows": rows}


def analyze(tmp_path, payloads):
    source = write_events(tmp_path / "survey.jsonl", [event("probe_snapshot", p) for p in payloads])
    return api().analyze_evidence(source, tmp_path / "out")


def test_native_jpeg_and_series_are_exactly_reassembled_with_exposure_provenance(tmp_path):
    parts, raw = image_parts()
    row = [2., 10., 0., 0., .055, 0., 0., 0., 1., 0., 0., .1]
    result = analyze(tmp_path, [core(), metadata(), series("trajectory_10hz", [row]),
        series("actual_commands", [[2., 10., 0., -.15, "TURN"], [2.05, 10.05, 0., 0., "STOP"]]), *reversed(parts)])
    assert result["status"] == "partial" and result["parse_errors"] == []
    assert result["mode"] == MODE and result["read_only"] is False
    assert result["official_scoring"] == result["task_completion"] == "unverified" and result["score"] is None
    assert result["motion"]["state"] == "FINISHED"
    assert result["series"]["trajectory_10hz"]["records"][0]["yaw_rad"] == 0.
    assert result["series"]["actual_commands"]["records"][0]["angular_z"] == -.15
    assert result["series"]["actual_commands"]["complete"] is True
    image = result["native_images"][0]
    assert (image["width"], image["height"]) == (800, 640)
    assert (tmp_path / "out" / image["file_name"]).read_bytes() == raw
    assert image["sha256"] == hashlib.sha256(raw).hexdigest()
    assert image["exposure"]["odom"]["child_frame_id"] == "uav/base_link"
    assert image["exposure_validation"] == "valid_prior_pose_observation"
    assert result["motion"]["pulse_response_causality"] == "unverified"
    assert result["survey"]["candidate_mapping"]["official_id_mapping_confirmed"] is False
    assert len(result["raw_parts"]) == 6


def test_missing_image_part_and_series_tail_remain_explicit_without_placeholder(tmp_path):
    parts, _ = image_parts()
    result = analyze(tmp_path, [metadata(), parts[0], series("actual_commands", [[0., None, 0., 0., "WAIT"]], total=3)])
    assert result["native_images"] == [] and not list((tmp_path / "out").glob("*.jpg"))
    assert result["image_groups"][0]["missing_parts"] == [1]
    assert result["series"]["actual_commands"]["missing_rows"] == [1, 2]
    assert result["series"]["actual_commands"]["complete"] is False


@pytest.mark.parametrize("change", [{"jpeg_sha256": "a" * 64}, {"width": 799}, {"encoding": "png"},
    {"part_index": True}, {"part_count": 0}, {"jpeg_base64_part": "not base64!"}])
def test_native_image_hash_shape_or_chunk_contract_failure_never_saves_image(tmp_path, change):
    parts, _ = image_parts()
    for part in parts: part.update(change)
    result = analyze(tmp_path, parts)
    assert result["status"] == "invalid" and result["native_images"] == []
    assert result["parse_errors"] or result["assembly_errors"]


def test_conflicting_duplicate_image_part_and_series_row_are_retained(tmp_path):
    parts, _ = image_parts(); conflict = copy.deepcopy(parts[0])
    conflict["jpeg_base64_part"] = base64.b64encode(b"different" * 20).decode("ascii")
    result = analyze(tmp_path, [parts[0], conflict, parts[1],
        series("actual_commands", [[0., None, 0., -.1, "TURN"]]),
        series("actual_commands", [[0., None, 0., 0., "STOP"]])])
    assert len(result["raw_parts"]) == 5
    assert result["conflicts"] and result["native_images"] == []
    assert result["series"]["actual_commands"]["complete"] is False


@pytest.mark.parametrize("change", [{"offset": -1}, {"total_rows": True}, {"offset": 1},
    {"columns": ["wrong"]}, {"rows": [[0., None, 0., "TURN"]]}])
def test_motion_series_contract_rejects_bad_offsets_counts_columns_and_row_width(tmp_path, change):
    value = series("actual_commands", [[0., None, 0., -.1, "TURN"]]); value.update(change)
    result = analyze(tmp_path, [value])
    assert result["status"] == "invalid" and result["parse_errors"][0]["line"] == 1


@pytest.mark.parametrize("fault", ["wrong_body", "future_odom", "old_odom", "association_error"])
def test_image_bytes_do_not_imply_valid_exposure_pose(tmp_path, fault):
    parts, _ = image_parts(); meta = metadata(); exposure = meta["survey"]["exposure_records"][0]
    if fault == "wrong_body": exposure["odom"]["child_frame_id"] = "base_footprint"
    elif fault == "future_odom": exposure["odom"]["stamp_s"] = 10.03
    elif fault == "old_odom": exposure["odom"]["stamp_s"] = 9.9
    else: exposure["association_error"] = "clock_epoch_invalid"
    result = analyze(tmp_path, [meta, *parts])
    assert len(result["native_images"]) == 1
    assert result["native_images"][0]["exposure_validation"] == "invalid_or_unverified_pose_association"


def test_absent_input_is_pending_and_wrong_mode_remains_invalid(tmp_path):
    result = api().analyze_evidence(tmp_path / "absent", tmp_path / "out")
    assert result["status"] == "pending" and result["native_images"] == []
    bad = core(); bad["mode"] = "bounded_ground_motion"
    assert analyze(tmp_path, [bad])["status"] == "invalid"


@pytest.mark.parametrize("association_error", [None, "prior_odom_too_old"])
def test_last_associated_native_phase_preserves_real_stamp_and_revalidates_pose(tmp_path, association_error):
    parts, raw = image_parts()
    for part in parts:
        part["phase"] = "last_associated"
    meta = metadata()
    exposure = meta["survey"]["exposure_records"][0]
    exposure["phase"] = "last_associated"
    exposure["association_error"] = association_error
    result = analyze(tmp_path, [meta, *parts])
    assert result["parse_errors"] == []
    assert len(result["native_images"]) == 1
    image = result["native_images"][0]
    assert image["phase"] == "last_associated" and image["stamp_s"] == 10.02
    assert (tmp_path / "out" / image["file_name"]).read_bytes() == raw
    assert image["exposure"] == exposure
    assert image["exposure_validation"] == ("valid_prior_pose_observation" if association_error is None
                                               else "invalid_or_unverified_pose_association")
