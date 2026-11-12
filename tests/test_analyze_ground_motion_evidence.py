"""The active probe analyzer must preserve evidence without inventing success."""
from __future__ import annotations

import base64
import copy
import hashlib
import importlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys

import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]


def analyze(source, output):
    name = "tools.analyze_ground_motion_evidence"
    assert importlib.util.find_spec(name) is not None, "active ground-motion evidence analyzer is missing"
    return importlib.import_module(name).analyze_evidence(source, output)


def event(name, data, **extra):
    return {"received_at": "2026-11-12T01:00:01+00:00", "source": "unverified_callback",
            "payload": {"event": name, "timestamp": "2026-11-12T01:00:00+00:00", "data": data}, **extra}


def write_events(path, events):
    path.write_text("".join(json.dumps(item, ensure_ascii=False) + "\n" for item in events), encoding="utf-8")
    return path


def odom(stamp, x):
    return {"header": {"stamp": {"sec": stamp, "nanosec": 0}, "frame_id": "odom"},
            "child_frame_id": "base_footprint",
            "pose": {"pose": {"position": {"x": x, "y": 0., "z": .055},
                              "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}},
            "twist": {"twist": {"linear": {"x": 0., "y": 0., "z": 0.},
                                  "angular": {"x": 0., "y": 0., "z": 0.}}}}


def full_snapshot():
    limits = {"speed_mps": .03, "pulse_wall_s": 1., "max_displacement_m": .06,
              "sensor_age_s": .2, "stable_s": 1., "wait_wall_s": 20., "stop_wall_s": 2.,
              "robot_radius_m": .447, "sensor_offset_bound_m": .114, "clearance_m": .02,
              "bounds_source": "explicit public candidate; official bounds unverified"}
    endpoints = {"/cmd_vel": {"declared_type": "geometry_msgs/msg/Twist", "silent": False,
                 "matched_subscriptions": 1, "subscriber_endpoints_omitted": 0,
                 "subscriber_endpoints": [{"node": "/public_bridge", "type": "geometry_msgs/msg/Twist",
                                            "reliability": "RELIABLE", "durability": "VOLATILE"}]}}
    startup = {"mode": "bounded_ground_motion", "read_only": False, "duration_s": 45.,
               "official_scoring": "unverified", "node": "inspection_bounded_ground_probe", "limits": limits}
    return {"mode": "bounded_ground_motion", "read_only": False, "official_scoring": "unverified",
            "section": "ground_motion_full", "partial": False, "startup": startup,
            "elapsed_s": 23., "state": "FINISHED", "reason": "pulse_time_limit", "limits": limits,
            "command_endpoints": endpoints, "command_endpoint_history": [{"wall_elapsed_s": 0., "endpoints": endpoints}],
            "command_endpoint_history_dropped": 0, "sample_counts": {"/clock": 30, "/odom": 100, "/scan": 30},
            "odom_first": odom(2, 0.), "odom_last": odom(23, .025),
            "odometry": {"/odom": {"first": odom(2, 0.), "last": odom(23, .025)}},
            "public_messages": {"/referee/ground_tasks": {"first": {"data": "ground_1,ground_4,ground_0"},
                                                          "last": {"data": "ground_1,ground_4,ground_0"}}},
            "tf_edges": {"0": {"parent_frame": "base_footprint", "child_frame_id": "laser_link"}},
            "max_observed_displacement_m": .027, "published_nonzero_count": 20, "published_zero_count": 120,
            "motion_log": [{"wall_elapsed_s": 1., "state": "PULSE", "reason": "bounded_ground_pulse", "linear_x": .03}],
            "motion_log_dropped": 0, "errors": [], "image_records": []}


def core_snapshot(full=None):
    full = full or full_snapshot()
    omitted = {"command_endpoint_history", "odometry", "public_messages", "tf_edges", "motion_log", "image_records"}
    return {**{key: value for key, value in full.items() if key not in omitted},
            "section": "ground_motion_core", "partial": True}


def jpeg_sample():
    image = io.BytesIO()
    Image.new("RGB", (12, 8), (140, 60, 20)).save(image, format="JPEG", quality=91)
    raw = image.getvalue()
    return {"topic": "/camera/image", "stamp": 7.25, "frame_id": "camera_link", "width": 12, "height": 8,
            "encoding": "jpeg", "jpeg_base64": base64.b64encode(raw).decode("ascii")}, raw


def test_complete_active_capture_preserves_real_evidence_and_never_claims_task_or_causality(tmp_path):
    full = full_snapshot(); image, raw = jpeg_sample()
    source = write_events(tmp_path / "capture.jsonl", [event("launcher_started", {"read_only": False}),
        event("probe_started", full["startup"]), event("probe_snapshot", core_snapshot(full)),
        event("image_sample", image), event("probe_snapshot", full), event("launcher_exit", {"exit_code": 0, "read_only": False})])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "collected" and result["read_only"] is False
    assert result["mode"] == "bounded_ground_motion"
    assert result["official_scoring"] == result["task_completion"] == "unverified"
    assert result["score"] is None
    assert result["motion"]["state"] == "FINISHED" and result["motion"]["reason"] == "pulse_time_limit"
    assert result["motion"]["published_nonzero_count"] == 20 and result["motion"]["published_zero_count"] == 120
    assert result["motion"]["odom_first"] == odom(2, 0.) and result["motion"]["odom_last"] == odom(23, .025)
    assert result["motion"]["observed_first_last_delta_m"] == pytest.approx(.025)
    assert result["motion"]["pulse_response_causality"] == "unverified"
    assert result["action_boundaries"]["declared_limits"]["sensor_age_s"] == .2
    assert result["action_boundaries"]["per_command_sensor_ages"] == "not_emitted"
    endpoint = result["command_endpoints"]["/cmd_vel"]
    assert endpoint["matched_subscriptions"] == 1 and endpoint["subscriber_endpoints"][0]["reliability"] == "RELIABLE"
    assert endpoint["subscriber_endpoints"][0]["type"] == "geometry_msgs/msg/Twist"
    assert result["snapshots"]["full_count"] == result["snapshots"]["core_count"] == 1
    assert result["missing_snapshot_fields"] == []
    assert result["images"][0]["sha256"] == hashlib.sha256(raw).hexdigest()
    assert (tmp_path / "out" / result["images"][0]["file_name"]).read_bytes() == raw
    assert result["source_evidence"]["sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert json.loads((tmp_path / "out/summary.json").read_text(encoding="utf-8")) == result


def test_v6_core_is_valid_partial_active_diagnostic_and_keeps_missing_sections(tmp_path):
    result = analyze(write_events(tmp_path / "core.jsonl", [event("probe_snapshot", core_snapshot())]), tmp_path / "out")
    assert result["status"] == "partial" and result["parse_errors"] == []
    assert result["read_only"] is False and result["snapshots"]["core_count"] == 1
    assert {"tf_edges", "motion_log", "public_messages"} <= set(result["missing_snapshot_fields"])
    assert {"launcher_started", "probe_started", "launcher_exit"} <= set(result["missing_events"])


def test_fragments_reassemble_offsets_without_silently_hiding_conflicts_gaps_or_unknown_parts(tmp_path):
    common = {"mode": "bounded_ground_motion", "read_only": False, "official_scoring": "unverified", "partial": True}
    parts = [core_snapshot(),
        {**common, "section": "odometry", "topic": "/odom", "value": {"first": odom(2, 0.), "last": odom(3, .01)}},
        {**common, "section": "tf_edges", "offset": 0, "transforms": [{"child_frame_id": "laser_link"}]},
        {**common, "section": "motion_log", "offset": 2, "value": [{"state": "STOP"}]},
        {**common, "section": "motion_log", "offset": 2, "value": [{"state": "FINISHED"}]},
        {**common, "section": "evidence_errors", "errors": [{"section": "public_messages", "error": "too large"}]},
        {**common, "section": "future_section", "new": "kept"}]
    result = analyze(write_events(tmp_path / "parts.jsonl", [event("probe_snapshot", item) for item in parts]), tmp_path / "out")
    assert result["status"] == "partial" and result["parse_errors"] == []
    assert len(result["snapshots"]["parts"]) == 7
    assert result["snapshots"]["assembled"]["odometry"]["/odom"]["last"] == odom(3, .01)
    assert result["snapshots"]["assembled"]["tf_edges"]["0"]["child_frame_id"] == "laser_link"
    assert result["fragment_gaps"]["motion_log"] == [0, 1]
    assert result["fragment_conflicts"][0]["section"] == "motion_log"
    assert result["diagnostic_errors"][0]["data"]["error"] == "too large"
    assert result["unknown_sections"][0]["data"]["new"] == "kept"


@pytest.mark.parametrize("kind", ["missing", "empty"])
def test_missing_input_stays_pending_without_invented_motion_or_images(tmp_path, kind):
    source = tmp_path / "pending.jsonl"
    if kind == "empty": source.write_bytes(b"")
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "pending" and result["record_count"] == 0
    assert result["read_only"] is None and result["mode"] is None
    assert result["motion"]["odom_first"] is None and result["images"] == []
    assert result["official_scoring"] == "unverified"


@pytest.mark.parametrize("bad", ['{"payload":', '{"payload":1,"payload":2}', '{"payload":NaN}',
    '{"payload":{"event":"probe_started","data":{"duration_s":1e309}}}'])
def test_strict_json_errors_are_line_numbered_and_good_records_survive(tmp_path, bad):
    source = write_events(tmp_path / "bad.jsonl", [event("probe_snapshot", core_snapshot())])
    with source.open("a", encoding="utf-8") as handle: handle.write(bad + "\n")
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "invalid" and result["record_count"] == 1
    assert result["parse_errors"][0]["line"] == 2 and result["motion"]["state"] == "FINISHED"


@pytest.mark.parametrize("change", [{"jpeg_base64": "invalid"}, {"width": 13},
    {"jpeg_base64": base64.b64encode(b"\xff\xd8\xfffake\xff\xd9").decode("ascii")}])
def test_invalid_images_never_write_placeholder_files(tmp_path, change):
    image, _ = jpeg_sample(); image.update(change)
    result = analyze(write_events(tmp_path / "bad_image.jsonl", [event("image_sample", image)]), tmp_path / "out")
    assert result["status"] == "invalid" and result["images"] == []
    assert not list((tmp_path / "out").glob("*.jpg"))


@pytest.mark.parametrize("change", [{"read_only": True}, {"mode": "read_only_probe"},
    {"partial": "true"}, {"published_nonzero_count": -1}, {"published_zero_count": True}])
def test_wrong_mode_flags_and_counts_are_invalid_and_never_relabelled_success(tmp_path, change):
    snapshot = core_snapshot(); snapshot.update(change)
    result = analyze(write_events(tmp_path / "invalid.jsonl", [event("probe_snapshot", snapshot)]), tmp_path / "out")
    assert result["status"] == "invalid" and result["parse_errors"][0]["line"] == 1
    assert result["task_completion"] == "unverified" and result["score"] is None


def test_unknown_fields_and_raw_provenance_survive_without_overwriting_image_metadata(tmp_path):
    image, _ = jpeg_sample(); image.update(line=999, source="forged")
    core = core_snapshot(); core["new_clock_evidence"] = {"age": .08}
    source = write_events(tmp_path / "future.jsonl", [event("probe_snapshot", core, future_wrapper=1),
        event("future_event", {"keep": "me"}), event("image_sample", image)])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "partial"
    assert result["unknown_events"][0]["data"] == {"keep": "me"}
    assert result["snapshots"]["parts"][0]["data"]["new_clock_evidence"] == {"age": .08}
    assert {item["scope"] for item in result["unknown_fields"]} >= {"wrapper", "probe_snapshot", "image_sample"}
    assert result["images"][0]["line"] == 3 and result["images"][0]["source"] == "unverified_callback"


def test_optional_graph_refresh_diagnostics_survive_core_without_becoming_required(tmp_path):
    snapshot = core_snapshot()
    diagnostics = {"interval_s": .5, "refresh_count": 3, "graph_query_count": 7,
        "graph_query_total_wall_s": .02, "graph_query_max_wall_s": .006, "match_check_count": 20}
    snapshot["graph_refresh_diagnostics"] = diagnostics
    source = write_events(tmp_path / "graph.jsonl", [event("probe_snapshot", snapshot)])
    result = analyze(source, tmp_path / "out")
    assert result["snapshots"]["assembled"]["graph_refresh_diagnostics"] == diagnostics
    assert not any("graph_refresh_diagnostics" in item["fields"] for item in result["unknown_fields"])
    assert "graph_refresh_diagnostics" not in result["missing_snapshot_fields"]


def test_graph_refresh_diagnostics_requires_an_object(tmp_path):
    snapshot = core_snapshot()
    snapshot["graph_refresh_diagnostics"] = []
    source = write_events(tmp_path / "graph_invalid.jsonl", [event("probe_snapshot", snapshot)])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "invalid"


def test_mismatched_odom_frames_do_not_create_distance_measurement(tmp_path):
    snapshot = full_snapshot(); snapshot["odom_last"]["header"]["frame_id"] = "map"
    result = analyze(write_events(tmp_path / "frames.jsonl", [event("probe_snapshot", snapshot)]), tmp_path / "out")
    assert result["motion"]["observed_first_last_delta_m"] is None
    assert result["motion"]["pulse_response_causality"] == "unverified"


@pytest.mark.parametrize("change", [{"section": None}, {"errors": {}}, {"motion_log": {}},
    {"odometry": []}, {"tf_edges": []}, {"command_endpoint_history": {}}])
def test_malformed_known_sections_cannot_be_counted_as_complete_capture(tmp_path, change):
    snapshot = full_snapshot(); snapshot.update(change)
    result = analyze(write_events(tmp_path / "malformed.jsonl", [event("probe_snapshot", snapshot)]), tmp_path / "out")
    assert result["status"] == "invalid" and result["parse_errors"][0]["line"] == 1


def test_v7_transition_and_final_timing_remain_observations_not_all_command_age_proof(tmp_path):
    snapshot = full_snapshot()
    timing = {"clock_sampling_lag_s": .12, "sensor_age_s": .2,
              "clock": {"stamp_s": 10., "wall_age_s": .08},
              "odom": {"stamp_s": 10.08, "wall_age_s": .01, "clock_minus_stamp_s": -.08, "fresh": True},
              "scan": {"stamp_s": 10., "wall_age_s": .09, "clock_minus_stamp_s": 0., "fresh": True}}
    snapshot["limits"]["clock_sampling_lag_s"] = .12
    snapshot["timing"] = timing
    snapshot["motion_log"][0]["timing"] = copy.deepcopy(timing)
    result = analyze(write_events(tmp_path / "timing.jsonl", [event("probe_snapshot", snapshot)]), tmp_path / "out")
    assert result["action_boundaries"]["final_timing"] == timing
    assert result["action_boundaries"]["declared_limits"]["clock_sampling_lag_s"] == .12
    assert result["action_boundaries"]["per_command_sensor_ages"] == "not_emitted"
    assert result["action_boundaries"]["timing_scope"] == "transition_and_final_observations_only"
    assert {item["scope"] for item in result["timing_observations"]} == {"snapshot", "motion_log_transition"}
    assert result["action_boundaries"]["independent_sensor_age_validation"] == "unverified"


def test_cli_handles_direct_invocation_and_pending_input(tmp_path):
    command = [sys.executable, str(ROOT / "tools/analyze_ground_motion_evidence.py"), str(tmp_path / "missing.jsonl"),
               "--output", str(tmp_path / "out")]
    run = subprocess.run(command, capture_output=True, text=True, encoding="utf-8")
    assert run.returncode == 0, run.stderr
    assert json.loads(run.stdout)["status"] == "pending"
