"""Offline evidence analysis must never invent execution or image evidence."""
from __future__ import annotations

import base64
import hashlib
import importlib
import io
import json
from pathlib import Path
import subprocess
import sys

import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]


def analyze(path, output):
    return importlib.import_module("tools.analyze_probe_evidence").analyze_evidence(path, output)


def event(name, data, **extra):
    return {"received_at": "2026-11-11T09:00:01+00:00", "source": "unverified_callback",
            "payload": {"event": name, "timestamp": "2026-11-11T09:00:00+00:00", "data": data}, **extra}


def write_events(path, events):
    path.write_text("".join(json.dumps(e, ensure_ascii=False) + "\n" for e in events), encoding="utf-8")
    return path


def jpeg_sample(topic="/uav/camera/image"):
    """Synthetic test fixture; analyzer outputs remain unchanged input bytes."""
    image = io.BytesIO()
    Image.new("RGB", (12, 8), (140, 60, 20)).save(image, format="JPEG", quality=91)
    raw = image.getvalue()
    return {"topic": topic, "stamp": 7.25, "frame_id": "uav/camera_optical",
            "width": 12, "height": 8, "encoding": "jpeg",
            "jpeg_base64": base64.b64encode(raw).decode("ascii")}, raw


def test_complete_capture_keeps_real_image_bytes_raw_order_and_provenance(tmp_path):
    image, raw = jpeg_sample("/uav/camera/../../image")
    order = {"topic": "/referee/ground_tasks", "sample_index": 1,
             "message": {"data": "ground_0,ground_4,ground_2"}}
    snapshot = {"read_only": True, "official_scoring": "unverified", "elapsed_s": 30,
                "topics": [{"name": "/camera/image", "type": "sensor_msgs/msg/Image"}],
                "subscriptions": {}, "samples": {order["topic"]: order},
                "sample_counts": {order["topic"]: 1}, "errors": {}, "public_endpoints": {}}
    source = write_events(tmp_path / "capture.jsonl", [
        event("launcher_started", {"read_only": True}), event("probe_started", {"duration_s": 30}),
        event("probe_snapshot", snapshot), event("image_sample", image), event("launcher_exit", {"exit_code": 0})])
    output = tmp_path / "out"
    result = analyze(source, output)
    assert result["status"] == "collected"
    assert result["official_scoring"] == "unverified"
    assert result["event_counts"]["probe_snapshot"] == 1
    assert "probe_finished" in result["missing_events"]
    assert result["launcher_exits"][0]["data"]["exit_code"] == 0
    assert result["launcher_exit_codes"] == [0]
    assert result["startup"] == {"launcher_started": True, "probe_started": True}
    assert result["sample_topics"] == ["/referee/ground_tasks", "/uav/camera/../../image"]
    assert result["missing_snapshot_fields"] == []
    assert result["reported_sample_counts"] == {"/referee/ground_tasks": 1}
    assert result["sample_record_counts"] == {"/referee/ground_tasks": 1}
    assert result["draw_order_payloads"][0]["data"] == order
    assert result["draw_order_payloads"][0]["line"] == 3
    saved = result["images"][0]
    assert (saved["topic"], saved["stamp"], saved["frame_id"]) == (image["topic"], 7.25, "uav/camera_optical")
    assert saved["sha256"] == hashlib.sha256(raw).hexdigest()
    assert saved["line"] == 4
    assert "/" not in saved["file_name"] and "\\" not in saved["file_name"]
    assert (output / saved["file_name"]).read_bytes() == raw
    assert json.loads((output / "summary.json").read_text(encoding="utf-8")) == result


def test_partial_snapshots_merge_metadata_sections_and_per_topic_values(tmp_path):
    common = {"read_only": True, "official_scoring": "unverified", "partial": True}
    sample = {"topic": "/odom", "sample_index": 1, "message": {"header": {"frame_id": "odom"}}}
    fragments = [
        {**common, "section": "metadata", "elapsed_s": 30, "topics": [], "samples": {},
         "subscriptions": [{"topic": "/odom", "value": {"type": "nav_msgs/msg/Odometry"}}],
         "sample_counts": [{"topic": "/odom", "value": 30}], "errors": [], "public_endpoints": []},
        {**common, "section": "samples", "topic": "/odom", "sample": sample},
        {**common, "section": "sample_counts", "value": {"/scan": 7}},
        {**common, "section": "public_endpoints", "topic": "/cmd_vel", "value": {"subscribers": 1}},
        {**common, "section": "future_sensor", "value": {"new": "kept"}},
    ]
    source = write_events(tmp_path / "parts.jsonl", [event("probe_snapshot", data) for data in fragments])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "partial"
    assert result["snapshots"]["full_count"] == 0 and result["snapshots"]["partial_count"] == 5
    assert result["snapshots"]["assembled"]["samples"]["/odom"] == sample
    assert result["snapshots"]["assembled"]["subscriptions"]["/odom"]["type"] == "nav_msgs/msg/Odometry"
    assert result["snapshots"]["assembled"]["public_endpoints"]["/cmd_vel"]["subscribers"] == 1
    assert result["reported_sample_counts"] == {"/odom": 30, "/scan": 7}
    assert result["sample_record_counts"] == {"/odom": 1}
    assert {item["line"] for item in result["samples"]} == {2}
    assert any(item["fields"] == {"future_sensor": {"new": "kept"}} for item in result["unknown_fields"])
    assert "launcher_started" in result["missing_events"]
    assert result["images"] == []


@pytest.mark.parametrize("mode", ["missing", "empty"])
def test_unrun_input_is_pending_and_writes_no_invented_samples_or_images(tmp_path, mode):
    source = tmp_path / "not_yet_run.jsonl"
    if mode == "empty":
        source.write_bytes(b"")
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "pending"
    assert result["record_count"] == 0 and result["samples"] == [] and result["images"] == []
    assert result["reported_sample_counts"] == {}
    assert {"launcher_started", "probe_started", "probe_snapshot", "launcher_exit"} <= set(result["missing_events"])
    assert not list((tmp_path / "out").glob("*.jpg"))


@pytest.mark.parametrize("bad", ["not base64!", base64.b64encode(b"\xff\xd8\xfffake\xff\xd9").decode("ascii")])
def test_invalid_image_is_line_numbered_and_not_saved(tmp_path, bad):
    data, _ = jpeg_sample()
    data["jpeg_base64"] = bad
    source = write_events(tmp_path / "bad.jsonl", [event("probe_started", {}), event("image_sample", data)])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "invalid"
    assert result["parse_errors"][0]["line"] == 2
    assert result["images"] == []
    assert not list((tmp_path / "out").glob("*.jpg"))


def test_image_dimensions_must_match_decoded_jpeg(tmp_path):
    data, _ = jpeg_sample()
    data["width"] = 13
    source = write_events(tmp_path / "wrong_size.jsonl", [event("image_sample", data)])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "invalid" and result["images"] == []
    assert result["parse_errors"][0]["line"] == 1


@pytest.mark.parametrize("bad_json", ['{"payload":', '{"payload":1,"payload":2}', '{"payload":NaN}',
    '{"payload":{"event":"probe_started","data":{"duration_s":1e309}}}'])
def test_invalid_json_is_line_numbered_without_hiding_good_records(tmp_path, bad_json):
    source = write_events(tmp_path / "bad.jsonl", [event("launcher_started", {})])
    with source.open("a", encoding="utf-8") as handle:
        handle.write(bad_json + "\n")
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "invalid"
    assert result["record_count"] == 1
    assert result["event_counts"] == {"launcher_started": 1}
    assert result["parse_errors"][0]["line"] == 2


def test_unknown_fields_and_probe_errors_survive_without_becoming_scoring(tmp_path):
    first = event("probe_started", {"read_only": True, "future_option": 10}, future_wrapper="wrapper")
    first["payload"]["future_payload"] = ["payload"]
    source = write_events(tmp_path / "future.jsonl", [first, event("sample_error", {"topic": "/scan", "error": "lost sample"}),
                       event("future_event", {"not_a_score": "keep me"})])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "partial"
    assert result["diagnostic_errors"][0]["data"]["error"] == "lost sample"
    assert result["unknown_events"][0]["event"] == "future_event"
    assert {entry["scope"] for entry in result["unknown_fields"]} >= {"wrapper", "payload", "probe_started"}
    assert result["official_scoring"] == "unverified"


def test_unknown_image_fields_cannot_overwrite_receiver_provenance(tmp_path):
    image, _ = jpeg_sample()
    image.update(line=999, source="forged", payload_timestamp="forged timestamp")
    source = write_events(tmp_path / "image.jsonl", [event("image_sample", image)])
    result = analyze(source, tmp_path / "out")
    assert result["images"][0]["line"] == 1
    assert result["images"][0]["source"] == "unverified_callback"
    assert result["images"][0]["payload_timestamp"] == "2026-11-11T09:00:00+00:00"
    assert result["unknown_fields"][0]["fields"]["line"] == 999


def test_empty_snapshot_does_not_claim_a_complete_capture(tmp_path):
    source = write_events(tmp_path / "empty_snapshot.jsonl", [event("launcher_started", {}),
        event("probe_started", {}), event("probe_snapshot", {}), event("launcher_exit", {"exit_code": 0})])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "partial"
    assert result["snapshots"]["full_count"] == 0
    assert {"samples", "sample_counts", "subscriptions", "topics"} <= set(result["missing_snapshot_fields"])


def test_partial_flag_must_be_boolean_instead_of_treating_string_as_full(tmp_path):
    source = write_events(tmp_path / "bad_partial.jsonl", [event("probe_snapshot", {"partial": "true"})])
    result = analyze(source, tmp_path / "out")
    assert result["status"] == "invalid" and result["parse_errors"][0]["line"] == 1


def test_cli_invalid_input_exits_nonzero_but_empty_input_exits_pending(tmp_path):
    source = tmp_path / "capture.jsonl"
    source.write_text("not json\n", encoding="utf-8")
    command = [sys.executable, str(ROOT / "tools/analyze_probe_evidence.py"), str(source), "--output", str(tmp_path / "out")]
    bad = subprocess.run(command, capture_output=True, text=True, encoding="utf-8")
    assert bad.returncode == 2
    assert json.loads(bad.stdout)["status"] == "invalid"
    source.write_bytes(b"")
    pending = subprocess.run(command, capture_output=True, text=True, encoding="utf-8")
    assert pending.returncode == 0 and json.loads(pending.stdout)["status"] == "pending"
