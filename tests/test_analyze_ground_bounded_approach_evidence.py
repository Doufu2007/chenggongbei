"""Approach observations keep distinct mode, larger caps and capture phases."""
import copy
import importlib
import math

import pytest

from test_analyze_ground_motion_evidence import event, full_snapshot, write_events
from test_analyze_ground_survey_evidence import image_parts, metadata, series


MODE = "ground_bounded_approach"


def payloads(phase="capture_0"):
    full = full_snapshot()
    full["mode"] = full["startup"]["mode"] = MODE
    full["state"], full["reason"] = "FINISHED", "approach_capture_complete"
    meta = metadata()
    meta["mode"] = MODE
    exposure = meta["survey"]["exposure_records"][0]
    exposure["phase"] = phase
    meta["survey"] = {"mode": MODE, "approach": {"mode": MODE,
        "phase": "CAPTURE", "state": "FINISHED", "reason": "approach_capture_complete",
        "target_id": "ground_4", "target_xy": [1.125, -.775], "target_yaw_rad": -.6,
        "measured_forward_m": .2, "total_path_m": .205, "max_origin_displacement_m": .201,
        "capture_count": 3, "capture_required": 3, "resume_count": 0, "max_resumes": 3,
        "task_completion": "unverified"}, "scan_records": [{"stage": "start", "payload": {"sample": "unchanged"}}],
        **{key: meta["survey"][key] for key in ("trajectory_dropped", "actual_commands_dropped",
            "task_completion", "image_errors", "exposure_records")}}
    parts, raw_image = image_parts()
    for part in parts:
        part["mode"], part["phase"] = MODE, phase
    rows = [series("trajectory_10hz", [[2., 10., .2, 0., .055, 0., 0., 0., 1., 0., 0., 0.]]),
            series("actual_commands", [[1., 9., .03, 0., "DRIVE"], [2., 10., 0., 0., "CAPTURE"]])]
    for value in rows:
        value["mode"] = MODE
    return [full, meta, *rows, *parts], raw_image


def all_capture_payloads():
    values, raw_image = payloads()
    exposures = values[1]["survey"]["exposure_records"]
    for index in (1, 2):
        parts, _ = image_parts()
        stamp = 10.02 + index * .1
        for part in parts:
            part.update(mode=MODE, phase=f"capture_{index}", stamp_s=stamp)
        exposure = copy.deepcopy(exposures[0])
        exposure.update(phase=f"capture_{index}", stamp_s=stamp)
        exposure["odom"]["stamp_s"] = stamp - .02
        exposures.append(exposure)
        values.extend(parts)
    return values, raw_image


def analyze(tmp_path, values, *, expected_mode=MODE):
    events = [event("launcher_started", {"mode": expected_mode, "read_only": False}),
        event("probe_started", {"mode": expected_mode, "read_only": False}),
        *[event("probe_snapshot", value) for value in values],
        event("launcher_exit", {"mode": expected_mode, "read_only": False, "exit_code": 0})]
    source = write_events(tmp_path / "approach.jsonl", events)
    before = source.read_bytes()
    result = importlib.import_module("tools.analyze_ground_survey_evidence").analyze_file(
        source, tmp_path / "out", expected_mode=expected_mode)
    assert source.read_bytes() == before
    return result


@pytest.mark.parametrize("phase", ["capture_0", "capture_1", "capture_2"])
def test_approach_capture_keeps_real_mode_original_image_and_raw_scan_record(tmp_path, phase):
    values, raw_image = payloads(phase)
    result = analyze(tmp_path, values)
    assert result["kind"] == "ground_bounded_approach_analysis" and result["status"] == "partial"
    assert result["missing_capture_phases"] == [f"capture_{i}" for i in range(3) if f"capture_{i}" != phase]
    assert result["mode"] == result["survey"]["mode"] == result["survey"]["approach"]["mode"] == MODE
    assert result["survey"]["scan_records"] == [{"stage": "start", "payload": {"sample": "unchanged"}}]
    assert result["score"] is None and result["task_completion"] == "unverified"
    image = result["native_images"][0]
    assert image["phase"] == phase and image["exposure_validation"] == "valid_prior_pose_observation"
    assert (tmp_path / "out" / image["file_name"]).read_bytes() == raw_image


@pytest.mark.parametrize("missing_phase", [None, "capture_0", "capture_1", "capture_2", "all"])
def test_entire_capture_group_loss_is_partial_even_when_received_fragments_are_complete(tmp_path, missing_phase):
    values, _ = all_capture_payloads()
    expected = [f"capture_{i}" for i in range(3)]
    if missing_phase is not None:
        values = [value for value in values if value.get("section") != "survey_image_part"
            or (missing_phase != "all" and value.get("phase") != missing_phase)]
    result = analyze(tmp_path, values)
    assert result["parse_errors"] == result["assembly_errors"] == []
    assert result["expected_capture_phases"] == expected
    assert result["missing_capture_phases"] == ([] if missing_phase is None else expected if missing_phase == "all" else [missing_phase])
    assert result["status"] == ("collected" if missing_phase is None else "partial")


def test_zero_recorded_captures_do_not_require_unobserved_three_frames(tmp_path):
    values, _ = payloads()
    values[1]["survey"]["approach"]["capture_count"] = 0
    values = [value for value in values if value.get("section") != "survey_image_part"]
    result = analyze(tmp_path, values)
    assert result["expected_capture_phases"] == result["missing_capture_phases"] == []
    assert result["status"] == "collected"
    assert result["task_completion"] == "unverified" and result["score"] is None


def test_v16_sensor_sync_and_core_graph_diagnostics_preserve_exact_received_values(tmp_path):
    values, _ = all_capture_payloads()
    diagnostics = {"interval_s": .5, "refresh_count": 2, "graph_query_count": 4,
        "total_wall_s": .01, "max_wall_s": .007, "match_check_count": 10}
    values[0]["graph_refresh_diagnostics"] = diagnostics
    sync = {"matched": 3, "pending": 1, "trace": [{"scan_stamp_s": 10.15, "prior_stamp_s": 10.14}]}
    values[1]["survey"]["approach"].update(revision="v16", sensor_sync=sync)
    result = analyze(tmp_path, values)
    assert result["snapshots"]["assembled"]["graph_refresh_diagnostics"] == diagnostics
    assert result["survey"]["approach"]["revision"] == "v16"
    assert result["survey"]["approach"]["sensor_sync"] == sync
    assert result["status"] == "collected" and result["task_completion"] == "unverified"


@pytest.mark.parametrize("section", ["ground_motion_core", "ground_motion_full"])
def test_v17_optional_callback_timing_preserves_real_recorder_payload_without_scoring(tmp_path, section):
    from inspection_mission.ground_callback_timing import CallbackTimingRecorder
    wall, cpu = [10.], [2.]
    recorder = CallbackTimingRecorder(perf_counter=lambda: wall[0], thread_time=lambda: cpu[0])
    token = recorder.begin_receive("/odom", {"header": {"stamp": {"sec": 8, "nanosec": 0}}},
        (8.18, 111.), "PAUSE")
    wall[0] += .03
    cpu[0] += .001
    recorder.end(token)
    recorder.record_input_qos("/odom", {"depth": 1, "reliability": "RELIABLE"},
        [{"reliability": "RELIABLE", "durability": "VOLATILE"}])
    timing = recorder.snapshot()
    values, _ = all_capture_payloads()
    values[0].update(section=section, partial=section == "ground_motion_core", callback_timing=timing)
    values[0]["startup"]["revision"] = "v17_timing"
    result = analyze(tmp_path, values)
    assembled = result["snapshots"]["assembled"]
    assert assembled["callback_timing"] == timing
    assert assembled["startup"]["revision"] == "v17_timing"
    assert not any("callback_timing" in value["fields"] for value in result["unknown_fields"])
    assert result["parse_errors"] == result["assembly_errors"] == []
    assert result["status"] == ("partial" if section == "ground_motion_core" else "collected")
    assert result["score"] is None and result["task_completion"] == "unverified"


@pytest.mark.parametrize("timing", [None, [], "v17_timing"])
def test_v17_callback_timing_rejects_nonobject_payload(tmp_path, timing):
    values, _ = all_capture_payloads()
    values[0]["callback_timing"] = timing
    result = analyze(tmp_path, values)
    assert result["status"] == "invalid"
    assert any("callback_timing must be an object" in item["error"] for item in result["parse_errors"])


@pytest.mark.parametrize("name,total,maximum", [("trajectory_10hz", 1200, 1200), ("actual_commands", 5000, 5000)])
def test_approach_larger_series_contract_is_mode_specific(tmp_path, name, total, maximum):
    row = [0., 10., 0., 0., .055, 0., 0., 0., 1., 0., 0., 0.] if name == "trajectory_10hz" else [0., 10., 0., 0., "WAIT"]
    fragments = []
    for offset in range(0, total, 300):
        value = series(name, [row[:] for _ in range(min(300, total-offset))], offset, total)
        value["mode"] = MODE
        fragments.append(value)
    result = analyze(tmp_path, fragments)
    assert result["parse_errors"] == [] and result["series"][name]["complete"] is True
    for value in fragments:
        value["mode"] = "ground_observation_survey"
    assert analyze(tmp_path, fragments, expected_mode="ground_observation_survey")["status"] == "invalid"
    fragments[0]["mode"], fragments[0]["total_rows"] = MODE, maximum+1
    assert analyze(tmp_path, fragments[:1])["status"] == "invalid"


@pytest.mark.parametrize("fault", ["metadata", "approach"])
def test_mixed_approach_metadata_mode_is_invalid(tmp_path, fault):
    values, _ = payloads()
    values[1]["survey"]["mode"] = "ground_short_drive" if fault == "metadata" else MODE
    if fault == "approach":
        values[1]["survey"]["approach"]["mode"] = "ground_short_drive"
    assert analyze(tmp_path, values)["status"] == "invalid"


def test_capture_phase_remains_invalid_in_older_mode(tmp_path):
    values, _ = payloads()
    parts = values[-2:]
    for value in parts:
        value["mode"] = "ground_observation_survey"
    assert analyze(tmp_path, parts, expected_mode="ground_observation_survey")["status"] == "invalid"


def test_real_approach_recorder_events_preserve_capture_and_each_scan_prior_pose(tmp_path):
    from test_ground_bounded_approach import guard, capture_ready, refresh, image, posed_odom
    from inspection_mission.telemetry import DiagnosticReporter
    state = guard()
    wall, yaw = capture_ready(state)
    for index in range(3):
        wall += .1
        refresh(state, wall, x=.201*math.cos(yaw), y=.201*math.sin(yaw), yaw=yaw)
        payload = posed_odom(round(10.+wall, 9), x=.201*math.cos(yaw), yaw=yaw)
        payload["pose"]["pose"]["position"]["y"] = .201*math.sin(yaw)
        payload["child_frame_id"] = "uav/base_link"
        state.observe_public("/uav/odom", payload, wall_s=wall)
        state.observe_image("/camera/image", image(10.+wall+.01, "camera_link"), wall_s=wall+.01)
        state.observe_image("/uav/camera/image", image(10.+wall+.01), wall_s=wall+.01)
        state.record_command(wall_s=wall+.01, linear_x=0., angular_z=0.)
    emitted = state.additional_events()
    for name, data in emitted:
        DiagnosticReporter._encode(name, data)
    # This is explicitly a local synthetic guard fixture, never platform data.
    wrapped = [event(name, data, source="synthetic_guard_fixture_not_platform") for name, data in emitted]
    source = write_events(tmp_path / "guard_fixture.jsonl", wrapped)
    result = importlib.import_module("tools.analyze_ground_bounded_approach_evidence").analyze_file(
        source, tmp_path / "real_events")
    assert result["parse_errors"] == result["assembly_errors"] == []
    assert result["mode"] == result["survey"]["approach"]["mode"] == MODE
    captures = [item for item in result["native_images"] if item["phase"].startswith("capture_")]
    assert len(captures) == 3 and len({item["stamp_s"] for item in captures}) == 3
    assert all(item["exposure_validation"] == "valid_prior_pose_observation" for item in captures)
    records = result["survey"]["scan_records"]
    assert {item["stage"] for item in records} == {"start", "after_turn", "mid_drive", "after_drive"}
    stamps = []
    for item in records:
        scan = item["payload"]
        stamp = scan["header"]["stamp"]["sec"]+scan["header"]["stamp"]["nanosec"]/1e9
        assert scan["header"]["frame_id"] == "laser_link"
        assert item["source_topic"] == "/scan" and item["odom"]["source_topic"] == "/odom"
        assert item["odom"]["frame_id"] == "odom" and item["odom"]["child_frame_id"] == "base_footprint"
        assert 0 <= stamp-item["odom"]["stamp_s"] <= .05+1e-9
        assert item["scan_minus_odom_s"] == pytest.approx(stamp-item["odom"]["stamp_s"])
        assert len(scan["ranges"]) == 360
        stamps.append(stamp)
    assert stamps == sorted(set(stamps))
