"""Route transport is an explicit mode, never a relabelled approach run."""
import copy
import importlib
import importlib.util

import pytest

from test_analyze_ground_bounded_approach_evidence import all_capture_payloads
from test_analyze_ground_motion_evidence import event, write_events
from test_analyze_ground_survey_evidence import series
from test_bounded_ground_probe import scan


MODE = "ground_observation_route"


def route_payloads():
    values, raw_image = all_capture_payloads()
    for value in values:
        value["mode"] = MODE
        if isinstance(value.get("startup"), dict):
            value["startup"]["mode"] = MODE
    meta = values[1]["survey"]
    meta["mode"] = meta["approach"]["mode"] = MODE
    meta["approach"]["route"] = {"mode": MODE, "status": "candidate_unverified", "requested_distance_m": 1.1}
    meta["scan_records"] = [{"stage": stage, "source_topic": "/scan",
        "payload": {**scan(index + 10., [1.0 + index * .01] * 360),
                    "time_increment": 0., "scan_time": 0., "intensities": [0.] * 360},
        "odom": {"stamp_s": index + 10., "source_topic": "/odom"}, "scan_minus_odom_s": 0.}
        for index, stage in enumerate(("start", "first_leg_end", "observation_arrival", "final_heading"))]
    return values, raw_image


def analyze(tmp_path, values, *, name="route"):
    module = "tools.analyze_ground_observation_route_evidence"
    assert importlib.util.find_spec(module) is not None, "explicit route analyzer entry is missing"
    events = [event("launcher_started", {"mode": MODE, "read_only": False}),
        event("probe_started", {"mode": MODE, "read_only": False}),
        *[event("probe_snapshot", value) for value in values],
        event("launcher_exit", {"mode": MODE, "read_only": False, "exit_code": 0})]
    source = write_events(tmp_path / f"{name}.jsonl", events)
    before = source.read_bytes()
    result = importlib.import_module(module).analyze_file(source, tmp_path / f"out_{name}")
    assert source.read_bytes() == before
    return result


def test_route_keeps_actual_modes_raw_scans_three_native_captures_and_unknown_scoring(tmp_path):
    values, raw_image = route_payloads()
    source_values = copy.deepcopy(values)
    result = analyze(tmp_path, values)
    assert values == source_values
    assert result["kind"] == "ground_observation_route_analysis" and result["status"] == "collected"
    assert result["mode"] == result["survey"]["mode"] == result["survey"]["approach"]["mode"] == MODE
    assert result["survey"]["approach"]["route"] == source_values[1]["survey"]["approach"]["route"]
    assert result["survey"]["scan_records"] == source_values[1]["survey"]["scan_records"]
    assert result["expected_capture_phases"] == ["capture_0", "capture_1", "capture_2"]
    assert result["missing_capture_phases"] == [] and len(result["native_images"]) == 3
    assert all((tmp_path / "out_route" / image["file_name"]).read_bytes() == raw_image for image in result["native_images"])
    assert result["task_completion"] == result["official_scoring"] == "unverified" and result["score"] is None


@pytest.mark.parametrize("missing_phase", ["capture_0", "capture_1", "capture_2", "all"])
def test_route_lost_complete_capture_group_remains_partial(tmp_path, missing_phase):
    values, _ = route_payloads()
    values = [value for value in values if value.get("section") != "survey_image_part"
        or (missing_phase != "all" and value["phase"] != missing_phase)]
    result = analyze(tmp_path, values)
    assert result["status"] == "partial"
    assert result["missing_capture_phases"] == ([f"capture_{i}" for i in range(3)] if missing_phase == "all" else [missing_phase])


@pytest.mark.parametrize("name,cap", [("trajectory_10hz", 3200), ("actual_commands", 14000)])
def test_only_route_accepts_extended_series_caps_and_rejects_overflow(tmp_path, name, cap):
    row = ([0., 10., 0., 0., .055, 0., 0., 0., 1., 0., 0., 0.] if name == "trajectory_10hz"
           else [0., 10., 0., 0., "WAIT"])
    fragments = []
    for offset in range(0, cap, 300):
        fragment = series(name, [row[:] for _ in range(min(300, cap-offset))], offset, cap)
        fragment["mode"] = MODE
        fragments.append(fragment)
    result = analyze(tmp_path, fragments)
    assert result["parse_errors"] == [] and result["series"][name]["complete"] is True
    for previous_mode in ("ground_bounded_approach", "ground_observation_survey", "ground_short_drive"):
        old = copy.deepcopy(fragments[0])
        old["mode"] = previous_mode
        source = write_events(tmp_path / f"{previous_mode}.jsonl", [event("probe_snapshot", old)])
        previous = importlib.import_module("tools.analyze_ground_survey_evidence").analyze_file(
            source, tmp_path / f"out_{previous_mode}", expected_mode=previous_mode)
        assert previous["status"] == "invalid"
    fragments[0]["total_rows"] = cap+1
    assert analyze(tmp_path, fragments[:1], name="overflow")["status"] == "invalid"


@pytest.mark.parametrize("scope", ["source", "metadata", "approach"])
def test_route_rejects_approach_mode_without_relabelling_source(tmp_path, scope):
    values, _ = route_payloads()
    if scope == "source":
        values[0]["mode"] = "ground_bounded_approach"
    elif scope == "metadata":
        values[1]["survey"]["mode"] = "ground_bounded_approach"
    else:
        values[1]["survey"]["approach"]["mode"] = "ground_bounded_approach"
    assert analyze(tmp_path, values)["status"] == "invalid"


def test_route_zero_captures_do_not_invent_images_and_dropped_rows_remain_partial(tmp_path):
    values, _ = route_payloads()
    values[1]["survey"]["approach"]["capture_count"] = 0
    values = [value for value in values if value.get("section") != "survey_image_part"]
    result = analyze(tmp_path, values)
    assert result["status"] == "collected" and result["expected_capture_phases"] == []
    values[1]["survey"]["trajectory_dropped"] = 1
    assert analyze(tmp_path, values, name="dropped")["status"] == "partial"
