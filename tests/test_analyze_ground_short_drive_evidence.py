"""Explicit short-drive analysis must preserve its real mode and transport."""
import importlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

from test_analyze_ground_motion_evidence import event, full_snapshot, write_events
from test_analyze_ground_survey_evidence import image_parts, metadata, series


MODE = "ground_short_drive"


def capture():
    # Synthetic fixture follows the emitted short-drive metadata. This does not
    # rewrite, relabel or patch any captured platform file.
    full = full_snapshot()
    full["mode"] = full["startup"]["mode"] = MODE
    full["state"], full["reason"] = "FINISHED", "short_drive_distance_reached"
    meta = metadata()
    meta["mode"] = MODE
    meta["survey"] = {"mode": MODE, "drive": {"mode": MODE,
        "limits": {"target_distance_m": .02, "max_motion_wall_s": 10.,
                   "max_heading_error_rad": .05, "max_reverse_m": .005, "global_wall_s": 25.},
        "target_distance_m": .02, "initial_yaw_rad": 0., "origin_position": [0., 0., .055],
        "first_motion_wall_s": 1., "measured_forward_m": .02,
        "max_observed_displacement_m": .0201, "resume_count": 0, "max_resumes": 3,
        "state": "FINISHED", "reason": "short_drive_distance_reached", "task_completion": "unverified"},
        **{key: meta["survey"][key] for key in ("trajectory_dropped", "actual_commands_dropped",
            "task_completion", "image_errors", "exposure_records")}}
    rows = [series("trajectory_10hz", [[2., 10., .02, 0., .055, 0., 0., 0., 1., 0., 0., 0.]]),
            series("actual_commands", [[1., 9., .03, 0., "DRIVE"], [2., 10., 0., 0., "STOP"]])]
    parts, raw_image = image_parts()
    for item in [*rows, *parts]:
        item["mode"] = MODE
    events = [event("launcher_started", {"mode": MODE, "read_only": False}),
        event("probe_started", full["startup"]), event("probe_snapshot", full),
        event("probe_snapshot", meta), *[event("probe_snapshot", item) for item in [*rows, *parts]],
        event("launcher_exit", {"mode": MODE, "read_only": False, "exit_code": 0})]
    return events, raw_image


def analyze(tmp_path, events, **kwargs):
    source = write_events(tmp_path / "drive.jsonl", events)
    raw = source.read_bytes()
    result = importlib.import_module("tools.analyze_ground_survey_evidence").analyze_file(
        source, tmp_path / "out", **kwargs)
    assert source.read_bytes() == raw
    return result


def test_explicit_mode_collects_exact_drive_evidence_without_inventing_score(tmp_path):
    events, raw_image = capture()
    result = analyze(tmp_path, events, expected_mode=MODE)
    assert result["kind"] == "ground_short_drive_analysis"
    assert result["mode"] == MODE and result["status"] == "collected"
    assert result["score"] is None and result["task_completion"] == "unverified"
    assert result["survey"]["mode"] == result["survey"]["drive"]["mode"] == MODE
    assert "candidate_mapping" not in result["survey"] and "target_yaw" not in result["survey"]
    assert result["series"]["actual_commands"]["records"][0]["state"] == "DRIVE"
    assert all(row["data"]["mode"] == MODE for row in result["raw_parts"] + result["lifecycle"])
    assert result["snapshots"]["assembled"]["startup"]["mode"] == MODE
    assert (tmp_path / "out" / result["native_images"][0]["file_name"]).read_bytes() == raw_image
    assert result["native_images"][0]["exposure_validation"] == "valid_prior_pose_observation"


@pytest.mark.parametrize("fault", ["snapshot", "startup", "lifecycle", "metadata", "drive"])
def test_mixed_mode_is_invalid_and_original_evidence_is_retained(tmp_path, fault):
    events, _ = capture()
    if fault == "snapshot": events[2]["payload"]["data"]["mode"] = "ground_observation_survey"
    elif fault == "startup": events[2]["payload"]["data"]["startup"]["mode"] = "ground_observation_survey"
    elif fault == "lifecycle": events[-1]["payload"]["data"]["mode"] = "ground_observation_survey"
    elif fault == "metadata": events[3]["payload"]["data"]["survey"]["mode"] = "ground_observation_survey"
    else: events[3]["payload"]["data"]["survey"]["drive"]["mode"] = "ground_observation_survey"
    result = analyze(tmp_path, events, expected_mode=MODE)
    assert result["status"] == "invalid" and result["parse_errors"]
    assert result["score"] is None


def test_default_survey_parser_still_rejects_drive_and_keeps_base_mode_constant(tmp_path):
    events, _ = capture()
    assert analyze(tmp_path, events)["status"] == "invalid"
    assert importlib.import_module("tools.analyze_ground_motion_evidence").MODE == "bounded_ground_motion"
    with pytest.raises(ValueError, match="mode"):
        analyze(tmp_path, events, expected_mode="anything")


def test_short_drive_cli_selects_real_mode_and_preserves_source(tmp_path):
    events, _ = capture()
    source = write_events(tmp_path / "drive.jsonl", events)
    raw = source.read_bytes()
    script = Path(__file__).parents[1] / "tools/analyze_ground_short_drive_evidence.py"
    proc = subprocess.run([sys.executable, str(script), str(source), "--output", str(tmp_path / "cli")],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    result = json.loads((tmp_path / "cli/summary.json").read_text(encoding="utf-8"))
    assert result["kind"] == "ground_short_drive_analysis" and result["mode"] == MODE
    assert source.read_bytes() == raw


def test_real_short_drive_recorder_events_need_no_mode_transformation(tmp_path):
    from test_ground_short_drive import guard, ready
    state = guard()
    decision = ready(state)
    state.record_command(wall_s=1.1, linear_x=decision.linear_x, angular_z=decision.angular_z)
    events = [event(name, data) for name, data in state.additional_events()]
    result = analyze(tmp_path, events, expected_mode=MODE)
    assert result["parse_errors"] == []
    assert result["survey"]["mode"] == result["survey"]["drive"]["mode"] == MODE
    assert result["series"]["actual_commands"]["records"][0]["state"] == "DRIVE"
    assert result["score"] is None and result["status"] == "partial"


@pytest.mark.parametrize("field", ["trajectory_dropped", "actual_commands_dropped"])
def test_complete_retained_fragments_do_not_hide_source_dropped_rows(tmp_path, field):
    events, _ = capture()
    events[3]["payload"]["data"]["survey"][field] = 1
    result = analyze(tmp_path, events, expected_mode=MODE)
    assert result["parse_errors"] == []
    assert all(item["complete"] for item in result["series"].values())
    assert result["status"] == "partial"
    assert result["survey"][field] == 1


@pytest.mark.parametrize("value", [-1, True, .5, "0"])
def test_source_drop_count_requires_nonnegative_integer(tmp_path, value):
    events, _ = capture()
    events[3]["payload"]["data"]["survey"]["trajectory_dropped"] = value
    result = analyze(tmp_path, events, expected_mode=MODE)
    assert result["status"] == "invalid" and result["parse_errors"]
