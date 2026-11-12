"""Independent ray geometry checks; never load referee or task programs."""
from __future__ import annotations

import importlib
import json
import math

import pytest


def validator():
    return importlib.import_module("tools.validate_ground_scan_geometry")


def world_text(*, wall_pose="2 0 .2 0 0 0", sensor_pose=".08 0 .08 0 0 0"):
    return f'''<sdf version="1.6"><world name="test">
      <model name="wall"><static>true</static><pose>{wall_pose}</pose><link name="link">
        <collision name="box"><geometry><box><size>.1 4 .4</size></box></geometry></collision>
        <visual name="irrelevant"><geometry><box><size>100 100 100</size></box></geometry></visual>
      </link></model>
      <model name="amphibious_drone"><pose>0 0 .055 0 0 0</pose><link name="base_footprint">
        <collision name="self"><geometry><box><size>.34 .22 .09</size></box></geometry></collision>
        <sensor type="ray" name="rplidar"><pose>{sensor_pose}</pose><plugin name="lidar" filename="public.so">
          <frame_name>laser_link</frame_name></plugin></sensor>
      </link></model>
    </world></sdf>'''


def test_axis_aligned_ray_intersection_returns_nearest_positive_surface_and_parallel_misses():
    api = validator()
    box = api.Box("wall/box", (2., 0., 0.), (.05, 2., .2), api.rpy_rotation(0., 0., 0.))
    assert api.ray_box_distance((.08, 0., 0.), (1., 0., 0.), box) == pytest.approx(1.87)
    assert api.ray_box_distance((.08, 3., 0.), (1., 0., 0.), box) is None
    assert api.ray_box_distance((2., 0., 0.), (1., 0., 0.), box) == pytest.approx(.05)


def test_rotated_box_and_translated_sensor_are_composed_in_world_coordinates(tmp_path):
    api = validator()
    path = tmp_path / "test.world"
    path.write_text(world_text(wall_pose="2 0 .2 0 0 1.5707963267948966"), encoding="utf-8")
    geometry = api.load_candidate_world(path)
    assert len(geometry.boxes) == 1  # A visual and the robot's own collision are excluded.
    assert geometry.sensor_parent == "base_footprint" and geometry.sensor_frame == "laser_link"
    assert geometry.sensor_translation_m == (.08, 0., .08)
    # Rotated 4m-wide wall now spans x=[0,4] and y=[-.05,.05].
    assert api.ray_box_distance((2., -1., .2), (0., 1., 0.), geometry.boxes[0]) == pytest.approx(.95)


def capture(*, frame="odom", scan_frame="laser_link", pose_nanosec=0):
    odom = {"header": {"frame_id": frame, "stamp": {"sec": 5, "nanosec": pose_nanosec}},
            "child_frame_id": "base_footprint", "pose": {"pose": {
                "position": {"x": 0., "y": 0., "z": .055},
                "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}}}
    scan = {"header": {"frame_id": scan_frame, "stamp": {"sec": 5, "nanosec": 0}},
            "angle_min": -.2, "angle_increment": .2, "range_min": .1, "range_max": 12.,
            "ranges": [1.87 / math.cos(.2), 1.87, 1.87 / math.cos(.2)]}
    full = {"section": "ground_motion_full", "partial": False, "mode": "bounded_ground_motion", "read_only": False,
            "odometry": {"/odom": {"first": odom, "last": odom}},
            "public_messages": {"/scan": {"first": scan, "last": scan}}}
    return {"payload": {"event": "probe_snapshot", "data": full}}


def test_exact_scan_matches_fixed_candidate_without_fitting_or_claiming_task_success(tmp_path):
    api = validator()
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(json.dumps(capture()) + "\n", encoding="utf-8")
    report = api.validate_capture(source, world)
    assert len(report["scans"]) == 2
    for scan in report["scans"]:
        assert scan["candidate_consistency"] is True and scan["metrics"]["finite_observed_beams"] == 3
        assert scan["metrics"]["rmse_m"] < 1e-12
        assert len(scan["beams"]) == 3 and scan["beams"][1]["hit"] == "wall/box"
        assert scan["sensor_origin_world_m"] == pytest.approx([.08, 0., .135])
    assert report["source_evidence"]["sha256"] and report["candidate_world"]["sha256"]
    assert report["score"] is None and report["task_completion"] == "unverified"
    assert report["official_scene_version"] == "unverified"
    assert report["scans"][0]["pose_message"]["header"]["stamp"] == {"sec": 5, "nanosec": 0}


@pytest.mark.parametrize("change", ["wrong_scan_frame", "wrong_odom_frame", "invalid_quaternion", "large_skew"])
def test_invalid_provenance_and_pose_do_not_produce_geometry_consistency(tmp_path, change):
    api = validator()
    payload = capture()
    full = payload["payload"]["data"]
    if change == "wrong_scan_frame":
        full["public_messages"]["/scan"]["first"]["header"]["frame_id"] = "other_laser"
    elif change == "wrong_odom_frame":
        full["odometry"]["/odom"]["first"]["header"]["frame_id"] = "map"
    elif change == "invalid_quaternion":
        full["odometry"]["/odom"]["first"]["pose"]["pose"]["orientation"]["w"] = 0.
    else:
        full["odometry"]["/odom"]["first"]["header"]["stamp"]["sec"] = 4
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(json.dumps(payload) + "\n", encoding="utf-8")
    with pytest.raises(ValueError):
        api.validate_capture(source, world)


def test_nonfinite_ranges_remain_missing_and_do_not_count_as_matching_free_space(tmp_path):
    api = validator()
    payload = capture()
    for phase in ("first", "last"):
        payload["payload"]["data"]["public_messages"]["/scan"][phase]["ranges"][0] = "inf"
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(json.dumps(payload) + "\n", encoding="utf-8")
    report = api.validate_capture(source, world)
    assert report["scans"][0]["metrics"]["missing_observed_beams"] == 1
    assert report["scans"][0]["beams"][0]["residual_m"] is None
    assert report["scans"][0]["candidate_consistency"] is False


@pytest.mark.parametrize("raw", ['{"payload":1,"payload":2}', '{"payload":NaN}',
                                  '{"payload":{"event":"probe_snapshot","data":{"x":1e309}}}'])
def test_strict_json_rejects_duplicate_keys_and_nonfinite_values(tmp_path, raw):
    api = validator()
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(raw + "\n", encoding="utf-8")
    with pytest.raises(ValueError):
        api.validate_capture(source, world)


def test_short_drive_requires_explicit_mode_and_preserves_actual_capture_mode_and_bytes(tmp_path):
    api = validator()
    payload = capture()
    payload["payload"]["data"]["mode"] = "ground_short_drive"
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    raw = (json.dumps(payload) + "\n").encode()
    source.write_bytes(raw)
    with pytest.raises(ValueError):
        api.validate_capture(source, world)
    report = api.validate_capture(source, world, expected_mode="ground_short_drive")
    assert report["mode"] == "ground_short_drive"
    assert report["source_modes_seen"] == ["ground_short_drive"]
    assert all(item["candidate_consistency"] for item in report["scans"])
    assert report["scans"][0]["pose_message"] == payload["payload"]["data"]["odometry"]["/odom"]["first"]
    assert source.read_bytes() == raw


def test_lifecycle_mode_mismatch_is_rejected_even_with_one_valid_full_snapshot(tmp_path):
    api = validator()
    payload = capture()
    payload["payload"]["data"]["mode"] = "ground_short_drive"
    foreign = {"payload": {"event": "launcher_started", "data": {"mode": "bounded_ground_motion"}}}
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(json.dumps(foreign) + "\n" + json.dumps(payload) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="mode"):
        api.validate_capture(source, world, expected_mode="ground_short_drive")


def test_legacy_default_keeps_bounded_motion_mode_in_report(tmp_path):
    api = validator()
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(json.dumps(capture()) + "\n", encoding="utf-8")
    report = api.validate_capture(source, world)
    assert report["mode"] == "bounded_ground_motion"
    assert report["source_modes_seen"] == ["bounded_ground_motion"]


def test_unsupported_expected_mode_is_not_a_generic_mode_bypass(tmp_path):
    api = validator()
    world, source = tmp_path / "test.world", tmp_path / "evidence.jsonl"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(json.dumps(capture()) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="expected_mode"):
        api.validate_capture(source, world, expected_mode="arbitrary_mode")


def test_cli_accepts_explicit_short_drive_mode_without_rewriting_the_capture(tmp_path):
    api = validator()
    payload = capture()
    payload["payload"]["data"]["mode"] = "ground_short_drive"
    world, source, output = tmp_path / "test.world", tmp_path / "evidence.jsonl", tmp_path / "report.json"
    world.write_text(world_text(), encoding="utf-8")
    source.write_text(json.dumps(payload) + "\n", encoding="utf-8")
    assert api.main([str(source), "--world", str(world), "--output", str(output),
                     "--expected-mode", "ground_short_drive"]) == 0
    assert json.loads(output.read_text(encoding="utf-8"))["mode"] == "ground_short_drive"
