"""Real parsing and safety gates, independent of an unavailable ROS runtime."""
import copy
import json
import math

import pytest

from tools.mission_scenario import demonstration_plan
from test_protocol import valid_profile


def configured_profile():
    profile = valid_profile()
    for source in ("ground", "uav"):
        profile["channels"][source + "_scan"].update(frame_id=source + "/laser", yaw_offset_rad=0.0)
    return profile


def odom(now=5.0, frame="odom"):
    return {"header": {"frame_id": frame, "stamp": {"sec": int(now), "nanosec": int(now % 1 * 1e9)}},
            "pose": {"pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.0},
                              "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}}


def scan(now=5.0, frame="ground/laser"):
    return {"header": {"frame_id": frame, "stamp": {"sec": int(now), "nanosec": 0}},
            "angle_min": -math.pi, "angle_increment": math.pi / 180,
            "range_min": 0.05, "range_max": 5.0, "ranges": [3.0] * 361}


def inputs():
    from inspection_mission.runtime import RuntimeInputs
    return RuntimeInputs(configured_profile(), {"g1", "g2", "g3"})


def test_configuration_is_importable_without_ros_and_rejects_unverified_profile(tmp_path):
    from inspection_mission.mission_node import load_configuration
    plan_path, profile_path = tmp_path / "plan.json", tmp_path / "profile.json"
    plan_path.write_text(json.dumps(demonstration_plan()), encoding="utf-8")
    profile_path.write_text(json.dumps({"verified": False}), encoding="utf-8")
    with pytest.raises(ValueError, match="verified"):
        load_configuration(plan_path, profile_path)
    profile_path.write_text(json.dumps(configured_profile()), encoding="utf-8")
    plan, profile, controller = load_configuration(plan_path, profile_path)
    assert controller.plan["ground_points"][0]["id"] == "g1" and profile["verified"]


def test_scan_transform_is_required_before_any_motion():
    from inspection_mission.runtime import RuntimeInputs
    with pytest.raises(ValueError, match="frame_id"):
        RuntimeInputs(valid_profile(), {"g1", "g2", "g3"})


def test_pose_uses_sensor_stamp_and_invalid_update_clears_previous_pose():
    data = inputs()
    data.update_pose("ground", odom(4), now=5)
    assert data.poses["ground"].stamp == 4
    assert not data.pose_fresh("ground", 5)
    data.update_pose("ground", odom(), now=5)
    assert data.pose_fresh("ground", 5)
    with pytest.raises(ValueError, match="frame"):
        data.update_pose("ground", odom(frame="map"), now=5)
    assert data.poses["ground"] is None
    bad = odom()
    bad["pose"]["pose"]["orientation"]["w"] = 0
    with pytest.raises(ValueError, match="quaternion"):
        data.update_pose("ground", bad, now=5)
    assert data.poses["ground"] is None


def test_direction_gate_checks_proposed_travel_sector_and_scan_expiry():
    data = inputs()
    data.update_pose("ground", odom(), now=5)
    rays = scan()
    rays["ranges"][180] = 0.12
    data.update_scan("ground", rays, now=5)
    assert not data.motion_clear("ground", (0.2, 0), 5, stop_distance=0.3)
    assert data.motion_clear("ground", (-0.2, 0), 5, stop_distance=0.3)
    assert not data.motion_clear("ground", (-0.2, 0), 6, stop_distance=0.3)
    with pytest.raises(ValueError):
        data.update_scan("ground", scan(frame="unconfirmed"), now=5)
    assert not data.motion_clear("ground", (-0.2, 0), 5, stop_distance=0.3)


def test_empty_new_camera_frame_replaces_old_detection_and_replay_is_rejected():
    data = inputs()
    payload = {"source": "ground", "stamp": 5.0, "detections": [
        {"target_id": "person", "center": [0.5, 0.5], "confidence": 0.9}]}
    data.update_detections(payload, now=5)
    assert data.fresh_detections(5)[0]["stamp"] == 5
    assert data.fresh_detections(6) == []
    data.update_detections({"source": "ground", "stamp": 5.1, "detections": []}, now=5.1)
    assert data.fresh_detections(5.1) == []
    with pytest.raises(ValueError, match="reversed"):
        data.update_detections(payload, now=5.1)
    assert data.fresh_detections(5.1) == []


def test_public_order_change_or_parse_error_latches_fault():
    data = inputs()
    data.update_order({"tasks": [{"id": "g1"}, {"id": "g2"}, {"id": "g3"}]})
    assert data.order == ["g1", "g2", "g3"]
    with pytest.raises(ValueError, match="changed"):
        data.update_order({"tasks": [{"id": "g3"}, {"id": "g2"}, {"id": "g1"}]})
    assert data.fault and data.order is None


def test_runtime_accepts_the_three_selected_ids_from_five_candidates():
    from inspection_mission.runtime import RuntimeInputs

    profile = configured_profile()
    profile["channels"]["ground_tasks"]["selection_count"] = 3
    data = RuntimeInputs(profile, {"g1", "g2", "g3", "g4", "g5"})
    data.update_order({"tasks": [{"id": "g4"}, {"id": "g2"}, {"id": "g1"}]})
    assert data.order == ["g4", "g2", "g1"]


def test_light_feedback_must_be_fresh_and_explicit():
    data = inputs()
    data.update_light({"data": True}, now=5)
    assert data.light_ready(5)
    assert not data.light_ready(6)
    with pytest.raises(ValueError):
        data.update_light({"data": "true"}, now=5)
    assert not data.light_ready(5)


def test_ack_parser_never_accepts_truthy_text():
    data = inputs()
    data.update_ack({"results": [{"event_id": "run:1", "accepted": True}]})
    assert data.acknowledgements == {"run:1"}
    with pytest.raises(ValueError):
        data.update_ack({"results": "broken"})
    assert data.acknowledgements == set()


def test_tilted_flight_is_rejected_before_yaw_only_scan_or_cloud_transform():
    data = inputs()
    data.update_pose("uav", odom(frame="uav/odom"), now=5)
    data.update_scan("uav", scan(frame="uav/laser"), now=5)
    pitched = odom(frame="uav/odom")
    pitched["pose"]["pose"]["orientation"] = {
        "x": 0.0, "y": math.sin(math.pi/6), "z": 0.0, "w": math.cos(math.pi/6)}
    with pytest.raises(ValueError, match="tilt"):
        data.update_pose("uav", pitched, now=5)
    # An upward WORLD command must not pass a yaw-only 3D guard at 60 degree pitch.
    assert not data.motion_clear("uav", (0, 0, 0.2, 0), 5, stop_distance=0.4)


def test_optional_body_cloud_blocks_level_vertical_motion_and_invalid_cloud_clears():
    from inspection_mission.runtime import RuntimeInputs
    profile = configured_profile()
    profile["channels"]["uav_cloud"] = {"topic": "/test/cloud", "type": "sensor_msgs/msg/PointCloud2", "frame_id": "uav/body"}
    data = RuntimeInputs(profile, {"g1", "g2", "g3"})
    data.update_pose("uav", odom(frame="uav/odom"), now=5)
    data.update_scan("uav", scan(frame="uav/laser"), now=5)
    header = {"frame_id": "uav/body", "stamp": {"sec": 5, "nanosec": 0}}
    data.update_cloud(header, [[0, 0, 0.3], [0.01, 0, 1.0], [0, 0.01, 1.1]], now=5)
    assert not data.motion_clear("uav", (0, 0, 0.2, 0), 5, stop_distance=0.4)
    header["frame_id"] = "unconfirmed"
    with pytest.raises(ValueError):
        data.update_cloud(header, [[0, 0, 1]] * 3, now=5)
    assert data.cloud is None


def test_json_nan_and_duplicate_fields_cannot_silently_override_configuration(tmp_path):
    from inspection_mission.mission_node import load_configuration
    profile_path, plan_path = tmp_path / "profile.json", tmp_path / "plan.json"
    plan_path.write_text(json.dumps(demonstration_plan()), encoding="utf-8")
    for raw in ('{"verified":true,"verified":false}', '{"verified":NaN}'):
        profile_path.write_text(raw, encoding="utf-8")
        with pytest.raises(ValueError):
            load_configuration(plan_path, profile_path)


def test_channel_templates_validate_against_only_real_runtime_fields():
    from inspection_mission.mission_node import validate_outbound_fields
    profile = configured_profile()
    validate_outbound_fields(profile)
    profile["channels"]["light"]["fields"] = {"data": "$target_id"}
    with pytest.raises(ValueError, match="target_id"):
        validate_outbound_fields(profile)
    profile = configured_profile()
    profile["channels"]["recognition"]["fields"] = {"data": "$enabled"}
    with pytest.raises(ValueError, match="enabled"):
        validate_outbound_fields(profile)


def test_trigger_flight_requires_confirmed_emergency_abort_contract():
    from inspection_mission.mission_node import validate_outbound_fields
    profile = configured_profile()
    profile["flight_mode"] = "trigger"
    for key in ("takeoff", "land"):
        profile["channels"][key] = {"topic": "/test/" + key, "type": "std_msgs/msg/Empty", "fields": {}}
    with pytest.raises(ValueError, match="abort"):
        validate_outbound_fields(profile)
    profile["trigger_abort_stops_motion"] = True
    profile["channels"]["flight_abort"] = {"topic": "/test/abort", "type": "std_msgs/msg/Empty", "fields": {}}
    profile["trigger_max_speed_mps"] = 0.4
    validate_outbound_fields(profile)


def test_explicit_runtime_paths_are_absolute_to_match_prelaunch_validation(tmp_path):
    from inspection_mission.mission_node import resolve_runtime_path
    assert resolve_runtime_path("", tmp_path / "plan.json") == tmp_path / "plan.json"
    assert resolve_runtime_path(str(tmp_path / "other.json"), tmp_path / "plan.json") == tmp_path / "other.json"
    with pytest.raises(ValueError, match="absolute"):
        resolve_runtime_path("relative/plan.json", tmp_path / "plan.json")


def test_scan_expiry_has_its_own_configurable_timeout():
    from inspection_mission.runtime import RuntimeInputs
    data = RuntimeInputs(configured_profile(), {"g1", "g2", "g3"}, sensor_timeout_s=1, scan_timeout_s=0.1)
    data.update_pose("ground", odom(), now=5)
    data.update_scan("ground", scan(), now=5)
    assert data.pose_fresh("ground", 5.2)
    assert not data.scan_fresh("ground", 5.2)


def test_trigger_intent_checks_true_first_frame_takeoff_and_landing_directions():
    from inspection_mission.runtime import RuntimeInputs, flight_motion_intent
    profile = configured_profile()
    profile.update(flight_mode="trigger", trigger_abort_stops_motion=True, trigger_max_speed_mps=0.4)
    for key in ("takeoff", "land", "flight_abort"):
        profile["channels"][key] = {"topic": "/test/" + key, "type": "std_msgs/msg/Empty", "fields": {}}
    profile["channels"]["uav_cloud"] = {"topic": "/test/cloud", "type": "sensor_msgs/msg/PointCloud2", "frame_id": "uav/body"}
    data = RuntimeInputs(profile, {"g1", "g2", "g3"})
    data.update_pose("uav", odom(frame="uav/odom"), now=5)
    data.update_scan("uav", scan(frame="uav/laser"), now=5)
    header = {"frame_id": "uav/body", "stamp": {"sec": 5, "nanosec": 0}}
    for state, direction in (("TAKEOFF", 1), ("LAND", -1)):
        intent = flight_motion_intent((0, 0, 0, 0), state, profile)
        assert intent == (0, 0, direction * 0.4, 0)
        data.update_cloud(header, [[0, 0, direction * distance] for distance in (0.3, 1.0, 1.1)], now=5)
        # At the transition frame controller's Twist is zero, but autonomous
        # trigger motion needs its real vertical corridor checked before send.
        assert data.motion_clear("uav", (0, 0, 0, 0), 5, stop_distance=0.4)
        assert not data.motion_clear("uav", intent, 5, stop_distance=0.4)
    assert flight_motion_intent((0.2, 0, 0, 0), "AIR_NAV_1", profile) == (0.2, 0, 0, 0)


@pytest.mark.parametrize("speed", [None, 0, -1, float("nan"), float("inf"), True])
def test_trigger_intent_cannot_guess_automatic_flight_speed(speed):
    from inspection_mission.runtime import flight_motion_intent
    profile = {"flight_mode": "trigger", "trigger_max_speed_mps": speed}
    with pytest.raises(ValueError):
        flight_motion_intent((0, 0, 0, 0), "TAKEOFF", profile)
