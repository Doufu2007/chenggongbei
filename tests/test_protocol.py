import math

import pytest

from inspection_mission.protocol import (
    build_payload, decode_ack, decode_order, decode_bool, validate_profile,
    planar_velocity, sensor_time,
)


def test_message_template_binds_observed_identity_and_encodes_json():
    event = {"event_id": "run42:g1", "id": "g1", "target_id": "actual-qr", "source": "ground"}
    assert build_payload({"data": "$json"}, event) == {
        "data": '{"event_id":"run42:g1","id":"g1","target_id":"actual-qr","source":"ground"}'
    }
    assert build_payload({"target": "$target_id", "nested": {"ok": True}}, event) == {
        "target": "actual-qr", "nested": {"ok": True}
    }
    with pytest.raises(ValueError):
        build_payload({"target": "$missing"}, event)


def test_order_rejects_partial_duplicate_and_unknown_task_ids():
    spec = {"path": "tasks", "id_field": "name"}
    assert decode_order({"data": '{"tasks":[{"name":"c"},{"name":"a"},{"name":"b"}]}'}, spec, {"a", "b", "c"}) == ["c", "a", "b"]
    for bad in (["a", "a", "c"], ["a", "b"], ["a", "b", "x"]):
        with pytest.raises(ValueError):
            decode_order({"tasks": [{"name": v} for v in bad]}, spec, {"a", "b", "c"})


def test_order_accepts_three_selected_ids_from_five_candidates():
    spec = {"path": "tasks", "id_field": "id", "selection_count": 3}
    payload = {"tasks": [{"id": "g4"}, {"id": "g2"}, {"id": "g1"}]}
    assert decode_order(payload, spec, {"g1", "g2", "g3", "g4", "g5"}) == ["g4", "g2", "g1"]
    for bad in (
        [{"id": "g4"}, {"id": "g2"}],
        [{"id": "g4"}, {"id": "g2"}, {"id": "g6"}],
        [{"id": "g4"}, {"id": "g4"}, {"id": "g2"}],
    ):
        with pytest.raises(ValueError):
            decode_order({"tasks": bad}, spec, {"g1", "g2", "g3", "g4", "g5"})


def test_official_csv_order_uses_data_and_preserves_selected_order():
    spec = {"encoding": "csv", "path": "data", "id_field": "", "selection_count": 3}
    known = {f"ground_{index}" for index in range(1, 6)}
    assert decode_order({"data": " ground_2, ground_4 ,ground_1 "}, spec, known) == [
        "ground_2", "ground_4", "ground_1"]


@pytest.mark.parametrize("raw", [
    "", "ground_2,,ground_1", "ground_2,ground_2,ground_1",
    "ground_2,ground_6,ground_1", "ground_2,ground_1",
    "ground_2,ground_4,ground_1,ground_3", "ground_2;ground_4;ground_1",
    '["ground_2","ground_4","ground_1"]',
])
def test_official_csv_order_rejects_malformed_or_unselected_ids(raw):
    spec = {"encoding": "csv", "path": "data", "id_field": "", "selection_count": 3}
    with pytest.raises(ValueError):
        decode_order({"data": raw}, spec, {f"ground_{index}" for index in range(1, 6)})


def test_order_encoding_must_be_explicitly_supported():
    spec = {"encoding": "guess", "path": "data", "id_field": "", "selection_count": 3}
    with pytest.raises(ValueError, match="encoding"):
        decode_order({"data": "ground_2,ground_4,ground_1"}, spec,
                     {f"ground_{index}" for index in range(1, 6)})


def test_ack_requires_explicit_acceptance_not_truthy_text():
    spec = {"path": "results", "id_field": "request", "accepted_field": "ok"}
    assert decode_ack({"results": [{"request": "new:g1", "ok": True}, {"request": "old:g1", "ok": "false"}]}, spec) == {"new:g1"}
    assert decode_bool({"data": False}, {"path": "data"}) is False
    with pytest.raises(ValueError):
        decode_bool({"data": "false"}, {"path": "data"})


def test_string_json_light_feedback_uses_configured_selector():
    assert decode_bool({"data": '{"enabled":true}'}, {"path": "enabled"}) is True
    assert decode_bool({"data": '{"enabled":false}'}, {"path": "enabled"}) is False


def test_unverified_profile_cannot_start_motion():
    with pytest.raises(ValueError, match="verified"):
        validate_profile({"verified": False})


def test_profile_cannot_subscribe_hidden_truth_even_if_marked_verified():
    from copy import deepcopy
    profile = valid_profile()
    assert validate_profile(profile) == profile
    for name in ("/gazebo/model_states", "/referee/events", "/referee/delivery_order"):
        bad = deepcopy(profile)
        bad["channels"]["ground_tasks"]["topic"] = name
        with pytest.raises(ValueError, match="forbidden"):
            validate_profile(bad)
    bad = deepcopy(profile)
    bad["uav_velocity_frame"] = "guess"
    with pytest.raises(ValueError):
        validate_profile(bad)


def test_ground_task_selection_count_must_be_positive_integer():
    from copy import deepcopy
    profile = valid_profile()
    profile["channels"]["ground_tasks"]["selection_count"] = 3
    assert validate_profile(profile) == profile
    for value in (0, -1, True, 3.0):
        bad = deepcopy(profile)
        bad["channels"]["ground_tasks"]["selection_count"] = value
        with pytest.raises(ValueError, match="selection_count"):
            validate_profile(bad)


def valid_profile():
    channels = {}
    for key, ros_type in {
        "ground_cmd": "geometry_msgs/msg/Twist", "uav_cmd": "geometry_msgs/msg/Twist",
        "ground_odom": "nav_msgs/msg/Odometry", "uav_odom": "nav_msgs/msg/Odometry",
        "ground_scan": "sensor_msgs/msg/LaserScan", "uav_scan": "sensor_msgs/msg/LaserScan",
        "ground_tasks": "std_msgs/msg/String", "recognition": "std_msgs/msg/String",
        "recognition_ack": "std_msgs/msg/String", "light": "std_msgs/msg/Bool",
        "light_feedback": "std_msgs/msg/Bool",
    }.items():
        channels[key] = {"topic": "/team_test/" + key, "type": ros_type}
    channels["recognition"]["fields"] = {"data": "$json"}
    channels["light"]["fields"] = {"data": "$enabled"}
    channels["ground_tasks"].update(path="tasks", id_field="id")
    channels["recognition_ack"].update(path="results", id_field="event_id", accepted_field="accepted")
    channels["light_feedback"]["path"] = "data"
    return {"verified": True, "uav_velocity_frame": "body", "ground_frame": "odom", "uav_frame": "uav/odom", "flight_mode": "velocity", "channels": channels}


def test_world_velocity_is_rotated_into_body_axes_only_when_requested():
    assert planar_velocity(1.0, 0.0, math.pi / 2, "body") == pytest.approx((0.0, -1.0))
    assert planar_velocity(1.0, 0.0, math.pi / 2, "world") == (1.0, 0.0)


def test_stale_future_and_nonfinite_sensor_stamps_do_not_become_fresh():
    assert sensor_time(0, 0, 0.0) == 0.0
    assert sensor_time(5, 0, 10.0) == 5.0
    with pytest.raises(ValueError):
        sensor_time(20, 0, 10.0)
    with pytest.raises(ValueError):
        planar_velocity(float("nan"), 0.0, 0.0, "world")
