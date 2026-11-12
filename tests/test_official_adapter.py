import pytest
import json

from inspection_mission.mission_node import outbound_payload, validate_outbound_fields
from test_protocol import valid_profile


def _official_configuration(tmp_path):
    """Artificial geometry plus the documented wire contract; no field map."""
    from tools.mission_scenario import demonstration_plan
    from test_runtime import configured_profile

    plan = demonstration_plan(protocol="inspection_v1")
    for index, point in enumerate(plan["ground_points"], 1):
        point["id"] = f"ground_{index}"
    for index, position in ((4, [-0.4, 0.0, 0.0]), (5, [0.0, -0.4, 0.0])):
        plan["ground_points"].append({
            "id": f"ground_{index}", "target_id": "*", "pose": position,
            "approach": [position[0] - 0.24, position[1], position[2]],
        })
    profile = configured_profile()
    profile["channels"].update({
        "ground_tasks": {"topic": "/referee/ground_tasks", "type": "std_msgs/msg/String",
                         "encoding": "csv", "path": "data", "id_field": "", "selection_count": 3},
        "recognition": {"topic": "/inspection/detections", "type": "std_msgs/msg/String",
                        "encoding": "inspection_phrase_v1", "fields": {"data": "$phrase"}},
        "current_target": {"topic": "/referee/current_target", "type": "std_msgs/msg/String",
                           "fields": {"data": "$value"}},
        "cruise_event": {"topic": "/referee/cruise_event", "type": "std_msgs/msg/String",
                         "fields": {"data": "$value"}},
        "light": {"topic": "/uav/laser_fire", "type": "std_msgs/msg/Bool", "fields": {"data": "$enabled"}},
    })
    plan_path, profile_path = tmp_path / "plan.json", tmp_path / "profile.json"
    plan_path.write_text(json.dumps(plan), encoding="utf8")
    profile_path.write_text(json.dumps(profile), encoding="utf8")
    return plan_path, profile_path


@pytest.mark.parametrize("channel,value", [
    ("current_target", "ground_1"),
    ("cruise_event", "window_1"),
    ("light", False),
])
def test_official_outputs_cannot_replace_runtime_values_with_constants(tmp_path, channel, value):
    from inspection_mission.mission_node import load_configuration

    plan_path, profile_path = _official_configuration(tmp_path)
    profile = json.loads(profile_path.read_text(encoding="utf8"))
    profile["channels"][channel]["fields"] = {"data": value}
    profile_path.write_text(json.dumps(profile), encoding="utf8")
    with pytest.raises(ValueError, match="official"):
        load_configuration(plan_path, profile_path)


@pytest.mark.parametrize("field,value", [
    ("encoding", "json"), ("path", "tasks"), ("id_field", "id"),
    ("selection_count", 5), ("topic", "/test/tasks"), ("type", "std_msgs/msg/Bool"),
])
def test_official_ground_order_must_keep_the_documented_csv_three_of_five_contract(tmp_path, field, value):
    from inspection_mission.mission_node import load_configuration

    plan_path, profile_path = _official_configuration(tmp_path)
    profile = json.loads(profile_path.read_text(encoding="utf8"))
    profile["channels"]["ground_tasks"][field] = value
    profile_path.write_text(json.dumps(profile), encoding="utf8")
    with pytest.raises(ValueError, match="official"):
        load_configuration(plan_path, profile_path)


@pytest.mark.parametrize("invalid", ["missing", "duplicate", "wrong-prefix", "empty"])
def test_official_map_requires_five_unique_ground_candidate_ids(tmp_path, invalid):
    from inspection_mission.mission_node import load_configuration

    plan_path, profile_path = _official_configuration(tmp_path)
    plan = json.loads(plan_path.read_text(encoding="utf8"))
    if invalid == "missing":
        plan["ground_points"].pop()
    elif invalid == "duplicate":
        plan["ground_points"][0]["id"] = plan["ground_points"][1]["id"]
    elif invalid == "empty":
        plan["ground_points"][0]["id"] = ""
    else:
        plan["ground_points"][0]["id"] = "g1"
    plan_path.write_text(json.dumps(plan), encoding="utf8")
    with pytest.raises(ValueError, match="five unique ground_.*nonnegative integer"):
        load_configuration(plan_path, profile_path)


@pytest.mark.parametrize("ground_ids", [
    [f"ground_{index}" for index in range(5)],
    ["ground_0", "ground_2", "ground_7", "ground_12", "ground_30"],
])
def test_official_map_preserves_configured_ground_ids_without_renumbering(tmp_path, ground_ids):
    from inspection_mission.mission_node import load_configuration
    from inspection_mission.runtime import RuntimeInputs

    plan_path, profile_path = _official_configuration(tmp_path)
    plan = json.loads(plan_path.read_text(encoding="utf8"))
    for point, ground_id in zip(plan["ground_points"], ground_ids):
        point["id"] = ground_id
    plan_path.write_text(json.dumps(plan), encoding="utf8")

    plan, profile, controller = load_configuration(plan_path, profile_path)
    assert [point["id"] for point in controller.plan["ground_points"]] == ground_ids
    data = RuntimeInputs(profile, set(ground_ids))
    selected = [ground_ids[3], ground_ids[1], ground_ids[0]]
    data.update_order({"data": ",".join(selected)})
    assert data.order == selected

    for unknown in {"ground_1", "ground_5"} - set(ground_ids):
        data = RuntimeInputs(profile, set(ground_ids))
        with pytest.raises(ValueError, match="selected ground points"):
            data.update_order({"data": ",".join([ground_ids[3], unknown, ground_ids[0]])})
        assert data.order is None


def test_official_documented_bindings_accept_all_five_measured_candidates(tmp_path):
    from inspection_mission.mission_node import load_configuration

    plan_path, profile_path = _official_configuration(tmp_path)
    plan, profile, controller = load_configuration(plan_path, profile_path)
    assert len(controller.plan["ground_points"]) == 5
    assert profile["channels"]["ground_tasks"]["selection_count"] == 3


def test_official_automatic_takeoff_speed_cannot_exceed_the_referee_limit(tmp_path):
    from inspection_mission.mission_node import load_configuration

    plan_path, profile_path = _official_configuration(tmp_path)
    profile = json.loads(profile_path.read_text(encoding="utf8"))
    profile.update(flight_mode="trigger", trigger_abort_stops_motion=True, trigger_max_speed_mps=0.51)
    for key in ("takeoff", "land", "flight_abort"):
        profile["channels"][key] = {"topic": "/test/" + key, "type": "std_msgs/msg/Empty", "fields": {}}
    profile_path.write_text(json.dumps(profile), encoding="utf8")
    with pytest.raises(ValueError, match="0.5"):
        load_configuration(plan_path, profile_path)


def test_official_recognition_adapter_uses_observed_category_and_required_phrase():
    channel = {"encoding": "inspection_phrase_v1", "fields": {"data": "$phrase"}}
    report = {"kind": "ground", "target_id": "被困人员", "point_id": "ground_2"}
    assert outbound_payload("recognition", channel, report) == {
        "data": "发现被困人员，地面巡航任务点确认，将巡航信息发送回指挥中心。"}
    with pytest.raises(ValueError):
        outbound_payload("recognition", channel, {**report, "target_id": "unknown"})


def test_declaration_adapter_preserves_only_explicit_current_stage_value():
    profile = valid_profile()
    for key in ("current_target", "cruise_event"):
        profile["channels"][key] = {"topic": "/test/" + key,
            "type": "std_msgs/msg/String", "fields": {"data": "$value"}}
    validate_outbound_fields(profile)
    assert outbound_payload("cruise_event", profile["channels"]["cruise_event"],
                            {"value": "window_1"}) == {"data": "window_1"}
    profile["channels"]["cruise_event"]["fields"] = {"data": "$unconfirmed"}
    with pytest.raises(ValueError, match="unconfirmed"):
        validate_outbound_fields(profile)


def test_official_plan_cannot_start_without_declared_official_output_channels(tmp_path):
    import json
    from tools.mission_scenario import demonstration_plan
    from inspection_mission.mission_node import load_configuration
    from test_runtime import configured_profile

    plan = demonstration_plan()
    plan["protocol"] = "inspection_v1"
    for point in plan["ground_points"]:
        point["approach"] = [point["pose"][0] - 0.4, point["pose"][1], point["pose"][2]]
    plan_path, profile_path = tmp_path / "plan.json", tmp_path / "profile.json"
    plan_path.write_text(json.dumps(plan), encoding="utf8")
    profile_path.write_text(json.dumps(configured_profile()), encoding="utf8")
    with pytest.raises(ValueError, match="official output channels"):
        load_configuration(plan_path, profile_path)
