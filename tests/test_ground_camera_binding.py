"""A ground task may explicitly use a real UAV camera without flying."""
from __future__ import annotations

import copy
import json

import pytest

from inspection_mission.mission import MissionController
from inspection_mission.mission_node import load_configuration, outbound_payload
from inspection_mission.runtime import RuntimeInputs
from tools.mission_scenario import demonstration_plan
from test_mission import at_ground, detection
from test_runtime import configured_profile


FRAME = "uav/camera_link"


def bound_plan():
    plan = demonstration_plan()
    plan["ground_points"][0].update(camera_source="uav", camera_frame_id=FRAME,
                                     target_id="被困人员")
    return plan


def first_recognition(plan=None, **settings):
    mission = MissionController(plan or bound_plan(), settings)
    mission.step(0., at_ground(), None, [], ground_order=["g1", "g2", "g3"])
    mission.step(.1, at_ground(now=.1), None, [])
    assert mission.state == "GROUND_RECOGNIZE_1"
    return mission


def observed(now, **changes):
    return detection(now, target="被困人员", source="uav", frame_id=FRAME, **changes)


def collect_report(mission, start=2, end=10, overrides=None):
    reports = []
    for tick in range(start, end):
        now = tick / 10
        item = observed(now)
        item.update(overrides or {})
        command = mission.step(now, at_ground(now=now), None, [item])
        assert command.uav == (0., 0., 0., 0.) and not command.light
        if command.recognition:
            reports.append(command.recognition)
    return reports


def test_explicit_ground_binding_uses_uav_observation_and_keeps_real_provenance_and_ground_phrase():
    mission = first_recognition(recognition_stable_s=.3)
    reports = collect_report(mission)
    assert reports, "explicitly bound UAV camera should satisfy ground recognition"
    report = reports[-1]
    assert mission.active_vehicle == "ground" and mission.state == "GROUND_RECOGNIZE_1"
    assert report["source"] == report["camera_source"] == "uav"
    assert report["vehicle"] == report["kind"] == "ground"
    assert report["frame_id"] == FRAME
    assert report["image_stamp"] <= report["stamp"]
    channel = {"encoding": "inspection_phrase_v1", "fields": {"data": "$phrase"}}
    message = outbound_payload("recognition", channel, report)
    assert message["data"] == "发现被困人员，地面巡航任务点确认，将巡航信息发送回指挥中心。"


def test_unconfigured_ground_stage_keeps_rejecting_uav_camera():
    plan = demonstration_plan()
    plan["ground_points"][0]["target_id"] = "被困人员"
    mission = first_recognition(plan, recognition_stable_s=.2)
    assert collect_report(mission, end=15) == []
    assert mission.state == "GROUND_RECOGNIZE_1"


@pytest.mark.parametrize("bad", [
    {"source": "ground"}, {"frame_id": "ground/camera_link"}, {"frame_id": ""},
    {"frame_id": None}, {"stamp": -1.}, {"stamp": 100.},
])
def test_explicit_binding_rejects_wrong_source_frame_missing_frame_and_old_or_future_exposure(bad):
    mission = first_recognition(recognition_stable_s=.2)
    assert collect_report(mission, end=15, overrides=bad) == []
    assert mission.state == "GROUND_RECOGNIZE_1"


def test_invalid_frame_breaks_prior_dwell_and_old_ack_cannot_advance():
    mission = first_recognition(recognition_stable_s=.2)
    first = collect_report(mission, end=7)[-1]
    command = mission.step(.7, at_ground(now=.7), None,
                           [observed(.7, center=[.5, .5]) | {"frame_id": "old_camera_frame"}],
                           recognition_ack={first["event_id"]})
    assert command.recognition is None and mission.state == "GROUND_RECOGNIZE_1"
    for tick in range(8, 10):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None, [observed(now)],
                               recognition_ack={first["event_id"]})
        assert command.recognition is None
    second = mission.step(1., at_ground(now=1.), None, [observed(1.)],
                          recognition_ack={first["event_id"]})
    assert second.recognition and second.recognition["event_id"] != first["event_id"]


def test_ground_visual_servo_with_uav_camera_only_returns_ground_yaw():
    mission = first_recognition(visual_servo_enabled=True, camera_yaw_sign=1)
    command = mission.step(.2, at_ground(now=.2), None, [observed(.2, center=[.8, .5])])
    assert command.ground[0] == 0. and command.ground[1] > 0.
    assert command.uav == (0., 0., 0., 0.) and not command.light
    assert mission.active_vehicle == "ground"


def test_camera_binding_is_scoped_to_one_ground_point_and_does_not_leak_to_next_stage():
    mission = first_recognition(recognition_stable_s=.2)
    report = collect_report(mission, end=8)[-1]
    command = mission.step(.8, at_ground(now=.8), None, [observed(.8)],
                           recognition_ack={report["event_id"]})
    assert command.state == "GROUND_NAV_2"
    mission.step(.9, at_ground(1, now=.9), None, [])
    assert mission.state == "GROUND_RECOGNIZE_2"
    for tick in range(10, 16):
        now = tick / 10
        command = mission.step(now, at_ground(1, now=now), None,
                               [detection(now, "ground-two", source="uav", frame_id=FRAME)])
        assert command.recognition is None and command.uav == (0., 0., 0., 0.)
    for tick in range(16, 21):
        now = tick / 10
        command = mission.step(now, at_ground(1, now=now), None, [detection(now, "ground-two")])
    assert command.recognition and command.recognition["source"] == "ground"


@pytest.mark.parametrize("updates", [
    {"camera_source": "uav"}, {"camera_source": "other", "camera_frame_id": FRAME},
    {"camera_source": "uav", "camera_frame_id": ""}, {"camera_source": True, "camera_frame_id": FRAME},
    {"camera_frame_id": FRAME}, {"camera_source": "uav", "camera_frame_id": None},
])
def test_explicit_camera_bindings_must_name_a_real_source_and_confirmed_frame(updates):
    plan = demonstration_plan()
    plan["ground_points"][0].update(updates)
    with pytest.raises(ValueError, match="camera"):
        MissionController(plan)


@pytest.mark.parametrize("group", ["aerial_points", "windows"])
def test_ground_camera_override_cannot_silently_change_air_or_window_camera(group):
    plan = bound_plan()
    plan[group][0].update(camera_source="ground", camera_frame_id="ground/camera")
    with pytest.raises(ValueError, match="camera"):
        MissionController(plan)


def test_runtime_keeps_uav_source_frame_and_exposure_for_ground_binding_and_clears_processing_error():
    inputs = RuntimeInputs(configured_profile(), {"g1", "g2", "g3"})
    mission = first_recognition(recognition_stable_s=.2)
    reports = []
    for tick in range(2, 8):
        now = tick / 10
        payload = {"source": "uav", "frame_id": FRAME, "stamp": now,
                   "detections": [{"target_id": "被困人员", "center": [.5, .5], "confidence": .95}]}
        original = copy.deepcopy(payload)
        inputs.update_detections(payload, now=now)
        retained = inputs.fresh_detections(now)
        assert retained[0]["source"] == "uav" and retained[0]["frame_id"] == FRAME
        assert retained[0]["stamp"] == now and payload == original
        command = mission.step(now, at_ground(now=now), None, retained)
        if command.recognition:
            reports.append(command.recognition)
    assert reports
    with pytest.raises(ValueError, match="processing error"):
        inputs.update_detections({"source": "uav", "stamp": .8, "frame_id": FRAME,
                                  "error": "frame processing failed", "detections": []}, now=.8)
    assert inputs.frames["uav"] is None
    command = mission.step(.8, at_ground(now=.8), None, inputs.fresh_detections(.8),
                           recognition_ack={reports[-1]["event_id"]})
    assert command.recognition is None and mission.state == "GROUND_RECOGNIZE_1"


def test_configuration_loader_preserves_explicit_binding_without_relabelling_camera_or_vehicle(tmp_path):
    plan_path, profile_path = tmp_path / "plan.json", tmp_path / "profile.json"
    plan_path.write_text(json.dumps(bound_plan(), ensure_ascii=False), encoding="utf-8")
    profile_path.write_text(json.dumps(configured_profile()), encoding="utf-8")
    plan, _, mission = load_configuration(plan_path, profile_path)
    assert plan["ground_points"][0]["camera_source"] == "uav"
    mission.step(0., at_ground(), None, [], ground_order=["g1", "g2", "g3"])
    mission.step(.1, at_ground(now=.1), None, [])
    assert collect_report(mission), "loaded explicit binding must reach runtime behavior"
