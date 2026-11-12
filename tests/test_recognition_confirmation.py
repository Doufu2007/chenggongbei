"""Transport delivery is an observation, never a fabricated referee ACK."""

import json

import pytest

from inspection_mission.control import Pose2D
from inspection_mission.mission import MissionController
from inspection_mission.mission_node import load_configuration
from inspection_mission.protocol import validate_profile
from inspection_mission.runtime import RuntimeInputs
from test_official_adapter import _official_configuration
from test_protocol import valid_profile
from test_runtime import configured_profile
from tools.mission_scenario import demonstration_plan


def transport_inspection():
    plan = demonstration_plan(protocol="inspection_v1")
    mission = MissionController(plan, {"recognition_confirmation_mode": "transport_only",
                                       "recognition_transport_wait_s": 0.2,
                                       "recognition_stable_s": 0.1})
    target = plan["ground_points"][0]["approach"]

    def step(now, images=True, acknowledgements=None):
        image = {"target_id": "ground-one", "source": "ground", "stamp": now,
                 "center": [0.5, 0.5], "confidence": 0.95}
        pose = Pose2D(x=target[0], y=target[1], yaw=target[2], stamp=now)
        return mission.step(now, pose, None, [image] if images else [],
                            ground_order=["g1", "g2", "g3"], recognition_ack=acknowledgements)

    step(0)
    step(0.1)
    step(0.2)
    report = step(0.3).recognition
    assert report
    return mission, step, report


def test_transport_profile_does_not_require_an_undocumented_ack_channel():
    profile = valid_profile()
    profile["recognition_confirmation_mode"] = "transport_only"
    del profile["channels"]["recognition_ack"]
    assert validate_profile(profile) is profile
    from inspection_mission.mission_node import required_input_channels
    assert "recognition_ack" not in required_input_channels(profile)
    assert "recognition_ack" in required_input_channels(valid_profile())


def test_event_ack_remains_default_and_unknown_modes_are_rejected():
    profile = valid_profile()
    del profile["channels"]["recognition_ack"]
    with pytest.raises(ValueError, match="recognition_ack"):
        validate_profile(profile)
    profile["recognition_confirmation_mode"] = "pretend_success"
    with pytest.raises(ValueError, match="recognition_confirmation_mode"):
        validate_profile(profile)


def test_only_profile_can_enable_transport_mode_in_loaded_configuration(tmp_path):
    plan_path, profile_path = _official_configuration(tmp_path)
    with pytest.raises(ValueError, match="profile"):
        load_configuration(plan_path, profile_path, {"recognition_confirmation_mode": "transport_only"})
    profile = json.loads(profile_path.read_text(encoding="utf8"))
    profile["recognition_confirmation_mode"] = "transport_only"
    del profile["channels"]["recognition_ack"]
    profile_path.write_text(json.dumps(profile), encoding="utf8")
    _, _, mission = load_configuration(plan_path, profile_path)
    assert mission.settings["recognition_confirmation_mode"] == "transport_only"


def test_transport_runtime_refuses_ack_injection_and_never_populates_ack_set():
    profile = configured_profile()
    profile["recognition_confirmation_mode"] = "transport_only"
    del profile["channels"]["recognition_ack"]
    inputs = RuntimeInputs(profile, {"g1", "g2", "g3"})
    with pytest.raises(ValueError, match="transport_only"):
        inputs.update_ack({"results": [{"event_id": "guessed", "accepted": True}]})
    assert inputs.acknowledgements == set()


def test_transport_mode_requires_successful_publication_then_wait_with_fresh_evidence():
    mission, step, report = transport_inspection()
    step(0.6, acknowledgements={report["event_id"]})
    assert mission.state == "GROUND_RECOGNIZE_1"  # ACK cannot substitute for publication.
    mission.mark_recognition_published(report["event_id"], 0.6)
    step(0.7)
    assert mission.state == "GROUND_RECOGNIZE_1"
    step(0.8)
    assert mission.state == "GROUND_ENTER_1"
    assert not mission.completed
    status = mission.recognition_status()
    assert status["recognition_confirmation_mode"] == "transport_only"
    assert status["recognition_reports_published"] == 1
    assert status["official_scoring"] == "unverified"


def test_stale_publication_cannot_confirm_a_reacquired_target():
    mission, step, first = transport_inspection()
    mission.mark_recognition_published(first["event_id"], 0.3)
    step(0.4, images=False)
    step(0.5)
    step(0.6)
    second = step(0.7).recognition
    assert second and second["event_id"] != first["event_id"]
    mission.mark_recognition_published(first["event_id"], 0.7)
    step(1.0)
    assert mission.state == "GROUND_RECOGNIZE_1"
    mission.mark_recognition_published(second["event_id"], 1.0)
    step(1.2)
    assert mission.state == "GROUND_ENTER_1"


def test_failed_publication_never_marks_transport_delivery():
    from inspection_mission.mission_node import publish_recognition_report
    mission, step, report = transport_inspection()

    def failed_publish(_report):
        raise RuntimeError("ROS publication failed")

    with pytest.raises(RuntimeError):
        publish_recognition_report(mission, report, failed_publish, 0.3)
    step(0.6)
    assert mission.state == "GROUND_RECOGNIZE_1"
    assert mission.recognition_status()["recognition_reports_published"] == 0
    published = []
    publish_recognition_report(mission, report, published.append, 0.6)
    step(0.8)
    assert published == [report]
    assert mission.state == "GROUND_ENTER_1"


@pytest.mark.parametrize("settings", [
    {"recognition_confirmation_mode": "guess"},
    {"recognition_transport_wait_s": 0},
    {"recognition_transport_wait_s": float("nan")},
])
def test_invalid_confirmation_settings_do_not_start_controller(settings):
    with pytest.raises(ValueError, match="recognition"):
        MissionController(demonstration_plan(protocol="inspection_v1"), settings)


def test_prelaunch_configuration_accepts_transport_profile_and_validates_wait(tmp_path):
    import yaml
    from inspection_mission.validation import validate_configuration

    plan_path, profile_path = _official_configuration(tmp_path)
    profile = json.loads(profile_path.read_text(encoding="utf8"))
    profile["recognition_confirmation_mode"] = "transport_only"
    del profile["channels"]["recognition_ack"]
    profile_path.write_text(json.dumps(profile), encoding="utf8")
    params = {"plan_path": str(plan_path), "interface_profile_path": str(profile_path),
              "use_sim_time": True, "recognition_transport_wait_s": 0.4}
    config = {"inspection_mission": {"ros__parameters": params},
              "inspection_vision": {"ros__parameters": {
                  "ground_image_topic": "/test/ground/image", "uav_image_topic": "/test/uav/image",
                  "use_sim_time": True}}}
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf8")
    checked = validate_configuration(path)
    assert checked["settings"]["recognition_confirmation_mode"] == "transport_only"
    assert checked["settings"]["recognition_transport_wait_s"] == 0.4
    params["recognition_transport_wait_s"] = 0
    path.write_text(yaml.safe_dump(config), encoding="utf8")
    with pytest.raises(ValueError, match="recognition_transport_wait_s"):
        validate_configuration(path)


def test_complete_offline_transport_flow_never_receives_referee_ack(monkeypatch):
    from inspection_mission.mission_node import publish_recognition_report
    from tools.mission_scenario import run_scenario

    original_step = MissionController.step
    observed = []
    missions = []

    def local_transport_step(self, now, *args, **kwargs):
        assert not kwargs.get("recognition_ack")
        command = original_step(self, now, *args, **kwargs)
        if command.recognition:
            publish_recognition_report(self, command.recognition, observed.append, now)
        missions[:] = [self]
        return command

    monkeypatch.setattr(MissionController, "step", local_transport_step)
    result = run_scenario(demonstration_plan(protocol="inspection_v1"), acknowledge=False,
                          settings={"recognition_confirmation_mode": "transport_only"})
    assert result.completed, result.fault
    assert {report["kind"] for report in observed} == {"ground", "window", "aerial"}
    assert missions[0].recognition_status()["official_scoring"] == "unverified"
