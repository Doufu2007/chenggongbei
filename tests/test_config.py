import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from inspection_mission.validation import validate_configuration

ROOT = Path(__file__).resolve().parents[1]


def test_unbound_official_profile_is_a_clear_startup_error():
    with pytest.raises(ValueError, match="verified"):
        validate_configuration(ROOT / "src/inspection_mission/config/inspection.yaml")


def test_configuration_validates_real_plan_and_camera_contract(tmp_path):
    from tools.mission_scenario import demonstration_plan
    from test_protocol import valid_profile
    (tmp_path / "plan.json").write_text(json.dumps(demonstration_plan()), encoding="utf-8")
    profile = valid_profile()
    for name in ("ground_scan", "uav_scan"):
        profile["channels"][name].update(frame_id=name, yaw_offset_rad=0.0)
    (tmp_path / "profile.json").write_text(json.dumps(profile), encoding="utf-8")
    config = {
        "inspection_mission": {"ros__parameters": {"plan_path": str(tmp_path / "plan.json"), "interface_profile_path": str(tmp_path / "profile.json"), "use_sim_time": True}},
        "inspection_vision": {"ros__parameters": {"ground_image_topic": "/test/ground/image", "uav_image_topic": "/test/uav/image", "use_sim_time": True}}
    }
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    result = validate_configuration(path)
    assert len(result["plan"]["ground_points"]) == 3
    config["inspection_mission"]["ros__parameters"]["plan_path"] = "plan.json"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(ValueError, match="absolute"):
        validate_configuration(path)
    config["inspection_mission"]["ros__parameters"]["plan_path"] = str(tmp_path / "plan.json")
    config["inspection_vision"]["ros__parameters"]["uav_image_topic"] = ""
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(ValueError, match="camera"):
        validate_configuration(path)


def test_metric_target_sizes_require_a_camera_info_topic(tmp_path):
    from test_protocol import valid_profile
    from tools.mission_scenario import demonstration_plan

    (tmp_path / "plan.json").write_text(json.dumps(demonstration_plan()), encoding="utf-8")
    profile = valid_profile()
    for name in ("ground_scan", "uav_scan"):
        profile["channels"][name].update(frame_id=name, yaw_offset_rad=0.0)
    (tmp_path / "profile.json").write_text(json.dumps(profile), encoding="utf-8")
    config = {
        "inspection_mission": {"ros__parameters": {
            "plan_path": str(tmp_path / "plan.json"),
            "interface_profile_path": str(tmp_path / "profile.json"),
            "use_sim_time": True,
        }},
        "inspection_vision": {"ros__parameters": {
            "ground_image_topic": "/test/ground/image",
            "uav_image_topic": "/test/uav/image",
            "target_sizes_json": '{"target-a": [0.2, 0.2]}',
            "use_sim_time": True,
        }},
    }
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(ValueError, match="CameraInfo"):
        validate_configuration(path)


def test_configuration_rejects_ignored_legacy_global_uav_altitude(tmp_path):
    from test_protocol import valid_profile
    from tools.mission_scenario import demonstration_plan

    (tmp_path / "plan.json").write_text(json.dumps(demonstration_plan()), encoding="utf-8")
    profile = valid_profile()
    for name in ("ground_scan", "uav_scan"):
        profile["channels"][name].update(frame_id=name, yaw_offset_rad=0.0)
    (tmp_path / "profile.json").write_text(json.dumps(profile), encoding="utf-8")
    config = {
        "inspection_mission": {"ros__parameters": {
            "plan_path": str(tmp_path / "plan.json"),
            "interface_profile_path": str(tmp_path / "profile.json"),
            "use_sim_time": True,
            "uav_altitude_m": 1.2,
        }},
        "inspection_vision": {"ros__parameters": {
            "ground_image_topic": "/test/ground/image",
            "uav_image_topic": "/test/uav/image",
            "use_sim_time": True,
        }},
    }
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(ValueError, match="uav_altitude_m"):
        validate_configuration(path)


def test_external_yaml_empty_paths_use_explicit_package_config_root(tmp_path):
    """Blank paths follow the installed package root, not the YAML directory."""
    from test_protocol import valid_profile
    from tools.mission_scenario import demonstration_plan

    package_config = tmp_path / "installed_share" / "config"
    package_config.mkdir(parents=True)
    (package_config / "mission_plan.json").write_text(json.dumps(demonstration_plan()), encoding="utf-8")
    profile = valid_profile()
    for name in ("ground_scan", "uav_scan"):
        profile["channels"][name].update(frame_id=name, yaw_offset_rad=0.0)
    (package_config / "interface_profile.json").write_text(json.dumps(profile), encoding="utf-8")

    external = tmp_path / "external"
    external.mkdir()
    config = {
        "inspection_mission": {"ros__parameters": {
            "plan_path": "", "interface_profile_path": "", "use_sim_time": True,
        }},
        "inspection_vision": {"ros__parameters": {
            "ground_image_topic": "/test/ground/image", "uav_image_topic": "/test/uav/image",
            "use_sim_time": True,
        }},
    }
    path = external / "settings.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")

    result = validate_configuration(path, default_config_dir=package_config)
    assert len(result["plan"]["ground_points"]) == 3


def test_validation_cli_blocks_unconfigured_package_without_ros_import():
    result = subprocess.run([sys.executable, "-m", "inspection_mission.validation", "--config",
                             str(ROOT / "src/inspection_mission/config/inspection.yaml")],
                            env={**__import__("os").environ, "PYTHONPATH": str(ROOT / "src/inspection_mission")},
                            text=True, encoding="utf-8", errors="replace", capture_output=True)
    assert result.returncode == 2
    assert "BLOCKED" in result.stdout
    assert "rclpy" not in result.stderr


@pytest.mark.parametrize(("key", "value"), [
    ("control_rate_hz", 0.0), ("control_rate_hz", 101.0),
    ("scan_timeout_s", 0.0), ("scan_timeout_s", -0.1),
])
def test_preflight_rejects_runtime_invalid_timing_before_launch(tmp_path, key, value):
    config = yaml.safe_load((ROOT / "src/inspection_mission/config/inspection.yaml").read_text(encoding="utf-8"))
    config["inspection_mission"]["ros__parameters"][key] = value
    path = tmp_path / "invalid_timing.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match=key):
        validate_configuration(path)


@pytest.mark.parametrize(("key", "value"), [
    ("min_detection_interval_s", -0.1),
    ("min_detection_interval_s", "invalid"),
    ("aruco_dictionary", "DICT_NOT_REAL"),
])
def test_preflight_rejects_invalid_vision_runtime_options(tmp_path, key, value):
    config = yaml.safe_load((ROOT / "src/inspection_mission/config/inspection.yaml").read_text(encoding="utf-8"))
    config["inspection_vision"]["ros__parameters"][key] = value
    path = tmp_path / "invalid_vision_option.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match=key):
        validate_configuration(path)


def test_preflight_rejects_empty_internal_detection_topic(tmp_path):
    config = yaml.safe_load((ROOT / "src/inspection_mission/config/inspection.yaml").read_text(encoding="utf-8"))
    config["inspection_mission"]["ros__parameters"]["detection_topic"] = ""
    config["inspection_vision"]["ros__parameters"]["detection_topic"] = ""
    path = tmp_path / "empty_detection_topic.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match="detection_topic"):
        validate_configuration(path)


@pytest.mark.parametrize(("key", "value"), [
    ("mission_status_topic", ""),
    ("mission_status_topic", "/bad-name"),
    ("runtime_status_topic", None),
    ("runtime_status_topic", "/status//heartbeat"),
])
def test_preflight_rejects_invalid_status_topic_names(tmp_path, key, value):
    config = yaml.safe_load((ROOT / "src/inspection_mission/config/inspection.yaml").read_text(encoding="utf-8"))
    config["inspection_mission"]["ros__parameters"][key] = value
    path = tmp_path / "invalid_status_topic.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match=key):
        validate_configuration(path)


@pytest.mark.parametrize("structure", [
    None,
    "bad-node",
    {"ros__parameters": None},
    {"ros__parameters": "bad-parameters"},
])
def test_preflight_reports_malformed_node_structure_as_blocked(tmp_path, structure):
    path = tmp_path / "malformed.yaml"
    path.write_text(yaml.safe_dump({"inspection_mission": structure, "inspection_vision": {}}), encoding="utf-8")

    result = subprocess.run([sys.executable, "-m", "inspection_mission.validation", "--config", str(path)],
                            env={**__import__("os").environ, "PYTHONPATH": str(ROOT / "src/inspection_mission")},
                            text=True, encoding="utf-8", errors="replace", capture_output=True)

    assert result.returncode == 2
    assert "BLOCKED" in result.stdout
    assert "Traceback" not in result.stderr
