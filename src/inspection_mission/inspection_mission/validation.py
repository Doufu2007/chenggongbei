"""Pre-launch validation. No ROS imports and no actuator side effects."""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import yaml

from .mission_node import load_configuration, number
from .protocol import uav_obstacle_sensor


def _validate_topic_name(name, value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty ROS topic name")
    normalized = value[1:] if value.startswith("/") else value
    tokens = normalized.split("/")
    if (not normalized or value.endswith("/")
            or any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", token) for token in tokens)):
        raise ValueError(f"{name} is not a valid ROS topic name")


def validate_configuration(config_path: Path | str, *, default_config_dir: Path | str | None = None) -> dict:
    path = Path(config_path).resolve()
    config_root = Path(default_config_dir).resolve() if default_config_dir is not None else path.parent
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("ROS parameter file must be a mapping")
    def node_parameters(name):
        node = data.get(name, {})
        if not isinstance(node, dict):
            raise ValueError(f"{name} must be a mapping")
        params = node.get("ros__parameters", {})
        if not isinstance(params, dict):
            raise ValueError(f"{name}.ros__parameters must be a mapping")
        return params

    params = node_parameters("inspection_mission")
    vision = node_parameters("inspection_vision")
    control_rate = number(params.get("control_rate_hz", 20.0), "control_rate_hz")
    if not 1 <= control_rate <= 100:
        raise ValueError("control_rate_hz must lie in [1,100]")
    scan_timeout = number(params.get("scan_timeout_s", 0.5), "scan_timeout_s")
    if scan_timeout <= 0:
        raise ValueError("scan_timeout_s must be positive")
    detection_interval = number(vision.get("min_detection_interval_s", 0.25), "min_detection_interval_s")
    if detection_interval < 0:
        raise ValueError("min_detection_interval_s must not be negative")
    for name in ("enable_qr", "enable_aruco"):
        if name in vision and type(vision[name]) is not bool:
            raise ValueError(f"{name} must be boolean")
    if vision.get("enable_aruco", True):
        from .vision_core import validate_aruco_dictionary
        validate_aruco_dictionary(vision.get("aruco_dictionary", "DICT_4X4_50"))
    observations = params.get("detection_topic", "/inspection_mission/observations")
    vision_observations = vision.get("detection_topic", "/inspection_mission/observations")
    _validate_topic_name("detection_topic", observations)
    _validate_topic_name("vision detection_topic", vision_observations)
    if observations != vision_observations:
        raise ValueError("vision and controller detection topics must match")
    for name, default in (("mission_status_topic", "/inspection/mission_status"),
                          ("runtime_status_topic", "/inspection_mission/status")):
        _validate_topic_name(name, params.get(name, default))
    if "uav_altitude_m" in params:
        raise ValueError("uav_altitude_m is unsupported; use measured per-stage plan altitudes and min_aerial_height_m")
    def resolve(name, default):
        raw = params.get(name)
        if not raw:
            return config_root / default
        value = Path(raw)
        if not value.is_absolute():
            raise ValueError(f"{name} must be absolute when configured")
        return value
    overrides = {key: value for key, value in params.items() if key in (
        "pose_timeout_s", "detection_timeout_s", "recognition_stable_s", "illumination_s",
        "phase_timeout_s", "no_progress_timeout_s", "obstacle_timeout_s", "require_ground_order",
        "require_detection", "recognition_transport_wait_s",
    )}
    plan, profile, controller = load_configuration(resolve("plan_path", "mission_plan.json"),
        resolve("interface_profile_path", "interface_profile.json"), overrides)
    settings = controller.settings
    for source in ("ground", "uav"):
        topic = vision.get(source + "_image_topic", "")
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ValueError(f"missing verified {source} camera topic")
        if source != "uav" or uav_obstacle_sensor(profile) != "pointcloud":
            scan = profile["channels"][source + "_scan"]
            if not scan.get("frame_id") or not isinstance(scan.get("yaw_offset_rad"), (float, int)):
                raise ValueError(f"missing verified {source} scan frame/extrinsics")
    if vision["ground_image_topic"] == vision["uav_image_topic"]:
        raise ValueError("ground/uav camera topics must be distinct")
    if params.get("use_sim_time") is not True or vision.get("use_sim_time") is not True:
        raise ValueError("both nodes must use the simulation clock")
    if observations in {channel["topic"] for channel in profile["channels"].values()}:
        raise ValueError("internal visual observations must not share a referee/control topic")
    references = json.loads(vision.get("template_paths_json", "{}"))
    if not isinstance(references, dict):
        raise ValueError("template_paths_json must be an object")
    if references:
        from .vision_core import TemplateMatcher, normalise_template_paths
        paths = normalise_template_paths(references, require_absolute=True)
        TemplateMatcher(paths)
    sizes = json.loads(vision.get("target_sizes_json", "{}"))
    if not isinstance(sizes, dict):
        raise ValueError("target_sizes_json must be an object")
    for size in sizes.values():
        if not isinstance(size, list) or len(size) != 2 or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0 for v in size):
            raise ValueError("physical target sizes must be positive finite [width,height]")
    info_topics = [vision.get(name, "") for name in (
        "camera_info_topic", "ground_camera_info_topic", "uav_camera_info_topic")]
    if any(not isinstance(topic, str) or (topic and not topic.startswith("/")) for topic in info_topics):
        raise ValueError("CameraInfo topics must be empty or verified absolute topic names")
    if sizes and not any(info_topics):
        raise ValueError("physical target sizes require at least one verified CameraInfo topic")
    return {"plan": plan, "profile": profile, "settings": settings, "vision": vision}


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--default-config-dir", help="installed package share/config root for blank plan/profile paths")
    opts = parser.parse_args(args)
    try:
        validate_configuration(opts.config, default_config_dir=opts.default_config_dir)
    except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as exc:
        print(f"BLOCKED: {exc}")
        return 2
    print("Configuration complete; platform execution still requires actual ROS interfaces.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
