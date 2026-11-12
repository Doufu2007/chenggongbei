"""Explicit, configurable ROS message bindings; this is not an official schema.

The platform must supply/confirm the message contract. An unverified profile
cannot start actuators. No task identity is invented when parsing fails.
"""
from __future__ import annotations

import json
import math
from typing import Any


REQUIRED_CHANNELS = {
    "ground_cmd", "uav_cmd", "ground_odom", "uav_odom", "ground_scan", "uav_scan",
    "ground_tasks", "recognition", "recognition_ack", "light", "light_feedback",
}
FIXED_TYPES = {
    "ground_cmd": "geometry_msgs/msg/Twist", "uav_cmd": "geometry_msgs/msg/Twist",
    "ground_odom": "nav_msgs/msg/Odometry", "uav_odom": "nav_msgs/msg/Odometry",
    "ground_scan": "sensor_msgs/msg/LaserScan", "uav_scan": "sensor_msgs/msg/LaserScan",
}


def recognition_confirmation_mode(profile: dict) -> str:
    mode = profile.get("recognition_confirmation_mode", "event_ack")
    if mode not in ("event_ack", "transport_only"):
        raise ValueError("recognition_confirmation_mode must be event_ack or transport_only")
    return mode


def uav_obstacle_sensor(profile: dict) -> str:
    mode = profile.get("uav_obstacle_sensor", "scan")
    if mode not in ("scan", "pointcloud"):
        raise ValueError("uav_obstacle_sensor must be scan or pointcloud")
    return mode


def validate_cloud_geometry(spec: dict) -> None:
    """Only the explicitly measured, unrotated sensor transform is supported."""
    if spec.get("type") != "sensor_msgs/msg/PointCloud2" or not spec.get("frame_id"):
        raise ValueError("uav_cloud requires PointCloud2 and its confirmed sensor frame_id")
    for key in ("translation_m", "rotation_rpy_rad"):
        vector = spec.get(key)
        if (not isinstance(vector, (list, tuple)) or len(vector) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in vector)):
            raise ValueError(f"uav_cloud.{key} must contain three finite values")
    if any(abs(v) > 1e-12 for v in spec["rotation_rpy_rad"]):
        raise ValueError("rotated uav_cloud sensors are unsupported; do not treat a downward scanner as horizontal")
    for key, maximum in (("horizontal_fov_rad", 2 * math.pi), ("vertical_fov_rad", math.pi)):
        value = spec.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0 < value <= maximum):
            raise ValueError(f"uav_cloud.{key} must be finite, positive and within its angular range")
    lower, upper = spec.get("range_min_m"), spec.get("range_max_m")
    if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (lower, upper))
            or not 0 <= lower < upper):
        raise ValueError("uav_cloud range_min_m/range_max_m must be explicitly confirmed")


def validate_profile(profile: dict) -> dict:
    if profile.get("verified") is not True:
        raise ValueError("interface profile must be verified against the actual platform")
    if profile.get("uav_velocity_frame") not in ("body", "world"):
        raise ValueError("uav_velocity_frame must be explicitly body or world")
    if profile.get("flight_mode") not in ("velocity", "trigger"):
        raise ValueError("flight_mode must be velocity or trigger (confirmed by platform)")
    for key in ("ground_frame", "uav_frame"):
        if not isinstance(profile.get(key), str) or not profile[key].strip():
            raise ValueError(f"missing {key}")
    mode = recognition_confirmation_mode(profile)
    channels = profile.get("channels", {})
    required = REQUIRED_CHANNELS - ({"recognition_ack"} if mode == "transport_only" else set())
    if uav_obstacle_sensor(profile) == "pointcloud":
        required = required - {"uav_scan"} | {"uav_cloud"}
    missing = required - channels.keys()
    if profile.get("flight_mode") == "trigger":
        missing |= {"takeoff", "land"} - channels.keys()
    if missing:
        raise ValueError(f"missing channels: {sorted(missing)}")
    if uav_obstacle_sensor(profile) == "pointcloud":
        validate_cloud_geometry(channels["uav_cloud"])
    topics = []
    for key, channel in channels.items():
        topic = channel.get("topic", "")
        if not isinstance(topic, str) or not topic.startswith("/") or " " in topic:
            raise ValueError(f"missing absolute topic for {key}")
        if topic.startswith("/gazebo/") or topic in ("/referee/events", "/referee/delivery_order"):
            raise ValueError(f"forbidden interface: {topic}")
        ros_type = channel.get("type", "")
        if not isinstance(ros_type, str) or len(ros_type.split("/")) != 3 or "/msg/" not in ros_type:
            raise ValueError(f"missing ROS message type for {key}")
        if key in FIXED_TYPES and ros_type != FIXED_TYPES[key]:
            raise ValueError(f"unsupported {key} type: {ros_type}")
        topics.append(topic)
    if len(topics) != len(set(topics)):
        raise ValueError("each channel must have a distinct topic")
    for key in ("recognition", "light"):
        if not isinstance(channels[key].get("fields"), dict) or not channels[key]["fields"]:
            raise ValueError(f"missing message field template: {key}")
    for key, fields in {
        "ground_tasks": ("path", "id_field"),
        "recognition_ack": ("path", "id_field", "accepted_field"),
        "light_feedback": ("path",),
    }.items():
        if key == "recognition_ack" and mode == "transport_only":
            continue
        for field in fields:
            if field not in channels[key] or not isinstance(channels[key][field], str):
                raise ValueError(f"missing selector {key}.{field}")
    selection_count = channels["ground_tasks"].get("selection_count")
    if selection_count is not None and (type(selection_count) is not int or selection_count < 1):
        raise ValueError("ground_tasks.selection_count must be a positive integer")
    if channels["ground_tasks"].get("encoding", "json") not in ("json", "csv"):
        raise ValueError("unsupported ground_tasks encoding")
    return profile


def build_payload(template: Any, values: dict) -> Any:
    if isinstance(template, dict):
        return {key: build_payload(value, values) for key, value in template.items()}
    if isinstance(template, list):
        return [build_payload(value, values) for value in template]
    if isinstance(template, str) and template.startswith("$"):
        name = template[1:]
        if name == "json":
            return json.dumps(values, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        if name not in values:
            raise ValueError(f"missing output field: {name}")
        return values[name]
    return template


def _unwrap(payload: Any) -> Any:
    if isinstance(payload, dict) and set(payload) == {"data"} and isinstance(payload["data"], str):
        try:
            return json.loads(payload["data"])
        except (json.JSONDecodeError, ValueError) as exc:
            raise ValueError("String payload must contain configured JSON") from exc
    return payload


def select(payload: Any, path: str) -> Any:
    value = payload
    for part in path.split(".") if path else []:
        if not isinstance(value, dict) or part not in value:
            raise ValueError(f"missing configured message path: {path}")
        value = value[part]
    return value


def decode_order(payload: Any, spec: dict, known_ids: set[str]) -> list[str]:
    """Decode an explicitly configured order, never guess String encoding.

    The official巡检表1 uses a comma-separated String.data.  JSON remains
    available for other explicitly verified profiles and offline fixtures.
    """
    encoding = spec.get("encoding", "json")
    if encoding == "csv":
        values = select(payload, spec["path"])
        if not isinstance(values, str) or spec.get("id_field", ""):
            raise ValueError("CSV ground task order requires string data without id_field")
        result = [value.strip() for value in values.split(",")]
    elif encoding == "json":
        values = select(_unwrap(payload), spec["path"])
        if not isinstance(values, list):
            raise ValueError("ground task order must be a list")
        result = [select(value, spec["id_field"]) if spec["id_field"] else value for value in values]
    else:
        raise ValueError("unsupported ground task encoding")
    if any(not isinstance(value, str) or not value for value in result):
        raise ValueError("task identities must be nonempty strings")
    selection_count = spec.get("selection_count")
    if selection_count is None:
        selection_count = len(known_ids)
    if (type(selection_count) is not int or selection_count < 1
            or selection_count > len(known_ids)):
        raise ValueError("ground task selection_count must be a valid positive integer")
    if (len(result) != selection_count or len(set(result)) != len(result)
            or not set(result).issubset(known_ids)):
        raise ValueError("order must contain the selected ground points exactly once")
    return result


def decode_ack(payload: Any, spec: dict) -> set[str]:
    values = select(_unwrap(payload), spec["path"])
    if isinstance(values, dict):
        values = [values]
    if not isinstance(values, list):
        raise ValueError("recognition confirmation must be a record/list")
    result = set()
    for item in values:
        if select(item, spec["accepted_field"]) is True:
            event_id = select(item, spec["id_field"])
            if not isinstance(event_id, str) or not event_id:
                raise ValueError("confirmed event ID must be a nonempty string")
            result.add(event_id)
    return result


def decode_bool(payload: Any, spec: dict) -> bool:
    value = select(_unwrap(payload), spec["path"])
    if type(value) is not bool:
        raise ValueError("feedback must be an explicit boolean")
    return value


def planar_velocity(vx: float, vy: float, yaw: float, frame: str) -> tuple[float, float]:
    if not all(math.isfinite(value) for value in (vx, vy, yaw)):
        raise ValueError("nonfinite velocity/pose")
    if frame == "world":
        return vx, vy
    if frame != "body":
        raise ValueError("velocity frame has not been configured")
    return math.cos(yaw) * vx + math.sin(yaw) * vy, -math.sin(yaw) * vx + math.cos(yaw) * vy


def sensor_time(sec: int, nanosec: int, now: float) -> float:
    stamp = float(sec) + float(nanosec) / 1e9
    if not math.isfinite(stamp) or stamp < 0 or stamp > now + 0.1:
        raise ValueError("invalid/future sensor stamp; check use_sim_time")
    return stamp
