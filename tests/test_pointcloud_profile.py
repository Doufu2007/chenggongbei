"""Explicit front-lidar geometry must never become a fictitious all-round scan."""
import math

import pytest

from inspection_mission.mission_node import required_input_channels
from inspection_mission.protocol import validate_profile
from inspection_mission.runtime import RuntimeInputs
from test_runtime import configured_profile, odom, scan


def cloud_profile():
    profile = configured_profile()
    profile["uav_obstacle_sensor"] = "pointcloud"
    del profile["channels"]["uav_scan"]
    profile["channels"]["uav_cloud"] = {
        "topic": "/uav/lidar/points", "type": "sensor_msgs/msg/PointCloud2",
        "frame_id": "uav/lidar_link", "translation_m": [0.12, 0.0, 0.18],
        "rotation_rpy_rad": [0.0, 0.0, 0.0],
        "horizontal_fov_rad": math.pi, "vertical_fov_rad": 1.1,
        "range_min_m": 0.05, "range_max_m": 10.0,
    }
    return profile


def cloud_inputs(points):
    data = RuntimeInputs(cloud_profile(), {"g1", "g2", "g3"})
    data.update_pose("uav", odom(frame="uav/odom"), now=5)
    data.update_cloud({"frame_id": "uav/lidar_link", "stamp": {"sec": 5, "nanosec": 0}}, points, now=5)
    return data


def test_cloud_only_profile_needs_no_uav_horizontal_scan():
    profile = cloud_profile()
    assert validate_profile(profile) is profile
    data = RuntimeInputs(profile, {"g1", "g2", "g3"})
    assert data.obstacle_fresh("uav", 5) is False
    required = required_input_channels(profile)
    assert "uav_cloud" in required and "uav_scan" not in required


def test_ground_stage_does_not_wait_for_unobserved_uav_interfaces():
    required = required_input_channels(cloud_profile(), active_vehicle="ground")
    assert {"ground_odom", "ground_scan", "ground_tasks"}.issubset(required)
    assert not {"uav_cloud", "uav_scan", "uav_odom", "light_feedback"}.intersection(required)


@pytest.mark.parametrize("field,value", [
    ("translation_m", None), ("rotation_rpy_rad", [0, math.pi / 2, 0]),
    ("horizontal_fov_rad", 0), ("vertical_fov_rad", float("nan")),
    ("range_min_m", -1), ("range_max_m", 0),
])
def test_cloud_profile_rejects_missing_or_unsupported_geometry(field, value):
    profile = cloud_profile()
    profile["channels"]["uav_cloud"][field] = value
    with pytest.raises(ValueError):
        RuntimeInputs(profile, {"g1", "g2", "g3"})


def test_cloud_offset_moves_real_obstacle_into_body_stopping_corridor():
    # Return z=0 is 0.18m above the body; choose a narrow corridor around body z=0.
    # Far returns at sensor z=-0.18 lie on that body corridor.
    clear_returns = [[1, 0, -0.18], [1.2, 0, -0.18], [1.4, 0, -0.18]]
    data = cloud_inputs(clear_returns)
    assert data.motion_clear("uav", (0.2, 0, 0, 0), 5, stop_distance=0.6, corridor_radius=0.05)
    data = cloud_inputs(clear_returns + [[0.35, 0, -0.18]])
    assert not data.motion_clear("uav", (0.2, 0, 0, 0), 5, stop_distance=0.6, corridor_radius=0.05)


@pytest.mark.parametrize("velocity", [(0, 0, 0.2, 0), (0, 0, -0.2, 0), (-0.2, 0, 0, 0), (0, 0, 0, 0.2)])
def test_front_cloud_rejects_vertical_rearward_or_rotating_blind_motion(velocity):
    data = cloud_inputs([[1, 0, 0], [1.2, 0, 0], [1.4, 0, 0],
                         [0, 0, 1], [0, 0, 1.2], [0, 0, 1.4]])
    assert not data.motion_clear("uav", velocity, 5, stop_distance=0.6, corridor_radius=0.05)


def test_stale_cloud_stops_motion_but_ground_uses_its_own_scan():
    data = cloud_inputs([[1, 0, -0.18], [1.2, 0, -0.18], [1.4, 0, -0.18]])
    assert data.obstacle_fresh("uav", 5)
    assert not data.obstacle_fresh("uav", 6)
    data.update_pose("ground", odom(now=6), now=6)
    data.update_scan("ground", scan(now=6), now=6)
    assert data.motion_clear("ground", (0.2, 0), 6, stop_distance=0.3)
    assert not data.motion_clear("uav", (0.2, 0, 0, 0), 6, stop_distance=0.6, corridor_radius=0.05)


def test_entire_stopping_cross_section_must_fit_sensor_fov():
    data = cloud_inputs([[1, 0, -0.18], [1.2, 0, -0.18], [1.4, 0, -0.18]])
    # Lower edge falls outside the lidar's vertical aperture even for forward motion.
    assert not data.motion_clear("uav", (0.2, 0, 0, 0), 5, stop_distance=0.3, corridor_radius=0.15)


def test_cloud_range_underflow_does_not_disappear_behind_far_returns():
    data = cloud_inputs([[1, 0, -0.18], [1.2, 0, -0.18], [1.4, 0, -0.18], [0.01, 0, 0]])
    assert not data.motion_clear("uav", (0.2, 0, 0, 0), 5, stop_distance=0.6, corridor_radius=0.05)


def test_prelaunch_accepts_cloud_only_official_profile_with_confirmed_geometry(tmp_path):
    import json
    import yaml
    from inspection_mission.validation import validate_configuration
    from test_official_adapter import _official_configuration
    plan_path, profile_path = _official_configuration(tmp_path)
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    profile["uav_obstacle_sensor"] = "pointcloud"
    del profile["channels"]["uav_scan"]
    profile["channels"]["uav_cloud"] = cloud_profile()["channels"]["uav_cloud"]
    profile_path.write_text(json.dumps(profile), encoding="utf-8")
    config = {"inspection_mission": {"ros__parameters": {
        "plan_path": str(plan_path), "interface_profile_path": str(profile_path), "use_sim_time": True}},
        "inspection_vision": {"ros__parameters": {"ground_image_topic": "/ground/image",
            "uav_image_topic": "/uav/image", "use_sim_time": True}}}
    path = tmp_path / "inspection.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    checked = validate_configuration(path)
    assert "uav_scan" not in checked["profile"]["channels"]
