import importlib
import math

import cv2
import numpy as np
import pytest


def _perception():
    assert importlib.util.find_spec("inspection_mission.perception") is not None, "safe perception geometry is missing"
    return importlib.import_module("inspection_mission.perception")


def _scan_clear(ranges, **kwargs):
    parameters = dict(angle_min=-math.pi, angle_increment=math.pi / 180,
                      world_direction=0.0, robot_yaw=0.0, stop_distance=0.8,
                      range_min=0.1, range_max=8.0)
    parameters.update(kwargs)
    return _perception().scan_path_clear(ranges, **parameters)


def test_scan_world_direction_rotates_into_robot_frame():
    scan = [5.0] * 361
    scan[90] = 0.35  # scan angle -pi/2; world east when robot faces north
    assert _scan_clear(scan, world_direction=0.0, robot_yaw=math.pi / 2) is False
    assert _scan_clear(scan, world_direction=math.pi / 2, robot_yaw=math.pi / 2) is True


def test_scan_mount_rotation_and_wrap_are_respected():
    scan = [5.0] * 361
    scan[180] = 0.3
    assert _scan_clear(scan, world_direction=math.pi, robot_yaw=0.0, scan_yaw_offset=math.pi) is False
    scan = [5.0] * 361
    scan[0] = 0.3
    assert _scan_clear(scan, world_direction=math.pi, robot_yaw=0.0) is False


@pytest.mark.parametrize("scan", [[], [float("nan")] * 361, [0.0] * 361, [-1.0] * 361,
                                  [float("nan")] * 180 + [5.0] + [float("nan")] * 180])
def test_insufficient_scan_observations_are_not_clear(scan):
    assert _scan_clear(scan) is False


def test_scan_outside_field_of_view_is_not_clear():
    assert _scan_clear([5.0] * 91, angle_min=-math.pi / 4, world_direction=math.pi) is False


def test_scan_no_return_is_clear_only_with_documented_sufficient_range():
    assert _scan_clear([float("inf")] * 361) is True
    assert _scan_clear([float("inf")] * 361, range_max=float("inf")) is False
    assert _scan_clear([float("inf")] * 361, range_max=0.5) is False


def test_scan_blind_zone_larger_than_stopping_margin_cannot_claim_clearance():
    assert _scan_clear([5.0] * 361, range_min=0.9) is False


def test_scan_below_range_min_in_motion_corridor_is_not_clear():
    scan = [3.0] * 361
    scan[180] = 0.03  # Forward return is below the declared 0.05 m minimum.
    assert _scan_clear(scan, range_min=0.05) is False


def test_nearby_obstacle_blocks_even_among_mostly_invalid_ranges():
    scan = [float("nan")] * 361
    scan[180] = 0.2
    assert _scan_clear(scan) is False


def test_scan_detects_near_side_obstacle_in_robot_swept_footprint():
    scan = [5.0] * 361
    scan[206] = 0.22  # x=.198,y=.096; inside a radius .11 collision corridor
    assert _scan_clear(scan, corridor_radius=0.11) is False


def test_aim_uses_both_axes_and_rejects_invalid_normalized_points():
    aim_centered = _perception().aim_centered
    assert aim_centered([0.51, 0.48], tolerance_x=0.03, tolerance_y=0.03) is True
    for point in [[0.7, 0.5], [0.5, 0.2], [-1, 0.5], [float("nan"), 0.5], None]:
        assert aim_centered(point) is False


def test_camera_projection_recovers_metric_target_position_from_known_size():
    camera_target_position = _perception().camera_target_position
    camera = np.float64([[480, 0, 320], [0, 480, 240], [0, 0, 1]])
    model = np.float64([[-0.4, -0.2, 0], [0.4, -0.2, 0], [0.4, 0.2, 0], [-0.4, 0.2, 0]])
    pixels, _ = cv2.projectPoints(model, np.float64([0.1, -0.15, 0.2]), np.float64([0.3, -0.1, 2.5]), camera, np.zeros(5))
    result = camera_target_position(pixels.reshape(4, 2), camera, target_size_m=(0.8, 0.4))
    assert result == pytest.approx([0.3, -0.1, 2.5], abs=0.001)


@pytest.mark.parametrize("size", [None, (0, 0.4), (-0.8, 0.4), (float("nan"), 0.4)])
def test_camera_projection_never_invents_unknown_physical_size(size):
    result = _perception().camera_target_position([[100, 100], [200, 100], [200, 200], [100, 200]],
                                                 np.eye(3), target_size_m=size)
    assert result is None


def test_camera_projection_rejects_degenerate_corners():
    result = _perception().camera_target_position([[100, 100]] * 4,
                                                 [[480, 0, 320], [0, 480, 240], [0, 0, 1]],
                                                 target_size_m=(0.8, 0.4))
    assert result is None


def test_pointcloud_checks_cylinder_along_actual_three_dimensional_velocity():
    geometry = _perception()
    assert hasattr(geometry, "pointcloud_path_clear"), "3D sampled corridor guard is missing"
    cloud = [[2, 0, 0], [2.5, 0.1, 0], [3, 0, 0.1], [0, 0, 0.3]]
    assert geometry.pointcloud_path_clear(cloud, [1, 0, 0], stop_distance=0.8, corridor_radius=0.25) is True
    assert geometry.pointcloud_path_clear(cloud, [0, 0, 1], stop_distance=0.8, corridor_radius=0.25) is False
    assert geometry.pointcloud_path_clear(cloud + [[0.5, 0.1, 0]], [1, 0, 0],
                                         stop_distance=0.8, corridor_radius=0.25) is False


@pytest.mark.parametrize("cloud", [[], [[float("nan"), 0, 0]] * 4, [[3, 0, 0]],
                                   [[2, 2, 0], [2, 3, 0], [2, 4, 0]],
                                   [[-2, 0, 0], [-3, 0, 0], [-4, 0, 0]]])
def test_pointcloud_unknown_or_unsampled_direction_is_not_clear(cloud):
    geometry = _perception()
    assert hasattr(geometry, "pointcloud_path_clear"), "3D sampled corridor guard is missing"
    assert geometry.pointcloud_path_clear(cloud, [1, 0, 0], stop_distance=0.8, corridor_radius=0.25) is False


def test_pointcloud_zero_or_invalid_direction_cannot_claim_clearance():
    geometry = _perception()
    assert hasattr(geometry, "pointcloud_path_clear"), "3D sampled corridor guard is missing"
    cloud = [[2, 0, 0], [3, 0, 0], [4, 0, 0]]
    for velocity in [[0, 0, 0], [float("nan"), 0, 0], [1, 0]]:
        assert geometry.pointcloud_path_clear(cloud, velocity, stop_distance=0.8, corridor_radius=0.25) is False
