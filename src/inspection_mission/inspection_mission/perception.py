"""Pure, conservative scan and image geometry; no ROS or simulator dependencies."""

from __future__ import annotations

import math

import cv2
import numpy as np


def scan_path_clear(ranges, *, angle_min: float, angle_increment: float,
                    world_direction: float, robot_yaw: float, stop_distance: float,
                    sector_half_angle: float = 0.26, range_min: float = 0.0,
                    range_max: float = math.inf, scan_yaw_offset: float = 0.0,
                    min_valid_fraction: float = 0.7, min_valid_beams: int = 3,
                    corridor_radius: float = 0.15) -> bool:
    """Check a sector around world travel direction in the actual scan frame.

    Angles are radians; ``scan_yaw_offset`` is the measured scan-to-base yaw.
    Missing/FOV-limited/invalid observations fail closed. Positive infinity is a
    clear ray only when a finite declared maximum range covers stop_distance.
    This is a local collision guard, not global navigation or a 3D free-space map.
    """
    try:
        scan = np.asarray(ranges, dtype=float).reshape(-1)
        metadata = [angle_min, angle_increment, world_direction, robot_yaw,
                    stop_distance, sector_half_angle, range_min, scan_yaw_offset, min_valid_fraction, corridor_radius]
        if (not scan.size or not np.isfinite(metadata).all() or angle_increment == 0
                or stop_distance <= 0 or corridor_radius <= 0 or not 0 < sector_half_angle < math.pi
                or range_min < 0 or range_min >= stop_distance or range_max <= range_min or math.isnan(range_max)
                or not 0 < min_valid_fraction <= 1 or min_valid_beams < 1):
            return False
        direction = world_direction - robot_yaw - scan_yaw_offset
        angles = angle_min + np.arange(scan.size) * angle_increment
        relative = np.arctan2(np.sin(angles - direction), np.cos(angles - direction))
        # A narrow forward cone misses close obstacles near the robot's side.
        # Check the full observed swept footprint before evaluating clear rays.
        # Sub-minimum positive returns are unknown, but must still stop motion
        # when their reported location falls inside the swept corridor.
        spatial_returns = np.isfinite(scan) & (scan > 0) & (scan <= range_max)
        forward = np.zeros_like(scan)
        lateral = np.zeros_like(scan)
        forward[spatial_returns] = scan[spatial_returns] * np.cos(relative[spatial_returns])
        lateral[spatial_returns] = scan[spatial_returns] * np.sin(relative[spatial_returns])
        if np.any(spatial_returns & (forward >= 0) & (forward <= stop_distance) & (np.abs(lateral) <= corridor_radius)):
            return False
        selected = np.abs(relative) <= sector_half_angle + 1e-9
        rays = scan[selected]
        offsets = relative[selected]
        step = abs(angle_increment)
        expected = max(min_valid_beams, int(math.floor(2 * sector_half_angle / step)))
        if (rays.size < expected or rays.size < min_valid_beams
                or offsets.min() > -sector_half_angle + step
                or offsets.max() < sector_half_angle - step):
            return False
        finite_valid = np.isfinite(rays) & (rays >= range_min) & (rays > 0) & (rays <= range_max)
        if np.any(finite_valid & (rays <= stop_distance)):
            return False
        no_return = np.isposinf(rays) & math.isfinite(range_max) & (range_max > stop_distance)
        valid = finite_valid | no_return
        return bool(valid.sum() >= min_valid_beams and valid.mean() >= min_valid_fraction)
    except (TypeError, ValueError, OverflowError):
        return False


def aim_centered(center, tolerance_x: float = 0.05, tolerance_y: float = 0.05) -> bool:
    """Whether an observed normalized image point lies in the aim deadband."""
    try:
        point = np.asarray(center, dtype=float)
        if (point.shape != (2,) or not np.isfinite(point).all()
                or (point < 0).any() or (point > 1).any()
                or not math.isfinite(tolerance_x) or not math.isfinite(tolerance_y)
                or tolerance_x < 0 or tolerance_y < 0):
            return False
        return bool(abs(point[0] - 0.5) <= tolerance_x and abs(point[1] - 0.5) <= tolerance_y)
    except (TypeError, ValueError):
        return False


def pointcloud_path_clear(points_xyz, velocity_xyz, *, stop_distance: float,
                          corridor_radius: float, min_valid_points: int = 3) -> bool:
    """Guard a sampled 3D cylinder in one explicitly confirmed local frame.

    Points and velocity must already share that frame. Any point within the
    forward stopping cylinder blocks. At least min_valid_points finite returns
    must sample this direction beyond the stopping distance to return True.
    This checks observed returns only; it does not prove unknown space is free.
    LaserScan cannot substitute for this 3D check during ascent/descent.
    """
    try:
        points = np.asarray(points_xyz, dtype=float)
        velocity = np.asarray(velocity_xyz, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or velocity.shape != (3,)
                or not np.isfinite(velocity).all() or np.linalg.norm(velocity) < 1e-9
                or not math.isfinite(stop_distance) or stop_distance <= 0
                or not math.isfinite(corridor_radius) or corridor_radius <= 0
                or min_valid_points < 1):
            return False
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < min_valid_points:
            return False
        direction = velocity / np.linalg.norm(velocity)
        forward = points @ direction
        radial = np.linalg.norm(points - forward[:, None] * direction, axis=1)
        in_corridor = (forward >= 0) & (radial <= corridor_radius)
        if np.any(in_corridor & (forward <= stop_distance)):
            return False
        return bool(np.count_nonzero(in_corridor & (forward > stop_distance)) >= min_valid_points)
    except (TypeError, ValueError, OverflowError):
        return False


def pointcloud_sensor_path_clear(points_xyz, velocity_xyz, spec, *, stop_distance, corridor_radius):
    """Transform measured returns and reject stopping corridors outside the aperture.

    This deliberately refuses front-lidar ascent/descent and rearward motion.
    No plan coordinate or assumed free space can extend the sensor's coverage.
    """
    try:
        points = np.asarray(points_xyz, dtype=float)
        velocity = np.asarray(velocity_xyz, dtype=float)
        offset = np.asarray(spec["translation_m"], dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or velocity.shape != (3,)
                or np.linalg.norm(velocity) < 1e-9 or not np.isfinite(velocity).all()
                or not math.isfinite(stop_distance) or stop_distance <= 0
                or not math.isfinite(corridor_radius) or corridor_radius <= 0):
            return False
        direction = velocity / np.linalg.norm(velocity)
        # Cover the stopping end-cap with its enclosing sphere. The angular
        # margin conservatively bounds every radial point in that cross-section.
        center = direction * stop_distance - offset
        distance = np.linalg.norm(center)
        if distance <= corridor_radius:
            return False
        margin = math.asin(min(1.0, corridor_radius / distance))
        azimuth = abs(math.atan2(center[1], center[0]))
        elevation = abs(math.atan2(center[2], math.hypot(center[0], center[1])))
        if (azimuth + margin > spec["horizontal_fov_rad"] / 2
                or elevation + margin > spec["vertical_fov_rad"] / 2
                or distance + corridor_radius > spec["range_max_m"]
                or distance - corridor_radius < spec["range_min_m"]):
            return False
        # Ignore nonfinite/no-return samples and reject an unobserved corridor.
        finite = points[np.isfinite(points).all(axis=1)]
        distances = np.linalg.norm(finite, axis=1)
        if np.any(distances < spec["range_min_m"]):
            return False  # An under-range return is unknown near-field occupancy.
        valid = finite[(distances >= spec["range_min_m"]) & (distances <= spec["range_max_m"])]
        if len(valid) < 3:
            return False
        azimuths = np.abs(np.arctan2(valid[:, 1], valid[:, 0]))
        elevations = np.abs(np.arctan2(valid[:, 2], np.hypot(valid[:, 0], valid[:, 1])))
        valid = valid[(azimuths <= spec["horizontal_fov_rad"] / 2)
                      & (elevations <= spec["vertical_fov_rad"] / 2)]
        return pointcloud_path_clear(valid + offset, velocity, stop_distance=stop_distance,
                                     corridor_radius=corridor_radius)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def yaw_level_rotation(quaternion_xyzw, yaw):
    """Rotate body coordinates to a level frame sharing the accepted pose yaw.

    Normalization is local to this derived transform; raw odometry is retained.
    """
    q = np.asarray(quaternion_xyzw, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]]) @ rotation


def pointcloud_envelope_path_clear(points_xyz, velocity_xyz, *, stop_distance,
                                    corridor_radius, below_origin_m, above_origin_m,
                                    min_valid_points=3):
    """Guard horizontal translation with separate level vertical body bounds.

    The forward range includes the full horizontal radius, as in the legacy
    gate. Side bounds retain that radius, and both origin-relative z bounds are
    inclusive. This is a sampled stopping corridor, not a free-space proof.
    """
    try:
        points = np.asarray(points_xyz, dtype=float)
        velocity = np.asarray(velocity_xyz, dtype=float)
        dimensions = [stop_distance, corridor_radius, below_origin_m, above_origin_m]
        if (points.ndim != 2 or points.shape[1] != 3 or velocity.shape != (3,)
                or not np.isfinite(velocity).all() or abs(velocity[2]) > 1e-8
                or np.linalg.norm(velocity[:2]) < 1e-9
                or not np.isfinite(dimensions).all() or min(dimensions) <= 0
                or min_valid_points < 1):
            return False
        points = points[np.isfinite(points).all(axis=1)]
        direction = velocity[:2] / np.linalg.norm(velocity[:2])
        forward = points[:, :2] @ direction
        lateral = points[:, :2] @ np.array([-direction[1], direction[0]])
        inside = ((forward >= 0) & (np.abs(lateral) <= corridor_radius)
                  & (points[:, 2] >= -below_origin_m) & (points[:, 2] <= above_origin_m))
        if np.any(inside & (forward <= stop_distance)):
            return False
        return bool(np.count_nonzero(inside & (forward > stop_distance)) >= min_valid_points)
    except (TypeError, ValueError, OverflowError):
        return False


def pointcloud_sensor_envelope_path_clear(points_xyz, velocity_xyz, spec, *, stop_distance,
                                           corridor_radius, envelope, body_to_level):
    """Check the full horizontal stopping rectangle inside the real front FOV.

    All four level end-cap corners are transformed back into the sensor frame.
    For a front rectangular aperture (at most pi radians) its angular cone is
    convex, so corner inclusion covers the entire cap without a spherical z
    radius. No vertical, rearward or rotating blind-motion permission is added.
    """
    try:
        points = np.asarray(points_xyz, dtype=float)
        velocity = np.asarray(velocity_xyz, dtype=float)
        offset = np.asarray(spec["translation_m"], dtype=float)
        rotation = np.asarray(body_to_level, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or velocity.shape != (3,)
                or not np.isfinite(velocity).all() or abs(velocity[2]) > 1e-8
                or velocity[0] <= 1e-8 or rotation.shape != (3, 3)
                or not np.isfinite(rotation).all()
                or not 0 < spec["horizontal_fov_rad"] <= math.pi
                or not 0 < spec["vertical_fov_rad"] < math.pi
                or not math.isfinite(stop_distance) or stop_distance <= 0
                or not math.isfinite(corridor_radius) or corridor_radius < envelope.horizontal_radius_m):
            return False
        direction = velocity / np.linalg.norm(velocity)
        side = np.array([-direction[1], direction[0], 0.])
        center = direction * stop_distance
        corners = np.array([center + sign*corridor_radius*side + [0, 0, z]
                            for sign in (-1, 1)
                            for z in (-envelope.below_origin_m, envelope.above_origin_m)])
        sensor_corners = corners @ rotation - offset
        azimuths = np.abs(np.arctan2(sensor_corners[:, 1], sensor_corners[:, 0]))
        elevations = np.abs(np.arctan2(sensor_corners[:, 2], sensor_corners[:, 0]))
        # z/x, rather than z/hypot(x,y), conservatively encloses the cone in
        # the lidar's spherical elevation aperture for every interior point.
        if (np.any(sensor_corners[:, 0] <= 0)
                or np.any(azimuths > spec["horizontal_fov_rad"]/2)
                or np.any(elevations > spec["vertical_fov_rad"]/2)
                or np.any(np.linalg.norm(sensor_corners, axis=1) > spec["range_max_m"])):
            return False
        sensor_origin = rotation @ offset
        lateral_gap = max(abs(sensor_origin @ side)-corridor_radius, 0.)
        vertical_gap = max(-envelope.below_origin_m-sensor_origin[2],
                           sensor_origin[2]-envelope.above_origin_m, 0.)
        nearest = math.sqrt((stop_distance-sensor_origin @ direction)**2
                            + lateral_gap**2 + vertical_gap**2)
        if nearest < spec["range_min_m"]:
            return False
        finite = points[np.isfinite(points).all(axis=1)]
        distances = np.linalg.norm(finite, axis=1)
        if np.any(distances < spec["range_min_m"]):
            return False
        valid = finite[(distances >= spec["range_min_m"]) & (distances <= spec["range_max_m"])]
        azimuths = np.abs(np.arctan2(valid[:, 1], valid[:, 0]))
        elevations = np.abs(np.arctan2(valid[:, 2], np.hypot(valid[:, 0], valid[:, 1])))
        valid = valid[(azimuths <= spec["horizontal_fov_rad"]/2)
                      & (elevations <= spec["vertical_fov_rad"]/2)]
        return pointcloud_envelope_path_clear((valid+offset) @ rotation.T, velocity,
            stop_distance=stop_distance, corridor_radius=corridor_radius,
            below_origin_m=envelope.below_origin_m, above_origin_m=envelope.above_origin_m)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def camera_target_position(corners_px, camera_matrix, distortion=None, *, target_size_m,
                           max_reprojection_error_px: float = 3.0) -> tuple[float, float, float] | None:
    """Solve a known rectangle's center in optical-camera metres (x right,y down,z forward).

    Corners must correspond to reference top-left, top-right, bottom-right,
    bottom-left. A real CameraInfo calibration and user-supplied physical width
    and height are required. This never invents size or an extrinsic transform.
    """
    try:
        size = np.asarray(target_size_m, dtype=np.float64)
        corners = np.asarray(corners_px, dtype=np.float64)
        camera = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        if (size.shape != (2,) or not np.isfinite(size).all() or (size <= 0).any()
                or corners.shape != (4, 2) or not np.isfinite(corners).all()
                or not np.isfinite(camera).all() or camera[0, 0] <= 0 or camera[1, 1] <= 0
                or abs(camera[2, 2] - 1) > 1e-6
                or not cv2.isContourConvex(corners.astype(np.float32))
                or abs(cv2.contourArea(corners.astype(np.float32))) < 4
                or not math.isfinite(max_reprojection_error_px) or max_reprojection_error_px <= 0):
            return None
        distortion = np.asarray(distortion if distortion is not None else np.zeros(5), dtype=np.float64)
        if not np.isfinite(distortion).all():
            return None
        width, height = size
        model = np.float64([[-width / 2, -height / 2, 0], [width / 2, -height / 2, 0],
                            [width / 2, height / 2, 0], [-width / 2, height / 2, 0]])
        success, rotation, translation = cv2.solvePnP(model, corners, camera, distortion,
                                                     flags=cv2.SOLVEPNP_ITERATIVE)
        if not success or not np.isfinite(translation).all() or translation[2, 0] <= 0:
            return None
        projected, _ = cv2.projectPoints(model, rotation, translation, camera, distortion)
        error = np.linalg.norm(projected.reshape(4, 2) - corners, axis=1).max()
        if not np.isfinite(error) or error > max_reprojection_error_px:
            return None
        return tuple(float(value) for value in translation.ravel())
    except (TypeError, ValueError, cv2.error):
        return None
