"""Explicit target/illumination geometry; standard library only, no ROS I/O.

T_AB maps B coordinates into A. Distances are metres, angles radians, and
timestamps seconds in the named clock/epoch. World frames must be right-handed
with +z up. Camera points use OpenCV optical axes (right, down, forward).
All transforms, source versions and conservative metric error bounds are
mandatory caller evidence: no platform calibration or trust flag is invented.

The target's world z defines a horizontal plane. A platform adapter must prove
that this plane also applies at the independent range beam's footprint. These
calculations do not certify a collision-free route, target identity or a score.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # x,y,z,w; active child-to-parent rotation.
_EPS = 1e-12


@dataclass(frozen=True)
class RigidTransform:
    parent_frame: str
    child_frame: str
    translation_m: Vec3
    rotation_xyzw: Quat
    version: str
    translation_error_m: float
    rotation_error_rad: float


@dataclass(frozen=True)
class StampedPose3D:
    transform: RigidTransform
    stamp_s: float
    clock_id: str
    epoch_id: str
    linear_speed_bound_mps: float
    angular_speed_bound_rps: float


@dataclass(frozen=True)
class CameraObservation:
    target_id: str
    point_id: str
    position_camera_m: Vec3
    camera_frame: str
    coordinate_convention: str
    stamp_s: float
    clock_id: str
    epoch_id: str
    calibration_version: str
    size_version: str
    position_error_m: float  # Includes calibration, size, corners and PnP ambiguity.


@dataclass(frozen=True)
class LocalizationLimits:
    world_frame: str
    body_frame: str
    camera_frame: str
    pose_version: str
    camera_extrinsic_version: str
    calibration_version: str
    size_version: str
    max_pose_image_skew_s: float
    max_observation_age_s: float
    max_pose_age_s: float
    max_position_error_m: float
    future_tolerance_s: float


@dataclass(frozen=True)
class TargetFix:
    target_id: str
    point_id: str
    position_world_m: Vec3
    world_frame: str
    body_frame: str
    stamp_s: float  # Original exposure time, never the processing/use time.
    clock_id: str
    epoch_id: str
    position_error_m: float
    pose_version: str
    calibration_version: str
    size_version: str
    camera_extrinsic_version: str


@dataclass(frozen=True)
class LightExtrinsics:
    transform: RigidTransform  # Light frame to the explicit odometry child/body.
    direction_light: Vec3


@dataclass(frozen=True)
class StampedRange:
    transform: RigidTransform  # Independent range-sensor frame to body.
    direction_sensor: Vec3
    distance_m: float
    range_min_m: float
    range_max_m: float
    measurement_error_m: float
    stamp_s: float
    clock_id: str
    epoch_id: str


@dataclass(frozen=True)
class StampedBool:
    value: bool
    stamp_s: float
    clock_id: str
    epoch_id: str
    version: str


@dataclass(frozen=True)
class IlluminationLimits:
    world_frame: str
    body_frame: str
    pose_version: str
    light_extrinsic_version: str
    range_extrinsic_version: str
    feedback_version: str
    max_target_age_s: float
    max_pose_age_s: float
    max_range_age_s: float
    max_feedback_age_s: float
    future_tolerance_s: float
    max_target_error_m: float
    max_total_error_m: float
    hit_radius_m: float
    max_range_residual_m: float
    max_down_tilt_rad: float
    min_body_height_m: float  # Body origin height above target's z plane.
    max_body_height_m: float
    localization_drift_mps: float  # Bound on target-coordinate drift after exposure.


@dataclass(frozen=True)
class HoverSolution:
    target_id: str
    point_id: str
    position_world_m: Vec3
    orientation_xyzw: Quat
    world_frame: str
    body_frame: str
    ray_distance_m: float


@dataclass(frozen=True)
class AimAssessment:
    allowed: bool
    reason: str
    spot_world_m: Vec3 | None
    total_error_m: float | None
    range_residual_m: float | None


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _text(value, label):
    _require(isinstance(value, str) and bool(value.strip()), "invalid_" + label)


def _number(value, label, *, nonnegative=False):
    _require(type(value) in (int, float) and math.isfinite(value), "finite_number:" + label)
    if nonnegative:
        _require(value >= 0, "negative_" + label)
    return float(value)


def _vector(value, size, label):
    _require(isinstance(value, (tuple, list)) and len(value) == size, "invalid_" + label)
    return tuple(_number(v, label) for v in value)


def _unit(value, size, label):
    result = _vector(value, size, label)
    # Only floating-point rounding is tolerated; no malformed vector is fixed.
    _require(abs(math.hypot(*result) - 1) <= 1e-9, "unit_" + label)
    return result


def _transform(value):
    _require(isinstance(value, RigidTransform), "invalid_transform")
    for name in ("parent_frame", "child_frame", "version"):
        _text(getattr(value, name), name)
    _vector(value.translation_m, 3, "translation")
    _unit(value.rotation_xyzw, 4, "quaternion")
    _number(value.translation_error_m, "translation_error", nonnegative=True)
    _require(0 <= _number(value.rotation_error_rad, "rotation_error") <= math.pi, "rotation_error_range")


def _pose(value):
    _require(isinstance(value, StampedPose3D), "invalid_pose")
    _transform(value.transform)
    for name in ("clock_id", "epoch_id"):
        _text(getattr(value, name), name)
    for name in ("stamp_s", "linear_speed_bound_mps", "angular_speed_bound_rps"):
        _number(getattr(value, name), name, nonnegative=True)


def _fresh(stamp, now, age, future, label):
    stamp = _number(stamp, label + "_stamp", nonnegative=True)
    now = _number(now, "now", nonnegative=True)
    _require(stamp - now <= future + _EPS, label + "_future")
    _require(now - stamp <= age + _EPS, label + "_stale")


def _same_session(first, second):
    _text(second.clock_id, "clock_id")
    _text(second.epoch_id, "epoch_id")
    _require(first.clock_id == second.clock_id, "clock_mismatch")
    _require(first.epoch_id == second.epoch_id, "epoch_mismatch")


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _scale(a, scalar):
    return tuple(x * scalar for x in a)


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _rotate(q, point):
    vector = q[:3]
    return _add(point, _scale(_add(_scale(_cross(vector, point), q[3]),
                                   _cross(vector, _cross(vector, point))), 2))


def _angular_delta(angle):
    return 2 * math.sin(min(math.pi, angle) / 2)


def _localization_limits(limits):
    _require(isinstance(limits, LocalizationLimits), "invalid_localization_limits")
    for name in ("world_frame", "body_frame", "camera_frame", "pose_version", "camera_extrinsic_version",
                 "calibration_version", "size_version"):
        _text(getattr(limits, name), name)
    for name in ("max_pose_image_skew_s", "max_observation_age_s", "max_pose_age_s", "max_position_error_m",
                 "future_tolerance_s"):
        _number(getattr(limits, name), name, nonnegative=True)


def localize_target(observation, optical_to_body, body_pose_at_exposure, limits, *, now_s):
    """Return a world fix; reject missing/invalid evidence with ValueError.

    A pose must already have been matched/interpolated to the exposure by the
    adapter. Residual skew is bounded and charged using the supplied motion
    bounds, which must cover that entire interval. Pixel confidence is not used
    as a metric uncertainty. The output preserves the original exposure stamp.
    """
    _localization_limits(limits)
    _require(isinstance(observation, CameraObservation), "invalid_observation")
    _transform(optical_to_body)
    pose = body_pose_at_exposure
    _pose(pose)
    for name in ("target_id", "point_id", "clock_id", "epoch_id"):
        _text(getattr(observation, name), name)
    p_camera = _vector(observation.position_camera_m, 3, "camera_position")
    _require(p_camera[2] > 0, "target_behind_camera")
    _require(observation.coordinate_convention == "opencv_optical", "optical_convention")
    _require(observation.camera_frame == limits.camera_frame == optical_to_body.child_frame, "camera_frame")
    _require(optical_to_body.parent_frame == limits.body_frame == pose.transform.child_frame, "body_frame")
    _require(pose.transform.parent_frame == limits.world_frame, "world_frame")
    for actual, expected, label in (
        (pose.transform.version, limits.pose_version, "pose_version"),
        (optical_to_body.version, limits.camera_extrinsic_version, "camera_extrinsic_version"),
        (observation.calibration_version, limits.calibration_version, "calibration_version"),
        (observation.size_version, limits.size_version, "size_version")):
        _require(actual == expected, label)
    _same_session(observation, pose)
    _fresh(observation.stamp_s, now_s, limits.max_observation_age_s, limits.future_tolerance_s, "observation")
    _fresh(pose.stamp_s, now_s, limits.max_pose_age_s, limits.future_tolerance_s, "pose")
    skew = abs(pose.stamp_s - observation.stamp_s)
    _require(skew <= limits.max_pose_image_skew_s + _EPS, "pose_image_skew")
    p_body = _add(_rotate(optical_to_body.rotation_xyzw, p_camera), optical_to_body.translation_m)
    p_world = _add(_rotate(pose.transform.rotation_xyzw, p_body), pose.transform.translation_m)
    error = (_number(observation.position_error_m, "camera_position_error", nonnegative=True)
             + optical_to_body.translation_error_m + pose.transform.translation_error_m
             + _angular_delta(optical_to_body.rotation_error_rad) * math.hypot(*p_camera)
             + _angular_delta(pose.transform.rotation_error_rad) * math.hypot(*p_body)
             + pose.linear_speed_bound_mps * skew
             + _angular_delta(pose.angular_speed_bound_rps * skew) * math.hypot(*p_body))
    _vector(p_world, 3, "world_position")
    _require(error <= limits.max_position_error_m + _EPS, "localization_error_exceeded")
    return TargetFix(observation.target_id, observation.point_id, p_world, limits.world_frame, limits.body_frame,
                     observation.stamp_s, observation.clock_id, observation.epoch_id, error, limits.pose_version,
                     observation.calibration_version, observation.size_version, optical_to_body.version)


def _illumination_limits(limits):
    _require(isinstance(limits, IlluminationLimits), "invalid_illumination_limits")
    for name in ("world_frame", "body_frame", "pose_version", "light_extrinsic_version",
                 "range_extrinsic_version", "feedback_version"):
        _text(getattr(limits, name), name)
    for name in ("max_target_age_s", "max_pose_age_s", "max_range_age_s", "max_feedback_age_s",
                 "future_tolerance_s", "max_target_error_m", "max_total_error_m", "hit_radius_m",
                 "max_range_residual_m", "max_down_tilt_rad", "min_body_height_m", "max_body_height_m",
                 "localization_drift_mps"):
        _number(getattr(limits, name), name, nonnegative=True)
    _require(limits.hit_radius_m > 0 and limits.min_body_height_m > 0, "invalid_physical_limits")
    _require(limits.max_body_height_m >= limits.min_body_height_m, "body_height_limits")
    _require(limits.max_down_tilt_rad < math.pi / 2, "down_tilt_limit")


def _target(target, limits):
    _require(isinstance(target, TargetFix), "invalid_target")
    _vector(target.position_world_m, 3, "world_target")
    for name in ("target_id", "point_id", "clock_id", "epoch_id", "calibration_version", "size_version",
                 "camera_extrinsic_version"):
        _text(getattr(target, name), name)
    _require(target.world_frame == limits.world_frame, "world_frame")
    _require(target.body_frame == limits.body_frame, "body_frame")
    _require(target.pose_version == limits.pose_version, "pose_version")
    _number(target.stamp_s, "target_stamp", nonnegative=True)
    error = _number(target.position_error_m, "target_error", nonnegative=True)
    _require(error <= limits.max_target_error_m + _EPS, "target_error_exceeded")


def _light(light, limits):
    _require(isinstance(light, LightExtrinsics), "invalid_light")
    _transform(light.transform)
    _unit(light.direction_light, 3, "light_direction")
    _require(light.transform.parent_frame == limits.body_frame, "light_body_frame")
    _require(light.transform.version == limits.light_extrinsic_version, "light_extrinsic_version")


def _down(direction, limit, label, angular_error_rad=0.):
    _require(direction[2] < -1e-9, label + "_not_downward")
    # The configured tilt is a hard bound on the possible physical ray, not
    # merely its nominal direction. Triangle inequality bounds rotations.
    tilt = math.acos(max(-1., min(1., -direction[2])))
    _require(tilt + angular_error_rad <= limit + _EPS, label + "_tilt_limit")


def _height(body_z, target_z, limits, error_m=0.):
    height = _number(body_z, "body_height") - target_z
    _require(limits.min_body_height_m - _EPS <= height - error_m
             and height + error_m <= limits.max_body_height_m + _EPS, "body_height")


def _intersect(origin, direction, plane_z, label):
    _require(direction[2] < -1e-9, label + "_parallel_or_upward")
    distance = (plane_z - origin[2]) / direction[2]
    _require(math.isfinite(distance) and distance > 0, label + "_behind_ray")
    return distance, _add(origin, _scale(direction, distance))


def hover_for_downlight(target, light_in_body, desired_orientation_xyzw, body_height_world_m, limits):
    """Compute a nominal hover origin for a downward ray and target-z plane.

    body_height_world_m is world z, not a range reading or light height. This
    setpoint needs separate trajectory/obstacle and live uncertainty checks.
    """
    _illumination_limits(limits)
    _target(target, limits)
    _light(light_in_body, limits)
    q = _unit(desired_orientation_xyzw, 4, "quaternion")
    _height(body_height_world_m, target.position_world_m[2], limits)
    offset = _rotate(q, light_in_body.transform.translation_m)
    direction = _rotate(q, _rotate(light_in_body.transform.rotation_xyzw, light_in_body.direction_light))
    _down(direction, limits.max_down_tilt_rad, "light", light_in_body.transform.rotation_error_rad)
    distance, _ = _intersect((offset[0], offset[1], body_height_world_m + offset[2]), direction,
                             target.position_world_m[2], "light")
    position = (target.position_world_m[0] - offset[0] - distance * direction[0],
                target.position_world_m[1] - offset[1] - distance * direction[1], float(body_height_world_m))
    _vector(position, 3, "hover_position")
    return HoverSolution(target.target_id, target.point_id, position, q, target.world_frame, target.body_frame, distance)


def _world_ray(pose, extrinsic, local_direction, time_offset_s):
    origin = _add(pose.transform.translation_m, _rotate(pose.transform.rotation_xyzw, extrinsic.translation_m))
    direction = _rotate(pose.transform.rotation_xyzw, _rotate(extrinsic.rotation_xyzw, local_direction))
    pose_angle = pose.transform.rotation_error_rad + pose.angular_speed_bound_rps * time_offset_s
    origin_error = (pose.transform.translation_error_m + pose.linear_speed_bound_mps * time_offset_s
                    + extrinsic.translation_error_m
                    + _angular_delta(pose_angle) * math.hypot(*extrinsic.translation_m))
    angular_error = pose_angle + extrinsic.rotation_error_rad
    direction_delta = _angular_delta(angular_error)
    return origin, direction, origin_error, direction_delta, angular_error


def _plane_error(origin_error, direction_delta, distance, direction):
    # For a perturbed ray, |delta lambda| <= (e_origin + lambda*e_direction)/k_min.
    # The additional 1 covers origin/endpoint motion. k_min bounds the vertical
    # direction away from a singular plane intersection, including angle error.
    k_min = abs(direction[2]) - direction_delta
    _require(k_min > 1e-9, "uncertain_ray_parallel")
    return (1 + 1 / k_min) * (origin_error + distance * direction_delta)


def assess_illumination(target, current_body_pose, light_in_body, downward_range, light_feedback,
                         limits, *, now_s):
    """Reject bad evidence explicitly; never issue a control or relabel old data.

    A cached world fix is allowed without a front-camera image, provided its
    age/drift budget remains valid. All error inputs are worst-case bounds.
    Range agreement includes measurement and transform uncertainty instead of
    allowing a large uncertainty to conceal a disagreement.
    """
    try:
        _illumination_limits(limits)
        _target(target, limits)
        pose, measured, feedback = current_body_pose, downward_range, light_feedback
        _pose(pose)
        _require(pose.transform.parent_frame == limits.world_frame, "world_frame")
        _require(pose.transform.child_frame == limits.body_frame, "body_frame")
        _require(pose.transform.version == limits.pose_version, "pose_version")
        _same_session(target, pose)
        _fresh(target.stamp_s, now_s, limits.max_target_age_s, limits.future_tolerance_s, "target")
        _fresh(pose.stamp_s, now_s, limits.max_pose_age_s, limits.future_tolerance_s, "pose")
        _light(light_in_body, limits)
        _require(isinstance(measured, StampedRange), "invalid_range")
        _transform(measured.transform)
        _unit(measured.direction_sensor, 3, "range_direction")
        _require(measured.transform.parent_frame == limits.body_frame, "range_body_frame")
        _require(measured.transform.version == limits.range_extrinsic_version, "range_extrinsic_version")
        _same_session(target, measured)
        _fresh(measured.stamp_s, now_s, limits.max_range_age_s, limits.future_tolerance_s, "range")
        _require(isinstance(feedback, StampedBool) and type(feedback.value) is bool, "invalid_feedback")
        _same_session(target, feedback)
        _require(feedback.version == limits.feedback_version, "feedback_version")
        _fresh(feedback.stamp_s, now_s, limits.max_feedback_age_s, limits.future_tolerance_s, "feedback")
        _require(feedback.value, "light_not_ready")
        minimum = _number(measured.range_min_m, "range_min", nonnegative=True)
        maximum = _number(measured.range_max_m, "range_max", nonnegative=True)
        distance = _number(measured.distance_m, "range_distance", nonnegative=True)
        measurement_error = _number(measured.measurement_error_m, "range_error", nonnegative=True)
        _require(0 < distance and minimum <= distance <= maximum and maximum > minimum, "range_limits")
        plane_z = target.position_world_m[2]
        drift = limits.localization_drift_mps * max(0., now_s - target.stamp_s,
                                                  pose.stamp_s - target.stamp_s, measured.stamp_s - target.stamp_s)
        target_error = target.position_error_m + drift
        _require(target_error <= limits.max_target_error_m + _EPS, "target_error_exceeded")
        pose_age = abs(now_s - pose.stamp_s)
        _height(pose.transform.translation_m[2], target.position_world_m[2], limits,
                target_error + pose.transform.translation_error_m + pose.linear_speed_bound_mps * pose_age)
        origin, direction, origin_error, direction_delta, angular_error = _world_ray(
            pose, light_in_body.transform, light_in_body.direction_light, pose_age)
        _down(direction, limits.max_down_tilt_rad, "light", angular_error)
        ray_distance, spot = _intersect(origin, direction, plane_z, "light")
        total_error = _plane_error(origin_error + target_error, direction_delta, ray_distance, direction)
        _require(total_error <= limits.max_total_error_m + _EPS, "total_error_exceeded")
        _require(math.dist(spot, target.position_world_m) + total_error <= limits.hit_radius_m + _EPS,
                 "miss_distance_exceeded")
        r_origin, r_direction, r_error, r_delta, r_angle_error = _world_ray(
            pose, measured.transform, measured.direction_sensor, abs(pose.stamp_s - measured.stamp_s))
        _down(r_direction, limits.max_down_tilt_rad, "range", r_angle_error)
        expected, _ = _intersect(r_origin, r_direction, plane_z, "range")
        _require(minimum <= expected <= maximum, "expected_range_limits")
        residual = abs(distance - expected)
        residual_bound = residual + measurement_error + _plane_error(r_error + target_error, r_delta,
                                                                      expected, r_direction)
        _require(residual_bound <= limits.max_range_residual_m + _EPS, "range_plane_mismatch")
        return AimAssessment(True, "ok", spot, total_error, residual)
    except (ValueError, TypeError, AttributeError, OverflowError) as exc:
        return AimAssessment(False, str(exc) or "invalid_evidence", None, None, None)
