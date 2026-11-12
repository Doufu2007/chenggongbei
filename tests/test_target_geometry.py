"""Hand-derived geometry and invalid-evidence boundaries; no ROS transport."""
import importlib
import importlib.util
import math
from dataclasses import replace

import pytest


def geometry():
    name = "inspection_mission.target_geometry"
    assert importlib.util.find_spec(name) is not None, "explicit target geometry module is missing"
    return importlib.import_module(name)


def inputs():
    g = geometry()
    camera = g.RigidTransform("body", "camera_optical", (.39, 0., .13), (-.5, .5, -.5, .5), "camera-v1", 0., 0.)
    pose = g.StampedPose3D(g.RigidTransform("odom", "body", (10., 20., .8),
                                         (0., 0., math.sqrt(.5), math.sqrt(.5)), "pose-v1", 0., 0.),
                           10., "sim", "session-1", 1., 1.)
    observation = g.CameraObservation("person", "aerial-1", (0., .93, 1.11), "camera_optical",
                                      "opencv_optical", 10., "sim", "session-1", "calib-v1", "size-v1", 0.)
    limits = g.LocalizationLimits("odom", "body", "camera_optical", "pose-v1", "camera-v1",
                                   "calib-v1", "size-v1", .05, 1., 1., .2, .01)
    return g, observation, camera, pose, limits


def illumination():
    g, observation, camera, exposure, limits = inputs()
    fix = g.localize_target(observation, camera, exposure, limits, now_s=10.)
    light = g.LightExtrinsics(g.RigidTransform("body", "light", (.1, 0., -.07), (0., 0., 0., 1.),
                                              "light-v1", 0., 0.), (0., 0., -1.))
    illum = g.IlluminationLimits("odom", "body", "pose-v1", "light-v1", "range-v1", "feedback-v1",
                                 10., .2, .2, .2, .01, .03, .04, .05, .03, .2, .8, 2., .001)
    hover = g.hover_for_downlight(fix, light, exposure.transform.rotation_xyzw, .9, illum)
    current = replace(exposure, transform=replace(exposure.transform, translation_m=hover.position_world_m), stamp_s=11.)
    # The independent range sensor is 0.1 m below the body, versus 0.07 m for the light.
    measured = g.StampedRange(g.RigidTransform("body", "range", (.2, 0., -.1), (0., 0., 0., 1.),
                                              "range-v1", 0., 0.),
                              (0., 0., -1.), .8, .05, 4., 0., 11., "sim", "session-1")
    feedback = g.StampedBool(True, 11., "sim", "session-1", "feedback-v1")
    return g, fix, light, illum, hover, current, measured, feedback


def assess(parts, **changes):
    g, fix, light, limits, _, pose, measured, feedback = parts
    return g.assess_illumination(changes.get("fix", fix), changes.get("pose", pose), changes.get("light", light),
                                  changes.get("measured", measured), changes.get("feedback", feedback),
                                  changes.get("limits", limits), now_s=changes.get("now_s", 11.))


def test_optical_axis_exchange_translation_and_ninety_degree_yaw_use_full_chain():
    g, observation, camera, pose, limits = inputs()
    fix = g.localize_target(observation, camera, pose, limits, now_s=10.)
    # Optical (0,.93,1.11) -> body (1.50,0,-.8) -> odom (10,21.50,0).
    assert fix.position_world_m == pytest.approx((10., 21.5, 0.), abs=1e-12)
    assert (fix.target_id, fix.point_id, fix.world_frame, fix.body_frame) == ("person", "aerial-1", "odom", "body")
    assert fix.stamp_s == 10. and fix.calibration_version == "calib-v1" and fix.size_version == "size-v1"


def test_roll_is_preserved_rather_than_replaced_by_yaw_only():
    g, observation, camera, pose, limits = inputs()
    pose = replace(pose, transform=replace(pose.transform, translation_m=(0., 0., 0.),
                                           rotation_xyzw=(math.sqrt(.5), 0., 0., math.sqrt(.5))))
    fix = g.localize_target(replace(observation, position_camera_m=(1., 2., 3.)), camera, pose, limits, now_s=10.)
    assert fix.position_world_m == pytest.approx((3.39, 1.87, -1.), abs=1e-12)


def test_roll_pitch_and_yaw_compound_in_child_to_parent_order():
    g, observation, camera, pose, limits = inputs()
    # Rz(90) Ry(45) Rx(90), hand-derived quaternion and Cartesian result.
    c, s = math.cos(math.pi / 8), math.sin(math.pi / 8)
    q = ((c-s)/2, (c+s)/2, (c-s)/2, (c+s)/2)
    pose = replace(pose, transform=replace(pose.transform, translation_m=(0., 0., 0.), rotation_xyzw=q))
    fix = g.localize_target(replace(observation, position_camera_m=(1., 2., 3.)), camera, pose, limits, now_s=10.)
    assert fix.position_world_m == pytest.approx((-1.87, 2.39 / math.sqrt(2), -4.39 / math.sqrt(2)), abs=1e-12)


def test_pose_image_skew_costs_translation_and_rotation_and_boundary_is_inclusive():
    g, observation, camera, pose, limits = inputs()
    pose = replace(pose, stamp_s=10.05)
    fix = g.localize_target(observation, camera, pose, limits, now_s=10.05)
    assert fix.position_error_m == pytest.approx(.05 + 2 * 1.7 * math.sin(.025), abs=1e-12)
    with pytest.raises(ValueError, match="localization_error_exceeded"):
        g.localize_target(observation, camera, pose, replace(limits, max_position_error_m=.1), now_s=10.05)
    with pytest.raises(ValueError, match="pose_image_skew"):
        g.localize_target(observation, camera, replace(pose, stamp_s=10.0501), limits, now_s=10.0501)


@pytest.mark.parametrize("field,value,reason", [
    ("position_camera_m", (0., 0., -1.), "target_behind_camera"),
    ("position_camera_m", (0., float("nan"), 1.), "finite"),
    ("position_camera_m", (True, 0., 1.), "finite"),
    ("camera_frame", "body", "camera_frame"),
    ("coordinate_convention", "body_xyz", "optical_convention"),
    ("calibration_version", "other", "calibration_version"),
    ("size_version", "other", "size_version"),
    ("clock_id", "wall", "clock_mismatch"),
    ("epoch_id", "session-2", "epoch_mismatch"),
    ("stamp_s", 8., "observation_stale"),
    ("stamp_s", 11., "observation_future"),
])
def test_bad_observations_cannot_create_world_fix(field, value, reason):
    g, observation, camera, pose, limits = inputs()
    with pytest.raises(ValueError, match=reason):
        g.localize_target(replace(observation, **{field: value}), camera, pose, limits, now_s=10.)


@pytest.mark.parametrize("changes,reason", [
    ({"rotation_xyzw": (0., 0., 0., 2.)}, "unit_quaternion"),
    ({"child_frame": "other_body"}, "body_frame"),
    ({"parent_frame": "map"}, "world_frame"),
    ({"version": "other"}, "pose_version"),
])
def test_bad_full_pose_is_rejected(changes, reason):
    g, observation, camera, pose, limits = inputs()
    with pytest.raises(ValueError, match=reason):
        g.localize_target(observation, camera, replace(pose, transform=replace(pose.transform, **changes)), limits, now_s=10.)


def test_extrinsic_uncertainty_is_meter_bound_not_pixel_confidence():
    g, observation, camera, pose, limits = inputs()
    fix = g.localize_target(replace(observation, position_error_m=.01),
                              replace(camera, translation_error_m=.02, rotation_error_rad=.01),
                              replace(pose, transform=replace(pose.transform, translation_error_m=.03,
                                                             rotation_error_rad=.02)), limits, now_s=10.)
    expected = .06 + 2 * math.sin(.005) * math.hypot(.93, 1.11) + 2 * math.sin(.01) * 1.7
    assert fix.position_error_m == pytest.approx(expected, abs=1e-12)


def test_hover_compensates_rotated_light_offset_and_preserves_body_height():
    g, fix, light, limits, _, pose, _, _ = illumination()
    hover = g.hover_for_downlight(fix, light, pose.transform.rotation_xyzw, .8, limits)
    assert hover.position_world_m == pytest.approx((10., 21.4, .8), abs=1e-12)
    assert hover.ray_distance_m == pytest.approx(.73)


@pytest.mark.parametrize("direction", [(1., 0., 0.), (0., 0., 1.), (0., 0., -2.)])
def test_hover_rejects_parallel_upward_or_nonunit_light(direction):
    g, fix, light, limits, _, pose, _, _ = illumination()
    with pytest.raises(ValueError):
        g.hover_for_downlight(fix, replace(light, direction_light=direction), pose.transform.rotation_xyzw, .8, limits)


def test_independent_range_ray_agrees_with_plane_without_becoming_body_height():
    parts = illumination()
    result = assess(parts)
    assert result.allowed and result.reason == "ok"
    assert result.spot_world_m == pytest.approx((10., 21.5, 0.), abs=1e-12)
    assert result.range_residual_m == pytest.approx(0., abs=1e-12)
    assert result.total_error_m > 0.  # One second of explicitly bounded world-frame drift.


@pytest.mark.parametrize("kind,reason", [
    ("target_stale", "target_stale"), ("pose_stale", "pose_stale"),
    ("range_stale", "range_stale"), ("feedback_stale", "feedback_stale"),
    ("range_mismatch", "range_plane_mismatch"), ("light_off", "light_not_ready"),
    ("pose_reset", "epoch_mismatch"), ("clock", "clock_mismatch"),
    ("miss", "miss_distance_exceeded"), ("uncertainty", "total_error_exceeded"),
    ("low", "body_height"), ("bad_input", "invalid_target"),
    ("range_frame", "range_body_frame"), ("feedback_version", "feedback_version"),
])
def test_invalid_illumination_evidence_returns_explicit_rejection(kind, reason):
    parts = illumination()
    g, fix, light, limits, hover, pose, measured, feedback = parts
    changes = {
        "target_stale": {"fix": replace(fix, stamp_s=0.)},
        "pose_stale": {"pose": replace(pose, stamp_s=10.)},
        "range_stale": {"measured": replace(measured, stamp_s=10.)},
        "feedback_stale": {"feedback": replace(feedback, stamp_s=10.)},
        "range_mismatch": {"measured": replace(measured, distance_m=.9)},
        "light_off": {"feedback": replace(feedback, value=False)},
        "pose_reset": {"pose": replace(pose, epoch_id="session-2")},
        "clock": {"pose": replace(pose, clock_id="wall")},
        "miss": {"pose": replace(pose, transform=replace(pose.transform, translation_m=(10.06, 21.4, .9)))},
        "uncertainty": {"fix": replace(fix, position_error_m=.025)},
        "low": {"pose": replace(pose, transform=replace(pose.transform, translation_m=(10., 21.4, .79)))},
        "bad_input": {"fix": None},
        "range_frame": {"measured": replace(measured, transform=replace(measured.transform, parent_frame="other"))},
        "feedback_version": {"feedback": replace(feedback, version="other")},
    }[kind]
    result = assess(parts, **changes)
    assert not result.allowed and result.reason == reason


def test_valid_cached_fix_requires_no_new_front_camera_image_and_expires_without_retimestamping():
    parts = illumination()
    _, fix, _, limits, _, pose, measured, feedback = parts
    assert assess(parts).allowed
    assert fix.stamp_s == 10.
    result = assess(parts, now_s=20.01, pose=replace(pose, stamp_s=20.01),
                    measured=replace(measured, stamp_s=20.01), feedback=replace(feedback, stamp_s=20.01))
    assert not result.allowed and result.reason == "target_stale"


@pytest.mark.parametrize("kind,reason", [("pose_lag", "total_error_exceeded"),
                                        ("range_lag", "range_plane_mismatch"),
                                        ("height_error", "body_height")])
def test_fresh_but_asynchronous_or_uncertain_pose_cannot_pass_illumination(kind, reason):
    parts = illumination()
    _, fix, _, _, _, pose, measured, _ = parts
    changes = {
        "pose_lag": {"pose": replace(pose, stamp_s=10.95)},
        "range_lag": {"measured": replace(measured, stamp_s=10.95)},
        "height_error": {"fix": replace(fix, position_error_m=.01),
                         "pose": replace(pose, transform=replace(pose.transform, translation_m=(10., 21.4, .8))),
                         "measured": replace(measured, distance_m=.7)},
    }[kind]
    result = assess(parts, **changes)
    assert not result.allowed and result.reason == reason


@pytest.mark.parametrize("ray,source", [("light", "extrinsic"), ("light", "pose"), ("light", "motion"),
                                       ("range", "extrinsic"), ("range", "pose"), ("range", "motion")])
def test_max_down_tilt_is_hard_bound_including_angular_uncertainty(ray, source):
    parts = illumination()
    g, fix, light, limits, _, pose, measured, _ = parts
    limits = replace(limits, max_total_error_m=.1, hit_radius_m=.1, max_range_residual_m=.1)
    tilt = (0., math.sin(.19 / 2), 0., math.cos(.19 / 2))
    pose = replace(pose, linear_speed_bound_mps=0., angular_speed_bound_rps=0.)
    if ray == "light":
        light = replace(light, transform=replace(light.transform, rotation_xyzw=tilt))
        nominal = g.hover_for_downlight(fix, light, pose.transform.rotation_xyzw, .9, limits)
        pose = replace(pose, transform=replace(pose.transform, translation_m=nominal.position_world_m))
        if source == "extrinsic":
            light = replace(light, transform=replace(light.transform, rotation_error_rad=.02))
    else:
        measured = replace(measured, transform=replace(measured.transform, rotation_xyzw=tilt),
                           distance_m=.8 / math.cos(.19))
        if source == "extrinsic":
            measured = replace(measured, transform=replace(measured.transform, rotation_error_rad=.02))
    if source == "pose":
        pose = replace(pose, transform=replace(pose.transform, rotation_error_rad=.02))
    elif source == "motion":
        pose = replace(pose, angular_speed_bound_rps=.2)
        if ray == "light":
            pose = replace(pose, stamp_s=10.9)
        else:
            measured = replace(measured, stamp_s=10.9)
    result = assess(parts, light=light, pose=pose, measured=measured, limits=limits)
    assert not result.allowed and result.reason == ray + "_tilt_limit"


def test_drift_must_remain_inside_target_error_hard_limit():
    parts = illumination()
    limits = replace(parts[3], max_target_error_m=.001, localization_drift_mps=.003,
                     max_total_error_m=.1, hit_radius_m=.1, max_range_residual_m=.1)
    result = assess(parts, limits=limits)
    assert not result.allowed and result.reason == "target_error_exceeded"


def test_hover_rejects_extrinsic_tilt_bound_even_before_live_pose_is_available():
    g, fix, light, limits, _, pose, _, _ = illumination()
    light = replace(light, transform=replace(light.transform,
                                             rotation_xyzw=(0., math.sin(.19 / 2), 0., math.cos(.19 / 2)),
                                             rotation_error_rad=.02))
    with pytest.raises(ValueError, match="light_tilt_limit"):
        g.hover_for_downlight(fix, light, pose.transform.rotation_xyzw, .9, limits)
