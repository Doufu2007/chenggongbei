"""Measured vertical extents must not be replaced by a horizontal rotor radius."""
import json
import math

import pytest

from inspection_mission.mission import MissionController
from inspection_mission.runtime import RuntimeInputs
import inspection_mission.mission_node as mission_node
from tools.mission_scenario import demonstration_plan
from test_pointcloud_profile import cloud_profile
from test_runtime import odom


def candidate_plan():
    # Public candidate world only; these are not verified platform bindings.
    plan = demonstration_plan()
    plan.update(robot_radius_m=.447, clearance_m=.03,
                uav_body_envelope={"below_origin_m": .068, "above_origin_m": .166,
                                   "max_tilt_rad": .05},
                settings={"position_tolerance_m": .01})
    for window in plan["windows"]:
        window.update(bottom=.4, top=1.15, crossing_z=.75, width_m=1.)
        window["exit"][2] = .75
        # Synthetic route fixture must also clear the unchanged rotor diameter.
        window["exit"][0] = window["approach"][0] + 1.2*math.cos(window["approach"][3])
    return plan


def envelope_controller():
    plan = candidate_plan()
    return MissionController(plan, plan["settings"])


def envelope_inputs(points, *, pitch=0., vertical_fov=1.1):
    profile = cloud_profile()
    profile["max_planar_tilt_rad"] = .1
    profile["channels"]["uav_cloud"].update(translation_m=[0., 0., 0.],
                                               vertical_fov_rad=vertical_fov)
    data = RuntimeInputs(profile, {"g1", "g2", "g3"},
                         uav_envelope=envelope_controller().uav_envelope)
    payload = odom(frame="uav/odom")
    payload["pose"]["pose"]["orientation"].update(y=math.sin(pitch/2), w=math.cos(pitch/2))
    data.update_pose("uav", payload, now=5)
    data.update_cloud({"frame_id": "uav/lidar_link", "stamp": {"sec": 5, "nanosec": 0}}, points, now=5)
    return data


FAR = [[2., 0., 0.], [2.2, 0., 0.], [2.4, 0., 0.]]


def test_public_candidate_window_fits_full_rotor_radius_and_separate_vertical_extents():
    controller = envelope_controller()
    assert controller.plan["robot_radius_m"] == .447
    assert controller.uav_envelope.horizontal_radius_m > .447 + .03


def test_legacy_missing_vertical_extent_keeps_conservative_sphere_rejection():
    plan = candidate_plan()
    del plan["uav_body_envelope"]
    with pytest.raises(ValueError, match="vertical clearance"):
        MissionController(plan)
    assert MissionController(demonstration_plan()).uav_envelope is None


@pytest.mark.parametrize("change", [None, {}, {"below_origin_m": .068},
    {"below_origin_m": -.01, "above_origin_m": .166, "max_tilt_rad": .05},
    {"below_origin_m": .068, "above_origin_m": float("nan"), "max_tilt_rad": .05},
    {"below_origin_m": .068, "above_origin_m": .166, "max_tilt_rad": .16}])
def test_partial_unknown_or_invalid_vertical_envelope_is_not_ignored(change):
    plan = demonstration_plan()
    plan["uav_body_envelope"] = change
    with pytest.raises(ValueError, match="uav_body_envelope"):
        MissionController(plan)


@pytest.mark.parametrize("crossing_z", [.55, .90])
def test_true_lower_emitter_and_upper_rotor_clearances_remain_enforced(crossing_z):
    plan = candidate_plan()
    for window in plan["windows"]:
        window["crossing_z"] = window["exit"][2] = crossing_z
    with pytest.raises(ValueError, match="vertical clearance"):
        MissionController(plan)


def test_expanded_vertical_bounds_cover_every_permitted_tilt():
    envelope = envelope_controller().uav_envelope
    assert envelope.below_origin_m == pytest.approx(.068 + .447*math.sin(.05) + .03)
    assert envelope.above_origin_m == pytest.approx(.166 + .447*math.sin(.05) + .03)
    assert envelope.horizontal_radius_m == pytest.approx(.447 + .166*math.sin(.05) + .03)


def test_pointcloud_uses_separate_vertical_extent_but_keeps_full_horizontal_radius():
    radius = envelope_controller().uav_envelope.horizontal_radius_m
    data = envelope_inputs(FAR + [[.6, 0., .30]])
    assert data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9, corridor_radius=radius)
    for obstacle in ([.6, 0., .20], [.6, .48, 0.], [.6, 0., -.12]):
        data = envelope_inputs(FAR + [obstacle])
        assert not data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9, corridor_radius=radius)


def test_pointcloud_fov_still_checks_both_vertical_edges_of_explicit_envelope():
    data = envelope_inputs(FAR, vertical_fov=.3)
    assert not data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9,
                                 corridor_radius=data.uav_envelope.horizontal_radius_m)


def test_runtime_rejects_tilt_outside_envelope_even_when_profile_allows_it():
    with pytest.raises(ValueError, match="tilt"):
        envelope_inputs(FAR, pitch=.051)


def test_runtime_levels_tilted_cloud_before_testing_world_vertical_extent():
    # Body-frame pitch .04 places this world-level point above the upper bound.
    pitch = .04
    level = [.6, 0., .225]
    body = [math.cos(pitch)*level[0] - math.sin(pitch)*level[2], 0.,
            math.sin(pitch)*level[0] + math.cos(pitch)*level[2]]
    far_body = [[math.cos(pitch)*p[0], 0., math.sin(pitch)*p[0]] for p in FAR]
    data = envelope_inputs(far_body + [body], pitch=pitch)
    assert data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9,
                              corridor_radius=data.uav_envelope.horizontal_radius_m)


@pytest.mark.parametrize("velocity", [(0, 0, .2, 0), (0, 0, -.2, 0), (-.2, 0, 0, 0), (0, 0, 0, .2)])
def test_explicit_envelope_does_not_open_front_sensor_blind_directions(velocity):
    data = envelope_inputs(FAR)
    assert not data.motion_clear("uav", velocity, 5, stop_distance=.9,
                                 corridor_radius=data.uav_envelope.horizontal_radius_m)


def test_prelaunch_binds_same_envelope_into_runtime(monkeypatch, tmp_path):
    profile = cloud_profile()
    calls = []
    real_inputs = mission_node.RuntimeInputs
    def capture(*args, **kwargs):
        calls.append(kwargs.get("uav_envelope"))
        return real_inputs(*args, **kwargs)
    monkeypatch.setattr(mission_node, "RuntimeInputs", capture)
    plan_path, profile_path = tmp_path / "plan.json", tmp_path / "profile.json"
    plan_path.write_text(json.dumps(candidate_plan()), encoding="utf-8")
    profile_path.write_text(json.dumps(profile), encoding="utf-8")
    _, _, controller = mission_node.load_configuration(plan_path, profile_path)
    assert calls == [controller.uav_envelope]


def test_node_motion_gate_passes_expanded_horizontal_radius_to_real_runtime():
    controller = envelope_controller()
    data = envelope_inputs(FAR + [[.55, .48, 0.]])
    # .48m is outside the old .477 radius, inside the allowed-tilt .485 radius.
    assert not mission_node.motion_path_clear(controller, data, "uav", (.2, 0, 0, 0), 5)


def test_window_side_separation_keeps_expanded_horizontal_extent():
    plan = candidate_plan()
    window = plan["windows"][0]
    window["exit"][0] = window["approach"][0] + .90
    with pytest.raises(ValueError, match="distinct sides"):
        MissionController(plan, plan["settings"])


def test_real_window_rejects_default_position_tolerance_instead_of_risking_rotor_contact():
    with pytest.raises(ValueError, match="horizontal clearance"):
        MissionController(candidate_plan())  # Default .08m is too wide for a 1m aperture.


@pytest.mark.parametrize("width", [None, 0., float("nan"), .98])
def test_explicit_envelope_requires_window_width_that_contains_control_tolerance(width):
    plan = candidate_plan()
    plan["windows"][0]["width_m"] = width
    with pytest.raises(ValueError, match="window.*width|horizontal clearance"):
        MissionController(plan, plan["settings"])


def test_explicit_envelope_does_not_invent_missing_window_width():
    plan = candidate_plan()
    del plan["windows"][0]["width_m"]
    with pytest.raises(ValueError, match="window.*width"):
        MissionController(plan, plan["settings"])


@pytest.mark.parametrize("edge", ["bottom", "top"])
def test_window_z_tolerance_must_fit_inside_true_expanded_vertical_bounds(edge):
    plan = candidate_plan()
    envelope = envelope_controller().uav_envelope
    window = plan["windows"][0]
    safe = (.4 + envelope.below_origin_m + .04 if edge == "bottom"
            else 1.15 - envelope.above_origin_m - .04)
    window["crossing_z"] = window["exit"][2] = safe + (1e-6 if edge == "bottom" else -1e-6)
    MissionController(plan, plan["settings"])
    window["crossing_z"] = window["exit"][2] = safe + (-1e-6 if edge == "bottom" else 1e-6)
    with pytest.raises(ValueError, match="vertical clearance"):
        MissionController(plan, plan["settings"])


def test_explicit_runtime_cannot_reduce_radius_or_reuse_skewed_cloud_pose():
    data = envelope_inputs(FAR)
    assert not data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9, corridor_radius=.477)
    data.cloud["stamp"] = 4.99
    assert not data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9,
                                 corridor_radius=data.uav_envelope.horizontal_radius_m)


def test_invalid_pose_clears_previously_accepted_envelope_motion_evidence():
    data = envelope_inputs(FAR)
    bad = odom(frame="uav/odom")
    bad["pose"]["pose"]["orientation"].update(y=math.sin(.06/2), w=math.cos(.06/2))
    with pytest.raises(ValueError, match="tilt"):
        data.update_pose("uav", bad, now=5)
    assert data.poses["uav"] is None and data.odometry_evidence["uav"] is None
    assert not data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9,
                                 corridor_radius=data.uav_envelope.horizontal_radius_m)


@pytest.mark.parametrize("boundary", ["below_origin_m", "above_origin_m"])
def test_vertical_envelope_boundary_contact_blocks_motion(boundary):
    envelope = envelope_controller().uav_envelope
    z = getattr(envelope, boundary) * (-1 if boundary == "below_origin_m" else 1)
    data = envelope_inputs(FAR + [[.6, 0., z]])
    assert not data.motion_clear("uav", (.2, 0, 0, 0), 5, stop_distance=.9,
                                 corridor_radius=envelope.horizontal_radius_m)
