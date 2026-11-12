"""Regression against the actually observed diagonal-motion mismatch."""
import importlib
import itertools
import math

import pytest


def module():
    return importlib.import_module("inspection_mission.ground_three_point_axis_plan")


ENDPOINTS = {"ground_0": (1.115, -1.35, 0.), "ground_1": (0., -1.35, math.pi),
             "ground_2": (0., -2.69, math.pi), "ground_3": (1.115, -1.925, -math.pi/2),
             "ground_4": (1.115, -.775, -math.pi/2)}


def test_g4_actual_regression_never_starts_with_diagonal():
    p = module().plan_ground_leg(None, "ground_4", (0., 0., 0.))
    drives = [o for o in p.to_observation_operations if o.target_xy_m is not None]
    assert [o.target_xy_m for o in drives] == [(.35, 0.), (.35, -.10)]


def test_all_sixty_orders_only_axis_drives_and_explicit_reverse():
    m = module()
    for order in itertools.permutations(ENDPOINTS, 3):
        previous, pose = None, (0., 0., 0.)
        for target in order:
            p = m.plan_ground_leg(previous, target, pose)
            assert p.minimum_laser_distance_m >= .641
            for op in p.to_observation_operations+p.entry_operations:
                if op.kind == "TURN_ONLY":
                    assert 0 < abs(op.signed_subturn_rad) <= math.pi/2+1e-12
                    continue
                dx, dy = [op.target_xy_m[i]-op.anchor_xy_m[i] for i in (0, 1)]
                assert abs(dx) < 1e-12 or abs(dy) < 1e-12
                body = (math.cos(op.target_yaw_rad), math.sin(op.target_yaw_rad))
                direction = dx*body[0]+dy*body[1]
                assert direction < 0 if op.kind == "DRIVE_REVERSE_TO_XY" else direction > 0
            previous, pose = target, ENDPOINTS[target]


def test_ground2_reverse_keeps_sensor_on_north_side_of_south_wall():
    p = module().plan_ground_leg(None, "ground_2", (0., 0., 0.))
    op = next(o for o in p.entry_operations if o.kind == "DRIVE_REVERSE_TO_XY")
    assert op.anchor_xy_m == (0., -1.8) and op.target_xy_m == (0., -2.69)
    assert op.target_yaw_rad == pytest.approx(math.pi/2)
    assert op.minimum_laser_distance_m >= .74-1e-12


def test_ground2_actual_v43_entry_keeps_real_row_without_a_millimetre_aim_point():
    # v43 reached this real observation pose. A 4.67mm Y correction cannot
    # be executed by the existing >20mm drive gate and its bearing moves
    # substantially under the observed millimetre-scale turn drift.
    pose = (.5888341895790857, -1.7953274616929733, -2.1343145342713865)
    operations = module().plan_ground_entry("ground_2", pose)
    drives = [op for op in operations if op.target_xy_m is not None]
    assert drives[0].anchor_xy_m == pose[:2]
    assert drives[0].target_xy_m == (0., pose[1])
    assert drives[0].length_m == pytest.approx(pose[0])
    assert all(op.length_m > .02 for op in drives)
    assert drives[-1].target_xy_m == (0., -2.69)
    assert drives[-1].kind == "DRIVE_REVERSE_TO_XY"
    for op in drives:
        dx, dy = [op.target_xy_m[i]-op.anchor_xy_m[i] for i in (0, 1)]
        assert min(abs(dx), abs(dy)) < 1e-12
        assert op.minimum_laser_distance_m >= .641


def test_entry_keeps_an_executable_axis_correction_outside_the_waypoint_gate():
    operations = module().plan_ground_entry("ground_2", (.5888, -1.779, -2.1343))
    first_drive = next(op for op in operations if op.target_xy_m is not None)
    assert first_drive.target_xy_m == (.5888, -1.8)
    assert first_drive.length_m == pytest.approx(.021)


def test_omitting_two_short_axes_cannot_expand_the_radial_waypoint_gate():
    # Each 15mm component is below the executable-drive floor, but the
    # combined 21.2mm error is outside the unchanged 20mm waypoint gate.
    with pytest.raises(ValueError, match="axis residual exceeds waypoint gate"):
        module().plan_ground_entry("ground_4", (1.100, -.760, -math.pi/2))


def test_actual_pose_is_preserved_and_unsafe_south_departure_rejected():
    m = module()
    p = m.plan_ground_leg("ground_0", "ground_4", (1.113, -1.346, .006))
    assert p.initial_pose == (1.113, -1.346, .006)
    drives = [o for o in p.to_observation_operations if o.kind.startswith("DRIVE")]
    assert drives[0].anchor_xy_m == (1.113, -1.346)
    assert drives[0].target_xy_m == (.6, -1.346)
    with pytest.raises(ValueError):
        m.plan_ground_leg("ground_2", "ground_0", (0., -2.71, math.pi))


def test_independent_sampled_laser_path_including_reverse_has_original_clearance():
    from inspection_mission.ground_three_point_plan import BOXES
    unique = {}
    for previous in (None, *ENDPOINTS):
        pose = (0., 0., 0.) if previous is None else ENDPOINTS[previous]
        for target in ENDPOINTS:
            if target == previous:
                continue
            p = module().plan_ground_leg(previous, target, pose)
            for op in p.to_observation_operations+p.entry_operations:
                unique[(op.kind, op.anchor_xy_m, op.target_xy_m, op.expected_start_yaw_rad,
                        op.target_yaw_rad, op.signed_subturn_rad)] = op
    for op in unique.values():
        for index in range(501):
            t = index/500
            if op.kind == "TURN_ONLY":
                xy = op.anchor_xy_m
                heading = op.expected_start_yaw_rad+op.signed_subturn_rad*t
            else:
                xy = tuple(op.anchor_xy_m[i]*(1-t)+op.target_xy_m[i]*t for i in (0, 1))
                heading = op.target_yaw_rad
            point = (xy[0]+.08*math.cos(heading), xy[1]+.08*math.sin(heading))
            clearance = min(math.hypot(max(xmin-point[0], 0., point[0]-xmax),
                                       max(ymin-point[1], 0., point[1]-ymax))
                            for _, (xmin, ymin, xmax, ymax) in BOXES)
            assert clearance >= .641-1e-12
            assert clearance >= op.minimum_laser_distance_m-1e-12
