"""Pure geometry for one live first-point observation, without ROS or control."""
import importlib
import importlib.util
import math

import pytest


def planner():
    name = "inspection_mission.ground_observation_route_plan"
    assert importlib.util.find_spec(name) is not None, "pure first-point route planner is missing"
    return importlib.import_module(name).plan_ground_observation_route


@pytest.mark.parametrize("first,observation,path,rotation", [
    ("ground_0", (.6, -.45), 1.05, 2.098870775221256),
    ("ground_1", (.6, -.5), 1.10, 2.185459278717062),
    ("ground_2", (.6, -1.8), 2.40, 2.158798930342464),
    ("ground_3", (.6, -1.025), 1.625, 2.098870775221256),
    ("ground_4", (.35, 0.), .35, .7853981633974483),
])
def test_live_first_id_selects_only_its_route_and_full_turn_budget(first, observation, path, rotation):
    plan = planner()(first, (0., 0., 0.))
    assert plan.first_id == first
    assert plan.waypoints_xy_m[-1] == observation
    assert len(plan.segments) == (1 if first == "ground_4" else 2)
    assert plan.total_path_m == pytest.approx(path)
    assert plan.planned_rotation_rad == pytest.approx(rotation)
    assert plan.budgets.max_path_m == pytest.approx(path + .30)
    assert plan.budgets.max_rotation_rad == pytest.approx(rotation + .40)
    assert plan.budgets.max_origin_displacement_m == pytest.approx(math.hypot(*observation) + .15)
    # The final facing rotation is part of the budget, not another drive segment.
    assert plan.segments[-1].end_xy_m == observation


def test_real_initial_pose_changes_first_leg_and_permanent_origin_budget():
    plan = planner()("ground_2", (.03, .04, .04))
    assert plan.segments[0].start_xy_m == (.03, .04)
    assert plan.segments[0].end_xy_m == (.6, 0.)
    assert plan.segments[0].length_m == pytest.approx(math.sqrt(.3265))
    assert plan.total_path_m == pytest.approx(math.sqrt(.3265) + 1.8)
    assert plan.planned_rotation_rad == pytest.approx(2.198798930342464)
    assert plan.budgets.max_origin_displacement_m == pytest.approx(math.sqrt(3.7105) + .15)
    assert plan.target_xy_m == (0., -2.7)  # Candidate world coordinates are not shifted with the robot.
    assert plan.final_yaw_rad == pytest.approx(-2.158798930342464)


def test_ground4_observation_uses_positive_x_leg_then_turns_with_complete_board_projection():
    plan = planner()("ground_4", (0., 0., 0.))
    assert plan.waypoints_xy_m == ((.35, 0.),)
    assert plan.segments[0].heading_rad == 0.
    assert plan.total_path_m == .35
    assert plan.final_yaw_rad == pytest.approx(-.7853981633974483)
    assert plan.candidate_projection.all_corners_in_frame
    assert plan.candidate_projection.bbox_extent_px == pytest.approx((456.14165063380693, 143.6529386542499))
    assert plan.target_board_clearance_m == pytest.approx(.7619875327064086)


def test_ground2_candidate_projection_matches_independent_corner_fixture():
    projection = planner()("ground_2", (0., 0., 0.)).candidate_projection
    assert projection.pixel_corners_xy[0] == pytest.approx((456.1004348132568, 482.1031713593204))
    assert projection.pixel_corners_xy[2] == pytest.approx((249.73832900371733, 621.7686018382337))
    assert projection.bbox_extent_px == pytest.approx((468.5122408128459, 139.66543047891332))
    assert projection.image_size_wh == (800, 800)
    assert projection.all_corners_in_frame
    assert projection.status == "prediction_not_platform_observation"


def test_each_observation_is_outside_its_target_board_and_corners_remain_in_frame():
    for first in ("ground_0", "ground_1", "ground_2", "ground_3", "ground_4"):
        plan = planner()(first, (0., 0., 0.))
        assert plan.target_board_clearance_m > .447 + .02
        assert plan.candidate_projection.all_corners_in_frame
        assert plan.candidate_mapping_status == "candidate_unverified"
        assert plan.budgets.global_wall_s == 300.
        assert plan.budgets.wall_budget_status == "provisional_pending_platform_measurement"
        assert plan.status == "candidate_plan_not_execution"


@pytest.mark.parametrize("first,pose", [
    ("ground_5", (0., 0., 0.)),
    (None, (0., 0., 0.)),
    ("ground_0", (.050001, 0., 0.)),
    ("ground_0", (.04, .04, 0.)),
    ("ground_0", (0., 0., .050001)),
    ("ground_0", (0., 0., float("nan"))),
    ("ground_0", (float("inf"), 0., 0.)),
    ("ground_0", (False, 0., 0.)),
    ("ground_0", (0., 0.)),
])
def test_invalid_id_or_initial_pose_is_rejected_before_a_plan_is_returned(first, pose):
    with pytest.raises(ValueError):
        planner()(first, pose)
