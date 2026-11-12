"""Independent control/entry review; no ROS, publishers or matcher execution."""
from pathlib import Path
import sys
import math

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src" / "inspection_mission"), str(ROOT / "tests")]

import pytest

from test_ground_bounded_approach import refresh
from test_ground_single_point import SingleRun, fixture_jobs


def report_ready(first="ground_4"):
    run = SingleRun(first).until(lambda item: item.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    for result in results:
        assert run.guard.accept_match_result(result, wall_s=run.wall)
    sent = []
    assert run.guard.dispatch_one(lambda *args: sent.append(args), wall_s=run.wall)["sent"]
    return run, sent


@pytest.mark.parametrize("first", ["ground_0", "ground_1", "ground_4"])
def test_entry_route_keeps_permanent_origin_all_measured_totals_and_frozen_caps(first):
    run, sent = report_ready(first)
    guard = run.guard
    origin = guard.global_origin
    caps = tuple(getattr(guard.approach_limits, key) for key in
                 ("max_path_m", "max_rotation_rad", "max_origin_displacement_m"))
    route_caps = tuple(getattr(guard.route_plan.budgets, key) for key in
                       ("max_path_m", "max_rotation_rad", "max_origin_displacement_m"))
    path, rotation = guard.total_path_m, guard.accumulated_rotation_rad
    run.until(lambda item: item.guard.in_entry)
    assert guard.global_origin == origin
    assert guard.total_path_m >= path and guard.accumulated_rotation_rad >= rotation
    assert tuple(getattr(guard.route_plan.budgets, key) for key in
                 ("max_path_m", "max_rotation_rad", "max_origin_displacement_m")) == route_caps
    previous_path, previous_rotation = guard.total_path_m, guard.accumulated_rotation_rad
    for _ in range(3000):
        run.tick()
        assert guard.global_origin == origin
        assert guard.total_path_m >= previous_path
        assert guard.accumulated_rotation_rad >= previous_rotation
        assert tuple(getattr(guard.approach_limits, key) for key in
                     ("max_path_m", "max_rotation_rad", "max_origin_displacement_m")) == caps
        previous_path, previous_rotation = guard.total_path_m, guard.accumulated_rotation_rad
        if guard.state == "DECLARE_WAIT":
            break
        assert guard.state not in ("STOP", "FINISHED"), guard.reason
    else:
        pytest.fail("entry never stopped inside the candidate region")
    assert len(sent) == 1
    assert guard.total_path_m > path
    assert guard._inside_candidate_region()
    assert guard._stable_dwell(run.wall)


@pytest.mark.parametrize("fault", ["scan_stale", "odom_stale", "clock_stale"])
def test_report_wait_freshness_gap_latches_and_never_enters_after_sensor_recovery(fault):
    run, sent = report_ready()
    guard = run.guard
    guard.temporal_fault(fault, run.wall)
    for _ in range(25):
        run.tick()
        assert run.last.linear_x == run.last.angular_z == 0.
    assert guard.state in ("STOP", "FINISHED")
    assert guard.reason == "single_point_report_wait_sensor_freshness_lost:"+fault
    assert not guard.in_entry and len(sent) == 1
    assert "declaration" not in guard.attempts


def test_rotation_budget_counts_observed_back_and_forth_motion_with_small_net_yaw():
    run = SingleRun("ground_0").until(
        lambda item: item.guard.phase == "TURN" and item.guard.leg_index == 1)
    guard = run.guard
    anchor_yaw = run.yaw
    cap = guard.approach_limits.max_rotation_rad
    assert cap <= 6.
    for index in range(400):
        run.wall = round(run.wall + .05, 9)
        run.yaw = anchor_yaw + (.01 if index % 2 == 0 else -.01)
        refresh(guard, run.wall, x=run.x, y=run.y, yaw=run.yaw, speed=0., yaw_speed=.4)
        decision = guard.step(wall_s=run.wall)
        if guard.state == "STOP":
            break
    else:
        pytest.fail("oscillating measured yaw escaped the accumulated rotation cap")
    assert abs(run.yaw-anchor_yaw) <= .011
    assert guard.accumulated_rotation_rad >= cap-1e-9
    assert guard.reason == "approach_rotation_limit"
    assert decision.linear_x == decision.angular_z == 0.


def test_final_observation_turn_settle_fault_cannot_start_drive_or_recognition():
    run = SingleRun().until(lambda item: item.guard.phase == "TURN_SETTLE"
                           and item.guard.leg_index == len(item.guard.route_plan.segments))
    endpoint = run.guard.route_plan.waypoints_xy_m[-1]
    run.tick(dx=math.copysign(.021, run.x-endpoint[0]))
    assert math.dist((run.x, run.y), endpoint) > .02
    # .02 is the final waypoint acceptance bound, whereas the independent
    # turn drift cap is .06. A 2.1cm offset must fail before CAPTURE after
    # re-establishing static dwell, while every intervening request stays zero.
    for _ in range(30):
        run.tick()
        assert run.last.linear_x == run.last.angular_z == 0.
        if run.guard.state == "STOP":
            break
    assert run.guard.state == "STOP"
    assert run.guard.reason == "route_waypoint_error_limit"
    assert not run.guard.attempts and not run.guard.in_entry


def test_turn_settle_displacement_above_actual_cap_stops_on_the_callback():
    run = SingleRun().until(lambda item: item.guard.phase == "TURN_SETTLE"
                           and item.guard.leg_index == len(item.guard.route_plan.segments))
    cap = run.guard.approach_limits.max_turn_drift_m
    run.tick(dx=cap+.001)
    assert run.guard.state == "STOP"
    assert run.guard.reason == "approach_turn_displacement_limit"
    assert run.last.linear_x == run.last.angular_z == 0.


@pytest.mark.parametrize("first", ["ground_0", "ground_1", "ground_4"])
def test_entry_last_leg_ends_with_zero_and_no_extra_turn_before_one_declaration(first):
    run, sent = report_ready(first)
    run.until(lambda item: item.guard.in_entry and item.guard.leg_index == 1
              and item.guard.phase == "DRIVE_SETTLE")
    guard = run.guard
    last_turn_start = guard.turn_started_wall
    stop_path = guard.leg_stop_path_m
    run.until(lambda item: item.guard.state == "DECLARE_WAIT")
    assert run.last.linear_x == run.last.angular_z == 0.
    assert guard.turn_started_wall == last_turn_start
    assert guard.leg_stop_path_m == stop_path
    assert guard.drive_origin is None and guard.drive_yaw is None
    assert guard.total_path_m-stop_path <= .01+1e-9
    assert guard._stable_dwell(run.wall)
    event = guard.dispatch_one(lambda *args: sent.append(args), wall_s=run.wall)
    assert event["sent"] and event["topic"] == "/referee/current_target"
    assert sent[-1] == ("/referee/current_target", first)
    for _ in range(45):
        run.tick()
        guard.dispatch_one(lambda *args: sent.append(args), wall_s=run.wall)
        assert run.last.linear_x == run.last.angular_z == 0.
    assert len(sent) == 2
    assert guard.reason == "single_point_transport_complete_official_score_unverified"


def test_declaration_dispatch_cannot_skip_current_sensor_age_even_after_dwell():
    run, sent = report_ready()
    run.until(lambda item: item.guard.state == "DECLARE_WAIT")
    guard = run.guard
    # Advance the actual wall input without relabeling any sensor exposure.
    event = guard.dispatch_one(lambda *args: sent.append(args), wall_s=run.wall+.201)
    assert event is None
    assert len(sent) == 1 and "declaration" not in guard.attempts


def test_actual_scan_silence_after_report_stops_at_existing_age_bound():
    run, sent = report_ready()
    guard = run.guard
    original_stamp = guard.scan["stamp"]
    for _ in range(5):
        run.wall = round(run.wall+.05, 9)
        refresh(guard, run.wall, x=run.x, y=run.y, yaw=run.yaw, laser=False)
        decision = guard.step(wall_s=run.wall)
        assert decision.linear_x == decision.angular_z == 0.
    assert guard.scan["stamp"] == original_stamp
    assert guard.state == "STOP"
    assert guard.reason == "single_point_report_wait_sensor_freshness_lost:scan_stale_or_unsafe"
    assert not guard.in_entry and len(sent) == 1
