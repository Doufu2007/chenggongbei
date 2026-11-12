"""v45 bounded command candidates; synthetic feedback is not platform proof."""
import pytest

from inspection_mission.ground_three_point import GroundThreePointGuard
from inspection_mission.ground_three_point_axis import GroundThreePointAxisGuard
from inspection_mission.ground_three_point_plan import GroundOperation
from test_bounded_ground_probe import endpoint, scan
from test_ground_bounded_approach import refresh


def live_guard(cls):
    g = cls(started_wall_s=0.)
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    g.observe_public("/referee/ground_tasks", {"data": "ground_1,ground_4,ground_2"}, wall_s=0.)
    refresh(g, 0., x=.6, y=-.5)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = g.odom["xyz"], 0.
    return g


def turn_guard(error):
    g = live_guard(GroundThreePointGuard)
    op = GroundOperation("TURN_ONLY", None, (.6, -.5), 0., error, error, "v45_turn", 1.)
    g._load_operations((op,), "observation", 0.)
    return g


@pytest.mark.parametrize("error,rate", [(1., .5), (.5, .5), (.3, .45), (.2, .35), (.0251, .35)])
@pytest.mark.parametrize("sign", [-1, 1])
def test_turn_gain_uses_bounded_high_tier_for_medium_errors_and_retains_near_tier(error, rate, sign):
    g = turn_guard(sign*error)
    decision = g.step(wall_s=0.)
    assert decision.linear_x == 0.
    assert decision.angular_z == pytest.approx(sign*rate)
    assert abs(decision.angular_z) <= .5
    assert g.turn_started_wall == 0.


@pytest.mark.parametrize("sign", [-1, 1])
def test_turn_within_original_aim_tolerance_still_requests_zero(sign):
    g = turn_guard(sign*.0249)
    decision = g.step(wall_s=0.)
    assert g.phase == "TURN_SETTLE"
    assert decision.linear_x == decision.angular_z == 0.


@pytest.mark.parametrize("fault", ["expired", "pause", "unsafe_scan", "over_rate", "deadline"])
def test_higher_turn_request_cannot_bypass_original_zero_and_stop_gates(fault):
    g = turn_guard(.5)
    wall = .05
    if fault == "expired":
        wall = .201
    elif fault == "pause":
        g.temporal_fault("scan_stale", .01)
        refresh(g, wall, x=.6, y=-.5)
    elif fault == "unsafe_scan":
        msg = scan(10.05); msg["ranges"][0] = .64
        g.update_scan(msg, wall_s=wall)
    elif fault == "over_rate":
        refresh(g, wall, x=.6, y=-.5, yaw_speed=.551)
    else:
        wall = 45.
        refresh(g, wall, x=.6, y=-.5)
    decision = g.step(wall_s=wall)
    assert decision.linear_x == decision.angular_z == 0.
    assert g.state in ("PAUSE", "STOP")
    assert g.turn_started_wall == 0.
    assert g.approach_limits.turn_wall_s == 45.
    assert g.limits.sensor_age_s == .2


def drive_guard(sign, length=.2, *, initial_slow_validated=True):
    g = live_guard(GroundThreePointAxisGuard)
    kind = "DRIVE_TO_XY" if sign > 0 else "DRIVE_REVERSE_TO_XY"
    op = GroundOperation(kind, (.6+sign*length, -.5), (.6, -.5), 0., 0., 0., "v45_drive", 1., length)
    g._load_operations((op,), "entry", 0.)
    g.calibration_verified = True
    g.initial_slow_validated = initial_slow_validated
    return g


def progress_with_current_scans(g, sign, distance):
    # Feed a bounded path with new real-parser scan anchors, rather than
    # teleporting farther than the unchanged 30mm local scan-motion reserve.
    steps = int(distance/.003)+1
    for i in range(1, steps+1):
        wall = round(i*.05, 9)
        refresh(g, wall, x=.6+sign*distance*i/steps, y=-.5)
    return wall


@pytest.mark.parametrize("remaining,rate", [(.08, .10), (.030001, .10), (.03, .03), (.029, .03), (.003, .03)])
@pytest.mark.parametrize("sign", [-1, 1])
def test_final_axis_drive_stays_fast_until_three_cm_tail_candidate(remaining, rate, sign):
    g = drive_guard(sign)
    wall = progress_with_current_scans(g, sign, .2-remaining)
    decision = g.step(wall_s=wall)
    assert g.state == "DRIVE", g.reason
    assert decision.linear_x == pytest.approx(sign*rate)
    assert decision.angular_z == 0.
    assert g.approach_limits.target_distance_m == pytest.approx(.198)
    assert g.scan["required_range_m"] == pytest.approx(.641)


@pytest.mark.parametrize("sign", [-1, 1])
def test_executable_segment_shorter_than_three_cm_remains_slow_for_its_whole_length(sign):
    g = drive_guard(sign, .025)
    for wall, remaining in ((.05, .025), (.10, .015), (.15, .003)):
        refresh(g, wall, x=.6+sign*(.025-remaining), y=-.5)
        assert g.step(wall_s=wall).linear_x == pytest.approx(sign*.03)


@pytest.mark.parametrize("sign", [-1, 1])
def test_first_slow_interval_still_requires_both_full_wall_and_source_second(sign):
    g = drive_guard(sign, initial_slow_validated=False)
    assert g.step(wall_s=0.).linear_x == pytest.approx(sign*.03)
    for i in range(1, 20):
        wall = round(i*.05, 9)
        refresh(g, wall, x=.6, y=-.5)
        assert g.step(wall_s=wall).linear_x == pytest.approx(sign*.03)
        assert not g.initial_slow_validated
    refresh(g, 1., x=.6, y=-.5)
    assert g.step(wall_s=1.).linear_x == pytest.approx(sign*.10)
    assert g.initial_slow_validated


@pytest.mark.parametrize("sign", [-1, 1])
def test_three_cm_tail_keeps_two_mm_early_zero_and_one_cm_accumulated_stop_path(sign):
    g = drive_guard(sign)
    wall = progress_with_current_scans(g, sign, .1981)
    decision = g.step(wall_s=wall)
    assert g.phase == "DRIVE_SETTLE"
    assert decision.linear_x == decision.angular_z == 0.
    anchor = g.total_path_m
    for _ in range(3):
        wall = round(wall+.05, 9)
        refresh(g, wall, x=g.odom["xyz"][0]+sign*.004, y=-.5)
    assert g.total_path_m-anchor > .01
    assert g.state == "STOP" and g.reason == "route_drive_settle_path_limit"
    assert g.step(wall_s=wall).linear_x == 0.
