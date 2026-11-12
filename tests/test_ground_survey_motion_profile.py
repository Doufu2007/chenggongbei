"""Bounded response calibration retains measured stopping and safety gates."""
import math

import pytest

from test_ground_observation_survey import api, ready, refresh
from inspection_mission.bounded_ground_probe import MotionLimits


def calibrated_guard():
    g = api()
    limits = g.SurveyLimits(minimum_yaw_rate=.35, max_turn_wall_s=45., global_wall_s=60.)
    return g.GroundSurveyGuard(MotionLimits(.447, .114, .02), started_wall_s=0., survey_limits=limits)


def test_calibrated_floor_keeps_command_bounded_and_stops_by_real_yaw():
    state = calibrated_guard()
    assert ready(state).angular_z == -.5
    refresh(state, 1.15, yaw=-1.2, yaw_speed=.03)
    decision = state.step(wall_s=1.15)
    assert decision.angular_z == -.35 and decision.linear_x == 0.
    refresh(state, 1.20, yaw=state.target_yaw, yaw_speed=.03)
    assert state.step(wall_s=1.20).angular_z == 0.
    assert state.reason == "target_yaw_reached"


def test_extended_turn_time_is_finite_and_never_resets_original_origin():
    state = calibrated_guard()
    ready(state)
    started, origin = state.pulse_started, state.pulse_origin
    for i in range(1, 522):
        wall = 1.1 + i * .05
        refresh(state, wall, yaw=-.3, yaw_speed=.03)
        decision = state.step(wall_s=wall)
    assert decision.state == "TURN" and decision.angular_z < 0.
    assert state.pulse_started == started and state.pulse_origin == origin
    refresh(state, started + 45., yaw=-.3, yaw_speed=.03)
    decision = state.step(wall_s=started + 45.)
    assert decision.angular_z == 0. and decision.reason == "turn_wall_time_limit"
    assert state.limits.sensor_age_s == .2
    assert state.limits.max_displacement_m == .06
    assert state.survey_limits.max_rotation_rad == 1.8


@pytest.mark.parametrize("kwargs", [
    {"minimum_yaw_rate": 0.}, {"minimum_yaw_rate": .351},
    {"minimum_yaw_rate": math.nan}, {"minimum_yaw_rate": .35, "yaw_rate": .3},
    {"max_turn_wall_s": 45.001},
])
def test_response_calibration_rejects_unbounded_or_inconsistent_parameters(kwargs):
    with pytest.raises(ValueError):
        api().SurveyLimits(**kwargs)


def test_prior_profile_defaults_remain_unchanged():
    limits = api().SurveyLimits()
    assert limits.minimum_yaw_rate == .04
    assert limits.max_turn_wall_s == 25. and limits.global_wall_s == 45.
