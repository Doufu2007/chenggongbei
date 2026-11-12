"""Known timing examples for descriptive command/odometry association."""
import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "analyze_survey_response", Path(__file__).parents[1] / "tools/analyze_survey_response.py")
response = importlib.util.module_from_spec(spec)
spec.loader.exec_module(response)


def command(wall, state, angular=0.):
    return {"wall_elapsed_s": wall, "clock_stamp_s": 10. + wall,
            "linear_x": 0., "angular_z": angular, "state": state}


def pose(wall, stamp, yaw):
    return {"wall_elapsed_s": wall, "stamp_s": stamp,
            "yaw_rad": yaw, "angular_speed": .1}


def summary(commands, poses):
    return {"survey": {"target_yaw": -1.}, "series": {
        "actual_commands": {"records": commands},
        "trajectory_10hz": {"records": poses}}}


def test_prior_command_never_uses_future_and_rates_keep_two_time_denominators():
    result = response.summarize(summary(
        [command(0., "TURN", -.5), command(.15, "TURN", -.35)],
        [pose(.01, 10., 0.), pose(.11, 10.2, -.02), pose(.21, 10.4, -.04)]))
    first = result["intervals"][0]
    assert first["prior_command"]["wall_elapsed_s"] == 0.
    assert first["prior_command"]["angular_z"] == -.5
    assert first["pose_rate_sim_rad_s"] == pytest.approx(-.1)
    assert first["pose_rate_wall_rad_s"] == pytest.approx(-.2)
    assert result["intervals"][1]["prior_command"]["wall_elapsed_s"] == .15


def test_turn_intervals_crossing_pause_are_not_retained():
    result = response.summarize(summary(
        [command(0., "TURN", -.5), command(.1, "PAUSE"),
         command(.2, "TURN", -.35), command(.3, "TURN", -.35)],
        [pose(.05, 10., 0.), pose(.15, 10.1, -.01),
         pose(.25, 10.2, -.02), pose(.35, 10.3, -.03)]))
    assert [row["retained_turn_interval"] for row in result["intervals"]] == [False, False, True]
    assert result["intervals"][0]["overlapping_command_states"] == ["PAUSE", "TURN"]


def test_no_retained_turn_reports_missing_observed_rate():
    result = response.summarize(summary(
        [command(0., "WAIT"), command(.2, "STOP")],
        [pose(.05, 10., 0.), pose(.15, 10.1, 0.), pose(.25, 10.2, 0.)]))
    assert result["command_bins"] == {}
    assert result["last_three_turn_seconds"]["n"] == 0
    assert result["last_three_turn_seconds"]["median_abs_pose_rate_sim_rad_s"] is None
    assert result["constant_observed_rate_scenario"]["angle_over_extra_20_sim_seconds_rad"] is None
