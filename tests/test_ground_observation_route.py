"""Exercise the new route against real guard parsers and measured feedback."""
import importlib
import importlib.util
import math

import pytest

from test_bounded_ground_probe import endpoint, install_ros_transport, scan
from test_ground_bounded_approach import image, refresh
from test_ground_observation_survey import posed_odom


def api():
    name = "inspection_mission.ground_observation_route"
    assert importlib.util.find_spec(name) is not None, "independent measured route controller is missing"
    return importlib.import_module(name)


class Run:
    # A finite non-unit actuator illustration, not a calibrated platform model.
    # v20 angular response/request ratios were about .0965 at .35 and .1655
    # at .5; unit gain makes .35*.05 rad steps skip the long leg's narrow aim.
    def __init__(self, first="ground_2", initial=(0., 0., 0.), gain=1., route_limits=None,
                 angular_gain=.15):
        self.guard = api().GroundObservationRouteGuard(started_wall_s=0., route_limits=route_limits)
        self.guard.update_endpoints([endpoint()], matched_subscriptions=1)
        others = [p for p in ("ground_0", "ground_1", "ground_2") if p != first][:2]
        if len(others) < 2:
            others.append("ground_3")
        self.guard.observe_public("/referee/ground_tasks", {"data": ",".join([first, *others])}, wall_s=0.)
        self.wall, self.x, self.y, self.yaw = 0., *initial
        self.last = None
        self.commands = []
        self.gain = gain
        self.angular_gain = angular_gain

    def tick(self, *, dx=0., dy=0.):
        dt = .05
        linear = 0. if self.last is None else self.last.linear_x
        linear = min(.10, linear*self.gain)
        angular = 0. if self.last is None else self.last.angular_z*self.angular_gain
        self.x += dt*linear*math.cos(self.yaw)+dx
        self.y += dt*linear*math.sin(self.yaw)+dy
        self.yaw += dt*angular
        self.wall = round(self.wall+dt, 9)
        refresh(self.guard, self.wall, x=self.x, y=self.y, yaw=self.yaw,
                speed=linear, yaw_speed=abs(angular))
        self.last = self.guard.step(wall_s=self.wall)
        self.commands.append((self.wall, self.last, self.guard.phase, self.x, self.y))
        return self.last

    def until(self, predicate, cap=5000):
        for _ in range(cap):
            self.tick()
            if predicate(self):
                return self
            assert self.guard.state not in ("STOP", "FINISHED"), self.guard.reason
        pytest.fail("route did not reach expected phase")


@pytest.mark.parametrize("first,xy", [("ground_0", (.6, -.45)), ("ground_1", (.6, -.5)),
                                     ("ground_2", (.6, -1.8)), ("ground_3", (.6, -1.025)),
                                     ("ground_4", (.35, 0.))])
def test_live_first_route_reaches_own_observation_then_final_turn_without_extra_drive(first, xy):
    run = Run(first).until(lambda r: r.guard.state == "CAPTURE")
    state = run.guard
    assert math.dist((run.x, run.y), xy) <= .02
    assert state.target_id == first
    assert state.global_origin == (0., 0., .055)
    assert state.total_path_m > .30
    assert state.scan["required_range_m"] == pytest.approx(.641)
    assert abs(state.target_yaw-run.yaw) <= .025
    assert state.route_plan.initial_pose == (0., 0., 0.)
    last_drive = max(i for i, row in enumerate(run.commands) if row[1].linear_x)
    assert all(row[1].linear_x == 0. for row in run.commands[last_drive+1:])
    assert max(row[1].linear_x for row in run.commands) == .10
    assert {row["stage"] for row in state.survey_evidence()["scan_records"]} == {
        "start", "first_leg_end", "observation_arrival", "final_heading"}


def test_initial_slow_tier_and_measured_stop_test_precede_route_continuation():
    run = Run().until(lambda r: r.guard.calibration_verified)
    first_drive = next(row[0] for row in run.commands if row[1].linear_x)
    first_fast = next(row[0] for row in run.commands if row[1].linear_x == .10)
    assert first_fast-first_drive >= 1.-1e-9
    evidence = run.guard.approach_evidence()["route"]["speed_validation"]
    assert evidence["stop_forward_m"] >= .05
    assert evidence["stop_path_increment_m"] <= .01
    assert evidence["stationary_wall_s"] >= 1.-1e-9
    assert evidence["stationary_sim_s"] >= 1.-1e-9
    stop_rows = [row for row in run.commands if row[2] == "CALIBRATION_SETTLE"]
    assert stop_rows and all(row[1].linear_x == 0. for row in stop_rows)


def test_stop_increment_is_real_accumulated_path_and_latches_before_new_scan():
    run = Run().until(lambda r: r.guard.phase == "CALIBRATION_SETTLE")
    for _ in range(3):
        run.tick(dx=.004)
    assert run.guard.state == "STOP"
    assert run.guard.reason == "route_calibration_stop_distance_limit"
    assert run.last.linear_x == 0.


@pytest.mark.parametrize("leg", [0, 1])
def test_each_leg_zero_request_target_leaves_only_two_mm_early_gap(leg):
    run = Run().until(lambda r: r.guard.leg_index == leg and r.guard.phase == "DRIVE")
    assert run.guard.approach_limits.target_distance_m == pytest.approx(run.guard.leg_target_distance_m-.002)


@pytest.mark.parametrize("leg", [0, 1])
def test_each_ordinary_drive_settle_latches_after_one_cm_accumulated_path(leg):
    run = Run().until(lambda r: r.guard.leg_index == leg and r.guard.phase == "DRIVE_SETTLE")
    origin = run.guard.total_path_m
    for dx in (.004, -.004, .004):
        run.tick(dx=dx)
    assert run.guard.total_path_m-origin > .01
    assert run.guard.state == "STOP" and run.guard.reason == "route_drive_settle_path_limit"
    reason = run.guard.reason
    run.tick()
    assert run.guard.reason == reason and run.last.linear_x == run.last.angular_z == 0.


def test_ordinary_stop_path_anchor_is_trigger_pose_and_pause_does_not_reset_it():
    run = Run("ground_4").until(lambda r: r.guard.phase == "DRIVE_SETTLE")
    state = run.guard
    anchor = state.leg_stop_path_m
    request = state.leg_records[-1]["stop_request"].copy()
    assert anchor == request["path_origin_m"] == state.total_path_m
    assert "actuator receipt not implied" in request["interpretation"]
    state.temporal_fault("scan_stale", run.wall)
    for dx in (.004, -.004, .004):
        run.tick(dx=dx)
    assert state.leg_stop_path_m == anchor
    assert state.state == "STOP" and state.reason == "route_drive_settle_path_limit"
    assert state.leg_records[-1]["stop_request"]["path_increment_m"] > .01


def test_early_stop_cannot_count_zero_dwell_as_initial_slow_movement():
    run = Run(gain=3.).until(lambda r: r.guard.calibration_verified)
    resumed = run.wall
    assert not run.guard.initial_slow_validated
    run.until(lambda r: r.last.linear_x == .10)
    assert run.wall-resumed >= 1.-1e-9


@pytest.mark.parametrize("fault,reason", [("reverse", "approach_reverse_limit"),
    ("cross", "approach_cross_track_limit"), ("heading", "approach_heading_limit"),
    ("speed", "unexpected_motion"), ("local", "local_scan_motion_reserve_exhausted")])
def test_route_retains_latched_real_feedback_hard_stops(fault, reason):
    run = Run().until(lambda r: r.guard.phase == "DRIVE")
    state = run.guard
    wall = run.wall+.01
    x, y, yaw, speed = run.x, run.y, run.yaw, 0.
    if fault == "reverse":
        x -= .0051
    elif fault == "cross":
        y += .0201
    elif fault == "heading":
        yaw += .051
    elif fault == "speed":
        speed = .1001
    else:
        x += .0301
    refresh(state, wall, x=x, y=y, yaw=yaw, speed=speed, laser=False)
    assert state.state == "STOP" and state.reason == reason
    state.update_scan(scan(round(10.+wall, 9)), wall_s=wall)
    assert state.step(wall_s=wall).linear_x == 0.


def test_late_real_scan_pair_promotes_in_route_without_changing_original_times():
    run = Run().until(lambda r: r.guard.phase == "DRIVE")
    state = run.guard
    wall = run.wall+.05
    state.update_clock(round(10.+wall, 9), wall_s=wall)
    state.update_scan(scan(round(10.+wall+.002, 9)), wall_s=wall+.001)
    state.update_odom(posed_odom(round(10.+wall, 9)), wall_s=wall+.01)
    assert state.scan["stamp"] == pytest.approx(10.+wall+.002)
    assert state.scan["wall"] == pytest.approx(wall+.001)
    assert state.scan_anchor["odom"]["stamp_s"] == pytest.approx(10.+wall)
    assert state.sync_diagnostics()["late_prior_promotions"] == 1


def test_second_leg_keeps_frozen_geometry_and_cumulative_time_budgets():
    run = Run().until(lambda r: r.guard.leg_index == 1 and r.guard.phase == "TURN")
    state = run.guard
    assert state.total_path_m >= .59
    assert state.max_observed_displacement_m >= .59
    assert state.approach_limits.max_path_m == pytest.approx(2.70)
    assert state.approach_limits.max_origin_displacement_m == pytest.approx(2.0473665961)
    assert state.drive_elapsed_wall_s > 1.
    state.turn_elapsed_wall_s = 99.
    state.phase_started_wall = run.wall
    state.temporal_fault("scan_stale", run.wall)
    assert state.watchdog_reason(run.wall+1.) == "route_turn_total_wall_time_limit"
    assert state.watchdog_reason(300.) == "route_global_wall_time_limit"


def test_settle_drift_reaims_long_leg_without_resetting_turn_deadlines():
    run = Run().until(lambda r: r.guard.leg_index == 1 and r.guard.phase == "TURN_SETTLE")
    state = run.guard
    started, origin, elapsed_start = state.turn_started_wall, state.turn_origin, state.phase_started_wall
    run.last = None
    run.yaw = state.target_yaw+.018
    for _ in range(23):
        run.tick()
    assert state.phase == "TURN" and run.last.linear_x == 0.
    assert run.last.angular_z < 0.
    assert (state.turn_started_wall, state.turn_origin, state.phase_started_wall) == (started, origin, elapsed_start)
    run.until(lambda r: r.guard.state == "CAPTURE")
    assert math.dist((run.x, run.y), (.6, -1.8)) <= .02


def test_real_v20_terminal_turn_residual_keeps_original_angular_floor_and_deadline():
    run = Run().until(lambda r: r.guard.leg_index == 1 and r.guard.phase == "TURN")
    state = run.guard
    deadline = state.turn_started_wall
    origin = state.turn_origin
    # Real v20 last TURN yaw and target: residual -0.0630582744 rad.
    state.target_yaw = -1.5658901744126794
    run.yaw = -1.5028319
    run.last = None
    decision = run.tick()
    assert decision.angular_z == pytest.approx(-.35)
    assert decision.linear_x == 0.
    assert (state.turn_started_wall, state.turn_origin) == (deadline, origin)
    assert state._turn_tolerance() < .006
    assert state.watchdog_reason(deadline+45.) == "route_single_turn_wall_time_limit"


def test_unit_angular_gain_can_skip_narrow_aim_but_original_rotation_limit_stops_before_next_drive():
    run = Run(angular_gain=1.).until(lambda r: r.guard.leg_index == 1 and r.guard.phase == "TURN")
    start = len(run.commands)
    run.until(lambda r: r.guard.state == "STOP")
    assert run.guard.reason == "approach_rotation_limit"
    assert all(row[1].linear_x == 0. for row in run.commands[start:])
    assert max(abs(row[1].angular_z) for row in run.commands[start:]) <= .5
    assert run.commands[-1][1].angular_z == 0.


def test_capture_deadline_and_fourth_resume_cannot_be_hidden_by_fresh_feedback():
    run = Run("ground_4").until(lambda r: r.guard.phase == "DRIVE")
    state = run.guard
    origin = state.global_origin, state.drive_origin
    for resume in range(4):
        state.temporal_fault("scan_stale", run.wall)
        run.last = None
        for _ in range(23):
            run.tick()
        if resume < 3:
            assert state.resume_count == resume+1 and state.state == "DRIVE"
        else:
            assert state.state == "STOP" and state.reason == "pause_resume_limit"
        assert (state.global_origin, state.drive_origin) == origin
    other = Run("ground_4").until(lambda r: r.guard.state == "CAPTURE")
    assert other.guard.watchdog_reason(other.guard.capture_started_wall+10.) == "route_capture_wall_time_limit"


def test_final_turn_drift_cannot_move_observation_outside_waypoint_contract():
    run = Run("ground_4").until(lambda r: r.guard.leg_index == 1 and r.guard.phase == "TURN_SETTLE")
    run.last = None
    run.x -= .025
    for _ in range(23):
        run.tick()
    assert run.guard.state == "STOP" and run.guard.reason == "route_waypoint_error_limit"


def test_capture_keeps_heading_hard_stop_after_clearing_last_drive_origin():
    run = Run("ground_4").until(lambda r: r.guard.state == "CAPTURE")
    state = run.guard
    refresh(state, run.wall+.05, x=run.x, y=run.y, yaw=state.target_yaw+.051)
    assert state.state == "STOP" and state.reason == "approach_heading_limit"


def test_capture_cannot_redwell_outside_frozen_observation_position():
    run = Run("ground_4").until(lambda r: r.guard.state == "CAPTURE")
    refresh(run.guard, run.wall+.05, x=run.x-.025, y=run.y, yaw=run.yaw)
    assert run.guard.state == "STOP" and run.guard.reason == "route_waypoint_error_limit"


def test_last_point_one_meter_is_slow_and_actual_nonzero_start_is_preserved():
    run = Run("ground_4", (.02, -.01, .02)).until(lambda r: r.guard.state == "CAPTURE")
    assert run.guard.global_origin == (.02, -.01, .055)
    assert run.guard.route_plan.initial_pose == (.02, -.01, .02)
    for _, command, phase, x, y in run.commands:
        if phase == "DRIVE" and math.dist((x, y), (.35, 0.)) <= .10:
            assert command.linear_x <= .03


def test_tighter_explicit_linear_cap_applies_to_initial_and_final_slow_requests():
    run = Run("ground_4", route_limits=api().RouteLimits(linear_command_mps=.01)).until(
        lambda r: r.guard.state == "CAPTURE")
    assert max(row[1].linear_x for row in run.commands) == .01


@pytest.mark.parametrize("phase,expected", [("TURN", "route_single_turn_wall_time_limit"),
                                           ("DRIVE", "route_drive_total_wall_time_limit")])
def test_pause_does_not_reset_phase_time_or_cumulative_stage_budget(phase, expected):
    run = Run("ground_4").until(lambda r: r.guard.phase == phase)
    state = run.guard
    origins = state.global_origin, state.drive_origin, state.turn_origin
    state.temporal_fault("scan_stale", run.wall)
    if phase == "TURN":
        deadline = state.turn_started_wall+45.
    else:
        state.drive_elapsed_wall_s = 189.
        state.phase_started_wall = run.wall
        deadline = run.wall+1.
    assert state.watchdog_reason(deadline) == expected
    assert (state.global_origin, state.drive_origin, state.turn_origin) == origins


def test_final_capture_requires_three_distinct_valid_original_frames_and_emits_true_mode():
    from inspection_mission.telemetry import DiagnosticReporter
    run = Run("ground_4").until(lambda r: r.guard.state == "CAPTURE")
    state = run.guard
    for index in range(3):
        run.tick()
        payload = posed_odom(round(10.+run.wall, 9), x=run.x, yaw=run.yaw)
        payload["pose"]["pose"]["position"]["y"] = run.y
        payload["child_frame_id"] = "uav/base_link"
        state.observe_public("/uav/odom", payload, wall_s=run.wall)
        msg = image(10.+run.wall+.01)
        state.observe_image("/uav/camera/image", msg, wall_s=run.wall+.01)
        state.observe_image("/uav/camera/image", msg, wall_s=run.wall+.01)
        assert len(state.capture_frames) == index+1
    assert state.state == "STOP" and state.reason == "route_observation_complete"
    events = state.additional_events()
    assert all(payload["mode"] == "ground_observation_route" for _, payload in events)
    assert all(len(DiagnosticReporter._encode(event, payload)) < 65536 for event, payload in events)
    survey = state.survey_evidence()
    assert survey["approach"]["mode"] == "ground_observation_route"
    assert survey["series_capacities"] == {"trajectory_10hz": 3200, "actual_commands": 14000}
    assert len(survey["scan_records"]) == 4


def test_route_ros_adapter_only_publishes_ground_twist_and_retains_sync_pairing(monkeypatch):
    _, published, _ = install_ros_transport(monkeypatch)
    wall = [0.]
    node = api().create_route_node(wall_clock=lambda: wall[0])
    assert node.experiment_mode == "ground_observation_route"
    assert node.guard._observations.mode == "ground_observation_route"
    assert node.graph_refresh_diagnostics["interval_s"] == .5
    node.tick()
    assert published and {topic for topic, *_ in published} == {"/cmd_vel"}
    node.close_watchdog()


def test_route_adapter_uses_two_best_effort_odoms_and_records_real_callbacks_without_relaxing_age(monkeypatch):
    from inspection_mission.telemetry import DiagnosticReporter
    from test_ground_odom_qos import capture_transport
    subscriptions, publications = capture_transport(monkeypatch)
    node = api().create_route_node(wall_clock=lambda: 0.)
    try:
        node.report_started(300.)
        assert subscriptions["/odom"] == subscriptions["/uav/odom"] == (1, "BEST_EFFORT", "VOLATILE")
        assert subscriptions["/scan"] == (1, "RELIABLE", "VOLATILE")
        assert subscriptions["/uav/camera/image"] == (1, "RELIABLE", "VOLATILE")
        assert subscriptions["/referee/ground_tasks"] == (1, "RELIABLE", "TRANSIENT_LOCAL")
        assert set(publications.values()) == {(1, "RELIABLE", "VOLATILE")}
        for stamp in (10., 10.02):
            node.receive("/odom", posed_odom(stamp))
            uav = posed_odom(stamp)
            uav["child_frame_id"] = "uav/base_link"
            node.receive("/uav/odom", uav)
        snap = node.evidence_snapshot()
        assert snap["startup"]["revision"] == "ground_observation_route_v2"
        timing = snap["callback_timing"]
        assert timing["revision"] == "ground_observation_route_v2"
        for topic in ("/odom", "/uav/odom"):
            assert timing["input_qos"][topic]["requested"]["reliability"] == "BEST_EFFORT"
            assert timing["spans"]["receive:"+topic]["wall_s"]["count"] == 2
        assert node.guard.limits.sensor_age_s == .20
        assert node.guard.limits.clock_sampling_lag_s == .12
        assert len(DiagnosticReporter._encode("probe_snapshot", snap)) < 65536
    finally:
        node.close_watchdog()


@pytest.mark.parametrize("changes,want", [
    ({}, {"global_wall_s": 300., "drive_wall_s": 190., "turn_wall_s": 45.,
          "turn_total_wall_s": 100., "capture_wall_s": 10., "wait_wall_s": 5.}),
    ({"global_wall_s": 270., "drive_wall_s": 180., "turn_wall_s": 40.,
      "turn_total_wall_s": 90., "capture_wall_s": 8., "wait_wall_s": 4.},
     {"global_wall_s": 270., "drive_wall_s": 180., "turn_wall_s": 40.,
      "turn_total_wall_s": 90., "capture_wall_s": 8., "wait_wall_s": 4.}),
])
def test_executed_route_plan_reports_actual_default_or_tighter_time_budgets(changes, want):
    run = Run(route_limits=api().RouteLimits(**changes)).until(lambda r: r.guard.phase == "DRIVE")
    evidence = run.guard.approach_evidence()
    budgets = evidence["route"]["plan"]["budgets"]
    for key, value in want.items():
        assert budgets[key] == value == evidence["limits"][key]
    assert run.guard.watchdog_reason(want["global_wall_s"]) == "route_global_wall_time_limit"
    run.guard.drive_elapsed_wall_s = want["drive_wall_s"]-1.
    run.guard.phase_started_wall = run.wall
    assert run.guard.watchdog_reason(run.wall+1.) == "route_drive_total_wall_time_limit"


@pytest.mark.parametrize("child_exit,setup_exit", [(0, 0), (23, 0), (0, 31)])
def test_route_launcher_uses_uploaded_source_and_preserves_exit(tmp_path, child_exit, setup_exit):
    import json
    import os
    from pathlib import Path
    import shlex
    import shutil
    import subprocess
    import sys
    from test_ground_probe_launch import BASH, shell_path
    if not BASH.is_file():
        pytest.skip("bash unavailable")
    workspace = tmp_path/"uploaded route workspace"
    package = workspace/"src/inspection_mission/inspection_mission"
    package.mkdir(parents=True)
    (package/"__init__.py").write_text("", encoding="utf-8")
    (package/"ground_observation_route.py").write_text(
        "import json,os,sys\nassert os.environ['FAKE_ROS_READY']=='1'\n"
        "print('ROUTE_ARGS '+json.dumps(sys.argv[1:]))\n"
        f"sys.exit({child_exit})\n", encoding="utf-8")
    source = Path(__file__).resolve().parents[1]/"scripts/run_ground_observation_route.sh"
    assert source.is_file(), "independent route launcher is missing"
    launcher = workspace/"scripts/run_ground_observation_route.sh"
    launcher.parent.mkdir()
    shutil.copyfile(source, launcher)
    setup = tmp_path/"fake ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
        +(f"return {setup_exit}\n" if setup_exit else "export FAKE_ROS_READY=1\n"), encoding="utf-8")
    env = dict(os.environ, INSPECTION_ROS_SETUP=shell_path(setup))
    env.pop("INSPECTION_DIAGNOSTIC_URL", None)
    result = subprocess.run([str(BASH), str(launcher)], cwd=tmp_path, env=env,
        capture_output=True, text=True, encoding="utf-8", timeout=10)
    assert result.returncode == (setup_exit or child_exit), result.stdout+result.stderr
    assert f"GROUND_OBSERVATION_ROUTE launch_exit exit_code={setup_exit or child_exit}" in result.stderr
    if not setup_exit:
        line = next(line for line in result.stdout.splitlines() if line.startswith("ROUTE_ARGS "))
        assert json.loads(line.removeprefix("ROUTE_ARGS ")) == ["--duration", "300"]
