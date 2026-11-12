"""Bounded v19 tier/stop measurements use the real guard and raw feedback."""
import importlib
import importlib.util
import json
import math

import pytest

from test_bounded_ground_probe import endpoint
from test_ground_bounded_approach import image, refresh
from test_ground_observation_survey import posed_odom
from test_ground_odom_qos import capture_transport


def api():
    name = "inspection_mission.ground_speed_calibration"
    assert importlib.util.find_spec(name) is not None, "independent v19 speed calibration missing"
    return importlib.import_module(name)


class Run:
    def __init__(self):
        self.guard = api().GroundSpeedCalibrationGuard(started_wall_s=0.)
        self.guard.update_endpoints([endpoint()], matched_subscriptions=1)
        self.guard.observe_public("/referee/ground_tasks", {"data": "ground_0,ground_1,ground_4"}, wall_s=0.)
        self.wall = self.x = self.y = self.yaw = 0.
        self.last = None
        self.gain = 1.
        self.commands = []

    def tick(self, *, dx=0., publish=True):
        dt = .05
        speed = 0. if self.last is None else self.last.linear_x*self.gain
        angular = 0. if self.last is None else self.last.angular_z
        self.x += dt*speed*math.cos(self.yaw)+dx
        self.y += dt*speed*math.sin(self.yaw)
        self.yaw += dt*angular
        self.wall = round(self.wall+dt, 9)
        refresh(self.guard, self.wall, x=self.x, y=self.y, yaw=self.yaw, speed=speed, yaw_speed=abs(angular))
        decision = self.guard.step(wall_s=self.wall)
        if publish:
            self.last = decision
            self.guard.record_command(wall_s=self.wall, linear_x=decision.linear_x, angular_z=decision.angular_z)
        self.commands.append((self.wall, decision, self.guard.phase))
        return decision

    def until(self, predicate, cap=2200):
        for _ in range(cap):
            self.tick()
            if predicate(self): return self
            assert self.guard.state not in ("STOP", "FINISHED"), self.guard.reason
        pytest.fail("calibration expected phase not reached")


def test_complete_sequence_separates_both_stops_and_finishes_twenty_cm_then_three_real_frames():
    run = Run().until(lambda r: r.guard.phase == "CAPTURE")
    state = run.guard
    phases = [row[2] for row in run.commands]
    assert phases.index("LOW_STOP") < phases.index("HIGH_PULSE") < phases.index("HIGH_STOP")
    fast = [row for row in run.commands if row[1].linear_x == .10]
    assert fast and all(row[2] == "HIGH_PULSE" for row in fast)
    first_stop = next(i for i, row in enumerate(run.commands) if row[2] == "LOW_STOP")
    assert all(row[1].linear_x in (0., .03) for row in run.commands[:first_stop])
    assert state.measured_forward() >= .20 and state.total_path_m < .30
    evidence = state.approach_evidence()["speed_calibration"]
    assert evidence["low_continuous_wall_s"] >= 1. and evidence["low_continuous_sim_s"] >= 1.
    assert len(evidence["stops"]) == 2 and all(row["verified"] for row in evidence["stops"])
    assert all(row["path_increment_m"] <= .01 for row in evidence["stops"])
    assert evidence["high_total_path_m"] <= .05
    for stage in ("low", "high", "final_slow"):
        tier = evidence["tiers"][stage]
        assert tier["sample_count"] > 0 and tier["max_speed_mps"] > 0
        assert tier["last"]["stamp_s"] >= tier["first"]["stamp_s"]
    for _ in range(3):
        run.tick()
        payload = posed_odom(round(10.+run.wall, 9), x=run.x, yaw=run.yaw)
        payload["pose"]["pose"]["position"]["y"] = run.y
        payload["child_frame_id"] = "uav/base_link"
        state.observe_public("/uav/odom", payload, wall_s=run.wall)
        state.observe_image("/uav/camera/image", image(10.+run.wall+.01), wall_s=run.wall+.01)
        run.wall += .01
    assert state.reason == "approach_observation_complete" and len(state.capture_frames) == 3


def test_slow_tier_cannot_borrow_zero_dwell_or_promote_before_one_second():
    run = Run().until(lambda r: r.guard.phase == "DRIVE")
    run.gain = 3.
    run.until(lambda r: r.guard.state == "STOP")
    assert run.guard.reason == "calibration_low_continuous_time_insufficient"
    assert all(row[1].linear_x <= .03 for row in run.commands)


@pytest.mark.parametrize("stop_phase", ["LOW_STOP", "HIGH_STOP"])
def test_each_stop_counts_cumulative_motion_and_latches_before_next_safe_scan(stop_phase):
    run = Run().until(lambda r: r.guard.phase == stop_phase)
    for _ in range(3):
        run.tick(dx=.004)
    assert run.guard.state == "STOP" and "stop_path_limit" in run.guard.reason
    reason = run.guard.reason
    run.tick()
    assert run.guard.reason == reason and run.commands[-1][1].linear_x == 0.


def test_phase_change_is_not_first_successful_zero_publish_and_pre_zero_motion_counts():
    run = Run().until(lambda r: r.guard.phase == "DRIVE" and r.guard.measured_forward() > .045)
    while run.guard.phase == "DRIVE": run.tick(publish=False)
    state = run.guard
    before = state.approach_evidence()["speed_calibration"]["stops"][0]
    assert before["first_successful_zero_publish"] is None
    run.tick(dx=.005)
    after = state.approach_evidence()["speed_calibration"]["stops"][0]
    assert after["first_successful_zero_publish"]["wall_elapsed_s"] > before["phase_switch"]["wall_elapsed_s"]
    assert after["path_increment_m"] >= .005
    assert after["first_successful_zero_publish"]["path_increment_m"] >= .005


@pytest.mark.parametrize("clock_kind", ["wall", "sim"])
def test_high_pulse_has_five_second_deadline_in_each_clock_and_pause_cannot_reset_it(clock_kind):
    run = Run().until(lambda r: r.guard.phase == "HIGH_PULSE")
    run.gain = 0.
    state = run.guard
    initial = state.approach_evidence()["speed_calibration"]["high_request_start"].copy()
    state.temporal_fault("scan_stale", run.wall)
    for _ in range(25): run.tick()
    assert state.phase == "HIGH_PULSE" and state.state == "HIGH_PULSE"
    assert state.approach_evidence()["speed_calibration"]["high_request_start"] == initial
    if clock_kind == "wall":
        state.step(wall_s=initial["wall_elapsed_s"]+5.)
    else:
        wall = run.wall+.05
        state.update_clock(initial["stamp_s"]+5., wall_s=wall)
        payload = posed_odom(initial["stamp_s"]+5., x=run.x, yaw=run.yaw)
        payload["pose"]["pose"]["position"]["y"] = run.y
        state.update_odom(payload, wall_s=wall)
    assert state.state == "STOP" and state.reason == "calibration_high_"+clock_kind+"_time_limit"


def test_high_pulse_stops_at_point_zero_four_and_keeps_point_zero_five_total_envelope():
    run = Run().until(lambda r: r.guard.phase == "HIGH_STOP")
    evidence = run.guard.approach_evidence()["speed_calibration"]
    assert .04 <= evidence["high_total_path_m"] <= .05
    assert run.commands[-1][1].linear_x == 0.
    run.tick(dx=.012)
    assert run.guard.state == "STOP" and run.guard.reason in (
        "calibration_high_total_path_limit", "calibration_high_stop_path_limit")


def test_real_high_tier_speed_above_original_hard_limit_latches_stop():
    run = Run().until(lambda r: r.guard.phase == "HIGH_PULSE")
    run.gain = 1.001
    run.tick()
    assert run.guard.state == "STOP" and run.guard.reason == "unexpected_motion"
    assert run.guard.approach_evidence()["speed_calibration"]["tiers"]["high"]["max_speed_mps"] > .10


def test_calibration_does_not_reset_original_origins_drive_deadline_or_resume_limit():
    run = Run().until(lambda r: r.guard.phase == "DRIVE")
    state = run.guard
    permanent = (state.global_origin, state.drive_origin, state.drive_started_wall)
    run.until(lambda r: r.guard.phase == "HIGH_STOP")
    assert (state.global_origin, state.drive_origin, state.drive_started_wall) == permanent
    assert state.watchdog_reason(state.drive_started_wall+55.) == "approach_drive_wall_time_limit"
    assert state.watchdog_reason(90.) == "approach_global_wall_time_limit"
    assert state.approach_evidence()["max_resumes"] == 3
    assert state.approach_limits.max_path_m == state.approach_limits.max_origin_displacement_m == .30


def test_factory_keeps_v18_odom_qos_v17_timing_and_bounded_final_metadata(monkeypatch):
    from inspection_mission.telemetry import DiagnosticReporter
    subscriptions, _ = capture_transport(monkeypatch)
    node = api().create_speed_calibration_node(wall_clock=lambda: 0.)
    try:
        node.report_started(90.)
        assert subscriptions["/odom"] == (1, "BEST_EFFORT", "VOLATILE")
        assert subscriptions["/scan"] == (1, "RELIABLE", "VOLATILE")
        snap = node.evidence_snapshot()
        assert snap["mode"] == "ground_bounded_approach"
        assert snap["startup"]["revision"] == "v19_speed_calibration"
        assert snap["callback_timing"]["revision"] == "v19_speed_calibration"
        assert len(DiagnosticReporter._encode("probe_snapshot", snap)) < 65536
    finally:
        node.close_watchdog()


@pytest.mark.parametrize("changes", [{"linear_command_mps": .01}, {"target_distance_m": .15}])
def test_fixed_tier_experiment_cannot_silently_override_a_tighter_explicit_contract(changes):
    from inspection_mission.ground_bounded_approach import ApproachLimits
    with pytest.raises(ValueError, match="fixed calibration contract"):
        api().GroundSpeedCalibrationGuard(started_wall_s=0., approach_limits=ApproachLimits(**changes))


def test_failed_ros_zero_publish_does_not_create_successful_publish_evidence(monkeypatch):
    capture_transport(monkeypatch)
    run = Run().until(lambda r: r.guard.phase == "DRIVE" and r.guard.measured_forward() > .045)
    while run.guard.phase == "DRIVE": run.tick(publish=False)
    node = api().create_speed_calibration_node(wall_clock=lambda: run.wall)
    node.guard = run.guard
    try:
        def fail_publish(message): raise RuntimeError("transport publish failed")
        node.publisher_by_topic["/cmd_vel"].publish = fail_publish
        with pytest.raises(RuntimeError, match="transport publish failed"):
            node.tick()
        assert node.guard.approach_evidence()["speed_calibration"]["stops"][0]["first_successful_zero_publish"] is None
    finally:
        node.close_watchdog()


def test_complete_calibration_metadata_survives_final_report_with_bounded_payloads(monkeypatch):
    from inspection_mission.telemetry import DiagnosticReporter
    capture_transport(monkeypatch)
    run = Run().until(lambda r: r.guard.phase == "CAPTURE")
    captured = []

    class Reporter:
        def emit(self, event, payload): return True
        def send_now(self, event, payload, timeout_s):
            encoded = DiagnosticReporter._encode(event, payload)
            assert len(encoded) < 65536
            captured.append(json.loads(encoded))

    node = api().create_speed_calibration_node(Reporter(), wall_clock=lambda: run.wall)
    node.guard = run.guard
    try:
        node.report_final()
        metadata = next(row["data"]["survey"] for row in captured if row["data"].get("section") == "survey_metadata")
        assert metadata["approach"]["speed_calibration"]["stops"][1]["verified"] is True
        assert metadata["approach"]["speed_calibration"]["tiers"]["high"]["max_speed_mps"] == .10
    finally:
        node.close_watchdog()


def test_proven_continuous_low_duration_does_not_grow_by_borrowing_a_later_pause():
    run = Run().until(lambda r: r.guard.low_duration_verified)
    state = run.guard
    before = (state.low_continuous_wall_s, state.low_continuous_sim_s)
    run.gain = 0.
    state.temporal_fault("scan_stale", run.wall)
    for _ in range(25): run.tick()
    assert state.state == state.phase == "DRIVE"
    assert (state.low_continuous_wall_s, state.low_continuous_sim_s) == before
