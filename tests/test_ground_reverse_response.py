"""Finite negative-X requests; synthetic feedback is not platform validation."""
from dataclasses import asdict
import importlib
import importlib.util
import math

import pytest

from test_bounded_ground_probe import endpoint, install_ros_transport, scan
from test_ground_direction_response import Run
from test_ground_observation_survey import posed_odom


def api():
    name = "inspection_mission.ground_reverse_response"
    assert importlib.util.find_spec(name), "independent reverse-response runtime is missing"
    return importlib.import_module(name)


def fixture():
    run = Run()
    run.guard = api().GroundReverseResponseGuard(started_wall_s=0., session_id="reverse-test")
    run.guard.update_endpoints([endpoint()], matched_subscriptions=1)
    return run


def first_pulse():
    return fixture().until(lambda r: r.last.linear_x < 0.)


def test_reverse_rotates_positive_before_all_three_negative_requests():
    run = fixture(); seen = []
    for _ in range(2400):
        decision = run.tick()
        if decision.angular_z:
            assert decision.angular_z > 0.
        if decision.linear_x:
            seen.append((run.guard.pulse_index, decision.linear_x, decision.linear_y))
        if run.guard.state == "STOP":
            break
    assert run.guard.reason == "direction_response_observed_official_score_unverified"
    assert set(seen) == {(0, -.03, 0.), (1, -.10, 0.), (2, -.10, 0.)}
    assert len(run.guard.pulse_records) == 3
    for record in run.guard.pulse_records:
        assert abs(record["start"]["yaw_rad"] - math.pi/2) <= .025
        assert record["settled"]["stable_wall_s"] >= 1.-1e-9
        assert record["settled"]["stable_source_s"] >= 1.-1e-9
    assert run.guard.accumulated_rotation_rad > 1.
    assert run.guard.accumulated_rotation_rad < 2.
    assert run.guard.pulse_records[1]["start"]["raw_odom"] != run.guard.pulse_records[2]["start"]["raw_odom"]


@pytest.mark.parametrize("field,value",[("global_wall_s",120.001),("max_path_m",.180001),
    ("max_origin_displacement_m",.120001),("max_rotation_rad",2.00001),
    ("pulse_wall_s",2.00001),("pulse_displacement_m",.040001),
    ("pulse_request_stop_m",.010001),("max_translation_speed_mps",.150001),
    ("max_translation_yaw_speed_rps",.150001),("max_pulse_yaw_drift_rad",.030001),
    ("stable_drift_m",.002001),("stop_drift_m",.010001),
    ("maximum_yaw_rate",.500001),("turn_wall_s",45.001),("stable_s",.99),
    ("max_turn_drift_m",.060001),("scan_anchor_motion_limit_m",.030001),
    ("local_swept_motion_budget_m",.060001),("minimum_yaw_rate",.350001),
    ("max_heading_error_rad",.050001),("local_stop_reserve_m",.030001)])
def test_only_rotation_budget_can_expand_from_frozen_direction_contract(field, value):
    with pytest.raises(ValueError):
        api().ReverseLimits(**{field: value})


def test_default_reverse_limits_change_only_the_rotation_budget():
    from inspection_mission.ground_direction_response import DirectionLimits
    reverse, original = asdict(api().ReverseLimits()), asdict(DirectionLimits())
    assert {key for key in original if reverse[key] != original[key]} == {"max_rotation_rad"}
    assert reverse["max_rotation_rad"] == 2.


def test_replacement_guard_keeps_original_creation_wall_origin(monkeypatch):
    install_ros_transport(monkeypatch)
    calls = []
    def wall_clock():
        result = 10.+.05*len(calls)
        calls.append(result)
        return result
    node = api().create_reverse_response_node(wall_clock=wall_clock)
    try:
        assert node.guard.started_wall == 10.
        assert len(calls) > 1
    finally:
        node.close_watchdog()


def test_reverse_measured_request_stop_and_zero_drift_budget_still_latch():
    run = first_pulse()
    run.tick(dx=.0101)  # allow the simultaneous negative-X body motion in this fixture
    assert run.guard.phase == "PULSE_SETTLE"
    assert run.last.linear_x == run.last.linear_y == 0.
    run.tick(dx=.011)
    assert run.guard.state == "STOP"
    assert run.guard.reason == "direction_zero_stop_path_limit"


@pytest.mark.parametrize("kind",["rotation", "turn_time", "global_time", "pulse_displacement"])
def test_reverse_cannot_escape_permanent_motion_budgets(kind):
    run = first_pulse() if kind == "pulse_displacement" else fixture().until(lambda r: r.last.angular_z > 0.)
    if kind == "rotation":
        run.guard.accumulated_rotation_rad = 2.
        run.tick()
    elif kind == "turn_time":
        run.wall = run.guard.turn_started_wall + 45.
        run.guard.step(wall_s=run.wall)
    elif kind == "global_time":
        run.wall = 120.
        run.guard.step(wall_s=run.wall)
    else:
        run.tick(dx=.041)
    assert run.guard.state == "STOP"
    for _ in range(2):
        run.tick()
        assert run.last.linear_x == run.last.linear_y == run.last.angular_z == 0.


def test_actual_subscriptions_and_tick_use_replacement_guard_for_reverse(monkeypatch):
    callbacks, _, _ = install_ros_transport(monkeypatch)
    wall = [0.]
    node = api().create_reverse_response_node(wall_clock=lambda: wall[0])
    node.arm_watchdog = lambda: None  # deterministic callback loop; watchdog has a separate test
    sent, other = [], []
    node.publisher_by_topic["/cmd_vel"].publish = lambda m: sent.append((m.linear.x, m.linear.y, m.angular.z))
    for topic, publisher in node.publisher_by_topic.items():
        if topic != "/cmd_vel":
            publisher.publish = lambda m, t=topic: other.append(t)
    x = y = yaw = 0.
    try:
        for i in range(1, 2401):
            vx, vy, wz = sent[-1] if sent else (0., 0., 0.)
            vx *= .2; vy *= .2; wz *= .15
            x += .05*(vx*math.cos(yaw)-vy*math.sin(yaw))
            y += .05*(vx*math.sin(yaw)+vy*math.cos(yaw)); yaw += .05*wz
            wall[0] = round(.05*i, 9); source = round(10.+wall[0], 9)
            sec = int(source)
            callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((source-sec)*1e9)}})
            msg = posed_odom(source, x=x, yaw=yaw)
            msg["pose"]["pose"]["position"]["y"] = y
            msg["twist"]["twist"]["linear"]["x"] = abs(vx)
            msg["twist"]["twist"]["angular"]["z"] = abs(wz)
            callbacks["/odom"](msg); callbacks["/scan"](scan(source))
            node.tick()
            if node.guard.state == "STOP":
                break
        assert isinstance(node.guard, api().GroundReverseResponseGuard)
        assert node.experiment_mode == "ground_reverse_response"
        assert node.guard.reason == "direction_response_observed_official_score_unverified"
        assert all(x <= 0. and y == 0. for x, y, _ in sent)
        assert any(w > 0. for _, _, w in sent) and not any(w < 0. for _, _, w in sent)
        assert not other
        assert {r["pulse_index"] for r in node.guard.direction_commands if r["linear_x"] < 0.} == {0, 1, 2}
        assert sent == [(r["linear_x"], r["linear_y"], r["angular_z"])
                        for r in node.guard.direction_commands if r["sent"]]
        core = node.guard.core_evidence()
        assert core["reverse_contract"]["target_yaw_rad"] == math.pi/2
        assert core["reverse_contract"]["actual_reverse_motion_confirmed"] is False
    finally:
        node.close_watchdog()


def test_negative_publish_failure_still_attempts_zero_with_silent_other_outputs(monkeypatch):
    install_ros_transport(monkeypatch)
    node = api().create_reverse_response_node(wall_clock=lambda: 0.)
    attempts = []
    def publish(msg):
        attempts.append((msg.linear.x, msg.linear.y, msg.angular.z))
        if msg.linear.x < 0.:
            raise RuntimeError("reverse transport failure")
    node.publisher_by_topic["/cmd_vel"].publish = publish
    try:
        from inspection_mission.ground_direction_response import DirectionDecision
        node.publish_direction(DirectionDecision(-.03,0.,0.,"PULSE","test"))
        assert attempts[:2] == [(-.03,0.,0.),(0.,0.,0.)]
        assert node.guard.state == "STOP"
        node.tick()
        assert attempts[-1] == (0.,0.,0.)
    finally:
        node.close_watchdog()


def test_reverse_dds_loss_immediately_before_output_cannot_publish_negative(monkeypatch):
    install_ros_transport(monkeypatch)
    run = first_pulse(); wall = [run.wall]
    node = api().create_reverse_response_node(wall_clock=lambda: wall[0]); node.guard = run.guard
    node.last_graph_refresh_wall = wall[0]
    matches = iter((1,0)); sent = []
    node.publisher_by_topic["/cmd_vel"].get_subscription_count = lambda: next(matches)
    node.publisher_by_topic["/cmd_vel"].publish = lambda m: sent.append((m.linear.x,m.linear.y,m.angular.z))
    try:
        decision = node.tick()
        assert sent == [(0.,0.,0.)]
        assert decision.linear_x == 0. and node.guard.state == "STOP"
    finally:
        node.close_watchdog()


def test_reverse_scan_pair_never_uses_future_source_pose():
    g = api().GroundReverseResponseGuard(started_wall_s=0.)
    g.update_endpoints([endpoint()],matched_subscriptions=1)
    g.update_clock(10.01,wall_s=0.)
    g.update_odom(posed_odom(10.001),wall_s=0.)
    g.update_scan(scan(10.),wall_s=0.)
    assert g.scan_anchor is None and g.step(wall_s=0.).linear_x == 0.
