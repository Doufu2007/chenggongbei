"""Continuous ground task contracts using synthetic feedback, not platform proof."""
from copy import deepcopy
import importlib
import importlib.util
import math

import pytest

from test_bounded_ground_probe import endpoint
from test_ground_observation_route import Run
from test_ground_bounded_approach import image
from test_ground_observation_survey import posed_odom
from test_ground_single_point import camera_info, fixture_jobs


def api():
    name = "inspection_mission.ground_three_point"
    assert importlib.util.find_spec(name), "continuous three-ground-point runtime missing"
    return importlib.import_module(name)


class ThreeRun(Run):
    def __init__(self, draw=("ground_0", "ground_3", "ground_2"), **limits):
        super().__init__(draw[0])
        self.guard = api().GroundThreePointGuard(started_wall_s=0., session_id="test-current-session",
            route_limits=api().ThreePointLimits(**limits))
        self.guard.update_endpoints([endpoint()], matched_subscriptions=1)
        self.guard.observe_public("/referee/ground_tasks", {"data": ",".join(draw)}, wall_s=0.)
        self.guard.observe_public("/uav/camera/camera_info", camera_info(), wall_s=0.)

    def tick(self, *, emit_image=True, **kwargs):
        decision = super().tick(**kwargs)
        stamp = round(10.+self.wall, 9)
        uav = posed_odom(stamp, x=self.x, yaw=self.yaw)
        uav["child_frame_id"] = "uav/base_link"
        uav["pose"]["pose"]["position"]["y"] = self.y
        self.guard.observe_public("/uav/odom", uav, wall_s=self.wall)
        if emit_image:
            self.guard.observe_image("/uav/camera/image", image(stamp), wall_s=self.wall)
        return decision

    def recognise(self):
        self.until(lambda r: r.guard.state == "RECOGNIZE_WAIT", cap=25000)
        _, results = fixture_jobs(self)
        for result in results:
            assert self.guard.accept_match_result(result, wall_s=self.wall)
        sent = []
        event = self.guard.dispatch_one(lambda *args: sent.append(args), wall_s=self.wall)
        assert event["sent"] and len(sent) == 1
        return sent

    def declare(self):
        self.until(lambda r: r.guard.state == "DECLARE_WAIT", cap=25000)
        self.until(lambda r: r.guard._stable_dwell(r.wall), cap=500)
        sent = []
        event = self.guard.dispatch_one(lambda *args: sent.append(args), wall_s=self.wall)
        assert event["sent"] and len(sent) == 1
        return sent


def test_new_limits_leave_frozen_single_route_caps_unchanged():
    from inspection_mission.ground_observation_route import RouteLimits
    assert RouteLimits().max_path_m == 3.
    limits = api().ThreePointLimits()
    assert limits.max_path_m == 12.5 and limits.max_rotation_rad == 25.
    with pytest.raises(ValueError):
        api().ThreePointLimits(max_path_m=12.5001)


def test_draw_is_immutable_and_mission_origin_is_initialised_only_once():
    run = ThreeRun().until(lambda r: r.last.linear_x != 0. or r.last.angular_z != 0.)
    assert run.guard.mission_draw == ("ground_0", "ground_3", "ground_2")
    assert run.guard.global_origin == (0., 0., .055)
    run.guard.observe_public("/referee/ground_tasks", {"data": "ground_1,ground_2,ground_3"}, wall_s=run.wall)
    assert run.guard.state == "STOP" and run.guard.reason == "public_draw_changed"
    assert run.guard.mission_draw == ("ground_0", "ground_3", "ground_2")


def test_completed_point_handoff_preserves_permanent_counters_and_evidence():
    run = ThreeRun()
    assert run.recognise()[0][0] == "/inspection/detections"
    assert run.declare() == [("/referee/current_target", "ground_0")]
    before = (run.guard.global_origin, run.guard.started_wall, run.guard.total_path_m,
              run.guard.accumulated_rotation_rad, run.guard._elapsed_group("turn", run.wall),
              run.guard._elapsed_group("drive", run.wall), run.guard.stationary_epoch)
    old_result = deepcopy(run.guard.point_visual["match_results"][0])
    run.until(lambda r: r.guard.point_index == 1)
    state = run.guard
    assert state.state not in ("STOP", "FINISHED")
    assert state.global_origin == before[0] and state.started_wall == before[1]
    assert state.total_path_m >= before[2] and state.accumulated_rotation_rad >= before[3]
    assert state._elapsed_group("turn", run.wall) >= before[4]
    assert state._elapsed_group("drive", run.wall) >= before[5]
    assert state.stationary_epoch > before[6]
    assert state.target_id == "ground_3" and state.recognition is None and state.attempts == {}
    assert not state.capture_frames and not state.expected_jobs and not state.match_results
    assert not state.accept_match_result({"job": old_result["job"], "matches_current_context": True}, wall_s=run.wall)
    first = state.completed_points[0]
    assert first["target_id"] == "ground_0" and first["attempts"]["declaration"]["sent"]
    assert len(first["match_results"]) == 3 and len(state.archived_capture_frames) == 3


@pytest.mark.parametrize("draw", [("ground_0", "ground_3", "ground_2"),
                                 ("ground_2", "ground_0", "ground_4")])
def test_three_points_have_individual_images_single_sends_and_real_final_headings(draw):
    run = ThreeRun(draw)
    sent = []
    epochs = []
    for index, target in enumerate(draw):
        sent += run.recognise()
        epochs.append(run.guard.stationary_epoch)
        sent += run.declare()
        expected = {"ground_0": 0., "ground_1": math.pi, "ground_2": math.pi,
                    "ground_3": -math.pi/2, "ground_4": -math.pi/2}[target]
        assert abs(math.atan2(math.sin(run.yaw-expected), math.cos(run.yaw-expected))) <= .025
        if index < 2:
            run.until(lambda r: r.guard.point_index == index+1)
    run.until(lambda r: r.guard.state == "STOP")
    assert run.guard.reason == "three_ground_transport_complete_official_score_unverified"
    assert [item for item in sent if item[0] == "/referee/current_target"] == [
        ("/referee/current_target", target) for target in draw]
    assert epochs[0] < epochs[1] < epochs[2]
    assert len(run.guard.completed_points) == 3
    assert run.guard.total_path_m <= 12.5 and run.guard.accumulated_rotation_rad <= 25.
    assert run.guard.approach_evidence()["three_point"]["full_mission_complete"] is False
    if "ground_2" in draw:
        from inspection_mission.ground_three_point_plan import _turn_clearance
        point = next(item for item in run.guard.completed_points if item["target_id"] == "ground_2")
        assert _turn_clearance(point["pose"][:2], point["pose"][2], 0.) >= .641


def test_hard_stop_is_absorbing_and_never_advances_point():
    run = ThreeRun().until(lambda r: r.last.linear_x != 0. or r.last.angular_z != 0.)
    run.guard.stop("synthetic_safety_failure", run.wall)
    for _ in range(10):
        run.tick()
    assert run.guard.point_index == 0 and run.guard.state in ("STOP", "FINISHED")
    assert run.guard.reason == "synthetic_safety_failure"


def test_point_transition_cannot_erase_or_expand_exhausted_path_budget():
    run = ThreeRun()
    run.recognise()
    run.declare()
    run.guard.total_path_m = 12.49
    run.guard._handoff(run.wall)
    assert run.guard.reason == "three_ground_remaining_path_budget_insufficient"
    assert run.guard.point_index == 0 and run.guard.total_path_m >= 12.49
    assert run.guard.approach_limits.max_path_m == 12.5
    assert len(run.guard.completed_points) == 1
    assert run.guard.completed_points[0]["attempts"]["declaration"]["sent"]


def test_entry_requires_current_successful_visual_transport_epoch():
    run = ThreeRun().until(lambda r: r.last.linear_x != 0. or r.last.angular_z != 0.)
    run.guard._begin_entry(run.wall)
    assert run.guard.state == "STOP" and run.guard.reason == "three_ground_entry_visual_gate_missing"
    assert not run.guard.in_entry


def test_declaration_rechecks_final_heading_after_last_turn():
    run = ThreeRun()
    run.recognise()
    run.until(lambda r: r.guard.phase == "DECLARE_WAIT")
    run.yaw = .04
    run.tick()
    assert run.guard.state == "STOP" and run.guard.reason == "three_ground_final_pose_outside_contract"
    assert run.guard.dispatch_one(lambda *_: pytest.fail("must not publish"), wall_s=run.wall) is None


def test_total_time_and_group_time_are_not_point_local_deadlines():
    run = ThreeRun()
    run.recognise()
    run.declare()
    run.until(lambda r: r.guard.point_index == 1)
    assert run.guard._budget_fault(1200.) == "three_ground_global_wall_time_limit"
    run.guard.turn_elapsed_wall_s = 700.
    # Every active turn is checked against the same accumulated group clock.
    run.guard.phase = "TURN"
    assert run.guard._budget_fault(run.wall) == "route_turn_total_wall_time_limit"


def test_declaration_pause_needs_new_dual_clock_stability_before_handoff():
    run = ThreeRun()
    run.recognise()
    run.declare()
    event = run.guard.attempts["declaration"]
    # A prior send does not excuse a later sensor freshness pause.
    event["wall_s"] = run.wall-3.
    run.guard.temporal_fault("scan_stale", run.wall)
    assert run.guard.state == "PAUSE"
    for _ in range(10):
        run.tick()
    assert run.guard.point_index == 0 and run.guard.state == "PAUSE"
    run.until(lambda r: r.guard.point_index == 1)
    assert run.guard.resume_count == 1


def test_same_coordinator_cancels_motion_context_and_keeps_job_sequence_monotone():
    from inspection_mission.ground_single_point import GroundSinglePointCoordinator
    run = ThreeRun()
    run.until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    calls = []
    class Worker:
        def cancel_context(self): calls.append(("cancel",))
        def set_context(self, *args): calls.append(("context", *args))
        def poll(self): return []
    def prepare(frames, **kwargs):
        calls.append(("prepare", kwargs["first_seq"]))
        return []
    coordinator = GroundSinglePointCoordinator(run.guard, Worker(), prepare=prepare)
    coordinator.poll_once(run.wall)
    coordinator.preparer.join(timeout=1.)
    assert coordinator.next_seq == 3
    run.recognise()
    run.until(lambda r: r.guard.in_entry)
    coordinator.poll_once(run.wall)
    assert coordinator.context is None and calls[-1] == ("cancel",)
    run.declare()
    run.until(lambda r: r.guard.point_index == 1)
    run.until(lambda r: r.guard.state == "RECOGNIZE_WAIT", cap=25000)
    coordinator.poll_once(run.wall)
    coordinator.preparer.join(timeout=1.)
    assert coordinator.next_seq == 6
    assert [row for row in calls if row[0] == "prepare"] == [("prepare", 0), ("prepare", 3)]
    contexts = [row for row in calls if row[0] == "context"]
    assert contexts[0][2] == "ground_0" and contexts[1][2] == "ground_3"
    assert contexts[0][3] < contexts[1][3]


def test_new_ros_adapter_keeps_flight_light_and_completion_silent(monkeypatch):
    from test_bounded_ground_probe import install_ros_transport
    _, published, _ = install_ros_transport(monkeypatch)
    node = api().create_three_point_node(wall_clock=lambda: 0.)
    snapshot = node.evidence_snapshot()
    assert snapshot["mode"] == "ground_three_point"
    for topic in ("/inspection/detections", "/referee/current_target"):
        assert snapshot["command_endpoints"][topic]["silent"] is False
        assert snapshot["command_endpoints"][topic]["actual_publish_attempt_count"] == 0
    for topic in ("/uav/cmd_vel", "/uav/laser_fire", "/referee/cruise_event"):
        assert snapshot["command_endpoints"][topic]["silent"] is True
    node.tick()
    assert all(topic == "/cmd_vel" for topic, _, _ in published)
    node.close_watchdog()


def test_only_last_drive_of_observation_or_entry_uses_final_approach_slow_tier():
    run = ThreeRun()
    run.until(lambda r: r.guard.phase == "DRIVE" and r.guard.leg_target_xy == (.6, 0.)
              and r.guard.leg_target_distance_m-r.guard.measured_forward() <= .08)
    assert run.last.linear_x == .10
    run.until(lambda r: r.guard.phase == "DRIVE" and r.guard.leg_target_xy == (.6, -.45)
              and r.guard.leg_target_distance_m-r.guard.measured_forward() <= .08)
    assert run.last.linear_x == .03


def test_original_nine_mm_ground2_endpoint_error_is_still_rejected_by_actual_arc_gate():
    from inspection_mission.ground_three_point_plan import plan_ground_leg
    plan = plan_ground_leg(None, "ground_2", (0., 0., 0.))
    guard = api().GroundThreePointGuard(started_wall_s=0.)
    actual_yaw = -3.0104704807897806
    payload = posed_odom(10., x=.0029968482356460095, yaw=actual_yaw)
    payload["pose"]["pose"]["position"]["y"] = -2.6993152857686487
    guard.update_odom(payload, wall_s=0.)
    guard.target_id = "ground_2"
    guard.current_operations_role = "entry"
    guard.operations = (plan.entry_operations[-1],)
    guard._start_operation(0.)
    assert guard.state == "STOP" and guard.reason == "three_ground_actual_turn_candidate_unsafe"
    assert guard.limits.robot_radius_m == .447


