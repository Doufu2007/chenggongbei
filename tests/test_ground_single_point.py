"""First real point experiment: current images, bounded entry, truthful transport."""
from copy import deepcopy
import importlib
import importlib.util
import math
import threading
import time

import pytest

from test_bounded_ground_probe import endpoint
from test_ground_observation_route import Run
from test_ground_bounded_approach import image, refresh
from test_ground_observation_survey import posed_odom


def api():
    name = "inspection_mission.ground_single_point"
    assert importlib.util.find_spec(name), "independent first-ground score loop missing"
    return importlib.import_module(name)


class SingleRun(Run):
    def __init__(self, first="ground_4"):
        super().__init__(first)
        draw = deepcopy(self.guard.draw)
        self.guard = api().GroundSinglePointGuard(started_wall_s=0., session_id="test-current-session")
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


def camera_info():
    return {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "uav/camera_link"},
            "width": 800, "height": 800, "k": [476.7, 0., 400.5, 0., 476.7, 400.5, 0., 0., 1.], "d": [0.]*5}


def fixture_jobs(run, *, category="trapped_people"):
    state = run.guard
    jobs, results = [], []
    board, size, _ = api()._POINTS[state.target_id]
    for index, frame in enumerate(state.capture_frames):
        record, pose = frame["record"], frame["record"]["odom"]
        result = candidate_result(state.target_id, category, stamp=record["stamp_s"], seq=index)
        result["job"].update(pose=deepcopy(pose), camera_info=deepcopy(frame["camera_info"]),
            K=[[476.7, 0., 400.5], [0., 476.7, 400.5], [0., 0., 1.]], stationary_epoch=state.stationary_epoch)
        yaw = api().yaw_from_q(pose["quaternion"])
        def local(point):
            x, y = point[0]-pose["position"][0], point[1]-pose["position"][1]
            return [math.cos(yaw)*x+math.sin(yaw)*y, -math.sin(yaw)*x+math.cos(yaw)*y]
        row = result["matcher"]["candidates"][0]
        row["body_plane_center_forward_left_m"] = local(board)
        row["body_plane_quad_forward_left_m"] = [local((board[0]+x*size[0]/2, board[1]+y*size[1]/2))
                                                for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        jobs.append({**deepcopy(result["job"]), "source_jpeg": b"unit-fixture-never-submitted-to-real-worker"})
        results.append(result)
    assert state.register_prepared_jobs(jobs, epoch=state.stationary_epoch)
    return jobs, results


def candidate_result(first="ground_4", category="trapped_people", world=None, stamp=10., seq=0):
    center = {"ground_0": (1.125, -1.35), "ground_1": (0., -1.35), "ground_2": (0., -2.7),
              "ground_3": (1.125, -1.925), "ground_4": (1.125, -.775)}[first]
    center = center if world is None else world
    return {"job": {"job_seq": seq, "session_id": "test-current-session", "frozen_first_ID": first,
                    "stationary_epoch": 1, "source_sha256": f"{seq+1:064x}", "image_stamp_s": stamp,
                    "frame_id": "uav/camera_link", "topic": "/uav/camera/image",
                    "pose": {"stamp_s": stamp, "position": [0., 0., .055], "quaternion": [0., 0., 0., 1.]}},
            "matches_current_context": True,
            "matcher": {"candidates": [{"accepted_candidate": True, "class_id": category,
                "source_quad_xy": [[10., 10.], [110., 10.], [110., 110.], [10., 110.]],
                "body_plane_center_forward_left_m": list(center),
                "body_plane_quad_forward_left_m": [[center[0]-.2, center[1]-.2], [center[0]+.2, center[1]-.2],
                    [center[0]+.2, center[1]+.2], [center[0]-.2, center[1]+.2]],
                "template_sha256": "a"*64, "provenance": {"source": "explicit-test-fixture"}}]}}


@pytest.mark.parametrize("first,total,allowed", [("ground_0", 2.465, True), ("ground_1", 2.55, True),
    ("ground_2", 3.9, False), ("ground_3", 3.04, False), ("ground_4", 1.89, True)])
def test_composite_plan_freezes_axis_entry_inside_original_caps(first, total, allowed):
    plan = api().plan_first_ground_point(first, (0., 0., 0.))
    assert plan.total_path_m == pytest.approx(total)
    assert plan.entry_allowed is allowed
    assert plan.budgets.max_path_m <= 3. and plan.budgets.max_rotation_rad <= 6.
    assert all(abs(a[0]-b[0]) < 1e-12 or abs(a[1]-b[1]) < 1e-12
               for a, b in zip([plan.observation.waypoints_xy_m[-1], *plan.entry_waypoints_xy_m], plan.entry_waypoints_xy_m))


def test_current_world_region_and_category_conflict_are_both_required():
    result = candidate_result()
    bound = api().bind_ground_candidate(result, "ground_4")
    assert bound["class_id"] == "trapped_people" and bound["label"] == "被困人员"
    assert api().bind_ground_candidate(candidate_result(world=(1.125, -1.35)), "ground_4") is None
    result["matcher"]["candidates"].append(deepcopy(result["matcher"]["candidates"][0]))
    result["matcher"]["candidates"][-1]["class_id"] = "rescue_camp"
    assert api().bind_ground_candidate(result, "ground_4") is None


def test_composite_budget_is_set_before_first_motion_and_old_route_is_unchanged():
    run = SingleRun().until(lambda r: r.last.linear_x != 0. or r.last.angular_z != 0.)
    assert run.guard.single_plan.budgets.max_path_m == pytest.approx(2.19)
    assert run.guard.approach_limits.max_path_m == pytest.approx(2.19)
    old = Run("ground_4").until(lambda r: r.last.linear_x != 0.)
    assert old.guard.approach_limits.max_path_m == pytest.approx(.65)


def test_only_exact_normal_three_frame_completion_enters_recognition():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    assert len(run.guard.capture_frames) == 3
    assert run.guard.reason == "route_recognize_wait" and not run.guard.attempts
    run.guard.stop("scan_invalid", run.wall)
    assert run.guard.state == "STOP" and run.guard.reason == "scan_invalid"
    run.guard.stop("approach_observation_complete", run.wall)
    assert run.guard.state == "STOP" and run.guard.reason == "scan_invalid"


def test_three_consistent_original_results_send_one_phrase_and_transport_is_not_ack():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    state = run.guard
    _, results = fixture_jobs(run)
    sent = []
    publish = lambda topic, data: sent.append((topic, data))
    for result in results[:2]:
        assert state.accept_match_result(result, wall_s=run.wall)
        assert state.dispatch_one(publish, wall_s=run.wall) is None
    assert state.accept_match_result(results[2], wall_s=run.wall)
    assert state.dispatch_one(publish, wall_s=run.wall)["sent"] is True
    assert sent == [("/inspection/detections", "发现被困人员，地面巡航任务点确认，将巡航信息发送回指挥中心。")]
    for _ in range(3):
        state.dispatch_one(publish, wall_s=run.wall)
    state.observe_public("/referee/recognition_status", {"data": '{"accepted":true}'}, wall_s=run.wall)
    evidence = state.approach_evidence()["single_point"]
    assert evidence["official_acceptance"] is False
    assert evidence["recognition_status"][-1]["data"] == '{"accepted":true}'
    assert evidence["attempts"]["recognition"]["interpretation"] == "local_transport_attempt_not_referee_ack"
    assert len(sent) == 1


@pytest.mark.parametrize("mutation", ["session", "epoch", "stamp", "hash", "context"])
def test_old_or_changed_result_identity_never_selects_a_category(mutation):
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    result = results[0]
    if mutation == "session": result["job"]["session_id"] = "another-run"
    if mutation == "epoch": result["job"]["stationary_epoch"] -= 1
    if mutation == "stamp": result["job"]["image_stamp_s"] += .001
    if mutation == "hash": result["job"]["source_sha256"] = "0"*64
    if mutation == "context": result["matches_current_context"] = False
    assert not run.guard.accept_match_result(result, wall_s=run.wall)
    assert not run.guard.match_results and run.guard.recognition is None


def test_multiframe_class_disagreement_latches_without_any_phrase():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    assert run.guard.accept_match_result(results[0], wall_s=run.wall)
    results[1]["matcher"]["candidates"][0]["class_id"] = "rescue_camp"
    assert not run.guard.accept_match_result(results[1], wall_s=run.wall)
    assert run.guard.reason == "single_point_multiframe_disagreement"
    assert not run.guard.attempts


def test_result_after_real_sift_delay_keeps_source_stamp_with_live_sensors():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    source_stamp = results[0]["job"]["image_stamp_s"]
    for _ in range(62):
        run.tick()
    assert run.guard.clock[0]-source_stamp > 3.
    assert run.guard.accept_match_result(results[0], wall_s=run.wall)
    assert run.guard.match_results[0]["image_stamp_s"] == source_stamp
    assert run.guard.state == "RECOGNIZE_WAIT"


@pytest.mark.parametrize("first", ["ground_0", "ground_1", "ground_4"])
def test_measured_axis_entry_stops_then_declares_exactly_once_without_final_turn(first):
    run = SingleRun(first).until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    state = run.guard
    origin = state.global_origin
    budgets = (state.approach_limits.max_path_m, state.approach_limits.max_rotation_rad)
    observation_path = state.total_path_m
    _, results = fixture_jobs(run)
    for result in results:
        assert state.accept_match_result(result, wall_s=run.wall)
    sent = []
    publish = lambda topic, data: sent.append((topic, data))
    state.dispatch_one(publish, wall_s=run.wall)
    assert state.phase == "REPORT_WAIT" and len(sent) == 1
    run.until(lambda r: r.guard.state == "DECLARE_WAIT")
    assert state.global_origin == origin and state.total_path_m > observation_path
    assert (state.approach_limits.max_path_m, state.approach_limits.max_rotation_rad) == budgets
    assert state._inside_candidate_region() and state._stable_dwell(run.wall)
    assert all(value.linear_x == value.angular_z == 0. for _, value, phase, _, _ in run.commands[-2:] if phase == "DECLARE_WAIT")
    state.dispatch_one(publish, wall_s=run.wall)
    state.dispatch_one(publish, wall_s=run.wall)
    assert sent[-1] == ("/referee/current_target", first) and len(sent) == 2
    for _ in range(42): run.tick()
    assert state.state == "STOP" and state.reason == "single_point_transport_complete_official_score_unverified"
    assert not state.approach_evidence()["single_point"]["full_mission_complete"]


@pytest.mark.parametrize("first", ["ground_2", "ground_3"])
def test_incompatible_entry_budget_never_moves_or_declares_after_reporting(first):
    run = SingleRun(first).until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    for result in results: assert run.guard.accept_match_result(result, wall_s=run.wall)
    sent = []
    run.guard.dispatch_one(lambda *args: sent.append(args), wall_s=run.wall)
    for _ in range(22): run.tick()
    assert run.guard.state == "STOP" and run.guard.reason.startswith("single_point_entry_not_permitted:")
    assert not run.guard.in_entry and len(sent) == 1
    assert "declaration" not in run.guard.attempts


def test_publish_exception_latches_attempt_before_call_and_does_not_retry():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    for result in results: run.guard.accept_match_result(result, wall_s=run.wall)
    calls = []
    def fail(topic, data):
        assert run.guard.attempts["recognition"]["attempted"]
        calls.append(topic)
        raise RuntimeError("uncertain transport")
    event = run.guard.dispatch_one(fail, wall_s=run.wall)
    run.guard.dispatch_one(fail, wall_s=run.wall)
    assert calls == ["/inspection/detections"] and not event["sent"]
    assert run.guard.reason == "single_point_recognition_publish_uncertain"


def test_reported_recognition_cannot_survive_an_unobserved_freshness_gap():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    for result in results: run.guard.accept_match_result(result, wall_s=run.wall)
    run.guard.dispatch_one(lambda *args: None, wall_s=run.wall)
    assert run.guard.state == "REPORT_WAIT"
    run.guard.temporal_fault("scan_stale", run.wall)
    assert run.guard.state == "STOP" and not run.guard.in_entry
    assert run.guard.reason == "single_point_report_wait_sensor_freshness_lost:scan_stale"


def test_stationary_pose_or_nonzero_command_invalidates_original_exposure():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    run.guard.record_command(wall_s=run.wall, linear_x=.001, angular_z=0.)
    assert not run.guard.accept_match_result(results[0], wall_s=run.wall)
    assert run.guard.epoch_invalid == "nonzero_command_since_exposure"
    run.tick()
    assert run.guard.state == "STOP" and "stationary_epoch_invalid" in run.guard.reason


def test_current_image_progress_is_required_without_rewriting_old_exposure():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    run.guard.last_uav_image = (run.wall-.501, results[0]["job"]["image_stamp_s"])
    assert not run.guard.accept_match_result(results[0], wall_s=run.wall)
    run.guard.step(wall_s=run.wall)
    assert run.guard.state == "STOP" and run.guard.reason == "single_point_uav_image_stale"


def test_v24b_measured_camera_gap_does_not_stop_zero_motion_recognition():
    # Raw line-5 capture_2 -> line-11 first STOP command sampled delta, with the
    # independent current odom/scan/clock refreshed; original image unchanged.
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    state = run.guard
    _, results = fixture_jobs(run)
    source_before = deepcopy(state.capture_frames[-1]["record"])
    state.last_uav_image = (run.wall-.21698330799834, state.clock[0]-.125)
    assert state._current_recognition_condition(run.wall) == ""
    assert state.step(wall_s=run.wall).state == "RECOGNIZE_WAIT"
    assert state.capture_frames[-1]["record"] == source_before
    assert state.accept_match_result(results[0], wall_s=run.wall)


@pytest.mark.parametrize("age,expected", [(.20, ""), (.38484501899802126, ""), (.50, ""),
    (.500002, "single_point_uav_image_stale")])
def test_camera_wall_age_has_own_half_second_boundary(age, expected):
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    run.guard.last_uav_image = (run.wall-age, run.guard.clock[0]-.1)
    assert run.guard._current_recognition_condition(run.wall) == expected


@pytest.mark.parametrize("delta,expected", [(-.12, ""), (.384845, ""), (.50, ""),
    (-.120002, "single_point_current_image_clock_inconsistent"),
    (.500002, "single_point_current_image_clock_inconsistent")])
def test_camera_original_stamp_age_is_independent_from_wall_arrival(delta, expected):
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    run.guard.last_uav_image = (run.wall, run.guard.clock[0]-delta)
    assert run.guard._current_recognition_condition(run.wall) == expected


def test_three_hz_camera_continues_while_control_sensors_stay_fresh():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    original = deepcopy(run.guard.capture_frames)
    for index in range(70):
        run.tick(emit_image=index % 7 == 0)
        assert run.guard.state == "RECOGNIZE_WAIT", run.guard.reason
    assert run.guard.clock[0]-results[0]["job"]["image_stamp_s"] > 3.
    assert run.guard.accept_match_result(results[0], wall_s=run.wall)
    assert run.guard.capture_frames == original


def test_duplicate_camera_stamp_cannot_refresh_arrival_freshness():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    original = run.guard.last_uav_image
    run.tick(emit_image=False)
    run.guard.observe_image("/uav/camera/image", image(original[1]), wall_s=run.wall)
    assert run.guard.last_uav_image == original


def test_camera_independent_gate_does_not_relax_control_sensor_age():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    state = run.guard
    state.last_uav_image = (run.wall-.3, state.clock[0]-.3)
    assert state._current_recognition_condition(run.wall) == ""
    assert state.limits.sensor_age_s == .20 and state.limits.clock_sampling_lag_s == .12
    assert state._condition(run.wall+.201)
    assert state.dispatch_one(lambda *_: pytest.fail("must not publish"), wall_s=run.wall+.201) is None


def test_real_encoder_preserves_800px_original_stamp_pose_and_camera_info():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    frames = [dict(frame) for frame in run.guard.capture_frames]
    for frame in frames:
        msg = deepcopy(frame["msg"])
        msg.width = msg.height = 800
        msg.step = 2400
        msg.data = bytes((100, 90, 80))*800*800
        frame["msg"] = msg
    jobs = api().prepare_capture_jobs(frames, session_id=run.guard.session_id, first_id=run.guard.target_id,
        epoch=run.guard.stationary_epoch, first_seq=7)
    from inspection_mission.ground_matcher_worker import LatestFrameMailbox
    mailbox = LatestFrameMailbox()
    mailbox.set_context(run.guard.session_id, run.guard.target_id, run.guard.stationary_epoch)
    for job, frame in zip(jobs, frames):
        mailbox.submit(job)
        assert job["image_stamp_s"] == frame["record"]["stamp_s"]
        assert job["pose"] == frame["record"]["odom"]
        assert job["camera_info"] == frame["camera_info"]
    assert [job["job_seq"] for job in jobs] == [7, 8, 9]


def test_coordinator_never_holds_guard_lock_during_slow_preparation():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    entered, release = threading.Event(), threading.Event()
    class Worker:
        def cancel_context(self): pass
        def set_context(self, *args): pass
        def poll(self): return []
    def slow(frames, **kwargs):
        entered.set()
        assert release.wait(2.)
        return []
    coordinator = api().GroundSinglePointCoordinator(run.guard, Worker(), prepare=slow)
    started = time.perf_counter()
    coordinator.poll_once(run.wall)
    assert time.perf_counter()-started < .1 and entered.wait(.2)
    assert run.guard._lock.acquire(timeout=.1)
    run.guard._lock.release()
    run.tick()
    assert run.guard.state == "RECOGNIZE_WAIT"
    release.set()
    coordinator.preparer.join(timeout=.5)


def test_single_point_core_fits_real_diagnostic_contract_with_bounded_raw_status():
    run = SingleRun().until(lambda r: r.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    for result in results: run.guard.accept_match_result(result, wall_s=run.wall)
    for _ in range(20):
        run.guard.observe_public("/referee/recognition_status", {"data": "中"*10000}, wall_s=run.wall)
    from inspection_mission.telemetry import DiagnosticReporter
    core = run.guard.additional_events()[0]
    encoded = DiagnosticReporter._encode(*core)
    assert len(encoded) < 64*1024
    assert core[1]["section"] == "single_point_core" and core[1]["official_acceptance"] is False
    assert len(core[1]["recognition_status"]) == 8
    assert core[1]["recognition_status"][0]["data_truncated"]


def test_asset_manifest_loads_exact_public_eight_and_linux_relative_paths():
    specs = api().load_ground_templates()
    assert len(specs) == 8 and {spec.class_id for spec in specs} == set(api().GROUND_LABELS)
    assert all(spec.expected_sha256 and spec.provenance["commit"] == api().TEMPLATE_COMMIT for spec in specs)


def test_node_changes_only_single_point_string_policy_and_tracks_actual_attempts(monkeypatch):
    from test_bounded_ground_probe import install_ros_transport
    _, published, _ = install_ros_transport(monkeypatch)
    node = api().create_single_point_node(wall_clock=lambda: 0.)
    snapshot = node.evidence_snapshot()
    assert snapshot["mode"] == "ground_single_point"
    for topic in ("/inspection/detections", "/referee/current_target"):
        endpoint = snapshot["command_endpoints"][topic]
        assert endpoint["silent"] is False and endpoint["actual_publish_attempt_count"] == 0
        assert endpoint["actual_publish_return_count"] == 0
    assert snapshot["command_endpoints"]["/uav/cmd_vel"]["silent"] is True
    assert snapshot["command_endpoints"]["/uav/laser_fire"]["silent"] is True
    node.tick()
    assert all(topic == "/cmd_vel" for topic, _, _ in published)
    node.close_watchdog()
