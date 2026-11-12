"""v36 camera gaps pause safely; synthetic fixtures are not platform evidence."""
from copy import deepcopy
import importlib
import importlib.util
from types import SimpleNamespace

import pytest

from test_bounded_ground_probe import endpoint, install_ros_transport
from test_ground_bounded_approach import refresh, image
from test_ground_observation_survey import posed_odom
from test_ground_single_point import camera_info, fixture_jobs


def api():
    name = "inspection_mission.ground_three_point_camera_recovery"
    # RED characterization uses the real frozen guard, not an import error.
    return importlib.import_module(name if importlib.util.find_spec(name) else
                                   "inspection_mission.ground_three_point_axis")


def guard_type():
    return getattr(api(), "GroundThreePointCameraRecoveryGuard", api().GroundThreePointAxisGuard)


class StationaryRun:
    def __init__(self):
        self.wall, self.x, self.y, self.yaw = 0., .6, -.5, -2.185459278717062
        self.guard = guard_type()(started_wall_s=0., session_id="test-current-session")
        g = self.guard
        g.update_endpoints([endpoint()], matched_subscriptions=1)
        g.target_id, g.target_xy, g.target_yaw = "ground_1", (0., -1.35), self.yaw
        g.observe_public("/uav/camera/camera_info", camera_info(), wall_s=0.)
        self.sensors()
        g.global_origin = g.pulse_origin = (0., 0., .055)
        g.previous_path_xyz, g.previous_yaw = (.6, -.5, .055), self.yaw
        g.total_path_m, g.accumulated_rotation_rad, g.first_motion_wall = 1.1, 2.18, 0.
        g._enter_phase("CAPTURE")
        for _ in range(25):
            self.tick()
            if g.state == "RECOGNIZE_WAIT":
                break
        assert g.state == "RECOGNIZE_WAIT", g.reason

    def sensors(self, *, emit_image=True, source_stamp=None):
        refresh(self.guard, self.wall, x=self.x, y=self.y, yaw=self.yaw)
        stamp = round(10.+self.wall, 9)
        uav = posed_odom(stamp, x=self.x, yaw=self.yaw)
        uav["child_frame_id"] = "uav/base_link"
        uav["pose"]["pose"]["position"]["y"] = self.y
        self.guard.observe_public("/uav/odom", uav, wall_s=self.wall)
        if emit_image:
            self.guard.observe_image("/uav/camera/image", image(stamp if source_stamp is None else source_stamp), wall_s=self.wall)

    def tick(self, *, emit_image=True, dt=.05):
        self.wall = round(self.wall+dt, 9)
        self.sensors(emit_image=emit_image)
        return self.guard.step(wall_s=self.wall)


def advance_no_image(run, elapsed):
    end = run.wall+elapsed
    while end-run.wall > .05+1e-9:
        run.wall += .05
        run.sensors(emit_image=False)
    run.wall = end
    run.sensors(emit_image=False)


def camera_gap(run, *, elapsed=.530038013):
    # Keep real control streams live throughout the camera-only gap. A single
    # giant sensor jump would instead exercise the old .20s control pause.
    advance_no_image(run, elapsed)
    return run.guard.step(wall_s=run.wall)


@pytest.mark.parametrize("phase", ["CAPTURE", "RECOGNIZE_WAIT"])
def test_real_v35_530ms_gap_pauses_without_relaxing_camera_gate(phase):
    run = StationaryRun(); g = run.guard
    if phase == "CAPTURE":
        g._enter_phase("CAPTURE")
    d = camera_gap(run)
    assert (g.state, d.linear_x, d.angular_z) == ("PAUSE", 0., 0.)
    assert g.epoch_invalid == "sensor_freshness_lost"
    assert g.last_camera_liveness_check["image_wall_age_s"] == pytest.approx(.530038013)
    event = g.approach_evidence()["camera_recovery"]["events"][0]
    assert event["event"] == "pause" and event["reason"] == "single_point_uav_image_stale"
    assert event["epoch"] == g.stationary_epoch and event["resume_count"] == 0
    assert event["clock_stamp_s"] == pytest.approx(10.+run.wall)
    assert event["camera_max_age_s"] == .5


def test_positive_source_age_gap_recovers_but_invalid_negative_age_is_terminal():
    run = StationaryRun(); g = run.guard
    g.last_uav_image = (run.wall, g.clock[0]-.543)
    g.step(wall_s=run.wall)
    assert g.state == "PAUSE"
    assert g.reason == "single_point_current_image_clock_inconsistent"
    run = StationaryRun(); g = run.guard
    g.last_uav_image = (run.wall, g.clock[0]+.120002)
    g.step(wall_s=run.wall)
    assert g.state == "STOP"


def test_camera_must_recover_then_both_clocks_dwell_before_new_epoch():
    run = StationaryRun(); g = run.guard
    old_epoch = g.stationary_epoch
    before = (g.global_origin, g.started_wall, g.total_path_m, g.accumulated_rotation_rad,
              g.frozen_limits, g.turn_elapsed_wall_s, g.drive_elapsed_wall_s)
    camera_gap(run)
    for _ in range(24):
        run.tick(emit_image=False)
        assert g.state == "PAUSE"
    for _ in range(19):
        run.tick()
        assert g.state == "PAUSE"
    for _ in range(4):
        run.tick()
        if g.state == "CAPTURE":
            break
    assert g.state == "CAPTURE" and g.stationary_epoch == old_epoch+1
    assert g.resume_count == g.recapture_count == 1
    assert not g.capture_frames and not g.expected_jobs and not g.match_results
    after = (g.global_origin, g.started_wall, g.total_path_m, g.accumulated_rotation_rad,
             g.frozen_limits, g.turn_elapsed_wall_s, g.drive_elapsed_wall_s)
    assert after == before
    event = g.approach_evidence()["camera_recovery"]["events"][-1]
    assert event["event"] == "recapture" and event["epoch"] == old_epoch+1


@pytest.mark.parametrize("resume,recapture", [(3, 0), (0, 1)])
def test_original_global_and_per_point_recovery_quotas_still_stop(resume, recapture):
    run = StationaryRun(); g = run.guard
    g.resume_count, g.recapture_count = resume, recapture
    camera_gap(run)
    for _ in range(25):
        run.tick()
        if g.state == "STOP":
            break
    assert g.state == "STOP" and "recapture_limit" in g.reason
    assert g.resume_count == resume and g.recapture_count == recapture


def test_old_worker_result_cannot_cross_camera_recovery_epoch():
    from inspection_mission.ground_single_point import GroundSinglePointCoordinator
    run = StationaryRun(); g = run.guard
    jobs, results = fixture_jobs(run)
    class Worker:
        def __init__(self): self.context = None; self.results = []
        def cancel_context(self): self.context = None
        def set_context(self, *value): self.context = value
        def poll(self): rows, self.results = self.results, []; return rows
        def submit(self, job): pass
    worker = Worker()
    coordinator = GroundSinglePointCoordinator(g, worker, prepare=lambda *a, **kw: jobs)
    coordinator.poll_once(run.wall)
    camera_gap(run)
    coordinator.poll_once(run.wall)
    assert coordinator.context is None and worker.context is None
    for _ in range(25):
        run.tick()
        if g.state == "CAPTURE": break
    for _ in range(4):
        run.tick()
        if g.state == "RECOGNIZE_WAIT": break
    assert g.state == "RECOGNIZE_WAIT" and g.stationary_epoch > jobs[0]["stationary_epoch"]
    fixture_jobs(run)
    assert not g.accept_match_result(results[0], wall_s=run.wall)
    assert not g.match_results and not g.attempts


@pytest.mark.parametrize("fault", ["frame", "stamp", "reversed", "shape", "stride", "negative"])
def test_illegal_or_reversed_image_cannot_be_recovered(fault):
    run = StationaryRun(); g = run.guard
    stamp = g.last_uav_image[1]+.01
    msg = image(stamp)
    if fault == "frame": msg.header.frame_id = "wrong_camera"
    if fault == "stamp": msg.header.stamp.nanosec = 10**9
    if fault == "reversed": msg = image(g.last_uav_image[1]-.001)
    if fault == "shape": msg.encoding = "uninterpreted8"
    if fault == "stride": msg.step = 1
    if fault == "negative": msg = image(g.clock[0]+.120002)
    g.observe_image("/uav/camera/image", msg, wall_s=run.wall)
    assert g.state == "STOP"
    run.tick()
    assert g.state == "STOP" and not g.attempts


def test_original_negative_sampling_tolerance_and_duplicate_no_refresh_remain():
    run = StationaryRun(); g = run.guard
    msg = image(g.clock[0]+.10)
    g.observe_image("/uav/camera/image", msg, wall_s=run.wall)
    assert g.state == "RECOGNIZE_WAIT"
    original = g.last_uav_image
    run.wall += .1
    run.sensors(emit_image=False)
    g.observe_image("/uav/camera/image", msg, wall_s=run.wall)
    assert g.last_uav_image == original


@pytest.mark.parametrize("phase,attempt", [("REPORT_WAIT", False), ("RECOGNIZE_WAIT", True)])
def test_reporting_or_uncertain_attempt_never_retries_or_republishes(phase, attempt):
    run = StationaryRun(); g = run.guard
    g.phase = g.state = phase
    if attempt: g.attempts["recognition"] = {"attempted": True, "sent": False}
    camera_gap(run)
    assert g.state == "STOP"
    sent = []
    for _ in range(4):
        run.tick()
        g.dispatch_one(lambda *a: sent.append(a), wall_s=run.wall)
    assert not sent and g.state in ("STOP", "FINISHED")


def test_stale_image_at_dispatch_boundary_cannot_publish_current_result():
    run = StationaryRun(); g = run.guard
    _, results = fixture_jobs(run)
    for row in results: assert g.accept_match_result(row, wall_s=run.wall)
    advance_no_image(run, .530038013)
    sent = []
    assert g.dispatch_one(lambda *a: sent.append(a), wall_s=run.wall) is None
    assert not sent and not g.attempts
    g.step(wall_s=run.wall)
    assert g.state == "PAUSE"


def test_actual_factory_constructs_new_guard_and_preserves_final_time_check(monkeypatch):
    install_ros_transport(monkeypatch)
    wall = [0.]
    create = getattr(api(), "create_camera_recovery_three_point_node", api().create_axis_three_point_node)
    node = create(wall_clock=lambda: wall[0])
    node.arm_watchdog = lambda: None
    assert type(node.guard).__name__ == "GroundThreePointCameraRecoveryGuard"
    run = StationaryRun()
    # Factory construction above is separately asserted; reusing stationary
    # physical state here isolates final-DDS time advancement at publication.
    node.guard = run.guard
    wall[0] = run.wall
    node.last_graph_refresh_wall = wall[0]
    count = [0]
    def matched():
        count[0] += 1
        if count[0] == 2:
            advance_no_image(run, .530038013); wall[0] = run.wall
        return 1
    node.publisher_by_topic["/cmd_vel"].get_subscription_count = matched
    sent = []
    node.publisher_by_topic["/cmd_vel"].publish = lambda msg: sent.append((msg.linear.x, msg.angular.z))
    node.tick()
    assert node.guard.state == "PAUSE" and sent and all(x == w == 0. for x, w in sent)
    node.close_watchdog()


def test_recovery_wait_does_not_expand_global_deadline():
    run = StationaryRun(); g = run.guard
    camera_gap(run)
    run.wall = 1200.001
    run.sensors(emit_image=False)
    g.step(wall_s=run.wall)
    assert g.state == "STOP" and "global_wall_time_limit" in g.reason


def test_camera_pause_step_keeps_wall_clock_monotonic_without_new_callbacks():
    run = StationaryRun(); g = run.guard
    camera_gap(run)
    g.step(wall_s=run.wall+.10)
    g.step(wall_s=run.wall+.05)
    assert g.state == "STOP" and g.reason == "wall_clock_reversed_or_invalid"


def test_current_clock_sampling_ahead_is_not_new_camera_stop_during_motion():
    run = StationaryRun(); g = run.guard
    g.phase = g.state = "TURN"
    g.observe_image("/uav/camera/image", image(g.clock[0]+.16), wall_s=run.wall)
    assert g.state == "TURN" and g.last_uav_image[1] == pytest.approx(g.clock[0]+.16)


def test_factory_injection_constructs_subclass_with_same_frozen_contract(monkeypatch):
    from inspection_mission.ground_three_point_axis import AxisThreePointLimits
    install_ros_transport(monkeypatch)
    class CustomGuard(guard_type()):
        pass
    limits = AxisThreePointLimits(global_wall_s=1199.)
    node = api().create_camera_recovery_three_point_node(route_limits=limits, guard_factory=CustomGuard)
    assert type(node.guard) is CustomGuard and node.guard.frozen_limits is limits
    node.close_watchdog()
    with pytest.raises(ValueError, match="same frozen contract"):
        api().create_camera_recovery_three_point_node(route_limits=limits,
            guard_factory=lambda **kwargs: CustomGuard(started_wall_s=kwargs["started_wall_s"]))
    with pytest.raises(ValueError, match="CameraRecoveryGuard"):
        api().create_camera_recovery_three_point_node(route_limits=limits,
            guard_factory=api().GroundThreePointAxisGuard)
