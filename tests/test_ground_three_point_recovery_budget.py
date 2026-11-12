"""Finite route retry resources; real guard fixtures, no platform claims."""
import importlib
import importlib.util
import pytest

import test_ground_three_point_camera_recovery as camera_fixture


def budget_api():
    name = 'inspection_mission.ground_three_point_recovery_budget'
    return importlib.import_module(name if importlib.util.find_spec(name) else
                                   'inspection_mission.ground_three_point_camera_recovery')


@pytest.fixture
def run(monkeypatch):
    mod = budget_api()
    cls = getattr(mod, 'GroundThreePointRecoveryBudgetGuard', mod.GroundThreePointCameraRecoveryGuard)
    monkeypatch.setattr(camera_fixture, 'guard_type', lambda: cls)
    return camera_fixture.StationaryRun()


def keep_fresh(run, seconds=1.2):
    end = run.wall+seconds
    while run.wall < end-1e-9 and run.guard.state not in ('STOP', 'FINISHED'):
        run.tick()


def control_pause(run):
    run.guard.temporal_fault('odom_stale_or_invalid', run.wall)
    assert run.guard.state == 'PAUSE'


def test_fourth_safe_resume_is_allowed_with_fresh_recapture_and_true_counter(run):
    g = run.guard
    old_epoch, old_path, old_rotation = g.stationary_epoch, g.total_path_m, g.accumulated_rotation_rad
    g.resume_count = 3
    control_pause(run)
    keep_fresh(run)
    assert g.state not in ('STOP', 'FINISHED'), g.reason
    assert g.resume_count == 4
    assert g.stationary_epoch == old_epoch+1 and g.recapture_count == 1
    assert g.total_path_m == pytest.approx(old_path)
    assert g.accumulated_rotation_rad == pytest.approx(old_rotation)
    assert not g.attempts and not g.match_results


def test_all_32_slots_can_be_used_but_slot_33_is_terminal(run):
    g = run.guard
    # Motion phase recovery never reuses a capture epoch. Every pause below
    # still receives actual fresh odom/clock/scan and a new stationary dwell.
    g.stationary_anchor = None
    g.phase = g.state = 'TURN'
    for expected in range(1, 33):
        control_pause(run)
        for _ in range(25):
            d = run.tick()
            assert (d.linear_x, d.angular_z) == (0., 0.)
            if g.state != 'PAUSE':
                break
        # Real odom may move the already-aligned turn into TURN_SETTLE even
        # during PAUSE; recovery restores that measured phase, with zero.
        assert g.state == g.phase and g.state in ('TURN', 'TURN_SETTLE'), g.reason
        assert g.resume_count == expected
    control_pause(run)
    keep_fresh(run)
    assert g.state in ('STOP', 'FINISHED')
    assert g.resume_count == 32
    assert 'resume' in g.reason


def test_single_pause_has_fixed_five_second_wall_deadline_even_if_never_fresh(run):
    g = run.guard
    control_pause(run)
    start = run.wall
    while run.wall-start < 5.1-1e-9 and g.state not in ('STOP', 'FINISHED'):
        run.wall = round(run.wall+.05, 9)
        # Images remain valid, but no new clock/odom may authorize movement.
        decision = g.step(wall_s=run.wall)
        assert (decision.linear_x, decision.angular_z) == (0., 0.)
    assert g.state in ('STOP', 'FINISHED')
    assert 'pause' in g.reason and 'limit' in g.reason
    assert g.resume_count == 0


def test_cumulative_pause_deadline_covers_watchdog_and_evidence(run):
    g = run.guard
    g.phase = g.state = 'TURN'
    for _ in range(20):
        control_pause(run)
        # Keep a real sensor gap for two seconds, then fresh + 1s dwell.
        for _ in range(40):
            run.wall = round(run.wall+.05, 9)
            d = g.step(wall_s=run.wall)
            assert (d.linear_x, d.angular_z) == (0., 0.)
            if g.state in ('STOP', 'FINISHED'):
                break
        if g.state in ('STOP', 'FINISHED'):
            break
        keep_fresh(run, 1.1)
        if g.state in ('STOP', 'FINISHED'):
            break
    assert g.state in ('STOP', 'FINISHED')
    assert 'cumulative_pause' in g.reason
    evidence = g.approach_evidence()
    assert evidence['recovery_resource']['max_resumes'] == 32
    assert evidence['recovery_resource']['cumulative_pause_wall_limit_s'] == 60.
    assert evidence['resume_count'] == g.resume_count
    assert evidence['max_resumes'] == 32


def test_stale_camera_wait_cannot_consume_fresh_dwell_or_report(run):
    g = run.guard
    g.resume_count = 3
    camera_fixture.camera_gap(run)
    assert g.state == 'PAUSE'
    epoch = g.stationary_epoch
    for _ in range(15):
        run.tick(emit_image=False)
    assert g.state == 'PAUSE' and g.resume_count == 3
    assert g.stationary_epoch == epoch and not g.attempts
    keep_fresh(run)
    assert g.resume_count == 4 and g.stationary_epoch == epoch+1


def test_original_global_wall_budget_is_never_restarted(run):
    g = run.guard
    control_pause(run)
    run.wall = 1200.
    d = g.step(wall_s=run.wall)
    assert (d.linear_x, d.angular_z) == (0., 0.)
    assert g.state in ('STOP', 'FINISHED')
    assert 'global_wall' in g.reason
    assert g.started_wall == 0.


def test_watchdog_and_sensor_callback_enforce_pause_deadline_without_tick(run):
    g = run.guard
    control_pause(run)
    end = run.wall+5.001
    assert g.watchdog_reason(end) == 'three_ground_single_pause_wall_limit'
    g.handle_watchdog(g.watchdog_reason(end), end)
    assert g.state == 'STOP'
    assert g.pause_records[-1]['outcome'].startswith('stopped:')
    assert g.pause_records[-1]['elapsed_wall_s'] == pytest.approx(5.001)


def test_new_resources_do_not_allow_second_recapture_at_same_point(run):
    g = run.guard
    g.recapture_count, g.resume_count = 1, 4
    control_pause(run)
    keep_fresh(run)
    assert g.state in ('STOP', 'FINISHED')
    assert g.reason == 'single_point_recapture_limit'
    assert g.resume_count == 4


def test_actual_factory_keeps_constructed_guard_and_final_dds_freshness(monkeypatch):
    from test_bounded_ground_probe import install_ros_transport
    install_ros_transport(monkeypatch)
    wall = [0.]
    node = budget_api().create_recovery_budget_three_point_node(wall_clock=lambda: wall[0])
    g = node.guard
    assert type(g).__name__ == 'GroundThreePointRecoveryBudgetGuard'
    # Populate the actual constructed instance through the same source helpers;
    # the node and coordinator retain that instance throughout this test.
    monkeypatch.setattr(camera_fixture, 'guard_type', lambda: lambda **kwargs: g)
    live = camera_fixture.StationaryRun()
    assert live.guard is g and node.guard is g
    node.arm_watchdog = lambda: None
    wall[0] = live.wall
    node.last_graph_refresh_wall = wall[0]
    calls = [0]
    def match():
        calls[0] += 1
        if calls[0] == 2:
            camera_fixture.advance_no_image(live, .530038013)
            wall[0] = live.wall
        return 1
    node.publisher_by_topic['/cmd_vel'].get_subscription_count = match
    sent = []
    node.publisher_by_topic['/cmd_vel'].publish = lambda msg: sent.append((msg.linear.x, msg.angular.z))
    node.tick()
    assert node.guard is g and g.state == 'PAUSE'
    assert sent and all(x == w == 0. for x, w in sent)
    assert g.pause_records and g.pause_records[0]['reason'] == 'single_point_uav_image_stale'
    node.close_watchdog()
