"""v40 independent return contracts; synthetic feedback is not platform proof."""
import importlib
import importlib.util
import itertools
import math

import pytest

from test_bounded_ground_probe import endpoint, install_ros_transport
from test_ground_bounded_approach import refresh
from test_ground_single_point import camera_info
from test_ground_three_point import ThreeRun


def plan_api():
    name = 'inspection_mission.ground_return_plan'
    assert importlib.util.find_spec(name), 'independent ground return planner missing'
    return importlib.import_module(name)


def test_all_60_nominal_draws_include_safe_return_and_fit_new_contract():
    from inspection_mission.ground_three_point_axis_plan import plan_ground_leg
    ids = ('ground_0', 'ground_1', 'ground_2', 'ground_3', 'ground_4')
    paths, angles = [], []
    for draw in itertools.permutations(ids, 3):
        pose, previous, path, angle = (0., 0., 0.), None, 0., 0.
        for target in draw:
            leg = plan_ground_leg(previous, target, pose)
            path += leg.path_m
            angle += leg.rotation_rad
            previous, pose = target, (*leg.endpoint_xy_m, leg.endpoint_yaw_rad)
        back = plan_api().plan_ground_return(pose)
        assert back.initial_pose == pose and back.home_pose == (0., 0., 0.)
        assert back.minimum_laser_distance_m >= .641 - 1e-12
        paths.append(path + back.path_m)
        angles.append(angle + back.rotation_rad)
    assert len(paths) == 60
    assert max(paths) == pytest.approx(13.99)
    assert max(angles) == pytest.approx(37.752432539826714)


def test_return_turn_anchor_precedes_north_reverse_and_preserves_terminal_heading():
    plan = plan_api().plan_ground_return((0., -2.69, math.pi))
    drives = [op for op in plan.operations if op.kind.startswith('DRIVE')]
    assert [op.target_xy_m for op in drives] == [(.6, -2.69), (.6, -.2), (.6, 0.), (0., 0.)]
    assert drives[2].kind == 'DRIVE_REVERSE_TO_XY' and drives[2].length_m == pytest.approx(.2)
    assert drives[2].target_yaw_rad == pytest.approx(-math.pi / 2)
    assert plan.operations[-1].target_yaw_rad == pytest.approx(0.)


def test_true_south_terminal_error_is_rejected_by_original_ring():
    with pytest.raises(ValueError, match='unsafe candidate initial laser ring'):
        plan_api().plan_ground_return((0., -2.71, math.pi))


def api():
    name = 'inspection_mission.ground_three_point_return'
    assert importlib.util.find_spec(name), 'independent v40 return guard missing'
    return importlib.import_module(name)


def third_guard(*, target='ground_3', actual_y=None):
    from inspection_mission.ground_three_point_axis_plan import plan_ground_leg
    coordinates = {'ground_3': (1.115, -1.925, -math.pi / 2), 'ground_2': (0., -2.69, math.pi)}
    x, y, yaw = coordinates[target]
    y = y if actual_y is None else actual_y
    g = api().GroundThreePointReturnGuard(started_wall_s=0., session_id='same-return-session')
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    refresh(g, 2.1, x=x, y=y, yaw=yaw)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = g.odom['xyz'], yaw
    g.mission_draw, g.point_index, g.target_id = ('ground_0', 'ground_1', target), 2, target
    g.point_plan = plan_ground_leg(None, target, (0., 0., 0.))
    g.completed_points = [{'point_index': 0}, {'point_index': 1}]
    g.phase = g.state = 'DECLARE_WAIT'
    g.in_entry = True
    g.attempts = {'recognition': {'sent': True}, 'declaration': {'sent': True, 'wall_s': 0.}}
    g.total_path_m, g.accumulated_rotation_rad = 7., 20.
    g.turn_elapsed_wall_s, g.drive_elapsed_wall_s = 300., 250.
    g.resume_count, g.completed_pause_wall_s = 4, 6.
    for i in range(1, 23):
        refresh(g, 2.1 + i * .05, x=x, y=y, yaw=yaw)
        g._stable_dwell(2.1 + i * .05)
    return g, 3.2


def settled_guard():
    g = api().GroundThreePointReturnGuard(started_wall_s=0.)
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    refresh(g, 1.)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = g.odom['xyz'], 0.
    g.return_started = True
    g.return_plan = plan_api().plan_ground_return((1.115, -1.925, -math.pi / 2))
    g.current_operations_role = 'return'
    g.phase = g.state = 'RETURN_SETTLE'
    return g


def settle(g, start=1., seconds=1.1, **pose):
    for i in range(round(seconds / .05) + 1):
        wall = round(start + i * .05, 9)
        refresh(g, wall, **pose)
        decision = g.step(wall_s=wall)
        assert (decision.linear_x, decision.angular_z) == (0., 0.)
    return wall


def zero_attempt(g, wall):
    return {'wall_elapsed_s': wall - g.started_wall, 'returned_wall_elapsed_s': wall - g.started_wall,
            'phase': 'RETURN_SETTLE', 'state': 'RETURN_SETTLE',
            'linear_x': 0., 'angular_z': 0., 'returned': True, 'error': None}


def test_new_limits_enforce_caps_and_keep_frozen_parent_contract():
    from inspection_mission.ground_three_point_axis import AxisThreePointLimits
    original = AxisThreePointLimits()
    assert (original.max_path_m, original.max_rotation_rad, original.global_wall_s) == (12.5, 28., 1200.)
    limits = api().ReturnThreePointLimits()
    assert (limits.max_path_m, limits.max_rotation_rad, limits.global_wall_s,
            limits.turn_total_wall_s, limits.drive_wall_s) == (15., 40., 1800., 1000., 650.)
    assert api().ReturnThreePointLimits(target_distance_m=2.5).target_distance_m == 2.5
    for field, value in (('max_path_m', 15.001), ('max_rotation_rad', 40.001),
                         ('global_wall_s', 1800.001), ('turn_total_wall_s', 1000.001),
                         ('drive_wall_s', 650.001), ('linear_command_mps', .1001)):
        with pytest.raises(ValueError):
            api().ReturnThreePointLimits(**{field: value})


def test_third_handoff_archives_once_and_preserves_all_permanent_ledgers():
    g, wall = third_guard()
    before = (g.session_id, g.started_wall, g.global_origin, g.total_path_m,
              g.accumulated_rotation_rad, g.turn_elapsed_wall_s, g.drive_elapsed_wall_s,
              g.resume_count, g.completed_pause_wall_s)
    g._handoff(wall)
    assert g.return_started and g.current_operations_role == 'return' and g.state not in ('STOP', 'FINISHED')
    assert len(g.completed_points) == 3 and g.return_plan.initial_pose == pytest.approx((1.115, -1.925, -math.pi / 2))
    assert before == (g.session_id, g.started_wall, g.global_origin, g.total_path_m,
                      g.accumulated_rotation_rad, g.turn_elapsed_wall_s, g.drive_elapsed_wall_s,
                      g.resume_count, g.completed_pause_wall_s)
    g._handoff(wall)
    assert len(g.completed_points) == 3
    assert g.dispatch_one(lambda *_: pytest.fail('return cannot declare'), wall_s=wall) is None
    assert not g.accept_match_result({'matches_current_context': True}, wall_s=wall)
    assert not g.register_prepared_jobs([], epoch=g.stationary_epoch)


@pytest.mark.parametrize('fault', ['not_sent', 'path', 'rotation', 'unsafe_pose'])
def test_return_refuses_missing_declaration_exhausted_budget_and_unsafe_actual_pose(fault):
    g, wall = third_guard(target='ground_2', actual_y=-2.71) if fault == 'unsafe_pose' else third_guard()
    if fault == 'not_sent':
        g.attempts['declaration']['sent'] = False
    elif fault == 'path':
        g.total_path_m = 14.99
    elif fault == 'rotation':
        g.accumulated_rotation_rad = 39.99
    g._handoff(wall)
    assert g.state == 'STOP' and not g.return_started
    assert g.reason.startswith(('return_third_declaration_gate', 'three_ground_remaining_', 'return_candidate_plan_rejected'))
    if fault != 'not_sent':
        assert len(g.completed_points) == 3


def test_return_completion_needs_dual_clock_dwell_and_successful_fresh_zero_return():
    g = settled_guard()
    wall = settle(g)
    assert g.return_ready and not g.return_completed and g.state == 'RETURN_SETTLE'
    failed = zero_attempt(g, wall)
    failed['returned'] = False
    assert not g.confirm_return_zero(failed, wall_s=wall)
    assert not g.return_completed
    assert g.confirm_return_zero(zero_attempt(g, wall), wall_s=wall)
    assert g.return_completed and g.state == 'STOP'
    assert g.reason == 'three_ground_return_complete_official_score_unverified'
    assert g.approach_evidence()['ground_return']['full_mission_complete'] is False


@pytest.mark.parametrize('pose', [{'x': .02001}, {'yaw': .02501}, {'speed': .011}])
def test_return_wrong_endpoint_or_unstable_motion_never_earns_completion(pose):
    g = settled_guard()
    wall = settle(g, **pose)
    assert not g.return_ready and not g.confirm_return_zero(zero_attempt(g, wall), wall_s=wall)


def test_return_source_clock_must_progress_and_bad_frame_erases_ready_dwell():
    g = settled_guard()
    wall = settle(g)
    assert g.return_ready
    # A real fresh sample may advance wall time while source dwell is still short.
    anchor = g.stable_anchor
    g.stable_anchor = (anchor[0] - 2., g.odom['stamp'] - .5, anchor[2], anchor[3])
    g.step(wall_s=wall)
    assert not g.return_ready
    g.step(wall_s=wall + .201)
    assert g.state == 'PAUSE' and not g.return_ready and g.stable_anchor is None
    assert g.resume_count == 0 and g.pause_started_wall is not None


def test_factory_constructs_new_guard_and_final_zero_publish_failure_cannot_complete(monkeypatch):
    install_ros_transport(monkeypatch)
    wall = [0.]
    node = api().create_return_three_point_node(wall_clock=lambda: wall[0])
    node.arm_watchdog = lambda: None
    assert isinstance(node.guard, api().GroundThreePointReturnGuard)
    g = node.guard
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    refresh(g, 0.)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = g.odom['xyz'], 0.
    g.return_started = True
    g.current_operations_role = 'return'
    g.phase = g.state = 'RETURN_SETTLE'
    sent = []
    def publish(msg):
        sent.append((msg.linear.x, msg.angular.z))
        raise RuntimeError('zero transport failed')
    publisher = node.publisher_by_topic['/cmd_vel']
    for i in range(23):
        wall[0] = i * .05
        refresh(g, wall[0])
        g.step(wall_s=wall[0])
    publisher.publish = publish
    node.last_graph_refresh_wall = wall[0]
    node.tick()
    assert sent and all(x == w == 0. for x, w in sent)
    assert g.state == 'STOP' and not g.return_completed
    for topic in ('/uav/cmd_vel', '/uav/laser_fire', '/referee/cruise_event'):
        assert node.command_endpoints[topic]['silent'] is True
    node.close_watchdog()


class ReturnRun(ThreeRun):
    def __init__(self, draw=('ground_0', 'ground_3', 'ground_2')):
        super().__init__(draw)
        self.guard = api().GroundThreePointReturnGuard(started_wall_s=0., session_id='test-current-session')
        self.guard.update_endpoints([endpoint()], matched_subscriptions=1)
        self.guard.observe_public('/referee/ground_tasks', {'data': ','.join(draw)}, wall_s=0.)
        self.guard.observe_public('/uav/camera/camera_info', camera_info(), wall_s=0.)


def test_synthetic_three_points_really_drive_home_without_fourth_declaration():
    run = ReturnRun(('ground_4', 'ground_3', 'ground_0'))
    sent = []
    for i in range(3):
        sent.extend(run.recognise())
        sent.extend(run.declare())
        if i < 2:
            run.until(lambda r: r.guard.point_index == i + 1)
    before = (run.guard.total_path_m, run.guard.accumulated_rotation_rad, run.guard.global_origin)
    run.until(lambda r: r.guard.return_ready, cap=30000)
    assert math.hypot(run.x, run.y) <= .02
    assert abs(math.atan2(math.sin(run.yaw), math.cos(run.yaw))) <= .025
    assert run.guard.total_path_m > before[0] and run.guard.accumulated_rotation_rad > before[1]
    assert run.guard.global_origin == before[2] and len(run.guard.completed_points) == 3
    assert len(sent) == 6 and len([row for row in sent if row[0] == '/referee/current_target']) == 3
    assert run.guard.dispatch_one(lambda *_: pytest.fail('fourth declaration'), wall_s=run.wall) is None
    assert run.guard.confirm_return_zero(zero_attempt(run.guard, run.wall), wall_s=run.wall)
    evidence = run.guard.approach_evidence()['ground_return']
    assert evidence['complete'] is True and evidence['official_acceptance'] is False


def test_actual_synthetic_turn_ledger_can_exhaust_40rad_before_return():
    run = ReturnRun(('ground_0', 'ground_3', 'ground_2'))
    for i in range(3):
        run.recognise()
        run.declare()
        if i < 2:
            run.until(lambda r: r.guard.point_index == i + 1)
    before_path, before_angle = run.guard.total_path_m, run.guard.accumulated_rotation_rad
    assert before_angle > 27.7  # Actual feedback, never replaced by nominal geometry.
    for _ in range(45):
        run.tick()
        if run.guard.state == 'STOP':
            break
    assert run.guard.state == 'STOP' and not run.guard.return_started
    assert run.guard.reason == 'three_ground_remaining_rotation_budget_insufficient'
    assert len(run.guard.completed_points) == 3
    assert run.guard.total_path_m >= before_path and run.guard.accumulated_rotation_rad >= before_angle
    assert run.guard.frozen_limits.max_rotation_rad == 40.


def test_actual_factory_records_completion_only_after_successful_zero_publish(monkeypatch):
    install_ros_transport(monkeypatch)
    wall = [0.]
    node = api().create_return_three_point_node(wall_clock=lambda: wall[0])
    node.arm_watchdog = lambda: None
    g = node.guard
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    refresh(g, 0.)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = g.odom['xyz'], 0.
    g.return_started = True
    g.current_operations_role = 'return'
    g.phase = g.state = 'RETURN_SETTLE'
    sent = []
    node.publisher_by_topic['/cmd_vel'].publish = lambda msg: sent.append((msg.linear.x, msg.angular.z))
    for i in range(23):
        wall[0] = i * .05
        refresh(g, wall[0])
        node.last_graph_refresh_wall = wall[0]
        node.tick()
        if g.return_completed:
            break
    assert node.guard is g and g.return_completed and g.state == 'STOP'
    assert sent and all(x == w == 0. for x, w in sent)
    attempt = g.return_completion['zero_publish_attempt']
    assert attempt['returned'] is True and attempt['error'] is None
    node.close_watchdog()


def test_zero_publish_return_that_consumes_freshness_cannot_latch_completion():
    g = settled_guard()
    wall = settle(g)
    assert not g.confirm_return_zero(zero_attempt(g, wall), wall_s=wall + .201)
    assert g.state == 'PAUSE' and not g.return_completed and not g.return_ready
    assert g.pause_started_wall is not None


def test_return_core_event_is_receiver_valid_and_retains_frozen_budget_contract():
    from inspection_mission.telemetry import DiagnosticReporter
    from tools.diagnostic_receiver import validate_event
    g = settled_guard()
    wall = settle(g)
    assert g.confirm_return_zero(zero_attempt(g, wall), wall_s=wall)
    events = [validate_event(DiagnosticReporter._encode(event, data))['data']
              for event, data in g.additional_events()]
    core = next(row for row in events if row.get('section') == 'ground_return_core')
    assert core['complete'] is True and core['official_acceptance'] is False
    assert core['full_mission_complete'] is False and core['mode'] == 'ground_three_point_return'
    three = next(row for row in events if row.get('section') == 'three_point_core')
    assert three['frozen_limits']['max_rotation_rad'] == 40.


@pytest.mark.parametrize('exit_code', [0, 19])
def test_return_launcher_runs_independent_module_and_preserves_exit(tmp_path, exit_code):
    import os
    import shlex
    import shutil
    import subprocess
    import sys
    from pathlib import Path
    from test_ground_probe_launch import BASH, shell_path
    if not BASH.is_file():
        pytest.skip('Git Bash unavailable')
    root = tmp_path / 'return uploaded workspace'
    package = root / 'src/inspection_mission/inspection_mission'
    package.mkdir(parents=True)
    (package / '__init__.py').write_text('', encoding='utf-8')
    (package / 'ground_three_point_return.py').write_text(
        'import os,sys\nassert os.environ["RETURN_FAKE_ROS"]=="1"\n'
        f'assert sys.argv[-1]=="1800"\nsys.exit({exit_code})\n', encoding='utf-8')
    source = Path(__file__).resolve().parents[1] / 'scripts/run_ground_three_point_return.sh'
    assert source.is_file(), 'independent v40 return launcher missing'
    launcher = root / 'scripts/run_ground_three_point_return.sh'
    launcher.parent.mkdir()
    shutil.copyfile(source, launcher)
    setup = tmp_path / 'ros setup.sh'
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
                     + 'export RETURN_FAKE_ROS=1\n', encoding='utf-8')
    env = dict(os.environ, INSPECTION_ROS_SETUP=shell_path(setup))
    env.pop('INSPECTION_DIAGNOSTIC_URL', None)
    done = subprocess.run([str(BASH), str(launcher)], cwd=tmp_path, env=env, capture_output=True,
                          text=True, encoding='utf-8', timeout=10)
    assert done.returncode == exit_code, done.stdout + done.stderr
    assert f'GROUND_THREE_POINT_RETURN launch_exit exit_code={exit_code}' in done.stderr
