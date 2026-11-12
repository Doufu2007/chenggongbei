"""Independent narrow review probes; never runs ROS, uploads or changes source."""
from __future__ import annotations
import ast
import dataclasses
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
import zipfile

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/inspection_mission')]
import pytest
import test_ground_three_point_camera_recovery as fx
from test_bounded_ground_probe import install_ros_transport, scan
from test_ground_observation_survey import posed_odom
from inspection_mission.ground_three_point_camera_recovery import (
    GroundThreePointCameraRecoveryGuard as CameraGuard, create_camera_recovery_three_point_node)
from inspection_mission.ground_three_point_recovery_budget import (
    GroundThreePointRecoveryBudgetGuard as BudgetGuard, create_recovery_budget_three_point_node)
from inspection_mission.ground_three_point_axis import AxisThreePointLimits

RESULTS = {'v36': [], 'v38': [], 'static': []}
def check(group, name, fn):
    try:
        detail = fn()
        RESULTS[group].append({'name': name, 'passed': True, 'detail': detail})
    except Exception:
        RESULTS[group].append({'name': name, 'passed': False, 'traceback': traceback.format_exc()})

def run(cls):
    fx.guard_type = lambda: cls
    return fx.StationaryRun()

def zeros(d):
    assert d.linear_x == d.angular_z == 0.

def hashes_and_ast():
    manifest = json.loads((ROOT/'runs/review/ground_camera_recovery_v36/freeze_manifest.json').read_text())
    rows = manifest['files']+manifest['frozen_parent_files']
    for r in rows:
        assert hashlib.sha256((ROOT/r['path']).read_bytes()).hexdigest() == r['sha256'], r['path']
    for rel, expected in {
        'src/inspection_mission/inspection_mission/ground_three_point_recovery_budget.py': 'f50c7058d8a6727ccac83d03c228d6a2c9e01249c55171aa55758a0b272cdc1d',
        'scripts/run_ground_three_point_recovery_budget.sh': 'ae32f4c32c20b1071c0d2eb7426f50c1748a9b9cb415d4547d2662f964fd24ce',
    }.items():
        assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() == expected, rel
    def tail(rel, name):
        tree = ast.parse((ROOT/rel).read_text(encoding='utf-8'))
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
        index = next(i for i, n in enumerate(fn.body) if isinstance(n, ast.Assign)
                     and any(isinstance(t, ast.Name) and t.id == 'node' for t in n.targets))
        return [ast.dump(n, include_attributes=False) for n in fn.body[index+1:]]
    prefix = 'src/inspection_mission/inspection_mission/'
    a = tail(prefix+'ground_three_point_axis.py', 'create_axis_three_point_node')
    b = tail(prefix+'ground_three_point_camera_recovery.py', 'create_camera_recovery_three_point_node')
    assert a == b
    return {'frozen_file_hashes': len(rows)+2, 'factory_tail_ast_statements_equal': len(a)}

def archive():
    p = ROOT/'runs/review/ground_recovery_budget_v38/package_manifest.json'
    m = json.loads(p.read_text())
    base, new = ROOT/m['base_archive'], ROOT/m['archive']
    assert hashlib.sha256(base.read_bytes()).hexdigest() == m['base_sha256']
    assert hashlib.sha256(new.read_bytes()).hexdigest() == m['archive_sha256']
    with zipfile.ZipFile(base) as b, zipfile.ZipFile(new) as n:
        bn, nn = b.namelist(), n.namelist()
        assert len(nn) == len(set(nn))
        assert set(nn)-set(bn) == set(m['added'])
        for name in bn: assert b.read(name) == n.read(name), name
        for name, expected in m['added'].items():
            assert hashlib.sha256(n.read(name)).hexdigest() == expected
    script = (ROOT/'scripts/run_ground_three_point_recovery_budget.sh').read_text()
    dist = (ROOT/'dist/run_three_point_recovery_v38.sh').read_text()
    lines = dist.splitlines(keepends=True)
    assert len(lines) == len(script.splitlines())+1
    assert lines[1].startswith('export INSPECTION_DIAGNOSTIC_URL=')
    assert ''.join(lines[:1]+lines[2:]) == script
    return {'unchanged_entries': len(bn), 'added_entries': list(m['added']),
            'archive_sha256': m['archive_sha256'], 'launcher_only_extra_diagnostic_env': True,
            'launcher_sha256': hashlib.sha256((ROOT/'dist/run_three_point_recovery_v38.sh').read_bytes()).hexdigest()}

def camera_boundary():
    r = run(CameraGuard); g = r.guard
    fx.advance_no_image(r, .5)
    zeros(g.step(wall_s=r.wall)); assert g.state == 'RECOGNIZE_WAIT', g.reason
    fx.advance_no_image(r, .00001)
    zeros(g.step(wall_s=r.wall)); assert g.state == 'PAUSE'
    return {'accepted_wall_age': .5, 'paused_wall_age': .50001}

def invalid_priority():
    for kind in ('negative_delta', 'future_arrival', 'missing_image'):
        r = run(CameraGuard); g = r.guard
        if kind == 'negative_delta': g.last_uav_image = (r.wall-1., g.clock[0]+.12001)
        elif kind == 'future_arrival': g.last_uav_image = (r.wall+.001, g.clock[0]-.6)
        else: g.last_uav_image = None
        zeros(g.step(wall_s=r.wall)); assert g.state == 'STOP', (kind, g.reason)
        assert not g.camera_recovery_pending and not g.attempts
    return {'terminal_variants': 3}

def scaled_sources(r, step=.01):
    g = r.guard; stamp = round(g.clock[0]+step, 9)
    r.wall = round(r.wall+.05, 9)
    g.update_clock(stamp, wall_s=r.wall)
    payload = posed_odom(stamp, x=r.x, yaw=r.yaw)
    payload['pose']['pose']['position']['y'] = r.y
    g.update_odom(payload, wall_s=r.wall); g.update_scan(scan(stamp), wall_s=r.wall)
    uav = posed_odom(stamp, x=r.x, yaw=r.yaw); uav['child_frame_id'] = 'uav/base_link'
    uav['pose']['pose']['position']['y'] = r.y
    g.observe_public('/uav/odom', uav, wall_s=r.wall)
    g.observe_image('/uav/camera/image', fx.image(stamp), wall_s=r.wall)
    return g.step(wall_s=r.wall)

def dual_clock(cls):
    r = run(cls); g = r.guard; epoch = g.stationary_epoch
    fx.camera_gap(r)
    for _ in range(24):
        zeros(scaled_sources(r)); assert g.state == 'PAUSE', g.reason
    assert g.resume_count == 0 and g.stationary_epoch == epoch
    for _ in range(20):
        zeros(scaled_sources(r, .05))
        if g.state != 'PAUSE': break
    assert g.state == 'CAPTURE' and g.resume_count == 1 and g.stationary_epoch == epoch+1
    assert not g.expected_jobs and not g.capture_frames and not g.match_results
    return {'wall_only_dwell_s': 1.2, 'source_dwell_s': .23, 'new_epoch': g.stationary_epoch}

def factory(cls, create):
    with pytest.MonkeyPatch.context() as mp:
        install_ros_transport(mp); wall = [0.]
        node = create(wall_clock=lambda: wall[0]); g = node.guard
        try:
            assert type(g) is cls
            fx.guard_type = lambda: lambda **kw: g
            live = fx.StationaryRun(); assert node.guard is live.guard is g
            node.arm_watchdog = lambda: None
            wall[0] = live.wall; node.last_graph_refresh_wall = wall[0]
            calls = [0]; sent = []
            def matched():
                calls[0] += 1
                if calls[0] == 2:
                    fx.advance_no_image(live, .530038013); wall[0] = live.wall
                return 1
            node.publisher_by_topic['/cmd_vel'].get_subscription_count = matched
            node.publisher_by_topic['/cmd_vel'].publish = lambda msg: sent.append((msg.linear.x, msg.angular.z))
            node.tick()
            assert g.state == 'PAUSE' and node.guard is g
            assert sent and all(x == w == 0. for x, w in sent)
            return {'retained_constructed_instance': True, 'transports': sent, 'final_dds_gap_s': .530038013}
        finally: node.close_watchdog()

def factory_rejects():
    with pytest.MonkeyPatch.context() as mp:
        install_ros_transport(mp); limits = AxisThreePointLimits()
        with pytest.raises(ValueError, match='same frozen contract'):
            create_camera_recovery_three_point_node(route_limits=limits,
                guard_factory=lambda **kw: CameraGuard(started_wall_s=kw['started_wall_s']+.001, route_limits=limits))
        with pytest.raises(ValueError, match='same frozen contract'):
            create_camera_recovery_three_point_node(route_limits=limits,
                guard_factory=lambda **kw: CameraGuard(started_wall_s=kw['started_wall_s']))
    return {'mismatched_start_and_limit_instances_rejected': 2}

def motion_run(phase):
    r = run(BudgetGuard); g = r.guard
    g.stationary_anchor = g.stationary_uav_anchor = None
    g.epoch_invalid = None; g.phase = g.state = phase
    g.turn_origin = None; g.initial_turn_error = .05
    if phase == 'DRIVE':
        g.drive_origin = (r.x-.005*math.cos(r.yaw), r.y-.005*math.sin(r.yaw), g.odom['xyz'][2])
        g.drive_yaw = r.yaw
        g.approach_limits = dataclasses.replace(g.approach_limits, target_distance_m=.004)
        g.leg_records.append({'origin':g.drive_origin, 'yaw_rad':r.yaw, 'started_wall_s':r.wall})
    else: g.drive_origin = g.drive_yaw = None
    return r

def phase_transition(phase):
    r = motion_run(phase); g = r.guard
    g.temporal_fault('odom_stale_or_invalid', r.wall)
    r.wall += .05; r.sensors()
    assert g.state == 'PAUSE' and g.phase == phase+'_SETTLE', (g.state, g.phase, g.reason)
    for _ in range(24):
        d = g.step(wall_s=r.wall); zeros(d)
        if g.state != 'PAUSE': break
        r.wall = round(r.wall+.05,9); r.sensors()
    assert g.state == phase+'_SETTLE' and g.resume_count == 1, (g.state,g.reason)
    return {'phase_after_odom': g.phase, 'resume_command': [d.linear_x,d.angular_z]}

def quota_real_count():
    r = motion_run('TURN'); g = r.guard
    for expected in range(1,5):
        g.temporal_fault('odom_stale_or_invalid', r.wall)
        for _ in range(24):
            zeros(r.tick())
            if g.state != 'PAUSE': break
        assert g.resume_count == expected and g.state not in ('STOP','FINISHED'), g.reason
    assert len(g.pause_records) == 4 and all(row['outcome'] == 'recovered' for row in g.pause_records)
    assert [row['resume_count_after'] for row in g.pause_records] == [1,2,3,4]
    g.resume_count = 31
    g.temporal_fault('odom_stale_or_invalid',r.wall)
    for _ in range(24):
        zeros(r.tick())
        if g.state != 'PAUSE': break
    assert g.resume_count == 32
    g.temporal_fault('odom_stale_or_invalid',r.wall)
    for _ in range(24):
        zeros(r.tick())
        if g.state != 'PAUSE': break
    assert g.state == 'STOP' and g.resume_count == 32 and 'resume_resource_limit' in g.reason
    e = g.approach_evidence()
    assert e['resume_count'] == e['recovery_resource']['resume_count'] == 32
    assert all(e[k] == 32 for k in ('max_resumes',))
    return {'four_independent_resumes': 4, 'seeded_boundary_last_granted': 32, 'next_request': g.reason}

def callback_deadline(topic):
    r = motion_run('TURN'); g=r.guard
    g.temporal_fault('odom_stale_or_invalid',r.wall); start=r.wall
    # Repeated faults may change reason, never the first wall deadline.
    g.temporal_fault('clock_stale',r.wall+4.)
    assert g.pause_started_wall == start
    end = start+5.
    if topic == 'clock': g.update_clock(g.clock[0]+5., wall_s=end)
    elif topic == 'odom': g.update_odom(posed_odom(g.clock[0]+5.,x=r.x,yaw=r.yaw),wall_s=end)
    else: g.update_scan(scan(g.clock[0]+5.),wall_s=end)
    assert g.state == 'STOP' and g.reason == 'three_ground_single_pause_wall_limit', g.reason
    assert g.pause_records[-1]['elapsed_wall_s'] == 5. and g.pause_started_wall is None
    return {'callback': topic, 'deadline': 5., 'no_tick': True}

def cumulative_boundary():
    r=motion_run('TURN');g=r.guard
    g.completed_pause_wall_s=59.4
    g.temporal_fault('odom_stale_or_invalid',r.wall); start=r.wall
    assert g.watchdog_reason(start+.599) != 'three_ground_cumulative_pause_wall_limit'
    reason=g.watchdog_reason(start+.6)
    assert reason=='three_ground_cumulative_pause_wall_limit'
    g.handle_watchdog(reason,start+.6)
    assert g.state=='STOP' and g.completed_pause_wall_s==pytest.approx(60.)
    e=g.approach_evidence()['recovery_resource']
    assert e['total_pause_wall_s']==pytest.approx(60.) and e['active_pause_wall_s']==0.
    return {'seeded_completed_s':59.4,'active_to_stop_s':.6,'total_s':e['total_pause_wall_s']}

def terminal_camera():
    for phase, attempt in [('REPORT_WAIT',False),('RECOGNIZE_WAIT',True)]:
        r=run(BudgetGuard);g=r.guard
        g.phase=g.state=phase
        if attempt:g.attempts['recognition']={'attempted':True,'sent':False}
        fx.camera_gap(r); assert g.state=='STOP'
        assert g.resume_count==0 and not g.pause_records
        for _ in range(4): zeros(r.tick())
        assert g.state in ('STOP','FINISHED')
    return {'terminal_no_retry_variants':2}

check('static','frozen_hashes_factory_ast',hashes_and_ast)
check('static','archive_and_launcher',archive)
check('v36','camera_half_second_boundary',camera_boundary)
check('v36','invalid_time_priority_and_missing_source',invalid_priority)
check('v36','dual_clock_dwell_with_slow_source',lambda:dual_clock(CameraGuard))
check('v36','actual_constructed_factory_final_dds',lambda:factory(CameraGuard,create_camera_recovery_three_point_node))
check('v36','factory_rejects_changed_start_or_contract',factory_rejects)
check('v38','turn_phase_changes_during_pause',lambda:phase_transition('TURN'))
check('v38','drive_phase_changes_during_pause',lambda:phase_transition('DRIVE'))
check('v38','dual_clock_dwell_with_slow_source',lambda:dual_clock(BudgetGuard))
check('v38','actual_constructed_factory_final_dds',lambda:factory(BudgetGuard,create_recovery_budget_three_point_node))
check('v38','real_resumes_and_32_boundary',quota_real_count)
for topic in ('clock','odom','scan'):
    check('v38','five_second_deadline_'+topic+'_callback',lambda topic=topic:callback_deadline(topic))
check('v38','cumulative_sixty_second_watchdog_boundary',cumulative_boundary)
check('v38','camera_after_attempt_is_terminal',terminal_camera)
for group, rows in RESULTS.items():
    out = ROOT/'runs/review'/('ground_recovery_budget_independent_20261112' if group=='v38' else 'ground_camera_recovery_independent_20261112')
    out.mkdir(parents=True,exist_ok=True)
    (out/(group+'_probe_results.json')).write_text(json.dumps({'group':group,'passed':sum(x['passed'] for x in rows),'total':len(rows),'checks':rows},ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(RESULTS,ensure_ascii=False,indent=2))
raise SystemExit(0 if all(r['passed'] for rows in RESULTS.values() for r in rows) else 1)
