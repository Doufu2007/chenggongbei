"""v37 fixed static column only; simulated inputs are not platform evidence."""
import base64
import dataclasses
import gzip
import hashlib
import importlib
import importlib.util
import json
import math
import sys
from types import SimpleNamespace
import pytest

from test_bounded_vertical_response import odom, tf_static, update, ready
from test_bounded_ground_probe import scan, install_ros_transport

def api():
    name='inspection_mission.bounded_cruise_height'
    assert importlib.util.find_spec(name) is not None, 'independent cruise module missing'
    return importlib.import_module(name)

def guard(): return api().CruiseHeightGuard(started_wall_s=0.)

def step(g,t):
    d=g.step(wall_s=t);g.record_command(wall_s=t,linear_z=d.linear_z);return d

def feed(g,start,end,z=.85,vz=0.):
    d=None
    for i in range(round(start*10),round(end*10)+1):
        update(g,i/10,z=z,vz=vz);d=step(g,i/10)
    return d

def hold(g):
    assert ready(g).linear_z==.05
    g.record_command(wall_s=1.1,linear_z=.05)
    update(g,1.2,z=.85);assert step(g,1.2).linear_z==0.
    update(g,1.3,z=.85);assert step(g,1.3).linear_z==pytest.approx(.02695)
    assert g.phase=='HOLD_CRUISE'

def test_independent_fixed_limits_and_parent_contract_unchanged():
    m=api();lim=m.CruiseHeightLimits()
    assert (lim.target_absolute_z_m,lim.hard_absolute_z_m,lim.motion_wall_s,lim.global_wall_s,lim.max_total_path_m)==(.85,1.,90.,110.,2.)
    assert (lim.sensor_age_s,lim.clock_sampling_lag_s,lim.max_tilt_rad,lim.max_horizontal_drift_m)==(.20,.12,.05,.04)
    from inspection_mission.bounded_height_hold import HEIGHT_LIMITS
    assert (HEIGHT_LIMITS.target_rise_m,HEIGHT_LIMITS.hard_rise_m,HEIGHT_LIMITS.motion_wall_s)==(.10,.15,25.)
    for key,value in [('target_absolute_z_m',.86),('hard_absolute_z_m',1.01),('sensor_age_s',.21),('motion_wall_s',91.),('max_total_path_m',2.1)]:
        with pytest.raises(ValueError):m.CruiseHeightLimits(**{key:value})

def test_static_column_includes_full_upper_body_but_stays_below_public_max_18():
    m=api();r=m.cruise_column_geometry((0.,0.,.055))
    assert r['clear'] and r['radius_m']==pytest.approx(.515296542099)
    assert r['top_m']==pytest.approx(1.208340688664) and r['top_m']<r['public_plugin_max_z_m']==1.8
    assert not m.cruise_column_geometry((0.,.2,.055))['clear']

def test_low_finite_scan_semantics_equal_frozen_height_parent():
    from inspection_mission.bounded_height_hold import HeightHoldGuard
    old=HeightHoldGuard(started_wall_s=0.);new=guard()
    ready(old);ready(new)
    for i in range(12,15):
        t=i/10;update(old,t,z=.07);update(new,t,z=.07)
        assert old.step(wall_s=t).linear_z==new.step(wall_s=t).linear_z==.05
        assert old.scan==new.scan
    assert new.initial_column_validation['clear']

def test_static_column_scan_can_keep_positive_inf_without_fabricating_ranges():
    g=guard();ready(g);g.update_clock(11.2,wall_s=1.2);g.update_odom(odom(11.2,z=.40),wall_s=1.2)
    payload=scan(11.2);payload['ranges'][0]=math.inf
    g.update_scan(payload,wall_s=1.2)
    assert g.step(wall_s=1.2).linear_z==.05
    assert math.isinf(g.last_scan_payload['ranges'][0])
    assert g.scan_semantics[-1]['positive_inf_fraction']==pytest.approx(1/360)
    payload=scan(11.3);payload['ranges']=[math.inf]*360
    g.update_clock(11.3,wall_s=1.3);g.update_odom(odom(11.3,z=.42),wall_s=1.3);g.update_scan(payload,wall_s=1.3)
    assert g.step(wall_s=1.3).linear_z==.05
    assert g.scan_semantics[-1]['finite_min_m'] is None
    assert g.scan_semantics[-1]['positive_inf_fraction']==1.

@pytest.mark.parametrize('bad',[math.nan,-math.inf,.640,.09,13.])
def test_static_inf_policy_still_rejects_invalid_or_near_finite_returns(bad):
    g=guard();ready(g);payload=scan(11.2);payload['ranges'][0]=bad
    g.update_scan(payload,wall_s=1.2)
    assert g.state=='STOP' and 'scan_invalid' in g.reason

def test_inf_before_static_column_validation_cannot_arm_outside_initial_column():
    g=guard();g.update_endpoints([{'node':'/external','type':'geometry_msgs/msg/Twist'}],matched_subscriptions=1)
    g.update_tf(tf_static(),wall_s=0.)
    for i in range(12):
        t=i/10;source=10+t;g.update_clock(source,wall_s=t);g.update_odom(odom(source,y=.2),wall_s=t)
        g.update_ground_odom(odom(source,y=.2,child='base_footprint'),wall_s=t)
        p=scan(source);p['ranges']=[math.inf]*360;g.update_scan(p,wall_s=t);g.step(wall_s=t)
    assert g.origin is None and g.state=='WAIT'

def test_dual_three_second_hold_then_descent_and_wall_support():
    g=guard();hold(g)
    feed(g,1.4,4.2);assert not g.hold_verified
    assert feed(g,4.3,4.3).linear_z==0. and g.hold_verified
    assert feed(g,4.4,4.4).linear_z==pytest.approx(.01195)
    assert g.phase=='DESCEND_SUPPORT'
    update(g,4.5,z=.079,vz=-.01);assert step(g,4.5).linear_z==0.
    assert g.protocol_complete and g.descent_verified
    feed(g,4.6,5.7,z=.055)
    assert g.state=='FINISHED' and g.support_return_verified
    assert g.origin==(0.,0.,.055) and g.motion_started_wall==1.

@pytest.mark.parametrize('bad',[{'z':.856},{'vz':.011}])
def test_any_bad_hold_frame_cannot_hide_before_tick(bad):
    g=guard();hold(g);feed(g,1.4,3.)
    update(g,3.05,z=bad.get('z',.85),vz=bad.get('vz',0.));update(g,3.1,z=.85);step(g,3.1)
    feed(g,3.2,4.4)
    assert not g.hold_verified and g.hold_dwell_resets

@pytest.mark.parametrize('vz,good',[(-.01,True),(-.021,False)])
def test_descent_same_gate_tick_below_cutoff_never_sends_nonzero(vz,good):
    g=guard();hold(g);feed(g,1.4,4.3)
    update(g,4.4,z=.079,vz=vz);assert step(g,4.4).linear_z==0.
    assert g.phase=='ZERO_RETURN' and g.descent_verified is good

@pytest.mark.parametrize('field,value,reason',[('z',1.,'absolute_height_limit'),('x',.041,'horizontal_drift_limit'),('pitch',.051,'tilt_limit'),('vz',.151,'speed_limit')])
def test_hard_bounds_latch_with_new_height_only(field,value,reason):
    g=guard();ready(g);g.update_odom(odom(11.2,**{field:value}),wall_s=1.2)
    assert g.state=='STOP' and reason in g.reason
    update(g,1.3);assert step(g,1.3).linear_z==0.

def test_hold_and_ascent_deadlines_abort_without_resetting_motion_origin():
    g=guard();hold(g);feed(g,1.4,9.3,z=.86)
    assert g.phase=='ZERO_RETURN' and g.protocol_abort_reason=='hold_wall_time_limit'
    g=guard();ready(g);feed(g,1.2,26.,z=.055)
    assert g.phase=='ZERO_RETURN' and g.protocol_abort_reason=='ascent_wall_time_limit'
    assert g.motion_started_wall==1.

def test_permanent_motion_global_and_zero_return_budgets():
    g=guard();ready(g);g.pause('injected',1.2)
    feed(g,1.3,9.2,z=.2)
    assert g.state=='STOP' and g.reason=='zero_return_wall_time_limit'
    g=guard();assert g.step(wall_s=110.).state=='STOP'
    g=guard();ready(g);g._budget(91.)
    assert g.state=='STOP' and g.reason=='motion_wall_time_limit'

def test_any_nonzero_interruption_permanently_zeroes():
    g=guard();hold(g);g.pause('gap',1.31);feed(g,1.4,1.6)
    assert g.phase=='ZERO_RETURN' and all(x['linear_z']==0. for x in g.commands if x['wall_elapsed_s']>=1.31)

def test_independent_protocol_completes_synthetic_dynamic_high_column_with_inf_ring():
    g=guard();g.update_endpoints([{'node':'/external','type':'geometry_msgs/msg/Twist'}],matched_subscriptions=1)
    g.update_tf(tf_static(),wall_s=0.)
    z=.055;speed=0.;u=0.;seen=set();positive_inf_seen=False
    for i in range(2000):
        t=i*.05;source=round(10+t,9)
        if i:
            speed=2*u-.0539 if z>.055+1e-9 or u>.02695 else 0.
            z=max(.055,z+speed*.05)
            if z<=.055:speed=0.
        g.update_clock(source,wall_s=t);g.update_odom(odom(source,z=z,vz=speed),wall_s=t)
        g.update_ground_odom(odom(source,child='base_footprint'),wall_s=t)
        p=scan(source)
        if z>.32:p['ranges']=[math.inf]*360;positive_inf_seen=True
        g.update_scan(p,wall_s=t);d=step(g,t);u=d.linear_z;seen.add(g.phase)
        if g.state=='FINISHED':break
    assert g.state=='FINISHED' and g.protocol_complete and g.support_return_verified
    assert positive_inf_seen and seen>={'ASCEND_CRUISE','HOLD_CRUISE','DESCEND_SUPPORT','ZERO_RETURN'}
    assert g.total_path_m<2. and t<90. and g.max_rise_m<.965
    assert g.hold_dwell_verified['ended_odom_stamp']-g.hold_dwell_verified['started_odom_stamp']>=3.

def test_new_schema_roundtrip_and_fixed_assumptions():
    g=guard();hold(g)
    parts=[v for _,v in g.diagnostic_events() if v['section']=='bounded_cruise_height_gzip_part']
    packed=base64.b64decode(''.join(p['base64_part'] for p in parts),validate=True);raw=gzip.decompress(packed)
    assert hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
    b=json.loads(raw);assert b['schema']=='bounded_cruise_height_v1' and b['mode']=='bounded_cruise_height'
    assert b['official_score'] is None and not b['full_mission_complete']
    assert b['assumptions']['positive_inf_scan_policy']=='static_initial_column_no_return_only'
    assert b['assumptions']['public_plugin_max_z_m']==1.8
    assert b['feedback']['target_absolute_z_m']==.85 and b['feedback']['trim']==.02695

def make(monkeypatch):
    callbacks,_,_=install_ros_transport(monkeypatch)
    class Twist:
        def __init__(self):self.linear=SimpleNamespace(x=0.,y=0.,z=0.);self.angular=SimpleNamespace(x=0.,y=0.,z=0.)
    monkeypatch.setattr(sys.modules['rosidl_runtime_py.utilities'],'get_message',lambda _:Twist)
    wall=[0.];cpu=[0.];node=api().create_cruise_node(wall_clock=lambda:wall[0],cpu_clock=lambda:cpu[0])
    callbacks['/tf_static'](tf_static())
    from test_height_runtime_timing import feed as transport_feed
    for i in range(14):transport_feed(node,callbacks,wall,i/10,.055 if i<12 else .85)
    assert node.guard.phase=='HOLD_CRUISE'
    return node,callbacks,wall,cpu

@pytest.mark.parametrize('kind',['delay','exception'])
def test_new_factory_preserves_final_dds_query_recheck_and_timing(monkeypatch,kind):
    node,callbacks,wall,cpu=make(monkeypatch);p=node.publisher_by_topic['/uav/cmd_vel'];out=[];calls=[0]
    p.publisher.publish=lambda msg:out.append(msg.linear.z)
    def query():
        calls[0]+=1
        if calls[0]==3:
            wall[0]+=.21;cpu[0]+=.001
            if kind=='exception':raise RuntimeError('DDS synthetic')
        return 1
    p.publisher.get_subscription_count=query;wall[0]=1.31
    assert node.tick().linear_z==0. and out and not any(out)
    snap=node.guard.evidence()['runtime_timing'];row=[s for s in snap['trace'] if s['label']=='dds:publish_boundary'][-1]
    assert row['wall_s']==pytest.approx(.21)
    assert '/uav/lidar/points' in callbacks
    node.close_watchdog()

def test_cloud_encoding_only_after_terminal_without_control_lock(monkeypatch):
    node,_,wall,_=make(monkeypatch)
    with pytest.raises(RuntimeError):node.finalize_cloud_capture()
    node.stop('test_cleanup')
    called=[]
    def snapshot(*,encode=False):
        assert encode and not node.guard._lock._is_owned()
        called.append(True);return {'encoded':True,'frames':[]}
    monkeypatch.setattr(node.cloud_capture,'snapshot',snapshot)
    node.finalize_cloud_capture();assert called


def test_v37_total_receive_and_cloud_capture_timing_cover_new_work(monkeypatch):
    from test_cruise_cloud_capture import cloud as cloud_message
    node,callbacks,wall,cpu=make(monkeypatch)
    # Inject time into the raw metadata copy, after the core receive returned.
    original_copy=api().copy.deepcopy
    def slow_copy(value):
        wall[0]+=.003;cpu[0]+=.001
        return original_copy(value)
    monkeypatch.setattr(api().copy,'deepcopy',slow_copy)
    wall[0]=1.31;callbacks['/uav/odom'](odom(11.31,z=.85))
    def capture(*args,**kwargs):wall[0]+=.025;cpu[0]+=.002
    monkeypatch.setattr(node.cloud_capture,'capture',capture)
    callbacks['/uav/lidar/points'](cloud_message(stamp=11.32))
    snap=node.guard.evidence()['capture_runtime_timing']
    assert snap['revision']=='v37_capture'
    receive=next(r for r in reversed(snap['trace']) if r['label']=='receive_total:/uav/odom')
    assert receive['wall_s']==pytest.approx(.003) and receive['cpu_s']==pytest.approx(.001)
    cloud=snap['trace'][-1]
    assert cloud['label']=='cloud_capture' and cloud['wall_s']==pytest.approx(.025)
    assert cloud['cpu_s']==pytest.approx(.002)
    assert node.guard.evidence()['runtime_timing']['revision']=='v34_timing'
    node.close_watchdog()


def test_cloud_capture_failure_is_recorded_without_changing_control(monkeypatch):
    from test_cruise_cloud_capture import cloud
    node,callbacks,wall,cpu=make(monkeypatch);before=(node.guard.state,node.guard.phase,node.guard.reason)
    def fail(*args,**kwargs):raise RuntimeError('synthetic collector error')
    monkeypatch.setattr(node.cloud_capture,'capture',fail)
    callbacks['/uav/lidar/points'](cloud(stamp=11.32))
    assert (node.guard.state,node.guard.phase,node.guard.reason)==before
    assert node.cloud_capture_errors==1
    assert node.guard.evidence()['capture_runtime_timing']['labels']['cloud_capture']['count']==1
    node.close_watchdog()


def test_factory_cloud_receives_real_prior_metadata_and_full_raw_bytes(monkeypatch):
    from test_cruise_cloud_capture import cloud,tf_record
    node,callbacks,wall,cpu=make(monkeypatch)
    callbacks['/tf_static'](tf_record()['message'])
    payload=cloud(stamp=11.32);callbacks['/uav/lidar/points'](payload)
    captured=node.cloud_capture.snapshot()['clouds'][0]
    association=captured['pose_association']
    assert association['status']=='valid_prior_pose'
    assert association['odom_stamp_s']==11.3 and association['skew_s']==pytest.approx(.02)
    assert association['record']['message']['header']['stamp']=={'sec':11,'nanosec':300000000}
    assert captured['tf_snapshot']['status']=='transform_chain_present'
    assert 'data_b64' not in captured
    node.stop('test_cleanup');node.finalize_cloud_capture()
    encoded=node.guard.evidence()['forward_cloud_collection']['clouds'][0]
    assert base64.b64decode(encoded['data_b64'])==bytes(payload.data)
    assert node.guard.evidence()['capture_runtime_timing']['labels']['cloud_encode']['count']==1
    from inspection_mission.telemetry import DiagnosticReporter
    from tools.diagnostic_receiver import validate_event
    envelopes=[validate_event(DiagnosticReporter._encode(event,data)) for event,data in node.guard.diagnostic_events()]
    parts=[e['data'] for e in envelopes if e['data']['section']=='bounded_cruise_height_gzip_part']
    packed=base64.b64decode(''.join(p['base64_part'] for p in parts),validate=True);raw=gzip.decompress(packed)
    assert len(packed)==parts[0]['compressed_bytes'] and len(raw)==parts[0]['uncompressed_bytes']
    assert hashlib.sha256(packed).hexdigest()==parts[0]['compressed_sha256']
    assert hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
    assert base64.b64decode(json.loads(raw)['forward_cloud_collection']['clouds'][0]['data_b64'])==bytes(payload.data)
    node.close_watchdog()


@pytest.mark.parametrize('kind',['terminal','previous_phase'])
def test_cloud_stage_gate_rejects_terminal_and_previous_phase_sources_without_using_slot(monkeypatch,kind):
    from test_cruise_cloud_capture import cloud
    node,callbacks,wall,cpu=make(monkeypatch)
    if kind=='terminal':
        wall[0]=1.31;node.stop('injected_terminal')
        wall[0]=1.32;stamp=11.32;reason='terminal_state'
    else:
        wall[0]=1.32;stamp=11.22;reason='source_before_phase_entry'
    callbacks['/uav/lidar/points'](cloud(stamp=stamp))
    assert node.cloud_capture.snapshot()['frame_count']==0
    assert node.cloud_stage_rejections[reason]==1
    if kind=='previous_phase':
        callbacks['/uav/lidar/points'](cloud(stamp=11.32))
        assert node.cloud_capture.snapshot()['frame_count']==1
    node.stop('test_cleanup');node.finalize_cloud_capture()
    snapshot=node.guard.evidence()['forward_cloud_collection']
    assert snapshot['stage_rejections']['counters'][reason]==1
    assert snapshot['stage_rejections']['records'][-1]['reason']==reason
    if snapshot['clouds']:
        context=snapshot['clouds'][0]['stage_context']
        assert context['state_at_callback_entry']=='ASCEND' and context['phase']=='HOLD_CRUISE'
        assert context['phase_entry_source_stamp_s']==11.3
    node.close_watchdog()


def test_watchdog_zero_still_runs_during_blocked_new_cloud_callback(monkeypatch):
    import threading,time
    from test_cruise_cloud_capture import cloud
    node,callbacks,wall,cpu=make(monkeypatch);outputs=[]
    node.publisher_by_topic['/uav/cmd_vel'].publisher.publish=lambda msg:outputs.append(msg.linear.z)
    entered,release=threading.Event(),threading.Event()
    def capture(*args,**kwargs):
        assert not node.guard._lock._is_owned()
        entered.set();release.wait(1.)
    monkeypatch.setattr(node.cloud_capture,'capture',capture)
    node.arm_watchdog();runner=threading.Thread(target=lambda:callbacks['/uav/lidar/points'](cloud(stamp=11.32)),daemon=True)
    runner.start();assert entered.wait(.2);wall[0]=1.51
    try:
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline and not outputs:time.sleep(.005)
        assert outputs and not any(outputs) and node.guard.phase=='ZERO_RETURN'
    finally:
        release.set();runner.join(timeout=1.);node.close_watchdog()

@pytest.mark.parametrize('child_exit,setup_exit',[(0,0),(17,0),(0,29)])
def test_new_launcher_real_exit(tmp_path,child_exit,setup_exit):
    import os,shlex,shutil,subprocess
    from pathlib import Path
    from test_ground_probe_launch import BASH,shell_path
    if not BASH.is_file():pytest.skip('bash unavailable')
    workspace=tmp_path/'cruise workspace';package=workspace/'src/inspection_mission/inspection_mission';package.mkdir(parents=True)
    (package/'__init__.py').write_text('',encoding='utf8')
    (package/'bounded_cruise_height.py').write_text('import os,sys\nassert os.environ["FAKE_ROS_READY"]=="1"\n'+f'sys.exit({child_exit})\n',encoding='utf8')
    source=Path(__file__).resolve().parents[1]/'scripts/run_bounded_cruise_height.sh';assert source.is_file()
    launcher=workspace/'scripts/run_bounded_cruise_height.sh';launcher.parent.mkdir();shutil.copyfile(source,launcher)
    setup=tmp_path/'fake ros setup.sh';setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'+(f'return {setup_exit}\n' if setup_exit else 'export FAKE_ROS_READY=1\n'),encoding='utf8')
    env=dict(os.environ,INSPECTION_ROS_SETUP=shell_path(setup));env.pop('INSPECTION_DIAGNOSTIC_URL',None)
    r=__import__('subprocess').run([str(BASH),str(launcher)],cwd=tmp_path,env=env,capture_output=True,text=True,encoding='utf8',timeout=10)
    assert r.returncode==(setup_exit or child_exit),r.stdout+r.stderr
    assert f'BOUNDED_CRUISE_HEIGHT launch_exit exit_code={setup_exit or child_exit}' in r.stderr
