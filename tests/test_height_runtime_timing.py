"""v34 only adds timing; synthetic transport is not platform success evidence."""
import importlib
import importlib.util
import copy
import base64
import gzip
import hashlib
import json
import math
import sys
from types import SimpleNamespace

from test_bounded_ground_probe import install_ros_transport,scan
from test_bounded_vertical_response import odom,tf_static
import pytest


def api():
    name='inspection_mission.height_runtime_timing'
    assert importlib.util.find_spec(name) is not None,'v34 timing adapter missing'
    return importlib.import_module(name)


def make(monkeypatch):
    callbacks,_,_=install_ros_transport(monkeypatch)
    class Twist:
        def __init__(self):
            self.linear=SimpleNamespace(x=0.,y=0.,z=0.)
            self.angular=SimpleNamespace(x=0.,y=0.,z=0.)
    monkeypatch.setattr(sys.modules['rosidl_runtime_py.utilities'],'get_message',lambda _:Twist)
    wall=[0.];cpu=[0.]
    node=api().create_timing_height_node(wall_clock=lambda:wall[0],cpu_clock=lambda:cpu[0])
    return node,callbacks,wall,cpu


def test_actual_factory_callbacks_and_spin_are_all_observed(monkeypatch):
    node,callbacks,wall,cpu=make(monkeypatch)
    callbacks['/tf_static'](tf_static())
    pending=[('/clock',{'clock':{'sec':10,'nanosec':0}}),('/uav/odom',odom(10.)),
             ('/odom',odom(10.,child='base_footprint')),('/scan',scan(10.))]
    class Ros:
        @staticmethod
        def ok():return bool(pending)
        @staticmethod
        def spin_once(n,timeout_sec):
            topic,payload=pending.pop(0);wall[0]+=.001;cpu[0]+=.0002
            callbacks[topic](payload)
    from inspection_mission.bounded_ground_probe import spin_control_loop
    spin_control_loop(node,Ros,duration_s=.1,wall_clock=lambda:wall[0])
    snap=node.guard.evidence()['runtime_timing']
    counts=snap['labels']
    assert counts['receive:/clock']['count']==1 and counts['receive:/odom']['count']==1
    assert counts['receive:/uav/odom']['count']==1 and counts['receive:/scan']['count']==1
    assert counts['spin_once']['count']==4
    assert counts['tick']['count']==counts['control_tick']['count']==1
    scanrow=next(x for x in snap['trace'] if x['label']=='receive:/scan')
    assert scanrow['source_stamp_s']==10. and scanrow['topic']=='/scan'
    assert scanrow['entry_wall_s']<=scanrow['exit_wall_s']
    assert scanrow['wall_s']>=0. and scanrow['cpu_s']>=0.
    assert node.guard.limits.sensor_age_s==.20 and node.guard.limits.motion_wall_s==25.
    node.close_watchdog()


def feed(node,callbacks,wall,t,z=.055,vz=0.):
    wall[0]=t;stamp=round(10.+t,9);sec=int(stamp)
    callbacks['/clock']({'clock':{'sec':sec,'nanosec':round((stamp-sec)*1e9)}})
    callbacks['/uav/odom'](odom(stamp,z=z,vz=vz))
    callbacks['/odom'](odom(stamp,child='base_footprint'))
    callbacks['/scan'](scan(stamp))
    return node.tick()


def hold(monkeypatch):
    node,callbacks,wall,cpu=make(monkeypatch);callbacks['/tf_static'](tf_static())
    for i in range(14):feed(node,callbacks,wall,i/10,.055 if i<12 else .155)
    assert node.guard.phase=='HOLD_010'
    return node,callbacks,wall,cpu


def test_receive_entry_is_captured_before_metadata_inspection(monkeypatch):
    node,callbacks,wall,cpu=make(monkeypatch)
    original=api().source_stamp
    def slow_metadata(topic,msg):wall[0]+=.03;return original(topic,msg)
    monkeypatch.setattr(api(),'source_stamp',slow_metadata)
    callbacks['/clock']({'clock':{'sec':10,'nanosec':0}})
    row=node.height_timing.snapshot()['trace'][-1]
    assert row['entry_wall_s']==0.
    assert row['exit_wall_s']==.03 and row['source_stamp_s']==10.


def test_metadata_observation_error_does_not_block_real_receive(monkeypatch):
    node,callbacks,wall,cpu=make(monkeypatch)
    def broken_metadata(topic,msg):raise RuntimeError('synthetic observer metadata failure')
    monkeypatch.setattr(api(),'source_stamp',broken_metadata)
    callbacks['/clock']({'clock':{'sec':10,'nanosec':0}})
    snap=node.height_timing.snapshot()
    assert node.guard.clock[0]==10.
    assert snap['instrumentation_errors']==1
    assert snap['labels']['receive:/clock']['count']==1
    assert snap['trace'][-1]['source_stamp_s'] is None


@pytest.mark.parametrize('kind',['delay','exception'])
def test_final_dds_query_is_measured_and_original_zero_boundary_survives(monkeypatch,kind):
    node,callbacks,wall,cpu=hold(monkeypatch)
    pub=node.publisher_by_topic['/uav/cmd_vel'];outputs=[];calls=[0]
    pub.publisher.publish=lambda msg:outputs.append(msg.linear.z)
    def query():
        calls[0]+=1
        if calls[0]==3:
            wall[0]+=.21;cpu[0]+=.001
            if kind=='exception':raise RuntimeError('DDS synthetic failure')
        return 1
    pub.publisher.get_subscription_count=query
    wall[0]=1.31
    d=node.tick()
    assert outputs and all(v==0. for v in outputs)
    assert d.linear_z==0. and node.guard.commands[-1]['linear_z']==0.
    timed=[x for x in node.height_timing.snapshot()['trace'] if x['label']=='dds:publish_boundary'][-1]
    assert timed['wall_s']==pytest.approx(.21) and timed['cpu_s']==pytest.approx(.001)
    assert node.guard.phase=='ZERO_RETURN' if kind=='delay' else node.guard.state=='STOP'
    node.close_watchdog()


def test_frozen_height_behavior_is_retained_for_the_same_synthetic_inputs(monkeypatch):
    from inspection_mission.bounded_height_hold import create_height_node
    all_outputs=[]
    for factory in (create_height_node,api().create_timing_height_node):
        callbacks,_,_=install_ros_transport(monkeypatch)
        class Twist:
            def __init__(self):
                self.linear=SimpleNamespace(x=0.,y=0.,z=0.);self.angular=SimpleNamespace(x=0.,y=0.,z=0.)
        monkeypatch.setattr(sys.modules['rosidl_runtime_py.utilities'],'get_message',lambda _:Twist)
        wall=[0.];node=factory(wall_clock=lambda:wall[0]);rows=[]
        for topic,pub in node.publisher_by_topic.items():
            real=getattr(pub,'publisher',pub)
            real.publish=lambda m,topic=topic:rows.append((topic,m.linear.z))
        callbacks['/tf_static'](tf_static());z=.055
        for i in range(400):
            cmd=next((v for t,v in reversed(rows) if t=='/uav/cmd_vel'),0.)
            vz=2*(cmd-.02695) if cmd else -.0539;z=max(.055,z+vz*.05)
            if z==.055:vz=0.
            feed(node,callbacks,wall,i*.05,z,vz)
            if node.guard.state=='FINISHED':break
        assert node.guard.protocol_complete and node.guard.support_return_verified
        all_outputs.append((rows,node.guard.commands,node.guard.phase_transitions))
        node.close_watchdog()
    assert all_outputs[0]==all_outputs[1]


def test_all_inputs_counts_capacity_and_gzip_preserve_bounded_timing(monkeypatch):
    node,callbacks,wall,cpu=make(monkeypatch)
    messages={'/clock':{'clock':{'sec':10,'nanosec':0}},'/uav/odom':odom(10.),
        '/odom':odom(10.,child='base_footprint'),'/scan':scan(10.),
        '/uav/laser/scan':scan(10.),'/tf':tf_static(),'/tf_static':tf_static()}
    for topic,msg in messages.items():callbacks[topic](msg)
    for i in range(2100):
        token=node.height_timing.begin_span('spin_once');wall[0]+=.021;cpu[0]+=.001
        node.height_timing.end(token)
    snap=node.height_timing.snapshot()
    assert all(snap['labels']['receive:'+t]['count']==1 for t in messages)
    assert len(snap['trace'])==2048 and snap['trace_dropped']>0
    assert len(snap['slow_spans'])==32 and snap['slow_dropped']==2068
    from inspection_mission.telemetry import DiagnosticReporter
    encoded=[json.loads(DiagnosticReporter._encode(e,d)) for e,d in node.guard.diagnostic_events()]
    parts=[x['data'] for x in encoded if x['data'].get('section')=='bounded_height_hold_gzip_part']
    packed=base64.b64decode(''.join(x['base64_part'] for x in parts),validate=True);raw=gzip.decompress(packed)
    assert len(raw)==parts[0]['uncompressed_bytes'] and hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
    body=json.loads(raw)
    assert body['runtime_timing']==snap and body['official_score'] is None and not body['full_mission_complete']


@pytest.mark.parametrize('child_exit,setup_exit',[(0,0),(17,0),(0,29)])
def test_timing_launcher_preserves_real_exit(tmp_path,child_exit,setup_exit):
    import os,shlex,shutil,subprocess
    from pathlib import Path
    from test_ground_probe_launch import BASH,shell_path
    if not BASH.is_file():pytest.skip('bash unavailable')
    package=tmp_path/'uploaded/src/inspection_mission/inspection_mission';package.mkdir(parents=True)
    (package/'__init__.py').write_text('',encoding='utf8')
    (package/'height_runtime_timing.py').write_text(f'import sys\nsys.exit({child_exit})\n',encoding='utf8')
    source=Path(__file__).resolve().parents[1]/'scripts/run_height_runtime_timing.sh'
    assert source.is_file(),'v34 timing launcher missing'
    launcher=package.parents[2]/'scripts/run_height_runtime_timing.sh';launcher.parent.mkdir();shutil.copyfile(source,launcher)
    setup=tmp_path/'setup.sh';setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'+(f'return {setup_exit}\n' if setup_exit else ''),encoding='utf8')
    env=dict(os.environ,INSPECTION_ROS_SETUP=shell_path(setup));env.pop('INSPECTION_DIAGNOSTIC_URL',None)
    result=subprocess.run([str(BASH),str(launcher)],cwd=tmp_path,env=env,capture_output=True,text=True,encoding='utf8',timeout=10)
    assert result.returncode==(setup_exit or child_exit),result.stdout+result.stderr
    assert f'HEIGHT_RUNTIME_TIMING launch_exit exit_code={setup_exit or child_exit}' in result.stderr
