"""Independent diagnostic-wrapper probes with strict publisher stand-ins."""
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
import base64
import gzip
import hashlib
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/inspection_mission'))
sys.path.insert(0, str(ROOT/'tests'))
import pytest
from inspection_mission import height_runtime_timing as api
from inspection_mission.bounded_height_hold import create_height_node
from inspection_mission.bounded_ground_probe import spin_control_loop
from test_bounded_ground_probe import install_ros_transport, scan
from test_bounded_vertical_response import odom, tf_static


class Fixture:
    def __init__(self, timed=True, customize=None):
        self.patch=pytest.MonkeyPatch()
        self.callbacks,_,_=install_ros_transport(self.patch)
        self.wall=self.cpu=0.
        self.sent=[]
        self.created={}
        owner=self
        class StrictPublisher:
            def __init__(self,node,topic):
                self.node,self.topic=node,topic
                self.handle=object()
                self.query_hook=None
                self.publish_error=None
                self.publish_result=object()
            def publish(self,message):
                assert owner.created[self.topic] is self
                if self.publish_error is not None:raise self.publish_error
                owner.sent.append((self.topic,message.linear.z))
                return self.publish_result
            def get_subscription_count(self):
                assert owner.created[self.topic] is self
                return 1 if self.query_hook is None else self.query_hook()
        self.StrictPublisher=StrictPublisher
        Node=sys.modules['rclpy.node'].Node
        def publisher(node,cls,topic,qos):
            raw=StrictPublisher(node,topic)
            self.created[topic]=raw
            if not hasattr(node,'registered_original_publishers'):node.registered_original_publishers=[]
            node.registered_original_publishers.append(raw)
            return raw
        def destroy(node):
            assert all(type(pub) is StrictPublisher for pub in node.registered_original_publishers)
            node.strict_destroy_passed=True
        self.patch.setattr(Node,'create_publisher',publisher)
        self.patch.setattr(Node,'destroy_node',destroy,raising=False)
        class Twist:
            def __init__(self):
                self.linear=SimpleNamespace(x=0.,y=0.,z=0.)
                self.angular=SimpleNamespace(x=0.,y=0.,z=0.)
        self.patch.setattr(sys.modules['rosidl_runtime_py.utilities'],'get_message',lambda name:Twist)
        if customize is not None:
            def base(reporter=None,*,wall_clock):
                node=create_height_node(reporter,wall_clock=wall_clock)
                customize(node)
                return node
            self.patch.setattr(api,'create_height_node',base)
        self.node=(api.create_timing_height_node(wall_clock=lambda:self.wall,cpu_clock=lambda:self.cpu)
            if timed else create_height_node(wall_clock=lambda:self.wall))

    def feed(self,t,z=.055,vz=0.):
        self.wall=t
        source=round(10.+t,9);sec=int(source)
        self.callbacks['/clock']({'clock':{'sec':sec,'nanosec':round((source-sec)*1e9)}})
        self.callbacks['/uav/odom'](odom(source,z=z,vz=vz))
        self.callbacks['/odom'](odom(source,child='base_footprint'))
        self.callbacks['/scan'](scan(source))
        self.cpu+=.0001
        return self.node.tick()

    def hold(self):
        self.callbacks['/tf_static'](tf_static())
        for i in range(14):self.feed(i/10,.055 if i<12 else .155)
        assert self.node.guard.phase=='HOLD_010'
        return self

    def close(self):
        self.node.close_watchdog()
        self.patch.undo()


def registered_callbacks_and_spin():
    f=Fixture()
    try:
        messages=[('/clock',{'clock':{'sec':10,'nanosec':0}}),('/uav/odom',odom(10.)),
            ('/odom',odom(10.,child='base_footprint')),('/scan',scan(10.)),
            ('/uav/laser/scan',scan(10.)),('/tf',tf_static()),('/tf_static',tf_static())]
        pending=list(messages)
        class Ros:
            @staticmethod
            def ok():return bool(pending)
            @staticmethod
            def spin_once(node,timeout_sec):
                topic,payload=pending.pop(0)
                f.wall+=.001;f.cpu+=.0002
                f.callbacks[topic](payload)
        spin_control_loop(f.node,Ros,duration_s=.1,wall_clock=lambda:f.wall)
        snap=f.node.height_timing.snapshot()
        assert all(snap['labels']['receive:'+topic]['count']==1 for topic,_ in messages)
        assert snap['labels']['spin_once']['count']==7
        assert snap['labels']['tick']['count']==snap['labels']['control_tick']['count']==1
        assert all(row['exit_wall_s']>=row['entry_wall_s'] and row['wall_s']>=0. and row['cpu_s']>=0. for row in snap['trace'])
        return {'pass':True,'labels':{key:row['count'] for key,row in snap['labels'].items()},'input_count':7}
    finally:f.close()


def baseline_behavior(kind):
    runs=[]
    for timed in (False,True):
        f=Fixture(timed)
        try:
            if kind=='nominal':
                f.callbacks['/tf_static'](tf_static());z=.055
                for i in range(400):
                    cmd=next((value for topic,value in reversed(f.sent) if topic=='/uav/cmd_vel'),0.)
                    vz=2*(cmd-.02695) if cmd else -.0539
                    z=max(.055,z+vz*.05)
                    if z==.055:vz=0.
                    f.feed(i*.05,z,vz)
                    if f.node.guard.state=='FINISHED':break
                assert f.node.guard.protocol_complete and f.node.guard.support_return_verified
            else:
                f.hold();f.sent.clear();calls=[]
                def query():
                    calls.append(f.wall)
                    if len(calls)==3:
                        f.wall+=.21;f.cpu+=.003
                        if kind=='dds_exception':raise RuntimeError('independent DDS query failure')
                    return 1
                f.created['/uav/cmd_vel'].query_hook=query
                f.wall=1.31
                f.node.tick()
                assert f.sent and all(value==0. for _,value in f.sent)
                assert len(calls)==3
                if timed:
                    spans=[row for row in f.node.height_timing.snapshot()['trace'] if row['label']=='dds:publish_boundary']
                    assert abs(spans[-1]['wall_s']-.21)<1e-12 and abs(spans[-1]['cpu_s']-.003)<1e-12
            g=f.node.guard
            runs.append({'sent':list(f.sent),'commands':g.commands,'phases':g.phase_transitions,
                'state':g.state,'reason':g.reason,'errors':f.node.errors})
        finally:f.close()
    assert runs[0]==runs[1]
    return {'pass':True,'scenario':kind,'publish_count':len(runs[0]['sent']),
        'final_state':runs[0]['state'],'reason':runs[0]['reason'],'outputs_commands_and_transitions_equal':True}


def strict_publisher_identity_and_exceptions():
    f=Fixture()
    marker=RuntimeError('independent raw publisher exception')
    try:
        raw=f.created['/uav/cmd_vel'];proxy=f.node.publisher_by_topic['/uav/cmd_vel']
        assert proxy is not raw and not isinstance(proxy,f.StrictPublisher)
        assert proxy.handle is raw.handle
        Twist=sys.modules['rosidl_runtime_py.utilities'].get_message('geometry_msgs/msg/Twist')
        assert proxy.publish(Twist()) is raw.publish_result
        def failed_query():raise marker
        raw.query_hook=failed_query
        try:proxy.get_subscription_count()
        except Exception as exc:assert exc is marker
        else:raise AssertionError('query exception swallowed')
        raw.publish_error=marker
        try:proxy.publish(Twist())
        except Exception as exc:assert exc is marker
        else:raise AssertionError('publish exception swallowed')
        f.node.destroy_node()
        assert f.node.strict_destroy_passed
        return {'pass':True,'delegated_methods_received_original_self':True,
            'original_registry_identity_retained':True,'proxy_is_publisher_instance':False,
            'raw_query_and_publish_exception_objects_preserved':True,
            'rclpy_available':False,'scope':'严格原对象 stand-in；非真实 rclpy 集成'}
    finally:f.close()


def original_callback_tick_publish_exceptions():
    result={}
    for name in ('receive','tick','_publish'):
        marker=RuntimeError('independent original '+name+' exception')
        def customize(node,name=name,marker=marker):
            def fail(*args,**kwargs):raise marker
            setattr(node,name,fail)
        f=Fixture(customize=customize)
        try:
            try:
                if name=='receive':f.callbacks['/clock']({'clock':{'sec':10,'nanosec':0}})
                elif name=='tick':f.node.tick()
                else:f.node._publish(0.)
            except Exception as exc:assert exc is marker
            else:raise AssertionError(name+' exception swallowed')
            result[name]='same exception object propagated'
            if name!='_publish':
                assert f.node.height_timing.snapshot()['trace'][-1]['label']==('receive:/clock' if name=='receive' else 'control_tick')
        finally:f.close()
    f=Fixture()
    try:
        marker=RuntimeError('independent original spin exception')
        class Ros:
            @staticmethod
            def ok():return True
            @staticmethod
            def spin_once(node,timeout_sec):raise marker
        try:spin_control_loop(f.node,Ros,duration_s=.1,wall_clock=lambda:f.wall)
        except Exception as exc:assert exc is marker
        else:raise AssertionError('spin exception swallowed')
        assert f.node.height_timing.snapshot()['trace'][-1]['label']=='spin_once'
        result['spin_once']='same exception object propagated'
    finally:f.close()
    return {'pass':True,'calls':result}


def observer_metadata_isolation():
    f=Fixture()
    try:
        original=api.source_stamp
        def slow(topic,msg):
            f.wall+=.03
            return original(topic,msg)
        f.patch.setattr(api,'source_stamp',slow)
        f.callbacks['/clock']({'clock':{'sec':10,'nanosec':0}})
        row=f.node.height_timing.snapshot()['trace'][-1]
        assert row['entry_wall_s']==0. and row['exit_wall_s']==.03
        def broken(topic,msg):raise RuntimeError('independent metadata failure')
        f.patch.setattr(api,'source_stamp',broken)
        f.wall=.04
        f.callbacks['/clock']({'clock':{'sec':10,'nanosec':10_000_000}})
        assert f.node.guard.clock[0]==10.01
        snap=f.node.height_timing.snapshot()
        assert snap['instrumentation_errors']==1 and snap['labels']['receive:/clock']['count']==2
        class PoisonPayload:
            def __deepcopy__(self,memo):raise AssertionError('payload copied')
            def __iter__(self):raise AssertionError('payload iterated')
        msg={'header':{'stamp':{'sec':12,'nanosec':0}},'ranges':PoisonPayload(),'data':PoisonPayload()}
        assert original('/scan',msg)==12.
        return {'pass':True,'metadata_cost_included_in_span':True,'observer_error_did_not_block_original_receive':True,
            'header_reader_ignored_large_payload':True}
    finally:f.close()


def bounded_trace_summary_and_gzip():
    f=Fixture()
    try:
        rec=f.node.height_timing
        defaults=api.HeightTimingRecorder.__init__.__kwdefaults__
        assert defaults['wall_clock'] is time.monotonic and defaults['cpu_clock'] is time.thread_time
        for _ in range(2100):
            token=rec.begin_span('spin_once');f.wall+=.021;f.cpu+=.001;rec.end(token)
        snap=rec.snapshot();stat=snap['labels']['spin_once']
        assert len(snap['trace'])==2048 and snap['trace_dropped']==52
        assert len(snap['slow_spans'])==32 and snap['slow_dropped']==2068
        assert stat['count']==2100 and abs(stat['wall_total_s']-44.1)<1e-9 and abs(stat['cpu_total_s']-2.1)<1e-9
        assert abs(stat['wall_max_s']-.021)<1e-12
        assert abs(stat['first_entry_wall_s'])<1e-12 and abs(stat['last_entry_wall_s']-2099*.021)<1e-9
        parts=[data for _,data in f.node.guard.diagnostic_events() if data.get('section')=='bounded_height_hold_gzip_part']
        packed=base64.b64decode(''.join(row['base64_part'] for row in sorted(parts,key=lambda row:row['part_index'])))
        raw=gzip.decompress(packed);body=json.loads(raw)
        assert len(packed)==parts[0]['compressed_bytes'] and hashlib.sha256(packed).hexdigest()==parts[0]['compressed_sha256']
        assert len(raw)==parts[0]['uncompressed_bytes'] and hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
        assert body['runtime_timing']==snap and body['official_score'] is None and body['full_mission_complete'] is False
        return {'pass':True,'trace_count':2048,'trace_dropped':52,'slow_count':32,'slow_dropped':2068,
            'summary_count':stat['count'],'wall_total_s':stat['wall_total_s'],'cpu_total_s':stat['cpu_total_s'],
            'gzip_integrity_and_optional_timing_equal':True,'original_mode':body['mode']}
    finally:f.close()


def main():
    source=ROOT/'src/inspection_mission/inspection_mission/height_runtime_timing.py'
    initial=hashlib.sha256(source.read_bytes()).hexdigest()
    cases=[('registered_callbacks_and_spin',registered_callbacks_and_spin),
        ('strict_publisher_identity_and_exceptions',strict_publisher_identity_and_exceptions),
        ('original_callback_tick_publish_exceptions',original_callback_tick_publish_exceptions),
        ('observer_metadata_isolation',observer_metadata_isolation),
        ('bounded_trace_summary_and_gzip',bounded_trace_summary_and_gzip)]
    cases += [('baseline_'+kind,lambda kind=kind:baseline_behavior(kind)) for kind in ('nominal','dds_delay','dds_exception')]
    probes={}
    for name,fn in cases:
        try:probes[name]=fn()
        except Exception as exc:probes[name]={'pass':False,'error':type(exc).__name__+':'+str(exc)}
    manifest_path=ROOT/'runs/review/height_runtime_timing_20261112/manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8-sig'))
    files=[{'path':row['path'],'expected_sha256':row['sha256'],
        'actual_sha256':hashlib.sha256((ROOT/row['path']).read_bytes()).hexdigest()}
        for row in manifest['frozen_baseline']+manifest['artifacts']]
    hashes_match=all(row['expected_sha256']==row['actual_sha256'] for row in files)
    result={'reviewed_at_utc':datetime.now(timezone.utc).isoformat(),'source_sha256':initial,
        'source_unchanged':initial==hashlib.sha256(source.read_bytes()).hexdigest(),
        'launcher_sha256':hashlib.sha256((ROOT/'scripts/run_height_runtime_timing.sh').read_bytes()).hexdigest(),
        'manifest_sha256':hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        'manifest_files':files,'all_manifest_files_match':hashes_match,
        'scope':'独立本地插桩审查；严格 Publisher stand-in；无真实 rclpy/平台性能或控制成功证据',
        'probe_count':len(cases),'pass_count':sum(bool(row.get('pass')) for row in probes.values()),'probes':probes}
    (Path(__file__).parent/'independent_metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=True,indent=2))
    return 0 if result['source_unchanged'] and hashes_match and result['pass_count']==len(cases) else 1


if __name__=='__main__':sys.exit(main())
