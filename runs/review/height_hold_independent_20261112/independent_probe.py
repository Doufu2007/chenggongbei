"""Independent height-hold transport probes; synthetic, never official evidence."""
from __future__ import annotations
import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
import pytest
from inspection_mission import bounded_height_hold as api
from test_bounded_ground_probe import install_ros_transport, scan
from test_bounded_vertical_response import odom, tf_static


class Transport:
    def __init__(self):
        self.patch = pytest.MonkeyPatch()
        self.callbacks,_,_ = install_ros_transport(self.patch)
        class Twist:
            def __init__(self):
                from types import SimpleNamespace
                self.linear = SimpleNamespace(x=0.,y=0.,z=0.)
                self.angular = SimpleNamespace(x=0.,y=0.,z=0.)
        self.patch.setattr(sys.modules['rosidl_runtime_py.utilities'],'get_message',lambda _:Twist)
        self.now = [0.]
        self.node = api.create_height_node(wall_clock=lambda:self.now[0])
        self.rows=[]
        for topic,pub in self.node.publisher_by_topic.items():
            pub.publish=lambda m,topic=topic:self.rows.append((topic,
                (m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)))
        self.callbacks['/tf_static'](tf_static())

    def feed(self,t,z=.055,vz=0.,tick=True):
        self.now[0]=round(t,9)
        stamp=round(10.+t,9); sec=int(stamp)
        self.callbacks['/clock']({'clock':{'sec':sec,'nanosec':round((stamp-sec)*1e9)}})
        self.callbacks['/uav/odom'](odom(stamp,z=z,vz=vz))
        self.callbacks['/odom'](odom(stamp,child='base_footprint'))
        self.callbacks['/scan'](scan(stamp))
        return self.node.tick() if tick else None

    def enter_hold(self):
        for i in range(23): self.feed(i*.05)
        self.feed(1.15,.155)
        assert self.node.guard.state=='PHASE_GATE'
        self.feed(1.2,.155)
        assert self.node.guard.phase=='HOLD_010'
        assert self.rows[-1][1][2]==pytest.approx(.02695)
        return self

    def close(self):
        self.node.close_watchdog(); self.patch.undo()


def actual_protocol():
    f=Transport(); z=.055
    try:
        for i in range(600):
            cmd=next((v[2] for t,v in reversed(f.rows) if t=='/uav/cmd_vel'),0.)
            # Independent synthetic local plant. This is not a measured model.
            vz=2.*(cmd-.02695) if cmd else -.0539
            z=max(.055,z+vz*.05)
            if z==.055: vz=0.
            f.feed(i*.05,z,vz)
            if f.node.guard.state=='FINISHED':break
        g=f.node.guard
        assert g.state=='FINISHED' and g.protocol_complete and g.support_return_verified,(g.state,g.reason)
        assert g.hold_verified and g.descent_verified
        assert set(f.node.publisher_by_topic)=={'/cmd_vel','/uav/cmd_vel'}
        assert all(v==(0.,)*6 for t,v in f.rows if t=='/cmd_vel')
        assert all(v[:2]+v[3:]==(0.,)*5 and 0.<=v[2]<=.05 for t,v in f.rows if t=='/uav/cmd_vel')
        actual=[v[2] for t,v in f.rows if t=='/uav/cmd_vel']
        assert actual==[x['linear_z'] for x in g.commands]
        for row in g.commands:
            if row['linear_z'] and row['phase'] in ('HOLD_010','DESCEND_002'):
                expected=max(.01,min(.05,.02695+.5*max(-.03,min(.03,.5*(row['target_z']-row['measured_z'])))))
                assert row['linear_z']==pytest.approx(expected)
        for phase in api.ACTIVE_PHASES:
            assert g.phase_samples[phase]['requested_command_count']>0
            assert g.phase_samples[phase]['advancing_odom_count']>0
        parts=[d for _,d in g.diagnostic_events() if d['section']=='bounded_height_hold_gzip_part']
        packed=base64.b64decode(''.join(x['base64_part'] for x in parts),validate=True)
        raw=gzip.decompress(packed)
        assert len(packed)==parts[0]['compressed_bytes']
        assert len(raw)==parts[0]['uncompressed_bytes']
        assert hashlib.sha256(packed).hexdigest()==parts[0]['compressed_sha256']
        assert hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
        data=json.loads(raw)
        assert data==g.evidence() and data['official_score'] is None
        assert not data['full_mission_complete'] and not data['forward_cloud_control_used']
        return {'state':g.state,'reason':g.reason,'commands':len(actual),'end_wall':f.now[0],
                'hold_dwell':g.hold_dwell_verified,'protocol_complete':g.protocol_complete,
                'support_return_verified':g.support_return_verified,'evidence_parts':len(parts)}
    finally:f.close()


def transient_hold_break(kind):
    f=Transport().enter_hold()
    try:
        for i in range(25,61):f.feed(i*.05,.155)
        g=f.node.guard
        assert g.hold_anchor==(1.2,11.2)
        f.now[0]=3.025
        f.callbacks['/uav/odom'](odom(13.025,z=.161 if kind=='z' else .155,vz=.011 if kind=='vz' else 0.))
        assert g.hold_anchor is None
        f.feed(3.05,.155)
        for i in range(62,86):f.feed(i*.05,.155)
        assert not g.hold_verified and g.phase=='HOLD_010'
        assert g.hold_anchor==(3.05,13.05)
        return {'kind':kind,'reset':g.hold_dwell_resets[-1],'new_anchor':g.hold_anchor}
    finally:f.close()


def descent_first_low(vz):
    f=Transport().enter_hold()
    try:
        for i in range(25,85):f.feed(i*.05,.155)
        g=f.node.guard
        assert g.state=='PHASE_GATE' and g.hold_verified
        before=len(f.rows)
        f.feed(4.25,.079,vz)
        assert g.phase=='ZERO_RETURN'
        assert all(v==(0.,)*6 for _,v in f.rows[before:])
        assert g.descent_verified is (abs(vz)<=.02)
        return {'initial_vz':vz,'descent_verified':g.descent_verified,'reason':g.reason,
                'nonzero_descent_count':g.phase_samples['DESCEND_002']['requested_command_count']}
    finally:f.close()


def permanent_budget(phase):
    f=Transport().enter_hold()
    try:
        g=f.node.guard; start=g.motion_started_wall; origin=g.origin
        if phase=='PHASE_GATE': g.state='PHASE_GATE'
        elif phase=='ZERO_RETURN':g.pause('review_abort',f.now[0])
        f.now[0]=start+25.
        f.node.tick()
        assert g.state=='STOP' and g.reason=='motion_wall_time_limit'
        assert g.origin==origin and g.motion_started_wall==start
        assert f.rows[-1][1]==(0.,)*6
        return {'phase_at_injection':phase,'reason':g.reason,'start':start,'stop':f.now[0]}
    finally:f.close()


def slow_boundary(kind):
    f=Transport().enter_hold()
    try:
        g=f.node.guard
        f.now[0]=1.21
        base=f.node.publisher_by_topic['/cmd_vel'].publish
        queries=[0]
        def ground(msg):
            base(msg)
            if kind=='ground_delay':f.now[0]=1.41
            if kind=='ground_failure':raise RuntimeError('independent ground fault')
            if kind=='match_lost':f.node.publisher_by_topic['/uav/cmd_vel'].get_subscription_count=lambda:0
            if kind=='request_changed':g.update_odom(odom(11.22,z=.156),wall_s=1.22);f.now[0]=1.22
            if kind=='motion_budget':f.now[0]=g.motion_started_wall+25.
            if kind in ('dds_delay','dds_error'):
                def delayed():
                    queries[0]+=1;f.now[0]=1.41
                    if kind=='dds_error':raise RuntimeError('independent DDS fault')
                    return 1
                f.node.publisher_by_topic['/uav/cmd_vel'].get_subscription_count=delayed
        f.node.publisher_by_topic['/cmd_vel'].publish=ground
        before=len(f.rows);counts=dict(f.node.publish_counts)
        d=f.node.tick()
        uav=[v for t,v in f.rows[before:] if t=='/uav/cmd_vel']
        assert uav and all(v==(0.,)*6 for v in uav),(kind,uav)
        assert d.linear_z==0. and g.commands[-1]['linear_z']==0.
        assert g.commands[-1]['wall_elapsed_s']==f.now[0]
        assert f.node.publish_counts['uav_positive_z']==counts['uav_positive_z']
        assert f.node.publish_counts['uav_zero']>counts['uav_zero']
        return {'fault':kind,'state':g.state,'phase':g.phase,'reason':g.reason,'after_wall':f.now[0],
                'queries_after_ground':queries[0],'uav_outputs':len(uav)}
    finally:f.close()


def hold_deadline_after_ground():
    f=Transport().enter_hold()
    try:
        for i in range(25,184):f.feed(i*.05,.162)
        g=f.node.guard
        deadline=g.phase_started_wall+8.
        assert g.phase=='HOLD_010' and g.state=='ASCEND'
        f.now[0]=deadline-.02
        original=f.node.publisher_by_topic['/cmd_vel'].publish
        def late(msg):original(msg);f.now[0]=deadline+.02
        f.node.publisher_by_topic['/cmd_vel'].publish=late
        before=len(f.rows);d=f.node.tick()
        assert d.linear_z==0. and g.phase=='ZERO_RETURN' and g.reason=='hold_wall_time_limit'
        assert all(v==(0.,)*6 for _,v in f.rows[before:])
        return {'deadline':deadline,'returned_wall':f.now[0],'reason':g.reason}
    finally:f.close()


def watchdog_no_tick():
    f=Transport().enter_hold();event=threading.Event()
    try:
        base=f.node.publisher_by_topic['/uav/cmd_vel'].publish
        def out(m):
            base(m)
            if not m.linear.z:event.set()
        f.node.publisher_by_topic['/uav/cmd_vel'].publish=out
        f.now[0]=1.36;f.node.arm_watchdog()
        assert event.wait(.4)
        assert f.node.guard.phase=='ZERO_RETURN'
        return {'phase':f.node.guard.phase,'reason':f.node.guard.reason}
    finally:f.close()


def failed_uav_publish():
    f=Transport().enter_hold()
    try:
        base=f.node.publisher_by_topic['/uav/cmd_vel'].publish
        count=f.node.publish_counts['uav_positive_z']
        def fail(m):
            if m.linear.z:raise RuntimeError('independent positive failure')
            base(m)
        f.node.publisher_by_topic['/uav/cmd_vel'].publish=fail
        f.now[0]=1.21;f.node.tick()
        assert f.node.guard.state=='STOP'
        assert f.rows[-1][1]==(0.,)*6
        assert f.node.publish_counts['uav_positive_z']==count
        assert f.node.guard.commands[-1]['linear_z']==0.
        return {'reason':f.node.guard.reason,'positive_count':count,'last':f.node.guard.commands[-1]}
    finally:f.close()


if __name__=='__main__':
    results={'actual_factory_protocol':actual_protocol(),
        'transient_bad_frame':[transient_hold_break(x) for x in ('z','vz')],
        'descent_first_low':[descent_first_low(v) for v in (-.01,-.021)],
        'permanent_budget':[permanent_budget(x) for x in ('HOLD_010','PHASE_GATE','ZERO_RETURN')],
        'post_ground_boundaries':[slow_boundary(x) for x in ('ground_delay','dds_delay','dds_error','ground_failure','match_lost','request_changed','motion_budget')],
        'hold_deadline':hold_deadline_after_ground(),'watchdog':watchdog_no_tick(),
        'failed_uav':failed_uav_publish(),
        'sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in
            ('src/inspection_mission/inspection_mission/bounded_height_hold.py','scripts/run_bounded_height_hold.sh')}}
    Path(__file__).with_name('metrics.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results,ensure_ascii=False,indent=2))
