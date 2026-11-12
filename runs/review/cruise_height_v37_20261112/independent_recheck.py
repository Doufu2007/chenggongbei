"""Focused v37 follow-up to independent stage-evidence counterexamples."""
from pathlib import Path
import base64,gzip,hashlib,json,sys,traceback
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'src/inspection_mission')]
import pytest
import test_bounded_cruise_height as fx
from test_cruise_cloud_capture import cloud,tf_record
from test_height_runtime_timing import feed as transport_feed
from test_bounded_vertical_response import odom
results=[]
def check(name,fn):
    try:results.append({'name':name,'passed':True,'detail':fn()})
    except Exception:results.append({'name':name,'passed':False,'traceback':traceback.format_exc()})
def frozen():
    manifest=json.loads((ROOT/'runs/review/cruise_height_v37_20261112/manifest.json').read_text(encoding='utf-8'))
    rows=manifest['files']+manifest['frozen_parents']
    saved=[]
    for row in rows:
        actual=hashlib.sha256((ROOT/row['path']).read_bytes()).hexdigest()
        assert actual==row['sha256'],row['path']
        if 'expected_sha256' in row:assert actual==row['expected_sha256']
        saved.append({'path':row['path'],'sha256':actual})
    return saved
def stage(kind):
    with pytest.MonkeyPatch.context() as mp:
        node,callbacks,wall,cpu=fx.make(mp)
        try:
            if kind=='terminal':
                wall[0]=1.31;node.stop('independent_terminal_recheck')
                wall[0]=1.32;stamp=11.32
            else:wall[0]=1.31;stamp=11.22
            before=(node.guard.state,node.guard.phase,node.guard.reason)
            callbacks['/uav/lidar/points'](cloud(stamp=stamp))
            rejected=node.cloud_capture.snapshot()
            assert rejected['frame_count']==0,rejected
            rejection_counters=dict(node.cloud_stage_rejections)
            assert rejection_counters['terminal_state' if kind=='terminal' else 'source_before_phase_entry']==1,rejection_counters
            assert before==(node.guard.state,node.guard.phase,node.guard.reason)
            if kind!='terminal':
                callbacks['/tf_static'](tf_record()['message'])
                payload=cloud(stamp=11.32)
                callbacks['/uav/lidar/points'](payload)
                snap=node.cloud_capture.snapshot()
                assert snap['frame_count']==1 and snap['clouds'][0]['phase']=='HOLD_CRUISE'
                assert node.cloud_stage_context['HOLD_CRUISE']['state_at_callback_entry']=='ASCEND'
                assert snap['clouds'][0]['pose_association']['skew_s']==pytest.approx(.02)
                original=node.cloud_capture.snapshot
                def lock_check(*,encode=False):
                    if encode:
                        assert not node.guard._lock._is_owned()
                        assert not node.command_lock._is_owned()
                    return original(encode=encode)
                mp.setattr(node.cloud_capture,'snapshot',lock_check)
                node.stop('independent_encoding_recheck');node.close_watchdog();node.finalize_cloud_capture()
                parts=[d for _,d in node.guard.diagnostic_events() if d['section']=='bounded_cruise_height_gzip_part']
                packed=base64.b64decode(''.join(d['base64_part'] for d in parts),validate=True)
                raw=gzip.decompress(packed)
                assert len(packed)==parts[0]['compressed_bytes'] and len(raw)==parts[0]['uncompressed_bytes']
                assert hashlib.sha256(packed).hexdigest()==parts[0]['compressed_sha256']
                assert hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
                restored=json.loads(raw)['forward_cloud_collection']['clouds'][0]
                assert base64.b64decode(restored['data_b64'],validate=True)==bytes(payload.data)
                assert json.loads(raw)['forward_cloud_collection']['stage_rejections']['counters']==rejection_counters
                return {'rejected':rejection_counters,'valid_stage_context':restored['stage_context'],
                        'gzip_parts':len(parts),'raw_cloud_bytes':restored['data_bytes'],'encoding_control_locks_free':True}
            return {'rejected':rejection_counters,'terminal_state':node.guard.state}
        finally:node.close_watchdog()
def ground_boundary():
    with pytest.MonkeyPatch.context() as mp:
        node,callbacks,wall,cpu=fx.make(mp)
        try:
            for i in range(14,46):transport_feed(node,callbacks,wall,i/10,.85)
            assert node.guard.phase=='DESCEND_SUPPORT',(node.guard.phase,node.guard.reason)
            out=[];fired=[False]
            node.publisher_by_topic['/uav/cmd_vel'].publisher.publish=lambda msg:out.append(msg.linear.z)
            def crossing(msg):
                if not fired[0]:
                    fired[0]=True
                    wall[0]=round(wall[0]+.01,9)
                    callbacks['/uav/odom'](odom(10.+wall[0],z=.079,vz=-.019))
            node.publisher_by_topic['/cmd_vel'].publish=crossing
            node.tick()
            assert fired[0] and out and not any(out),out
            assert node.guard.phase=='ZERO_RETURN' and node.guard.descent_verified
            return {'uav_requests':out,'phase_after_ground_callback':node.guard.phase,
                    'descent_verified':node.guard.descent_verified}
        finally:node.close_watchdog()
check('refrozen_files_and_six_parents',frozen)
check('terminal_cloud_rejected',lambda:stage('terminal'))
check('delayed_phase_cloud_rejected_valid_cloud_roundtrip',lambda:stage('old_phase'))
check('ground_publish_callback_crosses_descent_cutoff',ground_boundary)
out=ROOT/'runs/review/cruise_height_v37_20261112/independent_recheck_results.json'
out.write_text(json.dumps({'checks':results,'passed':sum(r['passed'] for r in results),'total':len(results)},ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False,indent=2))
raise SystemExit(0 if all(r['passed'] for r in results) else 1)
