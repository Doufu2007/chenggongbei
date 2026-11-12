"""Read-only independent regressions: synthetic transport, real production factories."""
from pathlib import Path
import json
import sys
import shutil

import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]


def completed_node(monkeypatch):
    from test_bounded_ground_probe import endpoint,install_ros_transport
    from test_ground_bounded_approach import refresh
    from inspection_mission.ground_three_point_return import create_return_three_point_node
    install_ros_transport(monkeypatch);wall=[0.]
    node=create_return_three_point_node(wall_clock=lambda:wall[0]);node.arm_watchdog=lambda:None
    g=node.guard;g.update_endpoints([endpoint()],matched_subscriptions=1);refresh(g,0.)
    g.global_origin=g.pulse_origin=(0.,0.,.055);g.previous_path_xyz=g.odom['xyz'];g.previous_yaw=0.
    g.return_started=True;g.current_operations_role='return';g.phase=g.state='RETURN_SETTLE'
    for i in range(23):
        wall[0]=i*.05;refresh(g,wall[0]);node.last_graph_refresh_wall=wall[0];node.tick()
        if g.return_completed:break
    assert g.return_completed
    node.report_started=lambda *a:None;node.report_final=lambda:None;node.destroy_node=lambda:None
    return node,wall


def test_actual_ground_return_proof_passes_flight_validator(monkeypatch):
    from inspection_mission.full_flight_inspection import validate_ground_return
    node,wall=completed_node(monkeypatch)
    assert validate_ground_return(node.guard.return_completion)==node.guard.return_completion
    node.close_watchdog()


def test_actual_cleanup_false_zero_must_prevent_flight(monkeypatch):
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    node,wall=completed_node(monkeypatch);created=[]
    def fail(msg):raise RuntimeError('injected zero failure after successful return')
    node.publisher_by_topic['/cmd_vel'].publish=fail
    class Ros:
        @staticmethod
        def spin_once(*args,**kwargs):pass
    def flight(*args,**kwargs):
        created.append(True)
        raise AssertionError('flight created despite failed cleanup zero')
    try:
        run_ground_and_flight(Ros(),None,worker=object(),ground_factory=lambda *a,**k:node,
            flight_factory=flight,spin=lambda *a,**k:None,wall_clock=lambda:wall[0])
    except RuntimeError:
        pass
    assert not created,'ground cleanup silently returns False and still constructs flight'


def test_recovered_manifest_wrong_class_is_rejected(tmp_path):
    from inspection_mission.ground_template_catalog import load_complete_ground_templates,additional_asset_root
    folder=tmp_path/'recovered';shutil.copytree(additional_asset_root(),folder)
    manifest=folder/'manifest.json';data=json.loads(manifest.read_text(encoding='utf-8'))
    data['templates'][0]['class_id']='damaged_vehicle' if data['templates'][0]['class_id']!='damaged_vehicle' else 'rescue_camp'
    manifest.write_text(json.dumps(data),encoding='utf-8')
    with pytest.raises(ValueError,match='class'):load_complete_ground_templates(additional_dir=folder)


def test_actual_ten_template_worker_shape_must_be_recordable_without_replacing_result():
    from inspection_mission.ground_match_result_recorder import GroundMatchResultRecorder
    from inspection_mission.telemetry import DiagnosticReporter
    frame=json.loads((Path(__file__).parent/'ten_template_replay.json').read_text(encoding='utf-8'))['frames'][0]
    result={'job':{'pose':frame['pose']},'matcher':frame['result'],'official_acceptance':False}
    class Worker:
        def poll(self):return [result]
    class Reporter:
        def __init__(self):self.events=[]
        def emit(self,event,payload):
            self.events.append(DiagnosticReporter._encode(event,payload));return True
    reporter=Reporter();recorder=GroundMatchResultRecorder(Worker(),reporter)
    assert recorder.poll()[0] is result
    assert recorder.flush(timeout_s=3.)
    assert reporter.events,'real worker result is rejected by 64KiB diagnostic transport'
    import base64,gzip,hashlib
    parts=[json.loads(e)['data'] for e in reporter.events]
    packed=base64.b64decode(''.join(p['base64_part'] for p in parts),validate=True);raw=gzip.decompress(packed)
    assert len(raw)==parts[0]['uncompressed_bytes'] and len(packed)==parts[0]['compressed_bytes']
    assert hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
    assert hashlib.sha256(packed).hexdigest()==parts[0]['compressed_sha256']
    assert json.loads(raw)==result
    recorder.close(timeout_s=.5)
