def test_rejected_worker_result_is_recorded_without_authorizing_or_replacing_it():
    from inspection_mission.ground_match_result_recorder import GroundMatchResultRecorder
    result = {'job': {'job_seq': 1}, 'matcher': {'candidates': []}, 'official_acceptance': False}
    class Worker:
        def poll(self):
            return [result]
    class Reporter:
        def __init__(self):
            self.events = []
        def emit(self, event, payload):
            self.events.append((event, payload))
    reporter = Reporter()
    recorder = GroundMatchResultRecorder(Worker(), reporter)
    assert recorder.poll()[0] is result
    assert recorder.flush(timeout_s=2.)
    assert reporter.events[0][0] == 'probe_snapshot'
    data = reporter.events[0][1]
    assert data['section'] == 'ground_worker_result_gzip_part'
    assert data['official_acceptance'] is False
    import base64,gzip,json
    assert json.loads(gzip.decompress(base64.b64decode(data['base64_part'])))==result
    recorder.close(timeout_s=.5)


def test_evidence_failure_never_changes_worker_results():
    from inspection_mission.ground_match_result_recorder import GroundMatchResultRecorder
    values = [{'error': 'no-match'}]
    class Worker:
        def poll(self):
            return values
    class BrokenReporter:
        def emit(self, *args):
            raise ValueError('payload limit')
    recorder = GroundMatchResultRecorder(Worker(), BrokenReporter())
    assert recorder.poll() is values
    assert recorder.flush(timeout_s=2.)
    assert recorder.report_errors == 1
    recorder.close(timeout_s=.5)


def test_large_realistic_result_roundtrips_full_receiver_contract_on_background_thread():
    import base64,gzip,hashlib,json,threading
    from inspection_mission.ground_match_result_recorder import GroundMatchResultRecorder
    from inspection_mission.telemetry import DiagnosticReporter
    from tools.diagnostic_receiver import validate_event
    result={'job':{'pose':{'position':[.1,.2,.055],'quaternion':[0.,0.,0.,1.]}},
        'matcher':{'diagnostics':[{'matches':[{'template_xy':[i+.1,i+.2],'bev_xy':[i*.3,i*.4],
            'distance':float(i),'inlier':i%2==0} for i in range(5000)]}]},'official_acceptance':False}
    values=[result];owner=threading.get_ident();events=[]
    class Worker:
        def poll(self):return values
    class Reporter:
        def emit(self,event,payload):
            assert threading.get_ident()!=owner
            events.append(validate_event(DiagnosticReporter._encode(event,payload)));return True
    recorder=GroundMatchResultRecorder(Worker(),Reporter())
    assert recorder.poll() is values and values[0] is result
    assert recorder.flush(timeout_s=3.) and not recorder.report_errors
    parts=[e['data'] for e in events];assert parts and len(parts)>1
    packed=base64.b64decode(''.join(p['base64_part'] for p in parts),validate=True);raw=gzip.decompress(packed)
    assert len(raw)==parts[0]['uncompressed_bytes'] and len(packed)==parts[0]['compressed_bytes']
    assert hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
    assert hashlib.sha256(packed).hexdigest()==parts[0]['compressed_sha256']
    assert json.loads(raw)==result
    recorder.close(timeout_s=.5)


def test_slow_diagnostic_transport_does_not_block_poll_and_pending_snapshots_are_bounded():
    import threading,time
    from inspection_mission.ground_match_result_recorder import GroundMatchResultRecorder
    entered,release=threading.Event(),threading.Event()
    values=[{'job':{'sequence':0},'matcher':{'candidates':[]}}]
    class Worker:
        def poll(self):return values
    class Reporter:
        def emit(self,*args):entered.set();release.wait(2.);return True
    recorder=GroundMatchResultRecorder(Worker(),Reporter())
    assert recorder.poll() is values and entered.wait(.5)
    try:
        started=time.monotonic()
        for _ in range(8):assert recorder.poll() is values
        assert time.monotonic()-started<.1
        assert recorder._queue.qsize()<=2 and recorder.recorded<=3 and recorder.dropped>=6
        assert not recorder.flush(timeout_s=.01)
    finally:
        release.set();assert recorder.flush(timeout_s=2.);recorder.close(timeout_s=.5)


def test_result_limit_snapshot_integrity_and_false_transport_are_honest():
    import base64,gzip,json
    from inspection_mission.ground_match_result_recorder import GroundMatchResultRecorder
    value={'job':{'id':1},'matcher':{'candidates':[]}};events=[]
    class Worker:
        def poll(self):return [value]
    class Reporter:
        def emit(self,event,payload):events.append(payload);return True
    recorder=GroundMatchResultRecorder(Worker(),Reporter())
    for _ in range(23):recorder.poll();assert recorder.flush(timeout_s=1.)
    assert recorder.recorded==recorder.completed==18 and len(events)==18
    assert json.loads(gzip.decompress(base64.b64decode(events[0]['base64_part'])))==value
    recorder.close(timeout_s=.5)
    class FailedReporter:
        def emit(self,*args):return False
    failed=GroundMatchResultRecorder(Worker(),FailedReporter());failed.poll()
    assert failed.flush(timeout_s=1.) and failed.report_errors==1 and failed.completed==0
    failed.close(timeout_s=.5)
