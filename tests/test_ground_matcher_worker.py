"""Pure worker contracts; synthetic input only, no external template library."""
from __future__ import annotations
from copy import deepcopy
import hashlib
import importlib.util
import time

import cv2
import numpy as np
import pytest


def api():
    spec=importlib.util.find_spec('inspection_mission.ground_matcher_worker')
    assert spec is not None,'production ground_matcher_worker is missing'
    from inspection_mission import ground_matcher_worker
    return ground_matcher_worker


def job(seq=1):
    ok,raw=cv2.imencode('.jpg',np.zeros((800,800,3),np.uint8));assert ok
    data=raw.tobytes();K=[[500.,0.,400.5],[0.,500.,400.5],[0.,0.,1.]]
    return {'job_seq':seq,'session_id':'session-a','frozen_first_ID':'ground_3','stationary_epoch':1,
      'source_jpeg':data,'source_sha256':hashlib.sha256(data).hexdigest(),
      'image_stamp_s':10.01,'topic':'/uav/camera/image','frame_id':'uav/camera_link',
      'pose':{'stamp_s':10.,'position':[.5,-1.,.055],'quaternion':[0,0,0,1]},
      'K':K,'camera_info':{'width':800,'height':800,'k':sum(deepcopy(K),[]),'d':[0.]*5,
         'header':{'stamp':{'sec':9,'nanosec':0},'frame_id':'uav/camera_link'}}}


def mailbox():
    m=api().LatestFrameMailbox();m.set_context('session-a','ground_3',1);return m


def test_only_latest_pending_survives_and_original_input_is_snapshotted():
    m=mailbox();one=job(1);m.submit(one);one['pose']['position'][0]=99.
    assert m.take_pending()['pose']['position'][0]==.5
    m.submit(job(2));m.submit(job(3));assert m.take_pending()['job_seq']==3
    assert m.take_pending() is None and m.replaced_pending==1


def test_cached_result_cannot_become_current_again_after_epoch_reversion():
    m=mailbox();m.submit(job());source=m.take_pending()
    m.record_result({'job':source,'matcher':{'candidates':[]}})
    assert m.latest_result['matches_current_context']
    m.set_context('session-a','ground_3',2)
    assert not m.latest_result['matches_current_context']
    m.set_context('session-a','ground_3',1)
    assert not m.latest_result['matches_current_context']
    assert not m.latest_result['eligible_for_recognition']


def test_cancel_clears_pending_and_invalidates_inflight_and_cached_bindings():
    m=mailbox();m.submit(job());source=m.take_pending();m.submit(job(2))
    m.cancel_context();assert m.take_pending() is None
    result=m.record_result({'job':source,'matcher':{'candidates':[]}})
    assert not result['matches_current_context'] and not result['official_acceptance']


@pytest.mark.parametrize('mutation',['future_pose','wrong_hash','wrong_K','distortion','oversize','wrong_context','old_seq','bad_quaternion'])
def test_invalid_original_binding_is_rejected(mutation):
    m=mailbox();m.submit(job());x=job(2)
    if mutation=='future_pose':x['pose']['stamp_s']=11.
    elif mutation=='wrong_hash':x['source_sha256']='0'*64
    elif mutation=='wrong_K':x['K'][0][0]=900.
    elif mutation=='distortion':x['camera_info']['d'][0]=.01
    elif mutation=='oversize':x['source_jpeg']=b'x'*(2*1024*1024+1)
    elif mutation=='wrong_context':x['frozen_first_ID']='ground_0'
    elif mutation=='old_seq':x['job_seq']=1
    elif mutation=='bad_quaternion':x['pose']['quaternion']=[0,0,0,0]
    with pytest.raises(ValueError):m.submit(x)


def test_jpeg_dimensions_are_validated_without_pillow():
    ok,raw=cv2.imencode('.jpg',np.zeros((240,320,3),np.uint8));assert ok
    with pytest.raises(ValueError):api().jpeg_dimensions(raw.tobytes())
    assert api().jpeg_dimensions(job()['source_jpeg'])==(800,800)


def test_error_result_and_current_context_do_not_authorize_recognition():
    m=mailbox();m.submit(job());result=m.record_result({'job':m.take_pending(),'error':'failure'})
    assert result['matches_current_context']
    assert not result['fresh_exposure_validated'] and not result['eligible_for_recognition']
    assert not result['official_acceptance']


def test_spawned_synthetic_worker_returns_original_binding_and_bounded_close(tmp_path):
    module=api()
    from inspection_mission.ground_plane_matcher import TemplateSpec
    rng=np.random.default_rng(90);pattern=rng.integers(0,256,(100,100,3),np.uint8)
    path=tmp_path/'pattern.png';ok,raw=cv2.imencode('.png',pattern);assert ok;path.write_bytes(raw.tobytes())
    spec=TemplateSpec(path,'synthetic','synthetic',{'source':'synthetic worker test'},hashlib.sha256(path.read_bytes()).hexdigest())
    worker=module.GroundMatcherWorker([spec]);worker.set_context('session-a','ground_3',1)
    try:
        worker.submit(job());result=None;deadline=time.perf_counter()+10.
        while time.perf_counter()<deadline:
            found=worker.poll()
            if found:result=found[0];break
            time.sleep(.01)
        assert result is not None
        assert 'error' not in result
        assert result['job']['image_stamp_s']==10.01
        assert result['job']['pose']['stamp_s']==10.
        assert result['worker_pid']!=__import__('os').getpid()
        assert result['matcher']['candidates']==[]
        assert result['worker_finished_monotonic_s']>=result['worker_started_monotonic_s']
        assert not result['official_acceptance']
    finally:
        before=time.perf_counter();status=worker.close(timeout_s=.5)
        assert time.perf_counter()-before<1.0
        assert status['closed']
    with pytest.raises(ValueError):worker.submit(job(2))


def test_close_is_bounded_during_initialization(tmp_path):
    module=api()
    from inspection_mission.ground_plane_matcher import TemplateSpec
    path=tmp_path/'blank.png';ok,raw=cv2.imencode('.png',np.zeros((64,64,3),np.uint8));assert ok;path.write_bytes(raw.tobytes())
    worker=module.GroundMatcherWorker([TemplateSpec(path,'x','x',{'source':'synthetic'})])
    start=time.perf_counter();status=worker.close(timeout_s=.2)
    assert time.perf_counter()-start<.8
    assert status['closed'] and not status['process_alive']


def test_unexpected_worker_exit_returns_bound_error_instead_of_waiting_forever(tmp_path):
    module=api()
    from inspection_mission.ground_plane_matcher import TemplateSpec
    path=tmp_path/'blank.png';ok,raw=cv2.imencode('.png',np.zeros((64,64,3),np.uint8));assert ok;path.write_bytes(raw.tobytes())
    worker=module.GroundMatcherWorker([TemplateSpec(path,'x','x',{'source':'synthetic crash test'})])
    try:
        worker.set_context('session-a','ground_3',1);worker.submit(job());worker.poll()
        worker._process.terminate();worker._process.join(timeout=1.)  # Simulate independent OS/process failure.
        result=worker.poll()
        assert result and 'worker_process_exited' in result[0]['error']
        assert result[0]['job']['source_sha256']==job()['source_sha256']
        assert result[0]['job_was_dispatched'] is True
        assert result[0]['job_was_executed'] is None
        assert not result[0]['eligible_for_recognition']
    finally:worker.close(timeout_s=.2)


def test_exit_before_first_dispatch_preserves_pending_source_binding(tmp_path):
    module=api()
    from inspection_mission.ground_plane_matcher import TemplateSpec
    path=tmp_path/'blank.png';ok,raw=cv2.imencode('.png',np.zeros((64,64,3),np.uint8));assert ok;path.write_bytes(raw.tobytes())
    worker=module.GroundMatcherWorker([TemplateSpec(path,'x','x',{'source':'synthetic'})])
    try:
        worker.set_context('session-a','ground_3',1);worker.submit(job())
        worker._process.terminate();worker._process.join(timeout=1.)
        result=worker.poll()[0]
        assert result['job']['source_sha256']==job()['source_sha256']
        assert result['job']['image_stamp_s']==10.01
        assert result['job_was_dispatched'] is False
        assert result['job_was_executed'] is False
    finally:worker.close(timeout_s=.2)


def test_initialization_error_before_dispatch_preserves_pending_source_binding(tmp_path):
    module=api()
    from inspection_mission.ground_plane_matcher import TemplateSpec
    worker=module.GroundMatcherWorker([TemplateSpec(tmp_path/'missing.png','x','x',{'source':'synthetic failure'})])
    try:
        worker.set_context('session-a','ground_3',1);worker.submit(job())
        # Exercise actual error handling without waiting for a costly template library.
        worker._completed.put_nowait({'initialization_error':'synthetic initialization failure'})
        result=worker.poll()[0]
        assert result['job']['source_sha256']==job()['source_sha256']
        assert result['job']['frozen_first_ID']=='ground_3'
        assert result['job_was_dispatched'] is False and result['job_was_executed'] is False
    finally:worker.close(timeout_s=.2)
