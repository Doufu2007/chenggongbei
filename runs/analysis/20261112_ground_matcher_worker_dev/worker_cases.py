"""Prototype-only tests, outside production src/tests."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
import importlib.util
import pytest


def load():
    spec=importlib.util.find_spec('latest_frame_worker')
    assert spec is not None,'worker prototype is missing'
    import latest_frame_worker
    return latest_frame_worker


def job(seq=1):
    import hashlib
    return {'job_seq':seq,'session_id':'run-a','first_ID':'ground_2','stationary_epoch':1,
            'source_jpeg':b'original-jpeg','source_sha256':hashlib.sha256(b'original-jpeg').hexdigest(),
            'image_stamp_s':65.379,'frame_id':'uav/camera_link','topic':'/uav/camera/image',
            'pose':{'stamp_s':65.36,'position':[.5,0,.055],'quaternion':[0,0,0,1]},
            'K':[[476.703,0,400.5],[0,476.703,400.5],[0,0,1]],
            'camera_info':{'width':800,'height':800,'k':[476.703,0,400.5,0,476.703,400.5,0,0,1],
                           'd':[0,0,0,0,0]}}


def test_mailbox_holds_only_latest_pending_and_counts_replacement():
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1)
    m.submit(job(1));m.submit(job(2));m.submit(job(3))
    assert m.take_pending()['job_seq']==3
    assert m.take_pending() is None
    assert m.replaced_pending==2


def test_snapshot_preserves_original_binding_against_caller_mutation():
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1)
    original=job();m.submit(original);original['pose']['position'][0]=99
    assert m.take_pending()['pose']['position']==[.5,0,.055]


def test_context_change_discards_pending_and_marks_old_result_ineligible():
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1);m.submit(job())
    m.set_context('run-a','ground_2',2)
    assert m.take_pending() is None
    assert m.context_discarded_pending==1
    result=m.record_result({'job':job(),'official_acceptance':False})
    assert result['matches_current_context'] is False
    assert result['eligible_for_current_context'] is False


@pytest.mark.parametrize('mutation',['old_seq','bad_stamp','bad_hash','context','large_payload'])
def test_invalid_jobs_are_rejected(mutation):
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1);m.submit(job())
    x=job(2)
    if mutation=='old_seq':x['job_seq']=1
    elif mutation=='bad_stamp':x['image_stamp_s']=float('nan')
    elif mutation=='bad_hash':x['source_sha256']='0'*64
    elif mutation=='context':x['first_ID']='ground_0'
    elif mutation=='large_payload':x['source_jpeg']=b'x'*(2*1024*1024+1)
    with pytest.raises(ValueError):m.submit(x)


def test_even_current_result_is_explicitly_not_official_acceptance():
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1)
    r=m.record_result({'job':job(),'official_acceptance':True,'candidates':[{'class_id':'x'}]})
    assert r['official_acceptance'] is False
    assert r['matches_current_context'] is True
    assert r['candidates']==[{'class_id':'x'}]
    assert r['eligible_for_recognition'] is False
    assert r['fresh_exposure_validated'] is False


def test_future_pose_and_nonzero_distortion_are_rejected():
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1)
    x=job();x['pose']['stamp_s']=66.
    with pytest.raises(ValueError):m.submit(x)
    x=job();x['camera_info']={'d':[.01,0,0,0,0]}
    with pytest.raises(ValueError):m.submit(x)


def test_changing_epoch_immediately_invalidates_already_cached_result():
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1)
    m.record_result({'job':job(),'official_acceptance':False})
    assert m.latest_result['matches_current_context'] is True
    m.set_context('run-a','ground_2',2)
    assert m.latest_result['matches_current_context'] is False
    assert m.latest_result['eligible_for_current_context'] is False


def test_K_must_match_original_camera_info():
    m=load().LatestMailbox();m.set_context('run-a','ground_2',1);x=job()
    x['K'][0][0]=900.
    with pytest.raises(ValueError):m.submit(x)


def test_original_jpeg_dimensions_are_checked_before_decode():
    import io
    from PIL import Image
    buffer=io.BytesIO();Image.new('RGB',(320,240)).save(buffer,format='JPEG')
    with pytest.raises(ValueError):load().validate_original_jpeg_header(buffer.getvalue())
    buffer=io.BytesIO();Image.new('RGB',(800,800)).save(buffer,format='JPEG')
    assert load().validate_original_jpeg_header(buffer.getvalue())==(800,800)
