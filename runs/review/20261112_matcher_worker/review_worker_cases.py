"""Independent mailbox counterexamples; never starts ROS or a worker process."""
from pathlib import Path
import hashlib
import importlib.util

import pytest

ROOT=Path(__file__).resolve().parents[3]
PATH=ROOT/'runs/analysis/20261112_ground_matcher_worker_dev/latest_frame_worker.py'
spec=importlib.util.spec_from_file_location('review_latest_frame_worker',PATH)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def job():
    raw=b'review-original-bytes'
    return {'job_seq':1,'session_id':'review-session','first_ID':'ground_4',
        'stationary_epoch':1,'source_jpeg':raw,'source_sha256':hashlib.sha256(raw).hexdigest(),
        'image_stamp_s':65.379,'frame_id':'uav/camera_link','topic':'/uav/camera/image',
        'pose':{'stamp_s':65.36,'position':[.5,0,.055],'quaternion':[0,0,0,1]},
        'K':[[476.703,0,400.5],[0,476.703,400.5],[0,0,1]]}


def test_changing_epoch_invalidates_already_cached_current_result():
    m=module.LatestMailbox();m.set_context('review-session','ground_4',1)
    m.record_result({'job':job(),'matcher':{'candidates':[]}})
    assert m.latest_result['matches_current_context'] is True
    m.set_context('review-session','ground_4',2)
    assert (m.latest_result is None or m.latest_result['matches_current_context'] is False)
    assert (m.latest_result is None or m.latest_result['eligible_for_current_context'] is False)


def test_rejected_context_does_not_consume_sequence_or_replace_pending():
    m=module.LatestMailbox();m.set_context('review-session','ground_4',1)
    first=job();m.submit(first)
    bad=job();bad['job_seq']=2;bad['stationary_epoch']=99
    with pytest.raises(ValueError):m.submit(bad)
    assert m.last_job_seq==1 and m.pending['job_seq']==1


def test_same_context_never_means_recognition_eligible_or_fresh():
    m=module.LatestMailbox();m.set_context('review-session','ground_4',1)
    r=m.record_result({'job':job(),'error':'decode failed'})
    assert r['official_acceptance'] is False
    assert r.get('eligible_for_recognition',False) is False
    assert r.get('fresh_exposure_validated',False) is False


@pytest.mark.parametrize('mutation',['future_pose','nonzero_distortion'])
def test_bad_exposure_geometry_binding_is_rejected(mutation):
    m=module.LatestMailbox();m.set_context('review-session','ground_4',1)
    item=job()
    if mutation=='future_pose':item['pose']['stamp_s']=66.
    else:item['camera_info']={'d':[.01,0,0,0,0]}
    with pytest.raises(ValueError):m.submit(item)
