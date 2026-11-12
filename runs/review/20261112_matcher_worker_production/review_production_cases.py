"""Independent production-worker lifecycle counterexamples; no ROS or movement."""
from pathlib import Path
from copy import deepcopy
import hashlib
import queue
import sys

import cv2
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/inspection_mission'))
from inspection_mission.ground_matcher_worker import GroundMatcherWorker, LatestFrameMailbox


def original_job(seq=1):
    ok,encoded=cv2.imencode('.jpg',np.zeros((800,800,3),np.uint8)); assert ok
    raw=encoded.tobytes(); K=[[476.7,0.,400.5],[0.,476.7,400.5],[0.,0.,1.]]
    return {'job_seq':seq,'session_id':'independent-review','frozen_first_ID':'ground_1','stationary_epoch':3,
        'source_jpeg':raw,'source_sha256':hashlib.sha256(raw).hexdigest(),'image_stamp_s':12.02,
        'topic':'/uav/camera/image','frame_id':'uav/camera_link',
        'pose':{'stamp_s':12.,'position':[.6,-.5,.055],'quaternion':[0.,0.,0.,1.]},
        'K':K,'camera_info':{'width':800,'height':800,'k':sum(deepcopy(K),[]),'d':[0.]*5,
            'header':{'stamp':{'sec':12,'nanosec':0},'frame_id':'uav/camera_link'}}}


class AlreadyExited:
    exitcode=7
    def is_alive(self): return False


def pending_owner():
    # Deterministic scheduling: submit happens before first poll; child exits in initialization.
    worker=GroundMatcherWorker.__new__(GroundMatcherWorker)
    worker._closed=False; worker._broken=False; worker._active=None
    worker._completed=queue.Queue(maxsize=1); worker._incoming=queue.Queue(maxsize=1)
    worker._process=AlreadyExited(); worker.dispatched=0
    worker.mailbox=LatestFrameMailbox(); worker.mailbox.set_context('independent-review','ground_1',3)
    worker.submit(original_job())
    return worker


@pytest.mark.parametrize('initialization_message',[False,True])
def test_failure_before_first_poll_preserves_submitted_source_binding(initialization_message):
    worker=pending_owner()
    if initialization_message: worker._completed.put_nowait({'initialization_error':'template load failed'})
    result=worker.poll()
    assert len(result)==1 and 'error'in result[0]
    assert result[0]['job'].get('source_sha256')==original_job()['source_sha256']
    assert result[0]['job'].get('image_stamp_s')==12.02
    assert not result[0]['eligible_for_recognition'] and not result[0]['official_acceptance']
    assert not worker.busy and worker._broken


def test_invalid_submission_preserves_original_pending_and_sequence():
    m=LatestFrameMailbox(); m.set_context('independent-review','ground_1',3);m.submit(original_job())
    bad=original_job(2); bad['pose']['stamp_s']=13.
    with pytest.raises(ValueError): m.submit(bad)
    assert m.last_job_seq==1 and m.pending['job_seq']==1


def test_dispatched_child_death_has_unknown_execution_without_inventing_failure_stage():
    worker=pending_owner(); worker._active=worker.mailbox.take_pending()
    result=worker.poll()[0]
    assert result['job']['source_sha256']==original_job()['source_sha256']
    assert result['job_was_dispatched']is True
    assert result['job_was_executed']is None
    assert not result['eligible_for_recognition']and not result['official_acceptance']


def test_result_after_context_round_trip_cannot_reactivate_old_generation():
    m=LatestFrameMailbox(); m.set_context('independent-review','ground_1',3);m.submit(original_job())
    old=m.take_pending(); m.set_context('independent-review','ground_1',4);m.set_context('independent-review','ground_1',3)
    result=m.record_result({'job':old,'matcher':{'candidates':[]}})
    assert not result['matches_current_context'] and not result['eligible_for_current_context']
    assert not result['eligible_for_recognition'] and not result['official_acceptance']


def test_zero_distortion_nan_in_original_camera_info_is_rejected():
    m=LatestFrameMailbox();m.set_context('independent-review','ground_1',3); bad=original_job()
    bad['camera_info']['k'][0]=float('nan')
    with pytest.raises(ValueError):m.submit(bad)
