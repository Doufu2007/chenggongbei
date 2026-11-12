"""Prototype only: one spawned matcher + one replaceable pending frame.

There is no ROS, controller, publish interface, or freshness override. All
public methods belong to one main-loop owner; no control lock is acquired.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import time
import io

ROOT=Path(__file__).resolve().parents[3]
FROZEN_MODULE_SHA='1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d'
MAX_JPEG_BYTES=2*1024*1024
_MATCHER=None
_INITIALIZATION=None


def validate_original_jpeg_header(raw):
    from PIL import Image
    with Image.open(io.BytesIO(raw)) as image:
        if image.format!='JPEG' or image.size!=(800,800):
            raise ValueError('worker prototype accepts only original 800x800 JPEG')
        return image.size


class LatestMailbox:
    """Single-owner bounded metadata/payload mailbox, independent of executor."""
    def __init__(self):
        self.context=None;self.pending=None;self.latest_result=None
        self.last_job_seq=-1;self.replaced_pending=0;self.context_discarded_pending=0
        self.results_received=0;self.replaced_latest_result=0

    def set_context(self,session_id,first_ID,stationary_epoch):
        if not isinstance(session_id,str) or not session_id or not isinstance(first_ID,str) or not first_ID:
            raise ValueError('session_id and first_ID must be explicit nonempty strings')
        if not isinstance(stationary_epoch,int) or isinstance(stationary_epoch,bool) or stationary_epoch<0:
            raise ValueError('stationary_epoch must be a nonnegative integer')
        context=(session_id,first_ID,stationary_epoch)
        if context!=self.context and self.pending is not None:
            self.pending=None;self.context_discarded_pending+=1
        self.context=context
        if self.latest_result is not None:
            job=self.latest_result['job']
            same=(job['session_id'],job['first_ID'],job['stationary_epoch'])==context
            self.latest_result['matches_current_context']=same
            self.latest_result['eligible_for_current_context']=same

    def submit(self,job):
        required=('job_seq','session_id','first_ID','stationary_epoch','source_jpeg','source_sha256',
                  'image_stamp_s','frame_id','topic','pose','K','camera_info')
        if any(k not in job for k in required):raise ValueError('job lacks original binding fields')
        seq=job['job_seq']
        if not isinstance(seq,int) or isinstance(seq,bool) or seq<=self.last_job_seq:raise ValueError('job_seq must increase')
        if (job['session_id'],job['first_ID'],job['stationary_epoch'])!=self.context:raise ValueError('job context differs')
        raw=job['source_jpeg']
        if not isinstance(raw,bytes) or not 0<len(raw)<=MAX_JPEG_BYTES:raise ValueError('source JPEG exceeds bounded payload')
        if hashlib.sha256(raw).hexdigest()!=job['source_sha256']:raise ValueError('original JPEG SHA256 mismatch')
        if not isinstance(job['image_stamp_s'],(int,float)) or not math.isfinite(job['image_stamp_s']) or job['image_stamp_s']<0:
            raise ValueError('original exposure stamp must be finite')
        if not isinstance(job['frame_id'],str) or not job['frame_id']:raise ValueError('original frame_id missing')
        if not isinstance(job['topic'],str) or not job['topic']:raise ValueError('original topic missing')
        pose=job['pose']
        if not isinstance(pose,dict) or not isinstance(pose.get('stamp_s'),(int,float)) or not math.isfinite(pose['stamp_s']) or pose['stamp_s']>job['image_stamp_s']:
            raise ValueError('pose must preserve a finite prior source stamp')
        if len(pose.get('position',[]))!=3 or len(pose.get('quaternion',[]))!=4:
            raise ValueError('pose must preserve position/quaternion')
        if any(x!=0 for x in job['camera_info'].get('d',[])):
            raise ValueError('nonzero distortion needs explicit upstream rectification')
        info=job['camera_info']
        if info.get('width')!=800 or info.get('height')!=800:
            raise ValueError('original CameraInfo must describe 800x800 pixels')
        try:
            K=job['K'];flat=[value for row in K for value in row]
            if len(K)!=3 or any(len(row)!=3 for row in K) or len(info['k'])!=9 or any(abs(a-b)>1e-9 for a,b in zip(flat,info['k'])):
                raise ValueError('K differs from original CameraInfo')
        except (KeyError,TypeError) as exc:raise ValueError('original K/CameraInfo missing') from exc
        meta={k:v for k,v in job.items() if k!='source_jpeg'}
        try:serialized=json.dumps(meta,allow_nan=False)
        except (TypeError,ValueError) as exc:raise ValueError('job metadata must be finite JSON') from exc
        if len(serialized.encode('utf-8'))>64*1024:raise ValueError('metadata exceeds bounded size')
        if self.pending is not None:self.replaced_pending+=1
        self.pending=deepcopy(job);self.last_job_seq=seq

    def take_pending(self):
        job=self.pending;self.pending=None;return job

    def record_result(self,result):
        value=deepcopy(result);job=value['job']
        same=(job['session_id'],job['first_ID'],job['stationary_epoch'])==self.context
        value['matches_current_context']=same
        value['eligible_for_current_context']=same  # Binding only; never image freshness or report authority.
        value['current_context_eligibility_is_not_freshness_validation']=True
        value['official_acceptance']=False
        value['fresh_exposure_validated']=False
        value['eligible_for_recognition']=False
        if self.latest_result is not None:self.replaced_latest_result+=1
        self.latest_result=value;self.results_received+=1
        return value


def _initialize(manifest_path):
    global _MATCHER,_INITIALIZATION
    import sys
    import cv2
    sys.path.insert(0,str(ROOT/'src/inspection_mission'))
    from inspection_mission.ground_plane_matcher import GroundPlaneMatcher,TemplateSpec
    start=time.perf_counter()
    module=ROOT/'src/inspection_mission/inspection_mission/ground_plane_matcher.py'
    if hashlib.sha256(module.read_bytes()).hexdigest()!=FROZEN_MODULE_SHA:raise ValueError('Frozen module changed')
    path=Path(manifest_path);data=json.loads(path.read_text(encoding='utf-8'))
    specs=[TemplateSpec(x['absolute_path'],x['class_id'],x['label'],
            {'source_url':x['source_url'],'git_blob_sha':x['git_blob_sha'],'official_current_library_verified':False},x['sha256'])
           for x in data['references']]
    cv2.setNumThreads(1);_MATCHER=GroundPlaneMatcher(specs)
    _INITIALIZATION={'worker_pid':os.getpid(),'opencv_threads':cv2.getNumThreads(),
          'template_load_ms':(time.perf_counter()-start)*1000,'module_sha256':FROZEN_MODULE_SHA,
          'template_manifest_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
          'templates':[{'path':str(s.path),'class_id':s.class_id,'label':s.label,
                        'sha256':s.expected_sha256,'provenance':dict(s.provenance)} for s in specs]}


def _match(job):
    import cv2
    import numpy as np
    start=time.perf_counter();raw=job['source_jpeg']
    validate_original_jpeg_header(raw)
    image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    decode_ms=(time.perf_counter()-start)*1000
    matched=time.perf_counter();result=_MATCHER.match(image,np.array(job['K'],np.float64),height_m=.173)
    match_ms=(time.perf_counter()-matched)*1000
    result.pop('bev_bgr')
    return {'job':{k:v for k,v in job.items() if k!='source_jpeg'},'matcher':result,
            'worker':deepcopy(_INITIALIZATION),'timing':{'decode_ms':decode_ms,'match_ms':match_ms,
             'total_worker_job_ms':(time.perf_counter()-start)*1000},'official_acceptance':False}


class LatestFrameWorker:
    def __init__(self,template_manifest):
        self.mailbox=LatestMailbox();self._future=None;self._active_job=None;self._closed=False
        self._executor=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),
                initializer=_initialize,initargs=(str(Path(template_manifest).resolve()),))
        self.dispatched=0;self.max_in_flight=0

    def set_context(self,*context):self.mailbox.set_context(*context)
    def submit(self,job):
        if self._closed:raise ValueError('worker closed')
        self.mailbox.submit(job)

    def poll(self):
        received=[]
        if self._future is not None and self._future.done():
            try:value=self._future.result()
            except Exception as exc:
                value={'job':{k:v for k,v in self._active_job.items() if k!='source_jpeg'},
                       'error':type(exc).__name__+': '+str(exc),'official_acceptance':False}
            received.append(self.mailbox.record_result(value));self._future=None;self._active_job=None
        if self._future is None and not self._closed:
            job=self.mailbox.take_pending()
            if job is not None:
                self._active_job=job;self._future=self._executor.submit(_match,job)
                self.dispatched+=1;self.max_in_flight=1
        return received

    @property
    def busy(self):return self._future is not None or self.mailbox.pending is not None

    def close(self):
        self._closed=True;self.mailbox.pending=None
        self._executor.shutdown(wait=True,cancel_futures=True)
