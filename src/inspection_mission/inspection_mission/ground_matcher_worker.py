"""Bounded latest-frame ground matcher process, independent of ROS/control.

One main-loop owner calls this API. A spawned process owns SIFT; a daemon reader
deserializes its result outside the owner loop. Context matching never validates
image freshness or authorizes recognition. Templates are explicit local specs.
"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import multiprocessing
import os
import queue
import struct
import threading
import time


MAX_JPEG_BYTES = 2*1024*1024
MAX_METADATA_BYTES = 64*1024


def jpeg_dimensions(raw):
    """Read JPEG SOF dimensions without decoding or requiring Pillow."""
    if not isinstance(raw, bytes) or not 4 <= len(raw) <= MAX_JPEG_BYTES or raw[:2] != b'\xff\xd8':
        raise ValueError('original JPEG bytes missing or exceed 2MB')
    index = 2
    while index+3 < len(raw):
        if raw[index] != 255:
            raise ValueError('invalid JPEG marker stream')
        while index < len(raw) and raw[index] == 255:
            index += 1
        if index >= len(raw): break
        marker = raw[index]; index += 1
        if marker in (0xd8, 0x01) or 0xd0 <= marker <= 0xd7: continue
        if marker in (0xd9, 0xda): break
        if index+2 > len(raw): break
        size = struct.unpack_from('>H', raw, index)[0]
        if size < 2 or index+size > len(raw): break
        if marker in (0xc0,0xc1,0xc2,0xc3,0xc5,0xc6,0xc7,0xc9,0xca,0xcb,0xcd,0xce,0xcf):
            if size < 8: break
            height, width = struct.unpack_from('>HH', raw, index+3)
            if (width,height) != (800,800):
                raise ValueError('only original 800x800 JPEGs are supported')
            return width,height
        index += size
    raise ValueError('JPEG dimensions unavailable before scan')


class LatestFrameMailbox:
    """Single-owner active-context metadata and one replaceable pending frame."""
    def __init__(self):
        self.context = None
        self.generation = 0
        self.pending = None
        self.latest_result = None
        self.last_job_seq = -1
        self.replaced_pending = 0
        self.discarded_pending = 0

    def _refresh(self, result):
        job = result.get('job') or {}
        same = (self.context is not None and job.get('_context_generation') == self.generation
                and (job.get('session_id'),job.get('frozen_first_ID'),job.get('stationary_epoch')) == self.context)
        result.update(matches_current_context=bool(same), eligible_for_current_context=bool(same),
                      fresh_exposure_validated=False, eligible_for_recognition=False, official_acceptance=False)

    def set_context(self, session_id, frozen_first_ID, stationary_epoch):
        if not isinstance(session_id,str) or not session_id or not isinstance(frozen_first_ID,str) or not frozen_first_ID:
            raise ValueError('session and frozen first ID must be explicit')
        if not isinstance(stationary_epoch,int) or isinstance(stationary_epoch,bool) or stationary_epoch < 0:
            raise ValueError('stationary epoch must be a nonnegative integer')
        context = (session_id,frozen_first_ID,stationary_epoch)
        if context != self.context:
            self.cancel_context()
            self.context = context

    def cancel_context(self):
        self.generation += 1
        self.context = None
        if self.pending is not None:
            self.pending = None; self.discarded_pending += 1
        if self.latest_result is not None: self._refresh(self.latest_result)

    def submit(self, job):
        keys = ('job_seq','session_id','frozen_first_ID','stationary_epoch','source_jpeg','source_sha256',
                'image_stamp_s','frame_id','topic','pose','K','camera_info')
        if not isinstance(job,dict) or any(k not in job for k in keys): raise ValueError('original job binding incomplete')
        seq = job['job_seq']
        if not isinstance(seq,int) or isinstance(seq,bool) or seq <= self.last_job_seq: raise ValueError('job_seq must increase')
        if (job['session_id'],job['frozen_first_ID'],job['stationary_epoch']) != self.context: raise ValueError('job context differs')
        raw = job['source_jpeg']; jpeg_dimensions(raw)
        if hashlib.sha256(raw).hexdigest() != job['source_sha256']: raise ValueError('original JPEG SHA256 mismatch')
        stamp = job['image_stamp_s']
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp < 0: raise ValueError('exposure stamp invalid')
        if not all(isinstance(job[k],str) and job[k] for k in ('frame_id','topic')): raise ValueError('source frame/topic missing')
        pose = job['pose']
        if not isinstance(pose,dict) or not isinstance(pose.get('stamp_s'),(int,float)) or not math.isfinite(pose['stamp_s']) or not 0 <= pose['stamp_s'] <= stamp:
            raise ValueError('pose must retain a finite prior stamp')
        if len(pose.get('position',[])) != 3 or len(pose.get('quaternion',[])) != 4:
            raise ValueError('original pose position/quaternion invalid')
        try:
            if not all(math.isfinite(x) for x in pose['position']+pose['quaternion']) or sum(x*x for x in pose['quaternion']) < 1e-12:
                raise ValueError('pose contains invalid values')
            info = job['camera_info']; K = job['K']; flat = [x for row in K for x in row]
            if (len(K)!=3 or any(len(row)!=3 for row in K) or len(info['k'])!=9
                    or not all(math.isfinite(x) for x in flat) or any(abs(a-b)>1e-9 for a,b in zip(flat,info['k']))
                    or flat[0]<=0 or flat[4]<=0 or flat[1]!=0 or flat[3]!=0 or flat[6:]!=[0,0,1]
                    or not 0 <= flat[2] < 800 or not 0 <= flat[5] < 800):
                raise ValueError('K differs from original pinhole CameraInfo')
            if info['width']!=800 or info['height']!=800 or len(info['d'])<4 or any(x!=0 for x in info['d']):
                raise ValueError('800x800 zero-distortion CameraInfo required')
            if info['header']['frame_id'] != job['frame_id']: raise ValueError('CameraInfo frame mismatch')
            camera_stamp = info['header']['stamp']
            if not isinstance(camera_stamp['sec'],int) or not isinstance(camera_stamp['nanosec'],int) or not 0 <= camera_stamp['nanosec'] < 10**9:
                raise ValueError('original CameraInfo stamp invalid')
            metadata = {k:v for k,v in job.items() if k!='source_jpeg'}
            encoded = json.dumps(metadata,allow_nan=False).encode('utf-8')
        except (KeyError,TypeError,OverflowError) as exc:
            raise ValueError('original pose/K/CameraInfo metadata invalid') from exc
        if len(encoded)>MAX_METADATA_BYTES: raise ValueError('metadata exceeds 64KB')
        if self.pending is not None: self.replaced_pending += 1
        self.pending = deepcopy(job)
        self.pending['_context_generation'] = self.generation
        self.last_job_seq = seq

    def take_pending(self):
        value = self.pending; self.pending = None; return value

    def record_result(self, result):
        value = deepcopy(result)
        self._refresh(value)
        self.latest_result = value
        return value


def _worker_main(templates, incoming, outgoing):
    """Top-level spawn target: imports same-package matcher, no project root."""
    try:
        import cv2
        import numpy as np
        from .ground_plane_matcher import GroundPlaneMatcher
        cv2.setNumThreads(1)
        initialized = time.perf_counter()
        matcher = GroundPlaneMatcher(templates)
        load_ms = (time.perf_counter()-initialized)*1000
    except Exception as exc:
        outgoing.put({'initialization_error':type(exc).__name__+': '+str(exc)})
        return
    while True:
        job = incoming.get()
        if job is None: return
        start = time.perf_counter(); utc = datetime.now(timezone.utc).isoformat()
        result = {'job':{k:v for k,v in job.items() if k!='source_jpeg'},'worker_pid':os.getpid(),
                  'worker_started_monotonic_s':start,'worker_started_utc':utc,'template_load_ms':load_ms}
        try:
            jpeg_dimensions(job['source_jpeg'])
            image = cv2.imdecode(np.frombuffer(job['source_jpeg'],np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape != (800,800,3): raise ValueError('original JPEG decode failed')
            decoded = time.perf_counter()
            matched = matcher.match(image,np.array(job['K'],np.float64),height_m=.173)
            matched.pop('bev_bgr')
            result.update(matcher=matched,decode_ms=(decoded-start)*1000,match_ms=(time.perf_counter()-decoded)*1000)
        except Exception as exc:
            result['error'] = type(exc).__name__+': '+str(exc)
        result.update(worker_finished_monotonic_s=time.perf_counter(),worker_finished_utc=datetime.now(timezone.utc).isoformat(),official_acceptance=False)
        outgoing.put(result)


class GroundMatcherWorker:
    """One spawned process, one in-flight job, one latest pending job/result.

    poll/submit/cancel do not wait for SIFT. close(timeout_s) first invalidates
    context, then allows a bounded graceful exit and terminates a busy child.
    No method acquires a caller's ROS/control lock or publishes any message.
    """
    def __init__(self, templates):
        if not templates: raise ValueError('explicit templates required')
        self.mailbox = LatestFrameMailbox()
        self._active = None; self._closed = False; self._broken = False
        context = multiprocessing.get_context('spawn')
        self._incoming = context.Queue(maxsize=1)
        self._outgoing = context.Queue(maxsize=1)
        self._completed = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._process = context.Process(target=_worker_main,args=(tuple(templates),self._incoming,self._outgoing),daemon=True)
        self._process.start()
        self._reader = threading.Thread(target=self._receive,name='ground-matcher-result-reader',daemon=True)
        self._reader.start()
        self.dispatched = 0

    def _receive(self):
        while not self._stop.is_set():
            try: result = self._outgoing.get(timeout=.05)
            except queue.Empty: continue
            except (EOFError,OSError,ValueError): return
            try: self._completed.put_nowait(result)
            except queue.Full:
                try: self._completed.get_nowait()
                except queue.Empty: pass
                try: self._completed.put_nowait(result)
                except queue.Full: pass

    def set_context(self,*context):
        if self._closed: raise ValueError('worker closed')
        self.mailbox.set_context(*context)

    def cancel_context(self):
        self.mailbox.cancel_context()

    def submit(self, job):
        if self._closed or self._broken: raise ValueError('worker closed or unavailable')
        self.mailbox.submit(job)

    def poll(self):
        if self._closed: return []
        received = []
        try: result = self._completed.get_nowait()
        except queue.Empty: result = None
        if result is None and not self._process.is_alive() and not self._broken:
            source = self._active or self.mailbox.pending or {}
            result = {'job':{k:v for k,v in source.items() if k!='source_jpeg'},
                      'job_was_dispatched':self._active is not None,
                      'job_was_executed':None if self._active is not None else False,
                      'error':'worker_process_exited: '+str(self._process.exitcode)}
            self._broken = True; self.mailbox.pending = None
        if result is not None:
            if 'initialization_error' in result:
                source = self._active or self.mailbox.pending or {}
                result = {'job':{k:v for k,v in source.items() if k!='source_jpeg'},
                          'job_was_dispatched':self._active is not None,'job_was_executed':False,
                          'error':result['initialization_error']}
                self._broken = True; self.mailbox.pending = None
            received.append(self.mailbox.record_result(result)); self._active = None
        if self._active is None and not self._broken:
            job = self.mailbox.take_pending()
            if job is not None:
                try: self._incoming.put_nowait(job)
                except queue.Full:
                    self.mailbox.pending = job
                else: self._active = job; self.dispatched += 1
        return received

    @property
    def busy(self): return self._active is not None or self.mailbox.pending is not None

    def close(self, *, timeout_s=.5):
        if not isinstance(timeout_s,(int,float)) or not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError('finite nonnegative close timeout required')
        start = time.perf_counter(); self._closed = True; self.mailbox.cancel_context()
        try: self._incoming.put_nowait(None)
        except (queue.Full,ValueError): pass
        deadline = start+timeout_s
        self._process.join(timeout=max(0,timeout_s*.5))
        terminated = self._process.is_alive()
        if terminated:
            self._process.terminate()
            self._process.join(timeout=max(0,deadline-time.perf_counter()))
        self._stop.set()
        # Never join queue feeder/reader threads inside a control-time budget.
        for transport in (self._incoming,self._outgoing):
            transport.cancel_join_thread()
            transport.close()
        return {'closed':True,'process_alive':self._process.is_alive(),'terminated':terminated,
                'close_elapsed_s':time.perf_counter()-start,'official_acceptance':False}
