"""Bounded asynchronous worker evidence, without changing original poll objects.

Serialization, compression, hashing and transport run in a separate daemon.
Diagnostic failures never authorize recognition, control or official acceptance.
"""
from __future__ import annotations

import base64
from copy import deepcopy
import gzip
import hashlib
import json
import math
import queue
import threading
import time
import uuid


class GroundMatchResultRecorder:
    MAX_RESULTS=18
    QUEUE_CAPACITY=2
    MAX_RAW_BYTES=4*1024*1024
    PART_CHARS=32000

    def __init__(self, worker, reporter):
        self.worker, self.reporter = worker, reporter
        self.recorded=self.completed=self.report_errors=self.dropped=self.oversize=0
        self.attempted=0;self._closed=False
        self.recording_id=uuid.uuid4().hex
        self._queue=queue.Queue(maxsize=self.QUEUE_CAPACITY)
        self._closing=threading.Event()
        self._thread=threading.Thread(target=self._run,name='ground-result-evidence',daemon=True)
        self._thread.start()

    def __getattr__(self, name):
        return getattr(self.worker, name)

    def poll(self):
        results = self.worker.poll()
        if self.reporter is not None and not self._closed:
            for result in results:
                if self.attempted>=self.MAX_RESULTS:break
                self.attempted+=1
                if self._queue.full():
                    self.dropped+=1;continue
                try:
                    # Keep evidence stable when later context refreshes mutate
                    # the original result. Large serialization is off-owner.
                    self._queue.put_nowait((self.attempted-1,deepcopy(result)))
                    self.recorded+=1
                except Exception:
                    self.report_errors+=1;self.dropped+=1
        return results

    def _send(self,payload):
        sender=getattr(self.reporter,'send_now',None)
        sent=(sender('probe_snapshot',payload,timeout_s=1.5) if callable(sender)
              else self.reporter.emit('probe_snapshot',payload))
        if sent is False:raise RuntimeError('diagnostic result part not accepted')

    def _record(self,index,result):
        raw=json.dumps(result,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode('utf-8')
        if len(raw)>self.MAX_RAW_BYTES:
            self.oversize+=1;raise ValueError('worker result exceeds bounded 4MiB evidence budget')
        packed=gzip.compress(raw,mtime=0);encoded=base64.b64encode(packed).decode('ascii')
        parts=[encoded[i:i+self.PART_CHARS] for i in range(0,len(encoded),self.PART_CHARS)]
        common={'mode':'full_inspection','section':'ground_worker_result_gzip_part',
            'official_acceptance':False,'recording_id':self.recording_id,'record_index':index,
            'encoding':'gzip+base64','uncompressed_bytes':len(raw),'compressed_bytes':len(packed),
            'uncompressed_sha256':hashlib.sha256(raw).hexdigest(),
            'compressed_sha256':hashlib.sha256(packed).hexdigest(),'part_count':len(parts)}
        for i,part in enumerate(parts):self._send({**common,'part_index':i,'base64_part':part})

    def _run(self):
        while True:
            try:index,result=self._queue.get(timeout=.05)
            except queue.Empty:
                if self._closing.is_set():break
                continue
            try:
                self._record(index,result);self.completed+=1
            except Exception:self.report_errors+=1
            finally:self._queue.task_done()
        if self.reporter is not None:
            try:self._send({'mode':'full_inspection','section':'ground_worker_recorder_summary',
                'official_acceptance':False,**self.snapshot()})
            except Exception:self.report_errors+=1

    def snapshot(self):
        return {'recording_id':self.recording_id,'attempted':self.attempted,'recorded':self.recorded,
            'completed':self.completed,'report_errors':self.report_errors,'dropped':self.dropped,
            'oversize':self.oversize,'pending':self._queue.unfinished_tasks,
            'max_results':self.MAX_RESULTS,'queue_capacity':self.QUEUE_CAPACITY,'max_raw_bytes':self.MAX_RAW_BYTES}

    def flush(self,*,timeout_s=.5):
        if not isinstance(timeout_s,(int,float)) or not math.isfinite(timeout_s) or not 0<=timeout_s<=30:
            raise ValueError('diagnostic flush timeout must be finite 0..30s')
        deadline=time.monotonic()+timeout_s
        with self._queue.all_tasks_done:
            while self._queue.unfinished_tasks:
                remaining=deadline-time.monotonic()
                if remaining<=0:return False
                self._queue.all_tasks_done.wait(remaining)
        return True

    def close(self,*,timeout_s=.5):
        if self._closed:return
        self._closed=True
        try:
            closer=getattr(self.worker,'close',None)
            if callable(closer):closer(timeout_s=timeout_s)
        finally:
            self._closing.set()
            self._thread.join(timeout=timeout_s)
