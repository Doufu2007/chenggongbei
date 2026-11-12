"""Measure a real spawned SIFT worker while the owner loop schedules 20Hz."""
from __future__ import annotations
from copy import deepcopy
import csv
from datetime import datetime,timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time

from latest_frame_worker import LatestFrameWorker

ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parent

def percentile(values,p):
    values=sorted(values);at=(len(values)-1)*p
    return values[math.floor(at)]*(math.ceil(at)-at)+values[math.ceil(at)]*(at-math.floor(at)) if at%1 else values[int(at)]

def main():
    source=json.loads((ROOT/'runs/analysis/20261112_route_v20_matcher/report.json').read_text(encoding='utf-8'))
    frame=source['frames'][0];meta=frame['source_metadata'];jpeg=(ROOT/frame['source_path']).read_bytes()
    original={'session_id':'v20-worker-offline','first_ID':source['first_ID'],'stationary_epoch':1,
       'source_jpeg':jpeg,'source_sha256':hashlib.sha256(jpeg).hexdigest(),'image_stamp_s':meta['stamp_s'],
       'topic':meta['topic'],'frame_id':meta['frame_id'],'pose':meta['exposure']['odom'],
       'exposure_validation':meta['exposure_validation'],'camera_info':frame['actual_camera_info'],
       'K':[frame['actual_camera_info']['k'][i:i+3] for i in (0,3,6)]}
    worker=LatestFrameWorker(ROOT/'runs/analysis/20261112_ground_ipm_sift_exploratory/report.json')
    worker.set_context('v20-worker-offline',source['first_ID'],1)
    start=time.perf_counter();ticks=[];events=[];results=[];submitted={};next_tick=start
    schedule=[(0.,1,1),(.20,2,2),(.25,3,2),(.30,4,2)]
    while time.perf_counter()-start<45:
        now=time.perf_counter()
        if now<next_tick:
            time.sleep(min(next_tick-now,.010));continue
        tick_start=time.perf_counter();elapsed=tick_start-start;skipped=max(0,int((tick_start-next_tick)/.05))
        scheduled=next_tick
        next_tick += (skipped+1)*.05
        while schedule and elapsed>=schedule[0][0]:
            due,seq,epoch=schedule.pop(0)
            worker.set_context('v20-worker-offline',source['first_ID'],epoch)
            job=deepcopy(original);job['job_seq']=seq;job['stationary_epoch']=epoch
            worker.submit(job);submitted[seq]=time.perf_counter()
            events.append({'event':'submitted','job_seq':seq,'epoch':epoch,'elapsed_s':time.perf_counter()-start})
        new=worker.poll()
        for value in new:
            seq=value['job']['job_seq'];value['received_elapsed_s']=time.perf_counter()-start
            value['queue_to_result_wall_s']=time.perf_counter()-submitted[seq]
            value['same_original_frame_used_for_mechanism_test']=True
            results.append(value)
            events.append({'event':'result','job_seq':seq,'elapsed_s':value['received_elapsed_s'],
                           'matches_current_context':value['matches_current_context']})
        work_ms=(time.perf_counter()-tick_start)*1000
        ticks.append({'tick':len(ticks),'elapsed_s':elapsed,'lateness_ms':(tick_start-scheduled)*1000,
                      'owner_work_ms':work_ms,'skipped_scheduled_ticks':skipped,
                      'worker_in_flight':worker._future is not None,'pending':worker.mailbox.pending is not None})
        if not schedule and len(results)==2 and not worker.busy:break
    elapsed=time.perf_counter()-start
    stopped=time.perf_counter();worker.close();close_ms=(time.perf_counter()-stopped)*1000
    intervals=[(b['elapsed_s']-a['elapsed_s'])*1000 for a,b in zip(ticks,ticks[1:])]
    work=[x['owner_work_ms'] for x in ticks];lag=[x['lateness_ms'] for x in ticks]
    summary={'duration_s':elapsed,'ticks':len(ticks),'effective_tick_rate_hz':(len(ticks)-1)/(ticks[-1]['elapsed_s']-ticks[0]['elapsed_s']),
      'max_heartbeat_interval_ms':max(intervals),'p99_heartbeat_interval_ms':percentile(intervals,.99),
      'median_heartbeat_interval_ms':percentile(intervals,.5),'owner_work_max_ms':max(work),'owner_work_p99_ms':percentile(work,.99),
      'lateness_max_ms':max(lag),'skipped_scheduled_ticks':sum(x['skipped_scheduled_ticks'] for x in ticks),
      'submitted_jobs':4,'executed_jobs':worker.dispatched,'pending_replacements':worker.mailbox.replaced_pending,
      'context_discarded_pending':worker.mailbox.context_discarded_pending,'latest_result_replacements':worker.mailbox.replaced_latest_result,
      'result_job_sequences':[x['job']['job_seq'] for x in results],'max_in_flight':worker.max_in_flight,'max_pending':1,
      'main_pid':os.getpid(),'worker_pids':sorted({x.get('worker',{}).get('worker_pid') for x in results}),
      'close_after_idle_ms':close_ms}
    report={'kind':'latest_frame_spawn_worker_real_matcher_20hz_prototype','created_at':datetime.now(timezone.utc).isoformat(),
      'worker_source_sha256':hashlib.sha256((OUT/'latest_frame_worker.py').read_bytes()).hexdigest(),
      'benchmark_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
      'summary':summary,'events':events,'results':results,'timing_ticks_path':'heartbeat_ticks.csv',
      'official_acceptance':False,'prototype_only':True,'production_modified':False,
      'source_sha256':hashlib.sha256(jpeg).hexdigest(),'source_stamp_s':meta['stamp_s'],
      'original_binding':{k:v for k,v in original.items() if k!='source_jpeg'},
      'limitations':['Two actual matcher calls use one retained JPEG; this does not prove multiple fresh exposures or class stability.',
       'Epoch change is injected to test stale-result rejection; the source pose is preserved without invented motion.',
       'A current epoch only validates binding, never image freshness; no .20s freshness is relaxed or renewed.',
       'One local main loop and one spawned worker, without ROS, simulator locks, CPU load control or deployment.',
       'close() joins the worker and belongs to shutdown, not a control tick; parent crash supervision and hard-kill timeout remain out of scope.']}
    report['timing_ticks_path']='heartbeat_ticks_final.csv'
    (OUT/'benchmark_report_final.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with (OUT/'heartbeat_ticks_final.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(ticks[0]));writer.writeheader();writer.writerows(ticks)
    print(json.dumps(summary,indent=2),flush=True)
    for r in results:
        print('job',r['job']['job_seq'],'same_context',r['matches_current_context'],'timing',r.get('timing'),
              'cold_init',r.get('worker',{}).get('template_load_ms'),'error',r.get('error'),flush=True)
    assert [x['job']['job_seq'] for x in results]==[1,4]
    assert results[0]['matches_current_context'] is False and results[1]['matches_current_context'] is True
    assert all(not x['official_acceptance'] for x in results)

if __name__=='__main__':main()
