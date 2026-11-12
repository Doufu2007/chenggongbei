"""One actual v22 matching call through the independent production worker."""
from datetime import datetime,timezone
from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/inspection_mission'))
from inspection_mission.ground_matcher_worker import GroundMatcherWorker
from inspection_mission.ground_plane_matcher import TemplateSpec
from benchmark_20hz import percentile

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    previous=json.loads((ROOT/'runs/analysis/20261112_route_v22_matcher/report.json').read_text(encoding='utf-8'))
    frame=previous['frames'][0];source=ROOT/frame['source_path'];meta=frame['source_metadata'];info=frame['actual_camera_info']
    library_path=ROOT/previous['template_manifest_path'];library=json.loads(library_path.read_text(encoding='utf-8'))
    specs=[TemplateSpec(x['absolute_path'],x['class_id'],x['label'],{'source_url':x['source_url'],'git_blob_sha':x['git_blob_sha'],'official_current_library_verified':False},x['sha256']) for x in library['references']]
    job={'job_seq':1,'session_id':'v22-offline-production-worker','frozen_first_ID':previous['first_ID'],'stationary_epoch':1,
      'source_jpeg':source.read_bytes(),'source_sha256':frame['source_sha256'],'image_stamp_s':meta['stamp_s'],
      'topic':meta['topic'],'frame_id':meta['frame_id'],'pose':meta['exposure']['odom'],
      'camera_info':info,'K':[info['k'][i:i+3] for i in (0,3,6)]}
    begin=time.perf_counter();worker=GroundMatcherWorker(specs);construct_ms=(time.perf_counter()-begin)*1000
    worker.set_context(job['session_id'],job['frozen_first_ID'],1);worker.submit(job)
    start=time.perf_counter();next_tick=start;ticks=[];result=None
    while time.perf_counter()-start<30:
        now=time.perf_counter()
        if now<next_tick:time.sleep(min(next_tick-now,.01));continue
        actual=time.perf_counter();skip=max(0,int((actual-next_tick)/.05));scheduled=next_tick;next_tick+=(skip+1)*.05
        found=worker.poll()
        if found:result=found[0]
        ticks.append({'tick':len(ticks),'elapsed_s':actual-start,'lateness_ms':(actual-scheduled)*1000,
                      'owner_work_ms':(time.perf_counter()-actual)*1000,'skipped_scheduled_ticks':skip})
        if result is not None:break
    assert result is not None and 'error' not in result,result
    assert not result['official_acceptance'] and not result['fresh_exposure_validated'] and not result['eligible_for_recognition']
    assert result['matches_current_context']
    observed=deepcopy(result)
    close=worker.close(timeout_s=.5)
    assert not result['matches_current_context'], 'close must invalidate even the cached delivered result'
    result=observed
    expected=previous['frames'][0]['result']['diagnostics'];actual=result['matcher']['diagnostics']
    assert [(x['template_sha256'],x['good_matches'],x['inliers'],x['accepted_candidate']) for x in expected]==[(x['template_sha256'],x['good_matches'],x['inliers'],x['accepted_candidate']) for x in actual]
    gaps=[(b['elapsed_s']-a['elapsed_s'])*1000 for a,b in zip(ticks,ticks[1:])]
    summary={'ticks':len(ticks),'duration_s':ticks[-1]['elapsed_s'],'effective_tick_rate_hz':(len(ticks)-1)/ticks[-1]['elapsed_s'],
      'max_heartbeat_interval_ms':max(gaps),'p99_heartbeat_interval_ms':percentile(gaps,.99),'skipped_scheduled_ticks':sum(x['skipped_scheduled_ticks'] for x in ticks),
      'owner_work_max_ms':max(x['owner_work_ms'] for x in ticks),'worker_construct_ms':construct_ms,'close':close,
      'template_load_ms':result['template_load_ms'],'match_ms':result['match_ms'],'calls':1,
      'v22_all_12_template_counts_and_acceptance_unchanged':True}
    report={'kind':'independent_production_worker_single_real_call_20hz','created_at':datetime.now(timezone.utc).isoformat(),
      'worker_source_sha256':sha(ROOT/'src/inspection_mission/inspection_mission/ground_matcher_worker.py'),
      'matcher_source_sha256':sha(ROOT/'src/inspection_mission/inspection_mission/ground_plane_matcher.py'),
      'benchmark_source_sha256':sha(Path(__file__)),'source_sha256':sha(source),'summary':summary,'result':result,
      'official_acceptance':False,'ROS_integrated':False,'limitations':['One actual v22 call and one process run; not a hard realtime bound or concurrency stress test.',
       'Candidate result is retained original v22 evidence, not a new exposure, live recognition, stable observation or official acceptance.']}
    (OUT/'production_benchmark_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with (OUT/'production_heartbeat_ticks.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(ticks[0]));writer.writeheader();writer.writerows(ticks)
    print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':main()
