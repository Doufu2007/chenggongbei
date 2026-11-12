"""Recompute production-worker evidence without launching another SIFT job."""
from pathlib import Path
import csv
import hashlib
import json
import math
import statistics

ROOT=Path(__file__).resolve().parents[3]
DEV=ROOT/'runs/analysis/20261112_ground_matcher_worker_dev'


def identity(path):
    raw=path.read_bytes()
    return {'path':path.relative_to(ROOT).as_posix(),'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()}


def quantile(values,p):
    values=sorted(values);where=(len(values)-1)*p;lo,hi=math.floor(where),math.ceil(where)
    return values[lo]+(values[hi]-values[lo])*(where-lo)


def main():
    report=DEV/'production_benchmark_report.json';csvfile=DEV/'production_heartbeat_ticks.csv'
    previous=ROOT/'runs/analysis/20261112_route_v22_matcher/report.json'
    before={name:identity(path)for name,path in [('report',report),('csv',csvfile),('previous',previous)]}
    data=json.loads(report.read_text('utf-8'));old=json.loads(previous.read_text('utf-8'))
    with csvfile.open(encoding='utf-8-sig',newline='')as f: rows=list(csv.DictReader(f))
    elapsed=[float(x['elapsed_s'])for x in rows]
    gaps=[(b-a)*1000 for a,b in zip(elapsed,elapsed[1:])]
    work=[float(x['owner_work_ms'])for x in rows]
    assert [int(x['tick'])for x in rows]==list(range(len(rows)))
    recomputed={'ticks':len(rows),'duration_s':elapsed[-1],
        'effective_tick_rate_hz':(len(rows)-1)/elapsed[-1],
        'max_heartbeat_interval_ms':max(gaps),'p99_heartbeat_interval_ms':quantile(gaps,.99),
        'skipped_scheduled_ticks':sum(int(x['skipped_scheduled_ticks'])for x in rows),
        'owner_work_max_ms':max(work)}
    for key,value in recomputed.items(): assert abs(value-data['summary'][key])<1e-8,(key,value,data['summary'][key])
    extra={'first_actual_tick_elapsed_s':elapsed[0],
        'first_to_last_actual_tick_rate_hz':(len(rows)-1)/(elapsed[-1]-elapsed[0]),
        'median_heartbeat_interval_ms':statistics.median(gaps),'owner_work_p99_ms':quantile(work,.99)}
    result=data['result'];job=result['job'];frame=old['frames'][0]
    counts=lambda values:[{k:x.get(k)for k in ('template_sha256','good_matches','inliers','accepted_candidate')}for x in values]
    actual=counts(result['matcher']['diagnostics']);expected=counts(frame['result']['diagnostics'])
    assert len(actual)==len(expected)==12 and actual==expected
    assert job['source_sha256']==data['source_sha256']==frame['source_sha256']
    assert job['pose']==frame['source_metadata']['exposure']['odom']
    assert job['image_stamp_s']==frame['source_metadata']['stamp_s']
    assert job['pose']['stamp_s']<=job['image_stamp_s']
    assert not any(result[x]for x in ('official_acceptance','fresh_exposure_validated','eligible_for_recognition'))
    assert result['matches_current_context'] and not data['ROS_integrated']
    assert data['benchmark_source_sha256']==identity(DEV/'benchmark_production.py')['sha256']
    assert data['matcher_source_sha256']==identity(ROOT/'src/inspection_mission/inspection_mission/ground_plane_matcher.py')['sha256']
    output={'kind':'production_worker_benchmark_independent_arithmetic','source_identity':before,
        'reported_worker_source_sha256':data['worker_source_sha256'],'reported_matcher_source_sha256':data['matcher_source_sha256'],
        'recomputed_summary':recomputed,'additional_arithmetic':extra,
        'result_binding':{'source_sha256':job['source_sha256'],'image_stamp_s':job['image_stamp_s'],
            'prior_pose_stamp_s':job['pose']['stamp_s'],'frozen_first_ID':job['frozen_first_ID'],
            'job_seq':job['job_seq'],'worker_pid':result['worker_pid'],'main_pid_retained':False,
            'matches_current_context':result['matches_current_context'],'fresh_exposure_validated':False,
            'eligible_for_recognition':False,'official_acceptance':False},
        'template_count_pairs':actual,'close_original':data['summary']['close'],
        'scope':'Single retained local production worker benchmark for reported exact worker SHA; new failure-path changes must not be relabelled as the timed source. No new SIFT call or ROS/control run.',
        'official_acceptance':False,'hard_realtime_or_platform_control_guarantee':False}
    assert before=={name:identity(path)for name,path in [('report',report),('csv',csvfile),('previous',previous)]}
    target=Path(__file__).with_name('benchmark_independent_arithmetic.json')
    target.write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf-8')
    print(json.dumps({'output':target.relative_to(ROOT).as_posix(),'source_identity':before,
        'recomputed':recomputed,'extra':extra,'result_binding':output['result_binding']},ensure_ascii=True,indent=2))


if __name__=='__main__':main()
