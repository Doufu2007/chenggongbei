"""Recompute retained offline heartbeat evidence without rerunning SIFT."""
from pathlib import Path
import csv
import hashlib
import json
import math
import statistics

ROOT=Path(__file__).resolve().parents[3]
DEV=ROOT/'runs/analysis/20261112_ground_matcher_worker_dev'


def identity(path):
    data=path.read_bytes()
    return {'path':path.relative_to(ROOT).as_posix(),'bytes':len(data),
            'sha256':hashlib.sha256(data).hexdigest()}


def percentile(values,p):
    values=sorted(values);at=(len(values)-1)*p;lo,hi=math.floor(at),math.ceil(at)
    return values[lo]+(values[hi]-values[lo])*(at-lo)


def main():
    report=DEV/'benchmark_report_final.json';ticks=DEV/'heartbeat_ticks_final.csv'
    before={'report':identity(report),'ticks':identity(ticks)}
    data=json.loads(report.read_text('utf-8'))
    rows=list(csv.DictReader(ticks.open(encoding='utf-8-sig')))
    elapsed=[float(r['elapsed_s'])for r in rows]
    intervals=[(b-a)*1000 for a,b in zip(elapsed,elapsed[1:])]
    work=[float(r['owner_work_ms'])for r in rows]
    lag=[float(r['lateness_ms'])for r in rows]
    recomputed={'ticks':len(rows),'effective_tick_rate_hz':(len(rows)-1)/(elapsed[-1]-elapsed[0]),
        'max_heartbeat_interval_ms':max(intervals),'p99_heartbeat_interval_ms':percentile(intervals,.99),
        'median_heartbeat_interval_ms':statistics.median(intervals),
        'owner_work_max_ms':max(work),'owner_work_p99_ms':percentile(work,.99),
        'lateness_max_ms':max(lag),'skipped_scheduled_ticks':sum(int(r['skipped_scheduled_ticks'])for r in rows)}
    for key,value in recomputed.items():assert abs(value-data['summary'][key])<1e-8,(key,value,data['summary'][key])
    assert data['worker_source_sha256']==identity(DEV/'latest_frame_worker.py')['sha256']
    assert data['benchmark_source_sha256']==identity(DEV/'benchmark_20hz.py')['sha256']
    assert all(r['eligible_for_recognition'] is False and r['fresh_exposure_validated'] is False for r in data['results'])
    assert data['summary']['main_pid'] not in data['summary']['worker_pids']
    assert [r['job']['job_seq']for r in data['results']]==[1,4]
    assert [r['matches_current_context']for r in data['results']]==[False,True]
    assert all(not r['official_acceptance']for r in data['results'])
    result={'kind':'independent_worker_benchmark_arithmetic','source_identity':before,
        'recomputed_summary':recomputed,'result_identity':[
            {'job_seq':r['job']['job_seq'],'worker_pid':r['worker']['worker_pid'],
             'source_sha256':r['job']['source_sha256'],'image_stamp_s':r['job']['image_stamp_s'],
             'prior_pose_stamp_s':r['job']['pose']['stamp_s'],'context':r['job']['stationary_epoch'],
             'matches_current_context':r['matches_current_context'],'diagnostic_count':len(r['matcher']['diagnostics']),
             'candidate_count':len(r['matcher']['candidates']),'job_ms':r['timing']['total_worker_job_ms'],
             'queue_to_result_wall_s':r['queue_to_result_wall_s']}for r in data['results']],
        'scope':'Final frozen local offline prototype only; original 800x800 JPEG repeated for a worker mechanism experiment, not multiple exposures or ROS/control validation.',
        'hard_realtime_or_platform_control_guarantee':False,
        'verified_current_src_sha256':data['worker_source_sha256'],'official_acceptance':False}
    assert before=={'report':identity(report),'ticks':identity(ticks)}
    out=Path(__file__).with_name('benchmark_final_independent_arithmetic.json')
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf-8')
    print(json.dumps(result,ensure_ascii=True,indent=2))


if __name__=='__main__':main()
