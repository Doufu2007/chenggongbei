"""Strict read-only v31 reconstruction with independent per-frame dwell audits."""
from __future__ import annotations
import argparse
import base64
import bisect
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics

PART='bounded_height_hold_gzip_part'
CORE='bounded_height_hold_core'
SCHEMA='bounded_height_hold_v1'
META=('session_id','encoding','uncompressed_bytes','compressed_bytes','uncompressed_sha256','compressed_sha256','part_count')


def dump(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def summary(values):
    a=list(values)
    return {'count':len(a),'min':min(a),'median':statistics.median(a),'max':max(a)} if a else {'count':0,'min':None,'median':None,'max':None}
def csv_dump(p,rows):
    if not rows:return
    with p.open('w',encoding='utf8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for row in rows for k in row)));w.writeheader();w.writerows(rows)


def reconstruct(source,out,platform_id=None):
    data=source.read_bytes();events=[json.loads(s) for s in data.decode('utf8').splitlines() if s.strip()]
    parts=[x['payload']['data'] for x in events if x['payload']['data'].get('section')==PART]
    assert parts,'no complete v31 gzip parts; this does not prove user code did not start'
    parts.sort(key=lambda x:x['part_index']);meta={k:parts[0][k] for k in META}
    assert meta['encoding']=='gzip+base64'
    assert all({k:p[k] for k in META}==meta for p in parts),'conflicting part metadata'
    assert [p['part_index'] for p in parts]==list(range(meta['part_count'])),'missing or duplicate part'
    packed=base64.b64decode(''.join(x['base64_part'] for x in parts),validate=True)
    assert len(packed)==meta['compressed_bytes'] and hashlib.sha256(packed).hexdigest()==meta['compressed_sha256']
    raw=gzip.decompress(packed)
    assert len(raw)==meta['uncompressed_bytes'] and hashlib.sha256(raw).hexdigest()==meta['uncompressed_sha256']
    b=json.loads(raw)
    assert b['schema']==SCHEMA and b['mode']=='bounded_height_hold' and b['session_id']==meta['session_id'],'wrong schema, mode or session'
    cores=[x['payload']['data'] for x in events if x['payload']['data'].get('section')==CORE]
    assert len(cores)==1,'must have exactly one core'
    assert all(b[k]==v for k,v in cores[0].items() if k in b),'core/reconstructed fields disagree'
    out.mkdir(parents=True,exist_ok=True)
    (out/'evidence.json.gz').write_bytes(packed);(out/'evidence.json').write_bytes(raw)
    dump(out/'events_without_parts.json',[x for x in events if x['payload']['data'].get('section')!=PART])
    v={'source':str(source),'source_bytes':len(data),'source_sha256':hashlib.sha256(data).hexdigest(),
        'event_count':len(events),'parts_verified':len(parts),'metadata':meta,'lengths_and_both_sha_match':True,
        'core_matches':True,'source_authenticated_independently':False,'callback_source':'unverified_callback',
        'platform_id':platform_id,'sample_counts':{k:len(b[k]) for k in ('trajectory','commands','scans','downscans','tf_records')},
        'dropped':b['dropped'],'launcher_exit_codes':[e['payload']['data'].get('exit_code') for e in events if e['payload'].get('event')=='launcher_exit']}
    dump(out/'validation.json',v)
    return b,v


def block_metrics(rows,target,kind):
    if not rows:return None
    anchor=rows[0]
    source_gaps=[b['stamp']-a['stamp'] for a,b in zip(rows,rows[1:])]
    arrival_gaps=[b['wall_elapsed_s']-a['wall_elapsed_s'] for a,b in zip(rows,rows[1:])]
    return {'samples':len(rows),'start_source_stamp':anchor['stamp'],'end_source_stamp':rows[-1]['stamp'],
        'start_arrival_wall_s':anchor['wall_elapsed_s'],'end_arrival_wall_s':rows[-1]['wall_elapsed_s'],
        'source_span_s':rows[-1]['stamp']-anchor['stamp'],'arrival_span_s':rows[-1]['wall_elapsed_s']-anchor['wall_elapsed_s'],
        'max_abs_height_error_m':max(abs(x['xyz'][2]-target) for x in rows),
        'max_abs_vz_mps':max(abs(x['linear'][2]) for x in rows),'max_speed_mps':max(x['speed'] for x in rows),
        'max_angular_speed_rps':max(x['angular_speed'] for x in rows),
        'position_radius_from_first_m':max(math.dist(x['xyz'],anchor['xyz']) for x in rows),
        'max_source_gap_s':max(source_gaps,default=None),'max_arrival_gap_s':max(arrival_gaps,default=None),
        'kind':kind}


def valid_blocks(rows,predicate,*,radius=None):
    blocks=[];current=[]
    for row in rows:
        if not predicate(row):
            if current:blocks.append(current);current=[]
            continue
        if current and ((row['stamp']-current[-1]['stamp']>.20+1e-9) or
                (row['wall_elapsed_s']-current[-1]['wall_elapsed_s']>.20+1e-9) or
                (radius is not None and math.dist(row['xyz'],current[0]['xyz'])>radius+1e-9)):
            blocks.append(current);current=[]
        current.append(row)
    if current:blocks.append(current)
    return blocks


def analyze(b,out,validation):
    tr,cmds=b['trajectory'],b['commands'];origin=b['origin'];trans=b['phase_transitions']
    assert all(x['stamp']<y['stamp'] for x,y in zip(tr,tr[1:])),'nonadvancing odom source'
    assert all(x['wall_elapsed_s']<=y['wall_elapsed_s'] for x,y in zip(tr,tr[1:])),'reversed odom arrival'
    assert all(x['wall_elapsed_s']<=y['wall_elapsed_s'] for x,y in zip(cmds,cmds[1:])),'reversed command arrival'
    startwall=tr[0]['wall']-tr[0]['wall_elapsed_s'] if tr else None
    motion=b['motion_started_wall']-startwall if startwall is not None and b['motion_started_wall'] is not None else None
    times=[r['wall_elapsed_s'] for r in tr]; stamps={r['stamp']:r for r in tr}
    entry={t['to']:t for t in trans}
    endtime=cmds[-1]['wall_elapsed_s'] if cmds else (times[-1] if times else 0.)
    intervals={t['to']:(t['wall_elapsed_s'],trans[i+1]['wall_elapsed_s'] if i+1<len(trans) else endtime) for i,t in enumerate(trans)}
    target=origin[2]+.10 if origin else None
    hold_interval=intervals.get('HOLD_010')
    hold_rows=[]
    if hold_interval:
        a,z=hold_interval
        prior=bisect.bisect_right(times,a)-1
        hold_rows=([tr[prior]] if prior>=0 else [])+[x for x in tr if a<x['wall_elapsed_s']<z]
    hold_pred=lambda x:abs(x['xyz'][2]-target)<=.005+1e-9 and abs(x['linear'][2])<=.01+1e-9
    blocks=valid_blocks(hold_rows,hold_pred) if target is not None else []
    hblocks=[block_metrics(x,target,'hold') for x in blocks]
    actual_hold=any(x['source_span_s']>=3.-1e-9 and x['arrival_span_s']>=3.-1e-9 for x in hblocks)
    dwell=b.get('hold_dwell_verified');dwell_audit={'reported':dwell,'reconstructed_window':None,'all_frames_inside_band':False,'source_span_ge_3s':False,'arrival_span_ge_3s':False}
    if dwell and target is not None:
        rows=[x for x in tr if dwell['started_odom_stamp']-1e-9<=x['stamp']<=dwell['ended_odom_stamp']+1e-9]
        dm=block_metrics(rows,target,'reported_hold_window')
        dwell_audit.update(reconstructed_window=dm,all_frames_inside_band=bool(rows) and all(hold_pred(x) for x in rows),
            exact_source_endpoints=bool(rows) and rows[0]['stamp']==dwell['started_odom_stamp'] and rows[-1]['stamp']==dwell['ended_odom_stamp'],
            source_span_ge_3s=bool(dm) and dm['source_span_s']>=3.-1e-9,
            arrival_span_ge_3s=bool(dm) and dm['arrival_span_s']>=3.-1e-9,
            reported_tick_wall_span_s=dwell['ended_wall_elapsed_s']-dwell['started_wall_elapsed_s'])
    zero=entry.get('ZERO_RETURN');qualifying=None
    if zero and origin is not None:
        measured=stamps.get(zero.get('odom_stamp'))
        if measured:
            rise=measured['xyz'][2]-origin[2];vz=measured['linear'][2]
            qualifying={'transition':zero,'odom':measured,'relative_height_m':rise,'vz_mps':vz,
                'height_le_025':rise<=.025+1e-9,'speed_le_020':abs(vz)<=.02+1e-9,
                'joint_descent_gate_measured':rise<=.025+1e-9 and abs(vz)<=.02+1e-9 and zero['from']=='DESCEND_002'}
    finished=next((x for x in cmds if x['state']=='FINISHED'),None)
    support_rows=[x for x in tr if zero and zero['wall_elapsed_s']<=x['wall_elapsed_s']<=(finished['wall_elapsed_s'] if finished else endtime)]
    support_pred=lambda x:abs(x['xyz'][2]-origin[2])<=.005+1e-9 and x['speed']<=.005+1e-9 and x['angular_speed']<=.01+1e-9
    sb=valid_blocks(support_rows,support_pred,radius=.002) if origin is not None else []
    support_metrics=[block_metrics(x,origin[2],'support') for x in sb]
    last_support=support_metrics[-1] if support_metrics else None
    support_measured=bool(last_support and support_rows and sb[-1][-1] is support_rows[-1]
        and last_support['arrival_span_s']>=1.-1e-9 and last_support['source_span_s']>=1.-1e-9)
    command_errors=[];low=[];deadline=[];budget=[]
    for c in cmds:
        value=c['linear_z'];phase=c.get('phase');sample=stamps.get(c.get('odom_stamp'))
        if not value:continue
        if sample is None:command_errors.append({'reason':'command_source_stamp_not_found','command':c});continue
        if c.get('measured_z')!=sample['xyz'][2] or c.get('measured_vz')!=sample['linear'][2]:command_errors.append({'reason':'command_measurement_disagrees_with_raw','command':c})
        want=.05 if phase=='ASCEND_TO_010' else max(.01,min(.05,.02695+.5*max(-.03,min(.03,.5*((origin[2]+(.02 if phase=='DESCEND_002' else .10))-sample['xyz'][2])))))
        if abs(value-want)>1e-12:command_errors.append({'reason':'fixed_formula_mismatch','expected':want,'command':c})
        if phase in ('HOLD_010','DESCEND_002') and sample['xyz'][2]-origin[2] <=(.025 if phase=='DESCEND_002' else .035)+1e-9:low.append(c)
        if phase=='HOLD_010' and c['wall_elapsed_s']>=entry[phase]['wall_elapsed_s']+8.-1e-9:deadline.append(c)
        if motion is not None and c['wall_elapsed_s']>=motion+b['limits']['motion_wall_s']-1e-9:budget.append(c)
    startidx=max((i for i,x in enumerate(tr) if motion is not None and x['wall_elapsed_s']<=motion),default=None)
    path=sum(math.dist(a['xyz'],z['xyz']) for a,z in zip(tr[startidx:],tr[startidx+1:])) if startidx is not None else 0.
    if startidx is not None:assert math.dist(tr[startidx]['xyz'],origin)<1e-9 and abs(path-b['total_path_m'])<1e-9
    rows=[{'wall_elapsed_s':x['wall_elapsed_s'],'source_stamp_s':x['stamp'],'z_m':x['xyz'][2],
        'relative_height_m':x['xyz'][2]-origin[2] if origin else None,'vz_mps':x['linear'][2],
        'speed_mps':x['speed'],'angular_speed_rps':x['angular_speed'],'tilt_rad':x['tilt']} for x in tr]
    csv_dump(out/'trajectory.csv',rows);csv_dump(out/'commands.csv',cmds)
    result={'kind':'independent_height_hold_raw_analysis','platform_id':validation['platform_id'],
        'reported':{k:b.get(k) for k in ('state','reason','phase','hold_verified','descent_verified','protocol_complete','protocol_abort_reason','support_return_verified','official_score','full_mission_complete')},
        'motion_start_elapsed_s':motion,'hold_interval':hold_interval,'hold_valid_blocks':hblocks,
        'independent_hold_3s_source_and_arrival':actual_hold,'reported_hold_dwell_audit':dwell_audit,
        'descent_transition_audit':qualifying,'support_valid_blocks':support_metrics,
        'support_last_block_source_and_arrival_ge_1s':support_measured,
        'independent_full_local_protocol_observed':bool(actual_hold and qualifying and qualifying['joint_descent_gate_measured'] and support_measured),
        'command_measurement_or_formula_errors':command_errors,'nonzero_at_or_below_phase_low_cutoff':low,
        'nonzero_after_hold_deadline':deadline,'nonzero_after_motion_budget':budget,
        'command_counts':{'positive':sum(c['linear_z']>0 for c in cmds),'negative':sum(c['linear_z']<0 for c in cmds),'zero':sum(c['linear_z']==0 for c in cmds)},
        'reconstructed_path_m':path,'reported_path_m':b['total_path_m'],
        'max_odom_source_gap_s':max((z['stamp']-a['stamp'] for a,z in zip(tr,tr[1:])),default=None),
        'max_odom_arrival_gap_s':max((z['wall_elapsed_s']-a['wall_elapsed_s'] for a,z in zip(tr,tr[1:])),default=None),
        'rise_m':summary(x['xyz'][2]-origin[2] for x in tr) if origin else None,
        'last_odom':tr[-1] if tr else None,'interpretation_limits':['sampled observations do not establish behavior between samples',
            'command time is process publication return time, not actuator application time','support proximity is not an independent touchdown sensor',
            'callback byte integrity is not independent source authentication','no task score or complete inspection mission is inferred']}
    dump(out/'metrics.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=Path('runs/evidence/20261112/callback_height_v31.jsonl'))
    p.add_argument('--out',type=Path,default=Path(__file__).resolve().parent)
    p.add_argument('--platform-id',default='f21b350d-dcba-4121-ae93-9781c554471c')
    a=p.parse_args();b,v=reconstruct(a.source,a.out,a.platform_id);r=analyze(b,a.out,v)
    print(json.dumps({'parts':v['parts_verified'],'counts':v['sample_counts'],'reported':r['reported'],
        'independent_hold':r['independent_hold_3s_source_and_arrival'],
        'independent_local_protocol':r['independent_full_local_protocol_observed'],
        'command_errors':len(r['command_measurement_or_formula_errors']),'low_height_nonzero':len(r['nonzero_at_or_below_phase_low_cutoff'])},ensure_ascii=False,indent=2))
