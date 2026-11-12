"""Independent, read-only reconstruction and bounded timing audit of v34."""
from __future__ import annotations
import base64
from collections import Counter
import csv
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import zipfile

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SOURCE = ROOT / 'runs/evidence/20261112/callback_height_timing_v34.jsonl'
PLATFORM_ID = '0d99c6fc-e6b7-4a69-aa35-abad82dbcbc0'
PART = 'bounded_height_hold_gzip_part'
CORE = 'bounded_height_hold_core'
INPUTS = ('/clock','/uav/odom','/odom','/scan','/uav/laser/scan','/tf','/tf_static')
META = ('session_id','encoding','uncompressed_bytes','compressed_bytes','uncompressed_sha256','compressed_sha256','part_count')

def dump(name, value):
    (OUT/name).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf8')

def summary(values):
    a = list(values)
    return {'count':len(a),'min':min(a),'median':statistics.median(a),'max':max(a)} if a else {'count':0,'min':None,'median':None,'max':None}

def csv_dump(name, rows):
    if rows:
        with (OUT/name).open('w', encoding='utf8', newline='') as f:
            w=csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
            w.writeheader(); w.writerows(rows)

def sha(data): return hashlib.sha256(data).hexdigest()

def stamp(message):
    s=message['header']['stamp']
    return s['sec']+s['nanosec']/1e9

def reconstruct():
    source=SOURCE.read_bytes()
    events=[json.loads(line) for line in source.decode('utf8').splitlines() if line.strip()]
    parts=[e['payload']['data'] for e in events if e['payload']['data'].get('section')==PART]
    parts.sort(key=lambda p:p['part_index'])
    assert parts, 'no gzip evidence'
    meta={k:parts[0][k] for k in META}
    assert meta['encoding']=='gzip+base64'
    assert [p['part_index'] for p in parts]==list(range(meta['part_count']))
    assert all({k:p[k] for k in META}==meta for p in parts)
    assert all(p['mode']=='bounded_height_hold' and p['read_only'] is False and p['official_scoring']=='unverified' for p in parts)
    packed=base64.b64decode(''.join(p['base64_part'] for p in parts), validate=True)
    assert len(packed)==meta['compressed_bytes'] and sha(packed)==meta['compressed_sha256']
    raw=gzip.decompress(packed)
    assert len(raw)==meta['uncompressed_bytes'] and sha(raw)==meta['uncompressed_sha256']
    b=json.loads(raw)
    assert b['schema']=='bounded_height_hold_v1'
    assert b['mode']=='bounded_height_hold' and b['session_id']==meta['session_id']
    assert b['read_only'] is False and b['full_mission_complete'] is False and b['official_score'] is None
    cores=[e['payload']['data'] for e in events if e['payload']['data'].get('section')==CORE]
    assert len(cores)==1
    core=cores[0]
    compared=[k for k in core if k in b]
    assert all(b[k]==core[k] for k in compared), 'core disagreement'
    assert b['runtime_timing']['revision']=='v34_timing'
    assert 'runtime_timing' not in core  # it is nested only in the full gzip evidence
    runtime=b['runtime_timing']
    assert runtime['wall_clock']=='time.monotonic' and runtime['cpu_clock']=='time.thread_time'
    assert all(e.get('source')=='unverified_callback' for e in events)
    assert len([e for e in events if e['payload']['event']=='launcher_exit'])==1
    assert [e['payload']['data']['exit_code'] for e in events if e['payload']['event']=='launcher_exit']==[0]
    (OUT/'evidence.json.gz').write_bytes(packed)
    (OUT/'evidence.json').write_bytes(raw)
    dump('events_without_parts.json',[e for e in events if e['payload']['data'].get('section')!=PART])
    validation={'source':str(SOURCE.relative_to(ROOT)),'source_bytes':len(source),'source_sha256':sha(source),
        'event_count':len(events),'parts_verified':len(parts),'metadata':meta,
        'lengths_and_both_sha_match':True,'core_matches':True,'core_fields_compared':compared,
        'mode':'bounded_height_hold','schema':'bounded_height_hold_v1','runtime_revision':'v34_timing',
        'runtime_nested_in_full_payload':True,'source_authenticated_independently':False,
        'callback_source':'unverified_callback','platform_id':PLATFORM_ID,
        'run_id':'run_20261112_085752_4f79c7da','evaluation_number':'#22362608',
        'sample_counts':{k:len(b[k]) for k in ('trajectory','commands','scans','downscans','tf_records')},
        'dropped':b['dropped'],'launcher_exit_codes':[0]}
    manifest=json.loads((ROOT/'runs/review/height_runtime_timing_20261112/manifest.json').read_text(encoding='utf8'))
    checked=[]
    for item in manifest['frozen_baseline']+manifest['artifacts']:
        actual=(ROOT/item['path']).read_bytes()
        matched=len(actual)==item['bytes'] and sha(actual)==item['sha256']
        checked.append({'path':item['path'],'bytes':len(actual),'sha256':sha(actual),'matches_review_manifest':matched})
        assert matched, item['path']+' differs from reviewed bytes'
    validation['local_review_manifest_checks']=checked
    archive=ROOT/'dist/inspection_height_timing_v34_src.zip'
    with zipfile.ZipFile(archive) as z:
        suffix='src/inspection_mission/inspection_mission/height_runtime_timing.py'
        names=[n for n in z.namelist() if n.replace('\\','/').endswith(suffix)]
        assert len(names)==1
        module=z.read(names[0]); expected=(ROOT/suffix).read_bytes()
        assert module==expected
        validation['local_archive_module']={'archive':str(archive.relative_to(ROOT)),'archive_sha256':sha(archive.read_bytes()),
            'entry':names[0],'sha256':sha(module),'matches_reviewed_worktree_module':True}
    dump('validation.json',validation)
    return b, validation

def span_key(s): return (s['label'],s['entry_wall_s'],s['exit_wall_s'])

def timing_audit(b):
    r=b['runtime_timing']; trace=r['trace']; slow=r['slow_spans']; labels=r['labels']
    assert set(labels)==set('receive:'+t for t in INPUTS)|{'spin_once','tick','control_tick','dds:control_tick','dds:publish_boundary'}
    assert len(trace)==r['trace_capacity']==2048 and len(slow)==r['slow_capacity']==32
    assert sum(x['count'] for x in labels.values())==len(trace)+r['trace_dropped']
    assert len({span_key(s) for s in trace})==len(trace)
    assert all(a['exit_wall_s']<=z['exit_wall_s'] for a,z in zip(trace,trace[1:]))
    assert all(a['wall_s']>=z['wall_s'] for a,z in zip(slow,slow[1:]))
    known={span_key(s):s for s in trace+slow}
    for s in known.values():
        assert s['wall_s']>=0 and s['cpu_s']>=0
        assert abs(s['exit_wall_s']-s['entry_wall_s']-s['wall_s'])<1e-12
        assert s['wall_s']<=labels[s['label']]['wall_max_s']+1e-12
        assert s['cpu_s']<=labels[s['label']]['cpu_max_s']+1e-12
        if s['label'].startswith('receive:'):
            assert s['topic']==s['label'][8:]
            assert all(k in s for k in ('source_stamp_s','state_at_entry','phase_at_entry'))
    receive_trace={t:sorted([s for s in trace if s['label']=='receive:'+t],key=lambda s:s['entry_wall_s']) for t in INPUTS}
    topic_metrics={}
    gap_rows=[]
    for topic,rows in receive_trace.items():
        pairs=[]
        for a,z in zip(rows,rows[1:]):
            pair={'topic':topic,'prior_entry_wall_s':a['entry_wall_s'],'next_entry_wall_s':z['entry_wall_s'],
                'entry_gap_s':z['entry_wall_s']-a['entry_wall_s'],'prior_exit_to_next_entry_s':z['entry_wall_s']-a['exit_wall_s'],
                'prior_source_stamp_s':a['source_stamp_s'],'next_source_stamp_s':z['source_stamp_s'],
                'source_gap_s':z['source_stamp_s']-a['source_stamp_s'] if a['source_stamp_s'] is not None and z['source_stamp_s'] is not None else None,
                'prior_callback_wall_s':a['wall_s'],'prior_callback_cpu_s':a['cpu_s'],'next_callback_wall_s':z['wall_s'],'next_callback_cpu_s':z['cpu_s']}
            pairs.append(pair); gap_rows.append(pair)
        topic_metrics[topic]={'full_count':labels['receive:'+topic]['count'],'retained_tail_count':len(rows),
            'retained_entry_start_s':rows[0]['entry_wall_s'] if rows else None,'retained_entry_end_s':rows[-1]['entry_wall_s'] if rows else None,
            'retained_missing_source_stamp_count':sum(s['source_stamp_s'] is None for s in rows),
            'retained_source_gap_s':summary(p['source_gap_s'] for p in pairs if p['source_gap_s'] is not None),
            'retained_entry_gap_s':summary(p['entry_gap_s'] for p in pairs),
            'retained_exit_to_next_entry_s':summary(p['prior_exit_to_next_entry_s'] for p in pairs),
            'largest_retained_entry_gaps':sorted(pairs,key=lambda p:p['entry_gap_s'],reverse=True)[:5]}
    # Guard and recorder clocks have distinct origins. Bound their offset using
    # exact source-stamp joins and the fact that original _receive_impl captures
    # wall between the measured receive entry and exit; do not silently equate them.
    tr_by_stamp={x['stamp']:x for x in b['trajectory']}
    joins=[(s,tr_by_stamp[s['source_stamp_s']]) for s in known.values() if s['label']=='receive:/uav/odom' and s['source_stamp_s'] in tr_by_stamp]
    offset_lower=max(x['wall_elapsed_s']-s['exit_wall_s'] for s,x in joins)
    offset_upper=min(x['wall_elapsed_s']-s['entry_wall_s'] for s,x in joins)
    assert offset_lower<=offset_upper
    rows_by_topic={
        '/uav/odom':[{'wall_elapsed_s':x['wall_elapsed_s'],'source_stamp_s':x['stamp']} for x in b['trajectory']],
        '/scan':[{'wall_elapsed_s':x['wall_elapsed_s'],'source_stamp_s':stamp(x['message'])} for x in b['scans']],
        '/uav/laser/scan':[{'wall_elapsed_s':x['wall_elapsed_s'],'source_stamp_s':stamp(x['message'])} for x in b['downscans']]}
    raw_gaps={}; raw_gap_rows=[]
    for topic,rows in rows_by_topic.items():
        pairs=[{'topic':topic,'prior_wall_elapsed_s':a['wall_elapsed_s'],'next_wall_elapsed_s':z['wall_elapsed_s'],
            'arrival_gap_s':z['wall_elapsed_s']-a['wall_elapsed_s'],
            'prior_source_stamp_s':a['source_stamp_s'],'next_source_stamp_s':z['source_stamp_s'],
            'source_gap_s':z['source_stamp_s']-a['source_stamp_s']} for a,z in zip(rows,rows[1:])]
        raw_gap_rows.extend(pairs)
        raw_gaps[topic]={'rows':len(rows),'arrival_gap_s':summary(p['arrival_gap_s'] for p in pairs),
            'source_gap_s':summary(p['source_gap_s'] for p in pairs),'largest_arrival_gaps':sorted(pairs,key=lambda p:p['arrival_gap_s'],reverse=True)[:5]}
    # Describe neighbors only where the detailed retained tail covers them.
    def neighborhood(span):
        start,end=span['entry_wall_s'],span['exit_wall_s']
        complete=start>=trace[0]['exit_wall_s']
        receives=[s for s in known.values() if s['label'].startswith('receive:')]
        prior=max((s for s in receives if s['exit_wall_s']<=start),key=lambda s:s['exit_wall_s'],default=None) if complete else None
        after=min((s for s in receives if s['entry_wall_s']>=end),key=lambda s:s['entry_wall_s'],default=None) if complete else None
        parent=[s for s in known.values() if s['label'] in ('spin_once','tick','control_tick') and s['entry_wall_s']<=start and end<=s['exit_wall_s']]
        return {'span':span,'within_complete_retained_tail':complete,
            'nearest_retained_receive_before':prior,'nearest_retained_receive_after':after,
            'gap_from_prior_receive_exit_s':start-prior['exit_wall_s'] if prior else None,
            'gap_to_next_receive_entry_s':after['entry_wall_s']-end if after else None,
            'containing_spans':parent}
    dds=sorted([s for s in trace if s['label']=='dds:publish_boundary'],key=lambda s:s['entry_wall_s'])
    peak_dds=max(dds,key=lambda s:s['wall_s'])
    global_peak_exact=abs(peak_dds['wall_s']-labels['dds:publish_boundary']['wall_max_s'])<1e-12
    max_receive=max((s for s in known.values() if s['label'].startswith('receive:')),key=lambda s:s['wall_s'])
    longest_receive=neighborhood(max_receive)
    source=max_receive['source_stamp_s']; idx=next(i for i,x in enumerate(b['trajectory']) if x['stamp']==source)
    longest_receive['nearby_full_trajectory']=[{'wall_elapsed_s':x['wall_elapsed_s'],'stamp':x['stamp'],'relative_height_m':x['xyz'][2]-b['origin'][2],'vz_mps':x['linear'][2]} for x in b['trajectory'][max(0,idx-2):idx+3]]
    # The original support contract uses _stable(wall): wall >= 1 second,
    # with speed/angular/radius/height gates. It has no source-span >= 1 test.
    # Exact anchor/step wall scalars are not logged; use enclosing tick bounds.
    zero_start=next(x['wall_elapsed_s'] for x in b['phase_transitions'] if x['to']=='ZERO_RETURN')
    finished=next(x for x in b['commands'] if x['state']=='FINISHED')
    def support_good(x):
        return abs(x['xyz'][2]-b['origin'][2])<=.005+1e-9 and x['speed']<=.005+1e-9 and x['angular_speed']<=.01+1e-9
    candidates=[c for c in b['commands'] if zero_start<=c['wall_elapsed_s']<=finished['wall_elapsed_s'] and support_good(tr_by_stamp[c['odom_stamp']])]
    first=candidates[0]
    ticks=[s for s in trace if s['label']=='control_tick']
    def tick_for(c):
        matches=[s for s in ticks if s['entry_wall_s']+offset_lower<=c['wall_elapsed_s']<=s['exit_wall_s']+offset_upper]
        assert len(matches)==1
        return matches[0]
    anchor_tick=tick_for(first); decision_tick=tick_for(finished)
    block=[x for x in b['trajectory'] if first['odom_stamp']<=x['stamp']<=finished['odom_stamp']]
    all_good=all(support_good(x) for x in block)
    radius=max(math.dist(x['xyz'],tr_by_stamp[first['odom_stamp']]['xyz']) for x in block)
    elapsed_bounds=[decision_tick['entry_wall_s']-anchor_tick['exit_wall_s'],decision_tick['exit_wall_s']-anchor_tick['entry_wall_s']]
    assert all_good and radius<=.002 and elapsed_bounds[0]>=1.
    cleanup_block=[x for x in b['trajectory'] if first['odom_stamp']<=x['stamp']]
    support_contract={
        'definition':'original _stable checks wall >= stable_s=1; no source-span >=1 requirement; speed<=.005, angular<=.01, radius<=.002, height offset<=.005',
        'first_eligible_command':first,'first_eligible_tick':anchor_tick,'finished_command':finished,'finished_tick':decision_tick,
        'exact_anchor_and_decision_wall_logged':False,'anchor_elapsed_guard_wall_bounds_s':[anchor_tick['entry_wall_s']+offset_lower,anchor_tick['exit_wall_s']+offset_upper],
        'decision_elapsed_guard_wall_bounds_s':[decision_tick['entry_wall_s']+offset_lower,decision_tick['exit_wall_s']+offset_upper],
        'anchor_to_decision_wall_span_bounds_s':elapsed_bounds,'all_source_frames_between_anchor_and_finish_good':all_good,
        'radius_from_anchor_sample_m':radius,'original_wall_1s_contract_supported':True,
        'source_span_at_first_finish_s':finished['odom_stamp']-first['odom_stamp'],
        'source_span_through_cleanup_s':cleanup_block[-1]['stamp']-cleanup_block[0]['stamp'],
        'arrival_span_through_cleanup_s':cleanup_block[-1]['wall_elapsed_s']-cleanup_block[0]['wall_elapsed_s'],
        'cleanup_all_frames_good':all(support_good(x) for x in cleanup_block),
        'extra_strict_source_and_arrival_1s_at_first_finish':False}
    label_rows=[]
    for label,s in labels.items():
        label_rows.append({'label':label,**s,'wall_mean_s':s['wall_total_s']/s['count'],
            'cpu_mean_s':s['cpu_total_s']/s['count'],'retained_trace_count':sum(x['label']==label for x in trace)})
    csv_dump('runtime_trace.csv',trace);csv_dump('slow_spans.csv',slow);csv_dump('label_summary.csv',label_rows)
    csv_dump('retained_callback_gaps.csv',gap_rows);csv_dump('full_sample_gaps.csv',raw_gap_rows)
    result={'kind':'independent_v34_timing_audit','labels':labels,'topic_retained_tail_metrics':topic_metrics,
        'complete_span_count':sum(s['count'] for s in labels.values()),'trace_count':len(trace),'trace_dropped':r['trace_dropped'],
        'retained_completion_range_s':[trace[0]['exit_wall_s'],trace[-1]['exit_wall_s']],
        'slow_count':len(slow),'slow_dropped':r['slow_dropped'],'instrumentation_errors':r['instrumentation_errors'],
        'guard_minus_recorder_origin_offset':{'joined_samples':len(joins),'lower_s':offset_lower,'upper_s':offset_upper,
            'meaning':'recorder.started_wall - guard.started_wall; derived bounds only'},
        'full_raw_sample_gaps':raw_gaps,'largest_receive_span':longest_receive,
        'largest_spin_span':max(known.values(),key=lambda s:s['wall_s']),
        'original_support_contract_audit':support_contract,
        'publish_boundary_maximum_detail':neighborhood(peak_dds),'maximum_detail_matches_full_label_max':global_peak_exact,
        'last_publish_boundary_detail':neighborhood(dds[-1]),
        'claims_not_supported':['DDS take/deserialize root cause','OS scheduling or quota root cause','separate guard-lock wait',
            'source-side wall publication time','all early-span adjacency','complete inspection mission or task score'],
        'nested_spans_overlap':True,'source_stamp_is_publication_wall_time':False}
    dump('timing_metrics.json',result)
    return result

def main():
    b,v=reconstruct()
    spec=importlib.util.spec_from_file_location('v31_raw_analysis',ROOT/'runs/analysis/20261112_height_v31/analyze.py')
    ref=importlib.util.module_from_spec(spec);spec.loader.exec_module(ref)
    protocol=ref.analyze(b,OUT,v)
    timing=timing_audit(b)
    protocol['legacy_v31_analysis_criterion_note']='independent_full_local_protocol_observed additionally requires pre-FINISHED support source and arrival spans >=1s; original guard requires wall dwell only'
    protocol['original_guard_wall_support_contract_supported']=timing['original_support_contract_audit']['original_wall_1s_contract_supported']
    protocol['original_guard_local_protocol_supported']=bool(protocol['independent_hold_3s_source_and_arrival'] and protocol['descent_transition_audit']['joint_descent_gate_measured'] and protocol['original_guard_wall_support_contract_supported'])
    dump('metrics.json',protocol)
    print(json.dumps({'event_count':v['event_count'],'parts':v['parts_verified'],'both_lengths_and_sha':v['lengths_and_both_sha_match'],
        'reported':protocol['reported'],'original_guard_local_protocol_supported':protocol['original_guard_local_protocol_supported'],
        'extra_strict_pre_finished_dual_clock_protocol':protocol['independent_full_local_protocol_observed'],
        'command_formula_errors':len(protocol['command_measurement_or_formula_errors']),
        'span_count':timing['complete_span_count'],'trace_dropped':timing['trace_dropped'],
        'largest_receive':timing['largest_receive_span']['span'],
        'max_retained_dds_query':timing['publish_boundary_maximum_detail']['span'],
        'full_label_max_dds_wall_s':timing['labels']['dds:publish_boundary']['wall_max_s'],
        'raw_gaps':{k:v['arrival_gap_s']['max'] for k,v in timing['full_raw_sample_gaps'].items()}},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
