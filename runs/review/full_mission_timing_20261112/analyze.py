"""Read-only saved-public-contract and submitted-v41 timing arithmetic.

No ROS, platform API, browser, private paths, new runs or production writes.
"""
from pathlib import Path
from dataclasses import asdict
import csv
import hashlib
import itertools
import json
import math
import re
import sys
import zipfile

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).parent
sys.path.insert(0,str(ROOT/'src/inspection_mission'))

def identity(path):
    p=ROOT/path;raw=p.read_bytes()
    return {'path':path,'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()}

def saved_contract():
    documents=[
        'docs/referee_rules_v1.txt','runs/evidence/20261111/evaluation_guide.json',
        '.firecrawl/ros-evaluation-notes.txt','docs/terminal_marker_fix_20261106.md',
        'docs/runner_contract_current.md','docs/full_mission_bridge_20261112.md',
    ]
    patterns=r'超时|时限|限时|分钟|时长|时间|timeout|deadline|time.limit|mission_completed|landing_marker|0\.5m/s'
    snippets=[]
    for name in documents:
        p=ROOT/name
        for line,text in enumerate(p.read_text(encoding='utf-8-sig').splitlines(),1):
            if re.search(patterns,text,re.I):snippets.append({'source':name,'line':line,'text':text[:1300]})
    frontend=[
        '.baf13c38c0939482.js','tmp_logchunks/239582d5d4bc9916.js',
        'runs/analysis/20261112_v41_log_contract/758ecc6acdc6d49b.js',
    ]
    front=[];abnormal_guards=[]
    for name in frontend:
        text=(ROOT/name).read_text(encoding='utf-8')
        words=r'timeout|deadline|timeLimit|time_limit|durationMs|abnormalEnd|超时|时限|分钟|倒计时|AbortSignal|300000|600000|1800000'
        matches=[]
        for m in re.finditer(words,text,re.I):
            # Static public JS only; do not copy payload state or signed URLs.
            excerpt=text[max(0,m.start()-100):min(len(text),m.end()+180)]
            matches.append({'offset':m.start(),'keyword':m.group(),'context':excerpt})
        front.append({'source':name,'matches':matches})
        for m in re.finditer(r'\w+&&"success"===\w+\.status&&\w+\.abnormalEnd\?',text):
            abnormal_guards.append({'source':name,'offset':m.start(),
                'condition':m.group()[:-1],
                'context':text[m.start():m.start()+1150],
                'meaning':'Public UI shows the suspected-timeout warning only for success plus abnormalEnd; this does not expose a numeric backend deadline.'})
    return {'sources':[identity(n) for n in documents+frontend],
        'document_snippets':snippets,'frontend_keyword_matches':front,
        'frontend_abnormal_end_guards':abnormal_guards,
        'official_overall_deadline_found':False,'official_user_startup_timeout_found':False,
        'scope':'Only saved public documents and downloaded frontend chunks listed above; absence here does not prove no server-side deadline.'}

def history():
    rows=[]
    # Whitelist current team's saved official-page snapshots and own records;
    # deliberately excludes access/session files and signed resource URLs.
    for p in sorted((ROOT/'runs/records').glob('*.json')):
        try:d=json.loads(p.read_text(encoding='utf-8-sig'))
        except (ValueError,UnicodeError):continue
        duration=d.get('durationMs')
        if isinstance(duration,str) and duration.replace('.','',1).isdigit():duration=float(duration)
        if duration is None and isinstance(d.get('duration_s'),(int,float)):duration=d['duration_s']*1000
        if isinstance(duration,(int,float)):
            rows.append({'source':p.relative_to(ROOT).as_posix(),'id':d.get('record_id'),
                'variant':d.get('variant'),'duration_s':duration/1000,'status':d.get('status'),
                'runtime_started':d.get('runtime_started'),'failure_reason':d.get('failure_reason')})
    evidence=ROOT/'runs/evidence/20261112'
    for p in sorted(evidence.glob('*official_page_final.json')):
        d=json.loads(p.read_text(encoding='utf-8-sig'));text=d.get('state',d.get('snapshot',''))
        match=re.search(r'已用\s*([\d.]+)\s*s',text)
        duration=d.get('duration_display_s')
        if match or isinstance(duration,(int,float)):
            rows.append({'source':p.relative_to(ROOT).as_posix(),
                'duration_s':float(match.group(1)) if match else float(duration),
                'source_kind':'saved_official_page_rounded_display',
                'status_display':d.get('status_display'),'failure':d.get('failure')})
    p=evidence/'full_v41_detail_final.json'
    d=json.loads(p.read_text(encoding='utf-8-sig'))['data']
    v41={k:d.get(k) for k in ('id','status','durationMs','result','failedReason','score','runId',
        'startedAt','finishedAt','serverDown','serverStatus','abnormalEnd','hasLogs','tabLogs','terminalLogs')}
    return {'own_saved_durations':rows,'v41_official_fields':v41,
            'interpretation':'durationMs is observed elapsed evaluation duration, not a published limit or measured user-control interval.'}

def compare_zip():
    zpath=ROOT/'dist/inspection_full_mission_v41_src.zip'
    selected=['ground_three_point_axis_plan','ground_return_plan','ground_three_point_plan',
        'ground_three_point','ground_three_point_axis','ground_observation_route',
        'ground_three_point_return','ground_three_point_recovery_budget','full_flight_inspection',
        'mission','bounded_height_hold','full_inspection_runtime']
    rows=[]
    with zipfile.ZipFile(zpath) as z:
        for module in selected:
            name='src/inspection_mission/inspection_mission/'+module+'.py'
            blob=z.read(name);current=(ROOT/name).read_bytes()
            rows.append({'path':name,'zip_sha256':hashlib.sha256(blob).hexdigest(),
                'current_sha256':hashlib.sha256(current).hexdigest(),'same':blob==current})
    return {'source_zip':identity(zpath.relative_to(ROOT).as_posix()),'comparisons':rows}

def ground_times():
    from inspection_mission.ground_three_point_axis_plan import plan_ground_leg
    from inspection_mission.ground_return_plan import plan_ground_return
    from inspection_mission.ground_three_point_return import ReturnThreePointLimits
    prior=json.loads((ROOT/'runs/analysis/20261112_ground_three_point_time/time_budget.json').read_text(encoding='utf-8'))
    drive_rate=prior['v25_accounting']['drive_wall_s_per_m']
    turn_rate=prior['v25_accounting']['turn_wall_s_per_rad']
    rows=[]
    for order in itertools.permutations(['ground_'+str(i) for i in range(5)],3):
        pose=(0.,0.,0.);previous=None;groups=[]
        for target in order:
            leg=plan_ground_leg(previous,target,pose)
            groups += [('observation',leg.to_observation_operations),('entry',leg.entry_operations)]
            pose=(*leg.endpoint_xy_m,leg.endpoint_yaw_rad);previous=target
        ret=plan_ground_return(pose);groups.append(('return',ret.operations))
        ops=[op for _,group in groups for op in group]
        path=sum(o.length_m for o in ops);angle=sum(o.rotation_rad for o in ops)
        nturn=sum(o.kind=='TURN_ONLY' for o in ops);ndrive=len(ops)-nturn
        # Not a hard physics lower bound: ideal command-following proxy only.
        simple=path/.1+angle/.5
        slow=sum(min(.1,group[-1].length_m if group[-1].kind!='TURN_ONLY' else
                next((o.length_m for o in reversed(group) if o.kind!='TURN_ONLY'),0.))
                for _,group in groups)
        # Every loaded group's final drive uses .03 in its last .10m.
        proxy=simple+slow*(1/.03-1/.1)+.7+len(ops)+1+1+3*(1+2)+1
        empirical=path*drive_rate+angle*turn_rate
        rows.append({'order':list(order),'path_m':path,'rotation_rad':angle,
            'turn_operations':nturn,'drive_operations':ndrive,'operation_count':len(ops),
            'ideal_max_command_motion_only_s':simple,
            'ideal_proxy_with_slow_tails_and_minimum_dwells_s':proxy,
            'slow_tail_length_m':slow,'v25_rate_extrapolated_motion_wall_s':empirical,
            'v25_rate_plus_45s_three_visual_and_30s_other_reserve_s':empirical+75})
    def span(key):
        return {'min':min(rows,key=lambda r:r[key]),'max':max(rows,key=lambda r:r[key])}
    with (OUT/'ground_sixty_orders.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    return {'limits':asdict(ReturnThreePointLimits()),'order_count':60,
        'path_span':span('path_m'),'angle_span':span('rotation_rad'),
        'ideal_command_motion_span':span('ideal_max_command_motion_only_s'),
        'ideal_proxy_span':span('ideal_proxy_with_slow_tails_and_minimum_dwells_s'),
        'v25_extrapolated_span':span('v25_rate_extrapolated_motion_wall_s'),
        'empirical_rate_source':identity('runs/analysis/20261112_ground_three_point_time/time_budget.json'),
        'v25_drive_wall_s_per_m':drive_rate,'v25_turn_wall_s_per_rad':turn_rate,
        'v25_saved_clock_accounting':prior['clock'],
        'v25_saved_command_aligned_response':prior['midpoint_wall_aligned_response'],
        'limitations':['No v41 draw or actual path was received; all 60 public orders shown.',
            'Ideal proxies assume exact nominal endpoints, velocities and clock=wall, not platform physics bounds.',
            'Dwells formula counts one second per operation, initial/calibration/home one each, report1+declare2 per point; no vision/network time guarantee.',
            'v25 rates describe one real earlier single-point run and include some settle overhead; extrapolation is a scenario, not current full-task measurement.']}

def flight_times():
    from inspection_mission.full_flight_inspection import FlightMissionController,AIR_CENTERS
    c=FlightMissionController('timing_readonly',.055);c.start_flight(30.)
    prev=(0.,0.,.055,0.);rows=[]
    for stage in c._stages:
        target=stage.target
        if stage.name.startswith('AIR_MOVE_TO_LIGHT_') or stage.name.startswith('AIR_INSPECT_'):
            # Ideal zero-XY-error visual fix only for route arithmetic.
            target=(*AIR_CENTERS[stage.point_id][:2],.912,stage.target[3])
        dx,dy,dz=[target[i]-prev[i] for i in range(3)]
        yaw=abs(math.atan2(math.sin(target[3]-prev[3]),math.cos(target[3]-prev[3])))
        xy=math.hypot(dx,dy);linear=math.sqrt(xy*xy+dz*dz)
        motion=max(linear/.12,yaw/.20)
        # Runtime adapter ignores controller z velocity: wanted physical-vz
        # candidate is max(-.03,min(.03,.5*(target_z-z))). Not measured cap.
        proxy=max(xy/.12,abs(dz)/.03,yaw/.20)
        rows.append({'stage':stage.name,'kind':stage.kind,'start':prev,'target':target,
            'nominal_xyz_distance_m':linear,'horizontal_m':xy,'vertical_abs_m':abs(dz),
            'yaw_abs_rad':yaw,'ideal_controller_max_speed_motion_s':motion,
            'ideal_trim_model_max_speed_motion_s':proxy})
        prev=target
    saved=json.loads((ROOT/'runs/review/full_flight_closed_loop_synthetic_20261112.json').read_text(encoding='utf-8'))
    saved_stages=[]
    for a,b in zip(saved['stages'],saved['stages'][1:]):
        saved_stages.append({'state':a['state'],'start_s':a['wall_s'],'end_s':b['wall_s'],
            'duration_s':b['wall_s']-a['wall_s']})
    return {'stage_count':len(rows),'settings':c.settings,'stages':rows,
        'nominal_distance_m':sum(r['nominal_xyz_distance_m'] for r in rows),
        'vertical_travel_m':sum(r['vertical_abs_m'] for r in rows),
        'ideal_controller_motion_only_s':sum(r['ideal_controller_max_speed_motion_s'] for r in rows),
        'ideal_trim_model_motion_only_s':sum(r['ideal_trim_model_max_speed_motion_s'] for r in rows),
        'minimum_explicit_visual_light_and_final_dwell_s':4*(.5+.2)+2*2+1,
        'existing_synthetic_source':identity('runs/review/full_flight_closed_loop_synthetic_20261112.json'),
        'existing_synthetic_closed_loop':{k:saved[k] for k in ('kind','synthetic_source_and_wall_elapsed_s',
            'integration_dt_s','z_model','visual_input','actual_python_wall_s','path_m','mission_completed')},
        'existing_synthetic_stage_times':saved_stages,
        'limitations':['No new simulation or timing test executed; saved synthetic 317s reused.',
            'Nominal light poses use public geometry as ideal route endpoints only, never assign class or claim actual visual fix.',
            'Nominal .03m/s vertical response is the code/synthetic trim model; platform speed/settling above .85m remains unmeasured.',
            'Recognition/detection cold-load delays, recovery, scheduler, transport, official scoring and runner overhead not guaranteed by this arithmetic.']}

def main():
    z=compare_zip()
    if not all(r['same'] for r in z['comparisons']):
        raise RuntimeError('Current timing dependencies differ from uploaded v41; inspect zip instead')
    result={'kind':'saved_public_contract_and_v41_readonly_timing_audit','zip_binding':z,
        'saved_contract':saved_contract(),'history':history(),'ground':ground_times(),'flight':flight_times()}
    g=result['ground'];f=result['flight']
    result['combined_scenarios']={
        'ideal_nominal_ground_plus_controller_motion_and_explicit_flight_dwells_s':[
            g['ideal_proxy_span'][k]['ideal_proxy_with_slow_tails_and_minimum_dwells_s']+
            f['ideal_controller_motion_only_s']+f['minimum_explicit_visual_light_and_final_dwell_s'] for k in ('min','max')],
        'ideal_nominal_command_proxy_ground_plus_trim_motion_and_explicit_flight_dwells_s':[
            g['ideal_proxy_span'][k]['ideal_proxy_with_slow_tails_and_minimum_dwells_s']+
            f['ideal_trim_model_motion_only_s']+f['minimum_explicit_visual_light_and_final_dwell_s'] for k in ('min','max')],
        'v25_ground_extrapolated_motion_plus_saved_317s_flight_s':[
            g['v25_extrapolated_span'][k]['v25_rate_extrapolated_motion_wall_s']+317 for k in ('min','max')],
        'team_stage_stop_budgets_s':1800+900,
        'not_official_deadlines_or_completion_guarantees':True}
    (OUT/'timing_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'zip_same':True,'ground_ideal_proxy_s':[
        g['ideal_proxy_span'][k]['ideal_proxy_with_slow_tails_and_minimum_dwells_s'] for k in ('min','max')],
        'flight_motion_s':f['ideal_trim_model_motion_only_s'],'flight_saved_s':317,
        'combined':result['combined_scenarios'],
        'saved_history_rows':len(result['history']['own_saved_durations'])},ensure_ascii=True))

if __name__=='__main__':main()
