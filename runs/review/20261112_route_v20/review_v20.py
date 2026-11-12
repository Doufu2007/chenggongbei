"""Independent arithmetic on v20 received evidence; no control or source changes."""
from pathlib import Path
import bisect
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from validate_ground_scan_geometry import compare_scan, load_candidate_world


def identity(path):
    raw = path.read_bytes()
    return {'path':path.relative_to(ROOT).as_posix(), 'bytes':len(raw),
            'sha256':hashlib.sha256(raw).hexdigest()}


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def xyz(row):
    return tuple(row[k] for k in ('x','y','z'))


def pose_metrics(rows):
    if not rows:
        return {'count':0}
    return {'count':len(rows), 'first':rows[0], 'last':rows[-1],
        'wall_span_s':rows[-1]['wall_elapsed_s']-rows[0]['wall_elapsed_s'],
        'sim_span_s':rows[-1]['stamp_s']-rows[0]['stamp_s'],
        'yaw_change_rad':wrap(rows[-1]['yaw_rad']-rows[0]['yaw_rad']),
        'sampled_path_m':sum(math.dist(xyz(a),xyz(b)) for a,b in zip(rows,rows[1:])),
        'maximum_distance_from_first_m':max(math.dist(xyz(r),xyz(rows[0])) for r in rows),
        'maximum_linear_speed_mps':max(r['linear_speed'] for r in rows),
        'maximum_angular_speed_radps':max(r['angular_speed'] for r in rows),
        'minimum_angular_speed_radps':min(r['angular_speed'] for r in rows)}


def main():
    analysis=ROOT/'runs/analysis/20261112_route_v20_independent/summary.json'
    raw=ROOT/'runs/evidence/20261112/callback_route_v20.jsonl'
    world=ROOT/'external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world'
    before={'analysis':identity(analysis),'raw':identity(raw),'world':identity(world)}
    s=json.loads(analysis.read_text('utf-8'))
    core=s['snapshots']['assembled'];p=s['survey']['approach'];route=p['route']
    commands=s['series']['actual_commands']['records']
    trajectory=s['series']['trajectory_10hz']['records']
    assert s['mode']=='ground_observation_route'
    assert not any(s[key] for key in ('parse_errors','assembly_errors','diagnostic_errors','conflicts','missing_events','missing_series'))
    assert all(v['complete'] and not v['conflicted'] and not v['missing_rows'] for v in s['series'].values())
    assert all(b['wall_elapsed_s']>=a['wall_elapsed_s'] for a,b in zip(commands,commands[1:]))
    assert all(b['stamp_s']>a['stamp_s'] for a,b in zip(trajectory,trajectory[1:]))
    groups=[]
    for c in commands:
        if not groups or groups[-1]['state']!=c['state']:
            groups.append({'state':c['state'],'rows':[]})
        groups[-1]['rows'].append(c)
    segments=[]
    for i,g in enumerate(groups):
        rows=g['rows'];start=rows[0]['wall_elapsed_s']
        end=groups[i+1]['rows'][0]['wall_elapsed_s'] if i+1<len(groups) else rows[-1]['wall_elapsed_s']
        segments.append({'state':g['state'],'command_count':len(rows),'first_command':rows[0],
            'last_command':rows[-1],'until_next_state_wall_s':end-start,
            'linear_range_mps':[min(c['linear_x'] for c in rows),max(c['linear_x'] for c in rows)],
            'angular_range_radps':[min(c['angular_z'] for c in rows),max(c['angular_z'] for c in rows)],
            'trajectory':pose_metrics([r for r in trajectory if start<=r['wall_elapsed_s']<end])})
    start_scan=next(r for r in s['survey']['scan_records'] if r['stage']=='start')
    # Both are recorded values from the same call; no sensor time is substituted.
    node_origin_wall=p['first_motion_wall_s']-start_scan['recorded_wall_elapsed_s']
    turn_start=p['turn_started_wall_s']-node_origin_wall
    stop_command=next(c for c in commands if c['state']=='STOP')
    turn_commands=[c for c in commands if c['state']=='TURN' and c['wall_elapsed_s']>=turn_start-1e-5]
    low=next(c for c in turn_commands if 0<abs(c['angular_z'])<.1)
    low_start=low['wall_elapsed_s']
    tr_times=[r['wall_elapsed_s'] for r in trajectory]
    def prior_pose(t):
        index=bisect.bisect_right(tr_times,t)-1
        return None if index<0 else trajectory[index]
    turn_review={
        'leg_index_original':route['leg_index'], 'completed_leg_records':route['legs'],
        'controller_target_yaw_rad':p['target_yaw_rad'],
        'recorded_turn_start_absolute_wall_s':p['turn_started_wall_s'],
        'node_origin_from_same_recorded_call_wall_s':node_origin_wall,
        'turn_start_elapsed_wall_s':turn_start,
        'first_STOP_command':stop_command,
        'turn_start_to_first_STOP_wall_s':stop_command['wall_elapsed_s']-turn_start,
        'small_request_first_command':low,'small_request_prior_10hz_pose':prior_pose(low_start),
        'small_request_prior_heading_error_rad':wrap(p['target_yaw_rad']-prior_pose(low_start)['yaw_rad']),
        'small_request_to_first_STOP_wall_s':stop_command['wall_elapsed_s']-low_start,
        'small_request_trajectory':pose_metrics([r for r in trajectory if low_start<=r['wall_elapsed_s']<stop_command['wall_elapsed_s']]),
        'last_5s_turn_trajectory':pose_metrics([r for r in trajectory if stop_command['wall_elapsed_s']-5<=r['wall_elapsed_s']<stop_command['wall_elapsed_s']]),
        'last_turn_command':turn_commands[-1],
        'last_turn_prior_heading_error_rad':wrap(p['target_yaw_rad']-prior_pose(turn_commands[-1]['wall_elapsed_s'])['yaw_rad']),
        'large_request_interval':pose_metrics([r for r in trajectory if turn_start<=r['wall_elapsed_s']<low_start]),
        'turn_position_drift_from_recorded_origin_m':max(math.dist(xyz(r),p['turn_origin']) for r in trajectory if turn_start<=r['wall_elapsed_s']<stop_command['wall_elapsed_s']),
        'no_second_leg_DRIVE':len(route['legs'])==1,
        'no_final_heading_scan':not any(r['stage']=='final_heading' for r in s['survey']['scan_records']),
        'no_capture_phase':p['capture_count']==0 and not any(r['phase'].startswith('capture_') for r in s['native_images']),
        'resume_count_original':p['resume_count'],
        'interpretation':'Strong evidence of inadequate small angular request response after the .08rad control-law discontinuity; not isolated actuator cause or future-command acceptance.'}
    c_times=[c['wall_elapsed_s'] for c in commands]
    bins={'large_request_abs_ge_0_35':[],'small_request_abs_lt_0_1':[]}
    for a,b in zip(trajectory,trajectory[1:]):
        ta,tb=a['wall_elapsed_s'],b['wall_elapsed_s']
        ia,ib=bisect.bisect_right(c_times,ta)-1,bisect.bisect_right(c_times,tb)-1
        if ia<0 or not 0<tb-ta<=.35 or b['stamp_s']<=a['stamp_s'] or ta-c_times[ia]>.1:
            continue
        active=commands[ia:ib+1]
        if not all(c['state']=='TURN' and c['linear_x']==0 and c['angular_z']<0 for c in active):
            continue
        name=('large_request_abs_ge_0_35' if all(abs(c['angular_z'])>=.35-1e-9 for c in active)
              else 'small_request_abs_lt_0_1' if all(abs(c['angular_z'])<.1 for c in active) else None)
        if name:bins[name].append({'wall_s':tb-ta,'sim_s':b['stamp_s']-a['stamp_s'],
            'yaw_change_rad':wrap(b['yaw_rad']-a['yaw_rad']),
            'abs_command_range':[min(abs(c['angular_z'])for c in active),max(abs(c['angular_z'])for c in active)]})
    turn_review['command_response_bins']={name:{'interval_count':len(rows),
        'wall_s':sum(r['wall_s']for r in rows),'sim_s':sum(r['sim_s']for r in rows),
        'yaw_change_rad':sum(r['yaw_change_rad']for r in rows),
        'abs_command_range':[min(r['abs_command_range'][0]for r in rows),max(r['abs_command_range'][1]for r in rows)],
        'observed_yaw_per_wall_s':sum(r['yaw_change_rad']for r in rows)/sum(r['wall_s']for r in rows),
        'observed_yaw_per_sim_s':sum(r['yaw_change_rad']for r in rows)/sum(r['sim_s']for r in rows)} for name,rows in bins.items() if rows}
    geom=load_candidate_world(world);scans=[]
    for record in s['survey']['scan_records']:
        prior=record['odom'];scan=record['payload']
        argument={'pose':{'pose':{'position':dict(zip(('x','y','z'),prior['position'])),
            'orientation':dict(zip(('x','y','z','w'),prior['quaternion']))}}}
        check=compare_scan(scan,argument,geom,tolerance_m=.001)
        scan_stamp=scan['header']['stamp']['sec']+scan['header']['stamp']['nanosec']*1e-9
        scans.append({'stage':record['stage'],'scan_header_original':scan['header'],
            'prior_odom_original':prior,'scan_minus_prior_s':scan_stamp-prior['stamp_s'],
            'recorded_scan_minus_odom_s':record['scan_minus_odom_s'],
            'quaternion_norm':math.hypot(*prior['quaternion']),
            'identity_hypothesis':check,
            'alternative_x_plus_0_1m_metrics':compare_scan(scan,argument,geom,tolerance_m=.001,translation=(.1,0,0))['metrics'],
            'alternative_yaw_plus_0_05rad_metrics':compare_scan(scan,argument,geom,tolerance_m=.001,yaw=.05)['metrics']})
    images=[]
    for image in s['native_images']:
        path=analysis.parent/image['file_name'];file=identity(path)
        assert file['sha256']==image['sha256']==image['jpeg_sha256']
        r=image.get('exposure');prior=r.get('odom') if r else None
        images.append({'image':file,'topic':image['topic'],'phase':image['phase'],
            'stamp_s':image['stamp_s'],'exposure_validation':image['exposure_validation'],
            'exposure_original':r,'image_minus_prior_stamp_s':None if prior is None else image['stamp_s']-prior['stamp_s']})
    timing=core['callback_timing'];input_qos=timing['input_qos']
    result={'kind':'v20_independent_offline_route_review','source_identity':before,
        'analysis_status':s['status'],'record_count':s['record_count'],
        'mode_original':s['mode'],'official_scoring':'unverified','task_completion':'unverified','score':None,
        'core_state_reason_elapsed':{k:core[k]for k in ('state','reason','elapsed_s')},
        'series_rows':{k:len(v['records'])for k,v in s['series'].items()},
        'command_state_segments':segments,'turn_failure_review':turn_review,
        'route_metadata_original':route,'speed_validation_original':route['speed_validation'],
        'sampled_total_path_m':sum(math.dist(xyz(a),xyz(b))for a,b in zip(trajectory,trajectory[1:])),
        'input_qos_original':input_qos,'callback_timing_revision':timing['revision'],
        'odom_timing_spans_original':{k:timing['spans'][k]for k in ('receive:/odom','receive:/uav/odom')},
        'command_endpoints_original':core['command_endpoints'],
        'scan_geometry_hypothesis':{'fit_parameters':[],'interpolation':False,
            'world_from_odom':'identity candidate hypothesis','official_map':'unverified','stages':scans},
        'native_image_review':images,
        'limits':['Commands are publish-call observations, not isolated actuator acceptance.',
            'Trajectory is rounded 10Hz sampled source data; original precise source pose records are kept separately.',
            'Scan consistency tests only recorded ray surfaces and the composite map/sensor hypothesis.',
            'No second leg, final heading, capture triplet, recognition report or official score was produced.']}
    assert before=={'analysis':identity(analysis),'raw':identity(raw),'world':identity(world)}
    out=Path(__file__).with_name('independent_metrics.json')
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf-8')
    print(json.dumps({'output':out.relative_to(ROOT).as_posix(),'turn':turn_review,
        'input_qos':input_qos,'scan_metrics':[{'stage':r['stage'],'metrics':r['identity_hypothesis']['metrics'],
            'consistent':r['identity_hypothesis']['candidate_consistency']}for r in scans],
        'images':[{k:v for k,v in r.items()if k!='exposure_original'}for r in images]},ensure_ascii=True,indent=2))


if __name__=='__main__':main()
