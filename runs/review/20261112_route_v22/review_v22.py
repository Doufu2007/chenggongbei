"""Independent v22 terminal-turn/endpoint arithmetic, preserving received evidence."""
from pathlib import Path
import hashlib
import json
import math
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from validate_ground_scan_geometry import compare_scan,load_candidate_world


def ident(path):
    raw=path.read_bytes()
    return {'path':path.relative_to(ROOT).as_posix(),'bytes':len(raw),
            'sha256':hashlib.sha256(raw).hexdigest()}


def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def xyz(r):return [r[k]for k in ('x','y','z')]
def summary(rows):
    if not rows:return {'count':0}
    return {'count':len(rows),'first':rows[0],'last':rows[-1],
        'wall_span_s':rows[-1]['wall_elapsed_s']-rows[0]['wall_elapsed_s'],
        'sim_span_s':rows[-1]['stamp_s']-rows[0]['stamp_s'],
        'yaw_delta_rad':wrap(rows[-1]['yaw_rad']-rows[0]['yaw_rad']),
        'sampled_path_m':sum(math.dist(xyz(a),xyz(b))for a,b in zip(rows,rows[1:])),
        'max_distance_from_first_m':max(math.dist(xyz(r),xyz(rows[0]))for r in rows),
        'max_linear_speed_mps':max(r['linear_speed']for r in rows),
        'max_angular_speed_radps':max(r['angular_speed']for r in rows)}


def main():
    analysis=ROOT/'runs/analysis/20261112_route_v22_independent/summary.json'
    raw=ROOT/'runs/evidence/20261112/callback_route_v22.jsonl'
    world=ROOT/'external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world'
    before={'analysis':ident(analysis),'raw':ident(raw),'world':ident(world)}
    s=json.loads(analysis.read_text('utf-8'));core=s['snapshots']['assembled'];p=s['survey']['approach'];route=p['route']
    cs=s['series']['actual_commands']['records'];tr=s['series']['trajectory_10hz']['records']
    assert p['draw'][0]=='ground_3' and route['leg_index']==2
    assert not any(s[k]for k in ('parse_errors','assembly_errors','diagnostic_errors','conflicts','missing_events','missing_series'))
    assert all(v['complete'] and not v['missing_rows'] and not v['conflicted']for v in s['series'].values())
    groups=[]
    for c in cs:
        if not groups or groups[-1]['state']!=c['state']:groups.append({'state':c['state'],'rows':[]})
        groups[-1]['rows'].append(c)
    segments=[]
    for i,g in enumerate(groups):
        rows=g['rows'];start=rows[0]['wall_elapsed_s']
        end=groups[i+1]['rows'][0]['wall_elapsed_s']if i+1<len(groups)else rows[-1]['wall_elapsed_s']
        segments.append({'state':g['state'],'first_command':rows[0],'last_command':rows[-1],
            'command_count':len(rows),'until_next_state_wall_s':end-start,
            'trajectory':summary([r for r in tr if start<=r['wall_elapsed_s']<end])})
    arrival=next(r for r in s['survey']['scan_records']if r['stage']=='observation_arrival')
    node_origin=p['turn_started_wall_s']-arrival['recorded_wall_elapsed_s']
    final_turn_start=arrival['recorded_wall_elapsed_s']
    turn_settle=[x for x in segments if x['state']=='TURN_SETTLE'][-1]
    stop=next(c for c in cs if c['state']=='STOP')
    waypoint=route['plan']['waypoints_xy_m'][-1]
    start=route['legs'][-1]['final_position']
    scan_end=p['scan_anchor']['odom']['position']
    last=core['odom_last']['pose']['pose']['position'];end=[last[k]for k in ('x','y','z')]
    def error(point):
        vector=[point[i]-waypoint[i]for i in range(2)]
        return {'xy_m':point[:2],'error_xy_m':vector,'error_norm_m':math.hypot(*vector)}
    drift=[scan_end[i]-start[i]for i in range(2)]
    last_leg=route['legs'][-1];yaw=last_leg['yaw_rad'];origin=last_leg['origin']
    def projection(point):
        dx,dy=point[0]-origin[0],point[1]-origin[1]
        return {'forward_m':dx*math.cos(yaw)+dy*math.sin(yaw),
            'cross_track_m':-dx*math.sin(yaw)+dy*math.cos(yaw)}
    fixed_drift_replays=[]
    for gap in (.01,.005,.002,0.):
        shift=.01-gap
        point=[scan_end[0]+shift*math.cos(yaw),scan_end[1]+shift*math.sin(yaw)]
        fixed_drift_replays.append({'assumed_advance_beyond_original_stop_m':shift,'assumed_stop_gap_m':gap,
            **error(point),'margin_to_0_02_m':.02-error(point)['error_norm_m'],
            'status':'fixed_displacement_geometric_replay_not_control_prediction'})
    final_turn={'node_origin_from_same_recorded_arrival_call_wall_s':node_origin,
        'start_elapsed_wall_s':final_turn_start,'target_yaw_rad':p['target_yaw_rad'],
        'start_error':error(start),'last_scan_prior_error':error(scan_end),'last_odom_error':error(end),
        'start_to_last_scan_prior_delta_xy_m':drift,'start_to_last_scan_prior_displacement_m':math.hypot(*drift),
        'original_last_scan_prior':p['scan_anchor']['odom'],
        'final_settle_trajectory':turn_settle['trajectory'],
        'final_turn_trajectory':summary([r for r in tr if final_turn_start<=r['wall_elapsed_s']<turn_settle['first_command']['wall_elapsed_s']]),
        'start_to_first_STOP_wall_s':stop['wall_elapsed_s']-final_turn_start,
        'last_prior_yaw_error_rad':wrap(p['scan_anchor']['odom']['yaw_rad']-p['target_yaw_rad']),
        'fixed_displacement_replays':fixed_drift_replays,
        'mechanism':'The existing endpoint error and observed translation during final turn jointly exceed the unchanged 0.02m waypoint check.'}
    last_drive_settle=[x for x in segments if x['state']=='DRIVE_SETTLE'][-1]
    zero=last_drive_settle['first_command'];pre=max((r for r in tr if r['wall_elapsed_s']<=zero['wall_elapsed_s']),key=lambda r:r['wall_elapsed_s'])
    post=min((r for r in tr if r['wall_elapsed_s']>zero['wall_elapsed_s']),key=lambda r:r['wall_elapsed_s'])
    settle_projection=projection(start);pre_proj=projection(xyz(pre));post_proj=projection(xyz(post))
    last_drive={'precise_start':origin,'planned_heading_rad':yaw,'target_projection_m':last_leg['target_projection_m'],
        'original_stop_trigger_target_m':p['limits']['target_distance_m'],
        'precise_settled_position':start,'settled_projection':settle_projection,
        'settled_remaining_forward_to_target_m':last_leg['target_projection_m']-settle_projection['forward_m'],
        'original_first_zero_command':zero,'bracketing_pre_zero_10hz':pre,'bracketing_post_zero_10hz':post,
        'pre_zero_to_settled_forward_increment_m':settle_projection['forward_m']-pre_proj['forward_m'],
        'post_zero_to_settled_forward_increment_m':settle_projection['forward_m']-post_proj['forward_m'],
        'bracket_sample_distance_m':math.dist(xyz(pre),xyz(post)),
        'settle_evidence_scope':'Pre sample may precede the zero request; this is sampled bracketing evidence, not exact or continuous postzero stopping distance.'}
    geom=load_candidate_world(world);scans=[]
    for r in s['survey']['scan_records']:
        prior=r['odom'];pose={'pose':{'pose':{'position':dict(zip(('x','y','z'),prior['position'])),
            'orientation':dict(zip(('x','y','z','w'),prior['quaternion']))}}}
        check=compare_scan(r['payload'],pose,geom,tolerance_m=.001)
        scans.append({'stage':r['stage'],'scan_header_original':r['payload']['header'],
            'prior_odom_original':prior,'scan_minus_odom_s_original':r['scan_minus_odom_s'],
            'candidate_identity_hypothesis':check})
    images=[]
    for image in s['native_images']:
        file=ident(analysis.parent/image['file_name']);assert file['sha256']==image['sha256']==image['jpeg_sha256']
        images.append({'file':file,'topic':image['topic'],'phase':image['phase'],'stamp_s':image['stamp_s'],
            'original_exposure':image['exposure'],'exposure_validation':image['exposure_validation']})
    result={'kind':'v22_independent_terminal_turn_review','source_identity':before,
        'official_scoring':'unverified','task_completion':'unverified','score':None,
        'core_state_reason_elapsed':{k:core[k]for k in ('state','reason','elapsed_s')},
        'record_count':s['record_count'],'series_rows':{k:len(v['records'])for k,v in s['series'].items()},
        'command_state_segments':segments,'route_metadata_original':route,
        'final_turn_review':final_turn,'last_drive_stop_review':last_drive,
        'scan_geometry_hypothesis':{'fitting':False,'interpolation':False,'world_from_odom':'identity hypothesis','stages':scans},
        'native_images':images,
        'limits':['Replay adds hypothetical displacement to the measured final pose; it predicts neither future slip nor control outcome.',
            'The 0.002m stop-gap replay has only about 0.34mm margin in this single run, not a stability guarantee.',
            'All position/heading/collision/freshness/runtime limits remain unchanged.',
            'There is no final_heading scan or CAPTURE triplet; two legs are motion evidence, not scored mission completion.']}
    assert before=={'analysis':ident(analysis),'raw':ident(raw),'world':ident(world)}
    out=Path(__file__).with_name('independent_metrics.json')
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf-8')
    print(json.dumps({'output':out.relative_to(ROOT).as_posix(),'final_turn':final_turn,
        'last_drive_stop':last_drive,'source_identity':before,
        'scan_metrics':[{'stage':r['stage'],'metrics':r['candidate_identity_hypothesis']['metrics'],
            'consistent':r['candidate_identity_hypothesis']['candidate_consistency']}for r in scans]},ensure_ascii=True,indent=2))


if __name__=='__main__':main()
