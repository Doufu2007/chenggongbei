"""Independent arithmetic for the complete v23 observation-only route evidence."""
from pathlib import Path
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from validate_ground_scan_geometry import compare_scan, load_candidate_world


def ident(path):
    raw = path.read_bytes()
    return {'path': path.relative_to(ROOT).as_posix(), 'bytes': len(raw),
            'sha256': hashlib.sha256(raw).hexdigest()}


def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def xyz(r): return [r[k] for k in ('x', 'y', 'z')]


def motion_summary(rows):
    if not rows: return {'count': 0}
    return {'count': len(rows), 'first': rows[0], 'last': rows[-1],
        'wall_span_s': rows[-1]['wall_elapsed_s']-rows[0]['wall_elapsed_s'],
        'sim_span_s': rows[-1]['stamp_s']-rows[0]['stamp_s'],
        'yaw_delta_rad': wrap(rows[-1]['yaw_rad']-rows[0]['yaw_rad']),
        'sampled_path_m': sum(math.dist(xyz(a), xyz(b)) for a,b in zip(rows,rows[1:])),
        'max_distance_from_first_m': max(math.dist(xyz(r), xyz(rows[0])) for r in rows),
        'max_linear_speed_mps': max(r['linear_speed'] for r in rows),
        'max_angular_speed_radps': max(r['angular_speed'] for r in rows)}


def main():
    analysis = ROOT/'runs/analysis/20261112_route_v23_independent/summary.json'
    raw = ROOT/'runs/evidence/20261112/callback_route_v23.jsonl'
    world = ROOT/'external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world'
    identities = {'analysis': ident(analysis), 'raw': ident(raw), 'world': ident(world)}
    s = json.loads(analysis.read_text('utf-8')); core = s['snapshots']['assembled']
    p = s['survey']['approach']; route = p['route']; plan = route['plan']
    cs = s['series']['actual_commands']['records']; tr = s['series']['trajectory_10hz']['records']
    assert not any(s[k] for k in ('parse_errors','assembly_errors','diagnostic_errors','conflicts','missing_events','missing_series'))
    assert not s['survey']['image_errors']
    assert all(v['complete'] and not v['missing_rows'] and not v['conflicted'] for v in s['series'].values())
    assert p['draw'][0] == plan['first_id'] == 'ground_1'
    assert core['state'] == 'FINISHED' and core['reason'] == 'route_observation_complete'
    assert route['leg_index'] == 2 and p['capture_count'] == 3
    raw_records = [json.loads(x) for x in raw.read_text('utf-8').splitlines() if x.strip()]
    exits = [r['payload']['data'] for r in raw_records if r['payload']['event']=='launcher_exit']
    assert len(exits)==1 and exits[0]['exit_code']==0

    groups=[]
    for c in cs:
        if not groups or groups[-1]['state'] != c['state']: groups.append({'state':c['state'],'rows':[]})
        groups[-1]['rows'].append(c)
    segments=[]
    for i,g in enumerate(groups):
        rows=g['rows']; start=rows[0]['wall_elapsed_s']
        end=groups[i+1]['rows'][0]['wall_elapsed_s'] if i+1<len(groups) else rows[-1]['wall_elapsed_s']
        segments.append({'state':g['state'],'first_command':rows[0],'last_command':rows[-1],
            'command_count':len(rows),'until_next_state_wall_s':end-start,
            'trajectory':motion_summary([r for r in tr if start<=r['wall_elapsed_s']<end])})

    scans={r['stage']:r for r in s['survey']['scan_records']}
    assert set(scans)=={'start','first_leg_end','observation_arrival','final_heading'}
    # Both values originate from the same observation-arrival call; no cross-clock fitting.
    node_origin=p['turn_started_wall_s']-scans['observation_arrival']['recorded_wall_elapsed_s']
    legs=[]
    for leg in route['legs']:
        target=leg['waypoint_xy_m']; final=leg['final_position']; stop=leg['stop_request']
        stop_elapsed=stop['wall_s']-node_origin; end_elapsed=leg['ended_wall_s']-node_origin
        index=leg['leg_index']; yaw=leg['yaw_rad']; origin=leg['origin']
        vector=[final[i]-target[i] for i in range(2)]
        projection=lambda point:(point[0]-origin[0])*math.cos(yaw)+(point[1]-origin[1])*math.sin(yaw)
        settle_commands=[c for c in cs if c['state']=='DRIVE_SETTLE' and stop_elapsed-1e-5<=c['wall_elapsed_s']<=end_elapsed+1e-5]
        assert settle_commands and all(c['linear_x']==c['angular_z']==0 for c in settle_commands)
        after=[r for r in tr if stop_elapsed<=r['wall_elapsed_s']<=end_elapsed]
        pre=max((r for r in tr if r['wall_elapsed_s']<=stop_elapsed),key=lambda r:r['wall_elapsed_s'])
        post=min((r for r in tr if r['wall_elapsed_s']>stop_elapsed),key=lambda r:r['wall_elapsed_s'])
        assert math.hypot(*vector)<.02 and abs(math.hypot(*vector)-leg['waypoint_error_m'])<1e-12
        assert 0<=stop['path_increment_m']<.01
        legs.append({'leg_index':index,'target_xy_m':target,'final_position_original':final,
            'recomputed_error_xy_m':vector,'recomputed_error_norm_m':math.hypot(*vector),
            'planned_target_projection_m':leg['target_projection_m'],
            'request_zero_projection_threshold_m':leg['target_projection_m']-.002,
            'final_projection_m':projection(final),'final_remaining_projection_m':leg['target_projection_m']-projection(final),
            'stop_request_original':stop,'stop_request_elapsed_wall_s':stop_elapsed,
            'first_settle_zero_publication_original':settle_commands[0],
            'request_to_first_recorded_settle_zero_s':settle_commands[0]['wall_elapsed_s']-stop_elapsed,
            'original_sampled_stop_path_increment_m':stop['path_increment_m'],
            'post_trigger_10hz_observation':motion_summary(after),
            'bracketing_pre_trigger_10hz':pre,'bracketing_post_trigger_10hz':post,
            'scope':'path_increment is original guard cumulative odom from phase request, including pre-first-zero time; 10Hz path is a sampled corroboration, not a continuous physical stopping bound.'})

    final_target=plan['waypoints_xy_m'][-1]
    final_prior=scans['final_heading']['odom']
    last=core['odom_last']['pose']['pose']['position']; last_xy=[last['x'],last['y']]
    final_error=lambda pos: {'xy_m':pos[:2], 'error_xy_m':[pos[i]-final_target[i] for i in range(2)],
        'error_norm_m':math.dist(pos[:2],final_target)}
    turn_start=route['legs'][-1]['final_position']; turn_end=final_prior['position']
    terminal={'target_xy_m':final_target,'actual_recomputed_heading_target_rad':p['target_yaw_rad'],
        'planning_candidate_yaw_rad':plan['final_yaw_rad'],
        'final_heading_prior_original':final_prior,
        'final_heading_prior_error':final_error(turn_end),
        'last_odom_error':final_error(last_xy),
        'recomputed_final_yaw_error_rad':wrap(final_prior['yaw_rad']-p['target_yaw_rad']),
        'final_turn_delta_xy_m':[turn_end[i]-turn_start[i] for i in range(2)],
        'final_turn_net_displacement_m':math.dist(turn_end[:2],turn_start[:2])}
    assert terminal['final_heading_prior_error']['error_norm_m']<.02
    assert abs(terminal['recomputed_final_yaw_error_rad'])<.025

    geom=load_candidate_world(world); scan_reviews=[]
    for r in s['survey']['scan_records']:
        prior=r['odom']; pose={'pose':{'pose':{'position':dict(zip(('x','y','z'),prior['position'])),
            'orientation':dict(zip(('x','y','z','w'),prior['quaternion']))}}}
        check=compare_scan(r['payload'],pose,geom,tolerance_m=.001)
        scan_reviews.append({'stage':r['stage'],'scan_header_original':r['payload']['header'],
            'prior_odom_original':prior,'scan_minus_odom_s_original':r['scan_minus_odom_s'],
            'candidate_identity_hypothesis':check})
    images=[]; capture=[]
    for img in s['native_images']:
        file=ident(analysis.parent/img['file_name'])
        assert file['sha256']==img['sha256']==img['jpeg_sha256']
        entry={'file':file,'topic':img['topic'],'phase':img['phase'],'stamp_s':img['stamp_s'],
            'original_exposure':img['exposure'],'exposure_validation':img['exposure_validation']}
        images.append(entry)
        if img['phase'].startswith('capture_'):
            e=img['exposure']; o=e['odom']
            assert e['association_error'] is None and o['association']=='prior_uav_odom'
            assert o['stamp_s']<=img['stamp_s'] and 0<=img['stamp_s']-o['stamp_s']<=.05
            assert o['wall_elapsed_s']<=e['wall_elapsed_s']
            assert img['stamp_s']>final_prior['stamp_s']
            entry['recomputed_image_minus_prior_s']=img['stamp_s']-o['stamp_s']
            entry['recomputed_waypoint_error_m']=math.dist(o['position'][:2],final_target)
            entry['recomputed_yaw_error_rad']=wrap(o['yaw_rad']-p['target_yaw_rad'])
            assert entry['recomputed_waypoint_error_m']<.02
            capture.append(entry)
    assert len(capture)==3 and {x['phase']for x in capture}=={'capture_0','capture_1','capture_2'}
    assert len({x['stamp_s']for x in capture})==3 and len({x['file']['sha256']for x in capture})==3
    silent_topics={topic:value.get('silent') for topic,value in core['command_endpoints'].items()}
    assert all(silent_topics[x] is True for x in ('/uav/cmd_vel','/inspection/detections','/referee/current_target','/referee/cruise_event','/uav/laser_fire'))
    assert '/referee/recognition_status' not in core['sample_counts']
    result={'kind':'v23_independent_observation_route_review','source_identity':identities,
        'official_scoring':'unverified','task_completion':'unverified','score':None,
        'local_route_observation_completion':'verified_from_received_source_evidence',
        'core_state_reason_elapsed':{k:core[k]for k in ('state','reason','elapsed_s')},
        'launcher_exit':exits[0], 'draw_original':p['draw'],
        'record_count':s['record_count'],'series_rows':{k:len(v['records'])for k,v in s['series'].items()},
        'command_state_segments':segments,'route_metadata_original':route,'leg_reviews':legs,
        'terminal_review':terminal,'captures':capture,'native_images':images,
        'scan_geometry_hypothesis':{'fitting':False,'interpolation':False,'world_from_odom':'identity hypothesis','stages':scan_reviews},
        'input_qos_original':{topic:core['callback_timing']['input_qos'][topic] for topic in ('/odom','/uav/odom')},
        'output_silent_original':silent_topics,'recognition_status_received_count':core['sample_counts'].get('/referee/recognition_status',0),
        'limits':['This run only exercised the ground_1 observation route; v22 first target differs, so this is not a paired improvement estimate.',
            'Candidate scan geometry checks do not independently validate official point ID, image category, all-route collision safety or scoring.',
            'The stop-path values are received guard sampled cumulative odometry from a phase request, not actuator acknowledgment or a continuous physical bound.',
            'Three real captures and local FINISHED do not establish recognition, entry, referee acceptance, full mission completion or 95% reliability.']}
    assert identities=={'analysis':ident(analysis),'raw':ident(raw),'world':ident(world)}
    output=Path(__file__).with_name('independent_metrics.json')
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf-8')
    print(json.dumps({'output':output.relative_to(ROOT).as_posix(),'source_identity':identities,
        'state':result['core_state_reason_elapsed'],'terminal':terminal,
        'legs':[{'leg':x['leg_index'],'error_m':x['recomputed_error_norm_m'],'stop_path_m':x['original_sampled_stop_path_increment_m'],
            'zero_delay_s':x['request_to_first_recorded_settle_zero_s']}for x in legs],
        'scan_metrics':[{'stage':x['stage'],'metrics':x['candidate_identity_hypothesis']['metrics'],
            'consistent':x['candidate_identity_hypothesis']['candidate_consistency']}for x in scan_reviews],
        'capture_stamps':[x['stamp_s']for x in capture]},ensure_ascii=True,indent=2))


if __name__=='__main__': main()
