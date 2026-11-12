"""Read-only v21 command/trajectory direction arithmetic; no actuation assumptions."""
from pathlib import Path
import bisect
import hashlib
import json
import math
import statistics

ROOT=Path(__file__).resolve().parents[3]


def ident(path):
    data=path.read_bytes()
    return {'path':path.relative_to(ROOT).as_posix(),'bytes':len(data),
            'sha256':hashlib.sha256(data).hexdigest()}


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def finite_difference(rows, commands, tier=None):
    times=[c['wall_elapsed_s']for c in commands]
    result=[]
    for a,b in zip(rows,rows[1:]):
        ta,tb=a['wall_elapsed_s'],b['wall_elapsed_s']
        ia,ib=bisect.bisect_right(times,ta)-1,bisect.bisect_right(times,tb)-1
        if ia<0 or not 0<tb-ta<=.35 or b['stamp_s']<=a['stamp_s'] or ta-times[ia]>.1:
            continue
        active=commands[ia:ib+1]
        if not all(c['state']=='DRIVE' and c['angular_z']==0 and c['linear_x']>0
                   and (tier is None or c['linear_x']==tier)for c in active):continue
        dx,dy=b['x']-a['x'],b['y']-a['y'];ds=math.hypot(dx,dy)
        yaw=a['yaw_rad']+wrap(b['yaw_rad']-a['yaw_rad'])/2
        q=(a['qx'],a['qy'],a['qz'],a['qw'])
        roll=math.atan2(2*(q[3]*q[0]+q[1]*q[2]),1-2*(q[0]*q[0]+q[1]*q[1]))
        pitch=math.asin(max(-1,min(1,2*(q[3]*q[1]-q[2]*q[0]))))
        result.append({'start_wall_s':ta,'end_wall_s':tb,'wall_s':tb-ta,
            'sim_s':b['stamp_s']-a['stamp_s'],'dx_m':dx,'dy_m':dy,'xy_path_m':ds,
            'direction_rad':math.atan2(dy,dx),'mean_yaw_rad':yaw,
            'direction_minus_yaw_rad':wrap(math.atan2(dy,dx)-yaw),
            'body_forward_increment_m':dx*math.cos(yaw)+dy*math.sin(yaw),
            'body_left_increment_m':-dx*math.sin(yaw)+dy*math.cos(yaw),
            'heading_aligned_predicted_dx_m':ds*math.cos(yaw),
            'heading_aligned_predicted_dy_m':ds*math.sin(yaw),
            'observed_reported_linear_speed_start_mps':a['linear_speed'],
            'observed_reported_angular_speed_start_radps':a['angular_speed'],
            'roll_rad':roll,'pitch_rad':pitch,
            'prior_command':commands[ia],'last_command_through_interval':commands[ib]})
    return result


def aggregate(rows):
    if not rows:return {'interval_count':0}
    total=lambda key:sum(r[key]for r in rows)
    dx,dy=total('dx_m'),total('dy_m')
    length=total('xy_path_m');wall=total('wall_s');sim=total('sim_s')
    yaw=math.atan2(sum(r['xy_path_m']*math.sin(r['mean_yaw_rad'])for r in rows),
                   sum(r['xy_path_m']*math.cos(r['mean_yaw_rad'])for r in rows))
    return {'interval_count':len(rows),'first_wall_s':rows[0]['start_wall_s'],
        'last_wall_s':rows[-1]['end_wall_s'],'wall_s':wall,'sim_s':sim,
        'dx_m':dx,'dy_m':dy,'xy_path_m':length,'net_direction_rad':math.atan2(dy,dx),
        'distance_weighted_body_yaw_rad':yaw,
        'net_direction_minus_weighted_body_yaw_rad':wrap(math.atan2(dy,dx)-yaw),
        'body_yaw_range_rad':[min(r['mean_yaw_rad']for r in rows),max(r['mean_yaw_rad']for r in rows)],
        'direction_minus_body_yaw_range_rad':[min(r['direction_minus_yaw_rad']for r in rows),max(r['direction_minus_yaw_rad']for r in rows)],
        'body_forward_increment_m':total('body_forward_increment_m'),
        'body_left_increment_m':total('body_left_increment_m'),
        'observed_world_vx_per_sim_s':dx/sim,'observed_world_vy_per_sim_s':dy/sim,
        'observed_body_forward_per_sim_s':total('body_forward_increment_m')/sim,
        'observed_body_left_per_sim_s':total('body_left_increment_m')/sim,
        'heading_aligned_predicted_dx_m':total('heading_aligned_predicted_dx_m'),
        'heading_aligned_predicted_dy_m':total('heading_aligned_predicted_dy_m'),
        'direction_median_rad':statistics.median(r['direction_rad']for r in rows),
        'max_observed_angular_speed_radps':max(r['observed_reported_angular_speed_start_radps']for r in rows),
        'max_abs_roll_rad':max(abs(r['roll_rad'])for r in rows),
        'max_abs_pitch_rad':max(abs(r['pitch_rad'])for r in rows)}


def main():
    analysis=ROOT/'runs/analysis/20261112_route_v21_independent/summary.json'
    raw=ROOT/'runs/evidence/20261112/callback_route_v21.jsonl'
    before={'analysis':ident(analysis),'raw':ident(raw)}
    s=json.loads(analysis.read_text('utf-8'));a=s['snapshots']['assembled'];p=s['survey']['approach']
    cs=s['series']['actual_commands']['records'];tr=s['series']['trajectory_10hz']['records']
    assert s['mode']=='ground_observation_route' and a['reason']=='approach_cross_track_limit'
    assert not any(s[k]for k in ['parse_errors','assembly_errors','conflicts','missing_series','missing_events','diagnostic_errors'])
    assert all(v['complete'] and not v['missing_rows'] and not v['conflicted']for v in s['series'].values())
    groups=[]
    for c in cs:
        if not groups or groups[-1]['state']!=c['state']:groups.append({'state':c['state'],'rows':[]})
        groups[-1]['rows'].append(c)
    segments=[]
    for i,g in enumerate(groups):
        cr=g['rows'];start=cr[0]['wall_elapsed_s'];end=groups[i+1]['rows'][0]['wall_elapsed_s']if i+1<len(groups)else cr[-1]['wall_elapsed_s']
        rows=[r for r in tr if start<=r['wall_elapsed_s']<end]
        d={'state':g['state'],'first_command':cr[0],'last_command':cr[-1],
            'until_next_state_wall_s':end-start,'command_count':len(cr),'trajectory_count':len(rows)}
        if rows:
            dx,dy=rows[-1]['x']-rows[0]['x'],rows[-1]['y']-rows[0]['y']
            d.update(first_pose=rows[0],last_pose=rows[-1],dx_m=dx,dy_m=dy,
                net_direction_rad=math.atan2(dy,dx),yaw_change_rad=wrap(rows[-1]['yaw_rad']-rows[0]['yaw_rad']),
                sample_wall_span_s=rows[-1]['wall_elapsed_s']-rows[0]['wall_elapsed_s'],
                sample_sim_span_s=rows[-1]['stamp_s']-rows[0]['stamp_s'],
                max_distance_from_first_m=max(math.dist((r['x'],r['y'],r['z']),(rows[0]['x'],rows[0]['y'],rows[0]['z']))for r in rows),
                max_angular_speed_radps=max(r['angular_speed']for r in rows))
        segments.append(d)
    first_drive=next(d for d in segments if d['state']=='DRIVE')
    last_stop=next(c for c in cs if c['state']=='STOP')
    all_intervals=finite_difference(tr,cs)
    high=finite_difference(tr,cs,.10);low=finite_difference(tr,cs,.03)
    resumed_high=[r for r in high if r['start_wall_s']>=8.839703]
    origin=p['drive_origin'];yaw=p['drive_yaw_rad']
    drive_start_pose=first_drive['first_pose'];drive_last=next(r for r in reversed(tr)if r['wall_elapsed_s']<last_stop['wall_elapsed_s'])
    dx,dy=drive_last['x']-origin[0],drive_last['y']-origin[1]
    actual_yaw=drive_start_pose['yaw_rad'];dist=math.hypot(dx,dy)
    # Same net distance under heading-aligned motion: decomposition only, no fitting.
    heading_only_cross=dist*math.sin(actual_yaw-yaw)
    observed_cross=-dx*math.sin(yaw)+dy*math.cos(yaw)
    result={'kind':'v21_independent_offline_direction_review','source_identity':before,
        'official_scoring':'unverified','task_completion':'unverified','score':None,
        'core_state_reason_elapsed':{k:a[k]for k in ['state','reason','elapsed_s']},
        'record_count':s['record_count'],'series_rows':{k:len(v['records'])for k,v in s['series'].items()},
        'original_route_metadata':p['route'],'original_approach':{k:v for k,v in p.items()if k not in ['sensor_sync','route']},
        'command_state_segments':segments,
        'drive_direction_intervals':all_intervals,
        'direction_aggregates':{'all_DRIVE':aggregate(all_intervals),'tier_0_03':aggregate(low),
            'tier_0_10':aggregate(high),'resumed_tier_0_10_after_full_calibration_stop':aggregate(resumed_high)},
        'heading_error_decomposition':{
            'frozen_drive_yaw_rad':yaw,'first_DRIVE_10hz_yaw_rad':actual_yaw,
            'first_DRIVE_heading_error_rad':wrap(actual_yaw-yaw),
            'first_DRIVE_pose':drive_start_pose,'last_pre_STOP_pose':drive_last,
            'net_displacement_from_precise_drive_origin_m':[dx,dy],
            'net_direction_from_drive_origin_rad':math.atan2(dy,dx),
            'observed_cross_track_recomputed_m':observed_cross,
            'heading_aligned_same_distance_cross_track_m':heading_only_cross,
            'extra_cross_over_heading_aligned_decomposition_m':observed_cross-heading_only_cross,
            'interpretation':'Initial turn residual alone is insufficient to explain the observed cross track; the translation direction differs from the received body yaw even after a full intermediate stop.'},
        'input_qos_original':a['callback_timing']['input_qos'],
        'observed_odom_periods_original':s['survey']['observed_odom_periods'],
        'speed_validation_original':p['route']['speed_validation'],
        'limits':['No isolated actuator/frame cause is proven by this single run.',
            '10Hz positions and quaternions are source-rounded; exact boundary metadata is retained.',
            'No linear_y command or wheel feedback is contained in these command series.',
            'Per-interval direction uses only preceding actual publish calls, never future commands.',
            'No cross-track, heading, collision, freshness or runtime bound is changed.']}
    assert before=={'analysis':ident(analysis),'raw':ident(raw)}
    out=Path(__file__).with_name('independent_metrics.json')
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf-8')
    print(json.dumps({'output':out.relative_to(ROOT).as_posix(),
        'direction_aggregates':result['direction_aggregates'],
        'heading_error_decomposition':result['heading_error_decomposition'],
        'source_identity':before},ensure_ascii=True,indent=2))


if __name__=='__main__':main()
