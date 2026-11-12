"""Independent byte/resource/motion review of the confirmed-complete v38 run.

No SIFT, production mutation, simulator, upload or referee-score inference.
"""
from pathlib import Path
import base64,collections,hashlib,importlib.util,json,math,sys,zlib
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tools')]
import analyze_ground_three_point_evidence as transport
spec=importlib.util.spec_from_file_location('v35_readonly_helpers',ROOT/'runs/analysis/20261112_three_point_axis_v35/analyze.py')
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
MODE='ground_three_point_recovery_budget'
helpers.MODE=MODE  # Reuse only schema-neutral, standard-library audit helpers.
OUT=Path(__file__).resolve().parent
source=ROOT/'runs/evidence/20261112/callback_three_point_recovery_v38.jsonl'
raw=source.read_bytes();digest=hashlib.sha256(raw).hexdigest()
assert digest=='ba8cc75e76b677ba5ef4e11861c2bc9af2c9b0033f1a08dc718b280cb1060854'
events=[transport._json(line) for line in raw.decode('utf-8').splitlines() if line.strip()]
records=[{'line':i+1,'event':row['payload']['event'],'data':row['payload']['data']} for i,row in enumerate(events)]
parts=[r['data'] for r in records if r['data'].get('section')==transport.SECTION]
body,meta=transport.decode_bundle(parts)
app=body['survey']['approach'];survey=body['survey'];point=body['current_point']
core=next(r['data'] for r in records if r['data'].get('section')=='three_point_core')
motion_core=next(r['data'] for r in records if r['data'].get('section')=='ground_motion_core')
bindings={
    'exact_mode':core['mode']==motion_core['mode']==survey['mode']==app['mode']==MODE,
    'recovery_revision':app['revision']=='ground_three_point_recovery_budget_v1',
    'same_session':core['session_id']==body['session_id']==app['three_point']['session_id']==point['session_id'],
    'state_reason':(core['state'],core['reason'])==(motion_core['state'],motion_core['reason'])==(app['state'],app['reason']),
    'origin':core['global_origin']==app['global_origin'],
    'path_rotation':core['total_path_m']==app['total_path_m'] and core['accumulated_rotation_rad']==app['accumulated_rotation_rad'],
    'draw_target_index':core['mission_draw']==app['three_point']['mission_draw'] and core['target_id']==app['target_id']==app['three_point']['target_id'] and core['point_index']==app['three_point']['point_index'],
    'frozen_limits':core['frozen_limits']==app['three_point']['frozen_limits'],
    'completed_count':core['completed_count']==len(body['completed_points'])==0,
    'current_point':point==app['single_point'],
    'parent_pretty_bundle_equal':body==json.loads((ROOT/'runs/analysis/20261112_three_point_recovery_v38/bundle.json').read_text(encoding='utf-8')),
}
assert all(bindings.values()),bindings
images=helpers.decode_images(body['capture_image_events'])
point_audit=helpers.audit_points(body,images)
motion=helpers.audit_motion(body,records)
assert not motion['command_contract_violations'] and not motion['ledger_errors']
assert motion['sampled_path_does_not_exceed_ledger'] and motion['sampled_rotation_does_not_exceed_ledger']
assert all(row['counts_agree'] for row in motion['string_publications'].values())
for i,im in enumerate(images):
    file=ROOT/f'runs/analysis/20261112_three_point_recovery_v38/point_0_capture_{i}.jpg'
    assert hashlib.sha256(file.read_bytes()).hexdigest()==im['jpeg_sha256']

resource=app['recovery_resource'];camera=app['camera_recovery'];pauses=resource['records']
pause_checks=[]
for row in pauses:
    elapsed=row['ended_wall_s']-row['started_wall_s']
    assert abs(elapsed-row['elapsed_wall_s'])<=1e-9
    assert row['resume_count_after']==row['resume_count_before']+1
    assert elapsed<resource['single_pause_wall_limit_s']
    pause_checks.append({'phase':row['phase'],'reason':row['reason'],'elapsed_wall_s':elapsed,
                         'count_before':row['resume_count_before'],'count_after':row['resume_count_after'],
                         'entry_timing':row['entry_timing'],'exit_timing':row['exit_timing'],
                         'start_elapsed_s':row['started_wall_elapsed_s'],
                         'end_elapsed_s':row['started_wall_elapsed_s']+elapsed})
assert sum(row['elapsed_wall_s'] for row in pauses)==resource['completed_pause_wall_s']
assert resource['resume_count']==app['resume_count']==1
assert resource['active_pause_wall_s']==0. and resource['total_pause_wall_s']==resource['completed_pause_wall_s']
assert resource['max_resumes']==app['max_resumes']==camera['max_resumes']==32
commands=survey['actual_commands'];logs=motion['motion_log']
stop=next(row for row in logs if row['state']=='STOP')
recognize=next(row for row in logs if row['state']=='RECOGNIZE_WAIT')
capture=next(row for row in logs if row['state']=='CAPTURE')
pause_commands=[]
for pause in pause_checks:
    rows=[row for row in commands if pause['start_elapsed_s']<=row['wall_elapsed_s']<=pause['end_elapsed_s']]
    after=next(row for row in commands if row['wall_elapsed_s']>=pause['end_elapsed_s'])
    next_nonzero=next(row for row in commands if row['wall_elapsed_s']>=pause['end_elapsed_s'] and (row['linear_x'] or row['angular_z']))
    assert all(row['linear_x']==row['angular_z']==0. for row in rows)
    assert after['linear_x']==after['angular_z']==0.
    pause_commands.append({'commands_inside_pause':len(rows),'all_zero':True,'first_command_after_resume':after,
                           'first_nonzero_after_resume':next_nonzero})
capture_commands=[row for row in commands if capture['wall_elapsed_s']<=row['wall_elapsed_s']<=stop['wall_elapsed_s']]
post_stop_commands=[row for row in commands if row['wall_elapsed_s']>=stop['wall_elapsed_s']]
assert all(row['linear_x']==row['angular_z']==0. for row in capture_commands+post_stop_commands)
callback=motion_core['callback_timing']
timing={label:{'count':data['wall_s']['count'],'wall_max_s':data['wall_s']['max'],
                       'wall_mean_s':data['wall_s']['total']/data['wall_s']['count'] if data['wall_s']['count'] else None,
                       'cpu_max_s':data['cpu_s']['max'],'receive_gap_max_s':data['entry_gap_wall_s']['max'],
                       'source_delta_max_s':data['header_delta_s']['max'],
                       'clock_minus_header_min_s':data['clock_minus_header_s']['min'],
                       'clock_minus_header_max_s':data['clock_minus_header_s']['max']}
        for label,data in callback['spans'].items()}
observed=[]
for row in motion['operations']:
    duration=row['ended_wall_s']-row['started_wall_s']
    observed.append({'index':row['operation_index'],'kind':row['operation']['kind'],'wall_s':duration,
        'actual_path_m':row['path_end_m']-row['path_origin_m'],
        'actual_rotation_rad':row['rotation_end_rad']-row['rotation_origin_rad'],
        'actual_start_pose':row['actual_start_pose'],'actual_end_pose':row['actual_end_pose']})
end=observed[-1]['actual_end_pose'];plan=app['three_point']['plan']
target_error={'position_error_m':math.dist(end[:2],plan['observation_xy_m']),
              'yaw_error_rad':helpers.wrap(end[2]-plan['observation_yaw_rad']),
              'actual_end_pose':end,'requested_observation_xy':plan['observation_xy_m'],
              'requested_observation_yaw':plan['observation_yaw_rad']}
resource_out={'raw':resource,'independent_pause_checks':pause_checks,'command_boundary':pause_commands,
              'camera':camera,'point_recapture_count':point['recapture_count'],
              'interpretation':'one successful ordinary scan pause; camera recapture and 32-slot extension not exercised'}
timing_out={'spans':timing,'stop_timing':stop['timing'],'camera_liveness_at_last_check':point['camera_liveness'],
            'snapshot_timing':motion_core['timing'],'graph':motion_core['graph_refresh_diagnostics'],
            'guard_lock_wait':callback['guard_lock_wait'],'pre_receive_transport':callback['pre_receive_transport']}
summary={'schema':'v38_independent_platform_analysis_v1','raw_sha256':digest,'raw_bytes':len(raw),'events':len(events),
    'transport':meta,'bindings':bindings,'state':app['state'],'reason':app['reason'],
    'mission_draw':core['mission_draw'],'point_index':core['point_index'],'target_id':core['target_id'],
    'completed_points':len(body['completed_points']),'capture_count':len(images),'accepted_match_results':len(point['match_results']),
    'matcher_failures':len(point['matcher_failures']),'recognition':point['recognition'],'attempts':point['attempts'],
    'in_entry':point['in_entry'],'camera_recapture_events':len(camera['events']),
    'resume_count':resource['resume_count'],'completed_pause_wall_s':resource['completed_pause_wall_s'],
    'motion_elapsed_at_core_s':motion_core['elapsed_s'],'stop_elapsed_s':stop['wall_elapsed_s'],
    'recognize_wait_to_stop_s':stop['wall_elapsed_s']-recognize['wall_elapsed_s'],
    'capture_start_to_stop_s':stop['wall_elapsed_s']-capture['wall_elapsed_s'],
    'reported_path_m':app['total_path_m'],'reported_rotation_rad':app['accumulated_rotation_rad'],
    'retained_10hz_path_lower_bound_m':motion['sampled_path_lower_bound_m'],
    'retained_10hz_rotation_lower_bound_rad':motion['sampled_rotation_lower_bound_rad'],
    'max_observed_displacement_m':app['max_origin_displacement_m'],
    'command_rows':len(commands),'nonzero_commands':motion['returned_nonzero'],'zero_commands':sum(not (r['linear_x'] or r['angular_z']) for r in commands),
    'core_nonzero_count':motion_core['published_nonzero_count'],'core_zero_count':motion_core['published_zero_count'],
    'command_vs_core_zero_count_delta':sum(not (r['linear_x'] or r['angular_z']) for r in commands)-motion_core['published_zero_count'],
    'command_violations':motion['command_contract_violations'],'negative_x_commands':motion['returned_negative_x'],
    'capture_through_stop_zero_rows':len(capture_commands),'post_stop_zero_rows':len(post_stop_commands),
    'ledger_errors':motion['ledger_errors'],'final_observation_error':target_error,
    'leg_errors_m':[row['recomputed_waypoint_error_m'] for row in motion['legs']],
    'source_prior_skews_s':[im['priors'][0]['reported_skew_s'] if 'reported_skew_s' in im['priors'][0] else im['priors'][0] for im in point_audit[0]['source_images']],
    'string_publications':motion['string_publications'],'axis_attempt_sequence_status':motion['axis_attempt_sequence_status'],
    'publication_failure_count':motion['publication_failure_count'],'launcher_exit_code':records[-1]['data'].get('exit_code'),
    'official_score':None,'full_mission_complete':False,'sift_reexecuted':False,'production_modified':False}
OUT.mkdir(parents=True,exist_ok=True)
def save(name,data):(OUT/name).write_text(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8')
save('summary.json',summary);save('resource_audit.json',resource_out);save('motion_audit.json',motion)
save('capture_source_audit.json',point_audit);save('timing_audit.json',timing_out);save('operation_rates.json',observed)
assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
print(json.dumps(summary,ensure_ascii=False,indent=2))
