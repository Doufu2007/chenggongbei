"""Single-point transport stays distinct and never upgrades sent claims to scores."""
from copy import deepcopy
import importlib
import importlib.util

import pytest

from test_analyze_ground_motion_evidence import event, write_events
from test_analyze_ground_observation_route_evidence import route_payloads
from test_analyze_ground_survey_evidence import series


MODE='ground_single_point'


def single_point_payloads():
    values,raw_image=route_payloads()
    for value in values:
        value['mode']=MODE
        if isinstance(value.get('startup'),dict):value['startup']['mode']=MODE
    meta=values[1]['survey'];meta['mode']=meta['approach']['mode']=MODE
    meta['approach']['route']['mode']=MODE
    detail={'session_id':'synthetic-local-session','plan':{'first_id':'ground_1','status':'candidate_unverified'},
        'in_entry':True,'stationary_epoch':1,'epoch_invalid':None,'recapture_count':0,
        'recognition':{'class_id':'rescue_camp','label':'救援营地','first_id':'ground_1',
            'stationary_epoch':1,'data':'发现救援营地，地面巡航任务点确认，将巡航信息发送回指挥中心。','official_acceptance':False},
        'match_results':[{'job_seq':1,'image_stamp_s':10.02,'source_sha256':'a'*64,
            'pose':{'stamp_s':10.},'candidate_calibration':True}],
        'attempts':{'recognition':{'topic':'/inspection/detections','data':'原始测试话术',
            'attempted':True,'sent':True,'wall_s':12.,'clock_stamp_s':21.,'odom_stamp_s':21.,
            'position':[0.,-1.35,.055],'first_id':'ground_1','epoch':1,'official_acceptance':False}},
        'recognition_status':[{'data':'{"accepted":true,"score":10}', 'wall_s':13.,
            'clock_stamp_s':22.,'phase':'REPORT_WAIT','acceptance_semantics':'unknown_string_not_parsed_as_ack'}],
        'official_acceptance':False,'body_fully_inside':'not_claimed_center_entry_only','full_mission_complete':False}
    meta['approach']['single_point']=deepcopy(detail)
    special={'mode':MODE,'read_only':False,'official_scoring':'unverified','partial':True,
        'section':'single_point_core','revision':'ground_single_point_v1','state':'FINISHED',
        'reason':'single_point_transport_complete_official_score_unverified',**detail}
    return [special,*values],raw_image


def analyze(tmp_path,values,name='point'):
    name_module='tools.analyze_ground_single_point_evidence'
    assert importlib.util.find_spec(name_module) is not None,'explicit single-point analyzer is missing'
    events=[event('launcher_started',{'mode':MODE,'read_only':False}),
        event('probe_started',{'mode':MODE,'read_only':False}),
        *[event('probe_snapshot',value)for value in values],
        event('launcher_exit',{'mode':MODE,'read_only':False,'exit_code':0})]
    source=write_events(tmp_path/f'{name}.jsonl',events);before=source.read_bytes()
    result=importlib.import_module(name_module).analyze_file(source,tmp_path/f'out_{name}')
    assert source.read_bytes()==before
    return result


def test_real_payloads_keep_native_bytes_attempts_raw_status_and_no_score_inference(tmp_path):
    values,raw=single_point_payloads();before=deepcopy(values)
    result=analyze(tmp_path,values)
    assert values==before
    assert result['kind']=='ground_single_point_analysis'and result['status']=='collected'
    assert result['mode']==result['survey']['mode']==result['survey']['approach']['mode']==MODE
    assert result['single_point']['attempts']==before[0]['attempts']
    assert result['single_point']['recognition_status'][0]['data']=='{"accepted":true,"score":10}'
    assert result['single_point']['match_results']==before[0]['match_results']
    assert result['single_point_observations'][0]['data']==before[0]
    assert result['missing_single_point_core']is False
    assert result['official_scoring']==result['task_completion']=='unverified'and result['score']is None
    assert len(result['native_images'])==3 and not result['missing_capture_phases']
    assert all((tmp_path/'out_point'/x['file_name']).read_bytes()==raw for x in result['native_images'])
    assert all(x['exposure_validation']=='valid_prior_pose_observation'for x in result['native_images'])
    assert result['series']['actual_commands']['records'][1]['state']=='CAPTURE'


@pytest.mark.parametrize('scope',['source','metadata','approach'])
def test_mixed_old_route_mode_is_rejected_without_relabelling(tmp_path,scope):
    values,_=single_point_payloads()
    if scope=='source':values[0]['mode']='ground_observation_route'
    elif scope=='metadata':values[2]['survey']['mode']='ground_observation_route'
    else:values[2]['survey']['approach']['mode']='ground_observation_route'
    assert analyze(tmp_path,values)['status']=='invalid'


@pytest.mark.parametrize('mutation',[{'partial':False},{'official_acceptance':True},
    {'full_mission_complete':True},{'match_results':{}},{'recognition_status':{}},{'attempts':[]}])
def test_invalid_single_point_core_claim_or_container_is_explicitly_invalid(tmp_path,mutation):
    values,_=single_point_payloads();values[0].update(mutation)
    result=analyze(tmp_path,values)
    assert result['status']=='invalid'and result['parse_errors']
    assert result['score']is None and result['task_completion']=='unverified'


def test_missing_dedicated_core_retains_embedded_evidence_and_marks_partial(tmp_path):
    values,_=single_point_payloads();result=analyze(tmp_path,values[1:])
    assert result['status']=='partial'and result['missing_single_point_core']is True
    assert result['single_point']['recognition']['class_id']=='rescue_camp'
    assert result['single_point_observations']==[]
    assert result['score']is None


def test_conflicting_dedicated_and_embedded_attempts_remain_visible(tmp_path):
    values,_=single_point_payloads();values[2]['survey']['approach']['single_point']['attempts']['recognition']['sent']=False
    result=analyze(tmp_path,values)
    assert result['status']=='partial'
    assert any(x['scope']=='single_point_metadata'for x in result['conflicts'])
    assert result['single_point']['attempts']['recognition']['sent']is True
    assert result['survey']['approach']['single_point']['attempts']['recognition']['sent']is False


def test_conflicting_duplicate_core_and_unknown_field_preserve_raw_observations(tmp_path):
    values,_=single_point_payloads();different=deepcopy(values[0]);different['attempts']['recognition']['sent']=False
    different['future_diagnostic']={'kept':'exactly'}
    values.insert(1,different);result=analyze(tmp_path,values)
    assert result['status']=='partial'and len(result['single_point_observations'])==2
    assert any(x['scope']=='single_point_core'for x in result['conflicts'])
    assert any(x['fields'].get('future_diagnostic')=={'kept':'exactly'}for x in result['unknown_fields'])
    assert result['score']is None


def test_new_mode_cannot_enter_old_route_analyzer_and_uses_explicit_route_capacity(tmp_path):
    values,_=single_point_payloads();source=write_events(tmp_path/'point.jsonl',[event('probe_snapshot',values[0])])
    old=importlib.import_module('tools.analyze_ground_observation_route_evidence').analyze_file(source,tmp_path/'old')
    assert old['status']=='invalid'
    fragments=[]
    for offset in range(0,14000,300):
        fragment=series('actual_commands',[[0.,10.,0.,0.,'WAIT']for _ in range(min(300,14000-offset))],offset,14000)
        fragment['mode']=MODE;fragments.append(fragment)
    good=analyze(tmp_path,fragments,name='capacity')
    assert not good['parse_errors']and good['series']['actual_commands']['complete']
    fragments[0]['total_rows']=14001
    assert analyze(tmp_path,fragments[:1],name='overflow')['status']=='invalid'


def test_original_success_job_and_failed_job_execution_unknown_remain_exact(tmp_path):
    values,_=single_point_payloads()
    job={'job_seq':1,'source_ros_sha256':'b'*64,'source_sha256':'a'*64,
        'frame_id':'uav/camera_link','topic':'/uav/camera/image','image_stamp_s':10.02,
        'source_ros_encoding':'rgb8','source_ros_stride':2408,
        'camera_info':{'header':{'frame_id':'uav/camera_link','stamp':{'sec':9,'nanosec':2}},
            'width':800,'height':800,'k':[1,0,0,0,1,0,0,0,1],'d':[0,0,0,0,0]}}
    failure={'job':deepcopy(job),'error':'worker_process_exited: -1',
        'job_was_dispatched':True,'job_was_executed':None}
    for target in (values[0],values[2]['survey']['approach']['single_point']):
        target['match_results'][0]['job']=deepcopy(job)
        target['matcher_failures']=[deepcopy(failure)]
    result=analyze(tmp_path,values)
    assert result['single_point']['match_results'][0]['job']==job
    assert result['single_point']['matcher_failures']==[failure]
    assert result['single_point']['matcher_failures'][0]['job_was_executed']is None
    assert result['single_point_observations'][0]['data']['matcher_failures']==[failure]
    assert result['score']is None and result['official_scoring']=='unverified'


def test_failed_job_list_cannot_silently_change_container_type(tmp_path):
    values,_=single_point_payloads();values[0]['matcher_failures']={}
    result=analyze(tmp_path,values)
    assert result['status']=='invalid' and result['parse_errors']


def test_camera_liveness_is_preserved_as_observed_timing_without_refreshing_exposures(tmp_path):
    values,_=single_point_payloads()
    camera={'wall_s':12.,'phase':'RECOGNIZE_WAIT','camera_max_wall_age_s':.5,
        'last_image_arrival_wall_s':11.7,'last_image_stamp_s':20.7,'image_wall_age_s':.3,
        'clock_minus_image_stamp_s':.3,'interpretation':'current liveness only'}
    for target in (values[0],values[2]['survey']['approach']['single_point']):
        target['camera_liveness']=deepcopy(camera)
    result=analyze(tmp_path,values)
    assert result['single_point']['camera_liveness']==camera
    assert result['single_point']['match_results'][0]['image_stamp_s']==10.02
    assert result['score']is None


def test_camera_liveness_bad_container_is_not_silently_accepted(tmp_path):
    values,_=single_point_payloads();values[0]['camera_liveness']=[]
    result=analyze(tmp_path,values)
    assert result['status']=='invalid' and result['parse_errors']


def compact_single_point_reference(values):
    approach=values[2]['survey']['approach']
    approach.pop('single_point')
    approach['single_point_transport']={'section':'single_point_core',
        'session_id':values[0]['session_id'],'interpretation':'dedicated_event_no_duplicate_payload'}
    return approach['single_point_transport']


def test_compact_metadata_refers_to_the_dedicated_original_session_without_duplication(tmp_path):
    values,_=single_point_payloads();reference=compact_single_point_reference(values)
    result=analyze(tmp_path,values)
    assert result['status']=='collected'
    assert 'single_point'not in result['survey']['approach']
    assert result['survey']['approach']['single_point_transport']==reference
    assert result['single_point']['session_id']==reference['session_id']


def test_compact_reference_to_another_session_stays_an_explicit_conflict(tmp_path):
    values,_=single_point_payloads();reference=compact_single_point_reference(values)
    reference['session_id']='different-synthetic-session'
    result=analyze(tmp_path,values)
    assert result['status']=='partial'
    assert any(item['scope']=='single_point_transport'for item in result['conflicts'])
    assert result['single_point']['session_id']!=result['survey']['approach']['single_point_transport']['session_id']


def test_compact_reference_cannot_silently_refer_to_another_section(tmp_path):
    values,_=single_point_payloads();reference=compact_single_point_reference(values)
    reference['section']='imaginary_success'
    result=analyze(tmp_path,values)
    assert result['status']=='invalid' and result['parse_errors']


def fallback_fragment(topic,value):
    return {'mode':MODE,'read_only':False,'official_scoring':'unverified','partial':True,
        'section':'survey','topic':topic,'value':deepcopy(value)}


def test_fallback_survey_top_fields_are_explicit_observations_without_creating_full_snapshot(tmp_path):
    values,_=single_point_payloads()
    compact_single_point_reference(values)
    values=[value for value in values if value.get('section')!='ground_motion_full']
    metadata=next(value['survey']for value in values if value.get('section')=='survey_metadata')
    fragments=[fallback_fragment(key,value)for key,value in metadata.items()]
    result=analyze(tmp_path,[*values,*fragments])
    assert result['unknown_sections']==[]
    assert result['survey_fragment_assembly']==metadata
    assert [item['data']for item in result['survey_fragment_observations']]==fragments
    assert result['snapshots']['full_count']==0 and result['status']=='partial'
    assert result['score']is None and result['task_completion']=='unverified'


def test_conflicting_fallback_survey_field_preserves_both_observations(tmp_path):
    values,_=single_point_payloads()
    result=analyze(tmp_path,[*values,fallback_fragment('trajectory_dropped',0),fallback_fragment('trajectory_dropped',1)])
    assert result['status']=='partial'
    assert any(item['scope']=='survey_fragment'for item in result['conflicts'])
    assert len(result['survey_fragment_observations'])==2


@pytest.mark.parametrize('mutation',[{'topic':None},{'partial':False},
    {'topic':'mode','value':'ground_observation_route'}])
def test_fallback_survey_fragment_shape_and_original_mode_are_enforced(tmp_path,mutation):
    values,_=single_point_payloads();fragment=fallback_fragment('mode',MODE);fragment.update(mutation)
    result=analyze(tmp_path,[*values,fragment])
    assert result['status']=='invalid' and result['parse_errors']


@pytest.mark.parametrize('fault',[None,'sensor_freshness_lost'])
def test_actual_single_point_emitter_preserves_nullable_epoch_fault_reason(tmp_path,fault):
    from inspection_mission.ground_single_point import GroundSinglePointGuard
    from inspection_mission.telemetry import DiagnosticReporter
    guard=GroundSinglePointGuard(started_wall_s=0.,session_id='synthetic-emitter-review')
    guard.epoch_invalid=fault
    emitted=guard.additional_events()
    for name,payload in emitted: DiagnosticReporter._encode(name,payload)
    source=write_events(tmp_path/'emitted.jsonl',[event(name,payload,source='synthetic_guard_fixture_not_platform')for name,payload in emitted])
    name='tools.analyze_ground_single_point_evidence'
    result=importlib.import_module(name).analyze_file(source,tmp_path/'out_emitted')
    assert result['parse_errors']==result['assembly_errors']==[]
    assert result['single_point']['epoch_invalid']==fault
    assert result['survey']['approach']['single_point_transport']['section']=='single_point_core'
    assert result['survey']['approach']['single_point_transport']['session_id']==guard.session_id
    assert 'single_point'not in result['survey']['approach']
    assert result['official_scoring']=='unverified'and result['score']is None
