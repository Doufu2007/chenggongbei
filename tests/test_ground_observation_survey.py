"""Ground survey rotates once toward a declared candidate, never reports a score."""
import importlib
import importlib.util
import math
import time
from types import SimpleNamespace

import pytest

from test_bounded_ground_probe import odom, scan, endpoint, install_ros_transport


def api():
    name="inspection_mission.ground_observation_survey"
    assert importlib.util.find_spec(name) is not None,"ground observation survey is missing"
    return importlib.import_module(name)


def posed_odom(stamp=10.,*,yaw=0.,x=0.,yaw_speed=0.):
    payload=odom(stamp,x=x)
    payload["pose"]["pose"]["orientation"]={"x":0.,"y":0.,"z":math.sin(yaw/2),"w":math.cos(yaw/2)}
    payload["twist"]["twist"]["angular"]["z"]=yaw_speed
    return payload


def guard(*,target_yaw=None):
    g=api()
    from inspection_mission.bounded_ground_probe import MotionLimits
    return g.GroundSurveyGuard(MotionLimits(.447,.114,.02),started_wall_s=0.,
        survey_limits=g.SurveyLimits(target_yaw=target_yaw))


def ready(state,*,draw="ground_1,ground_4,ground_0"):
    state.update_endpoints([endpoint()],matched_subscriptions=1)
    state.observe_public("/referee/ground_tasks",{"data":draw},wall_s=0.)
    decisions=[]
    for i in range(23):
        wall=i*.05
        state.update_clock(10.+wall,wall_s=wall)
        state.update_odom(posed_odom(10.+wall),wall_s=wall)
        state.update_scan(scan(10.+wall),wall_s=wall)
        decisions.append(state.step(wall_s=wall))
    return decisions[-1]


def refresh(state,wall,*,yaw=0.,x=0.,yaw_speed=0.):
    state.update_clock(10.+wall,wall_s=wall)
    state.update_odom(posed_odom(10.+wall,yaw=yaw,x=x,yaw_speed=yaw_speed),wall_s=wall)
    state.update_scan(scan(10.+wall),wall_s=wall)


def test_draw_first_candidate_determines_yaw_and_only_bounded_angular_command():
    state=guard()
    decision=ready(state)
    assert decision.state=="TURN" and decision.linear_x==0.
    assert -.5<=decision.angular_z<0.
    metadata=state.survey_evidence()
    assert metadata["draw"]==["ground_1","ground_4","ground_0"]
    assert metadata["target_id"]=="ground_1"
    assert metadata["target_yaw"]==pytest.approx(-math.pi/2)
    assert metadata["candidate_mapping"]["status"]=="candidate_unverified"
    assert metadata["candidate_mapping"]["hypothesis"]=="ground_N -> static ground_task_point_N"


@pytest.mark.parametrize("draw",["ground_5,ground_1,ground_2","ground_0,ground_0,ground_1",
    "ground_1,ground_2","ground_1,,ground_2","junk","ground_1,ground_2,ground_3,ground_4"])
def test_unknown_duplicate_or_malformed_draw_latches_stop(draw):
    state=guard(); decision=ready(state,draw=draw)
    assert decision.linear_x==decision.angular_z==0. and state.state in ("STOP","FINISHED")
    state.observe_public("/referee/ground_tasks",{"data":"ground_0,ground_1,ground_2"},wall_s=1.2)
    refresh(state,1.2)
    assert state.step(wall_s=1.2).angular_z==0.


def test_changed_draw_after_arming_stops_instead_of_replanning():
    state=guard(); ready(state)
    state.observe_public("/referee/ground_tasks",{"data":"ground_0,ground_1,ground_2"},wall_s=1.2)
    refresh(state,1.2)
    decision=state.step(wall_s=1.2)
    assert decision.angular_z==0. and "draw_changed" in decision.reason


def test_no_draw_cannot_move_even_with_explicit_target_yaw():
    state=guard(target_yaw=-1.)
    state.update_endpoints([endpoint()],matched_subscriptions=1)
    for i in range(23):
        wall=i*.05; refresh(state,wall)
        decision=state.step(wall_s=wall)
    assert decision.linear_x==decision.angular_z==0.


def test_target_reached_dwell_is_zero_for_two_seconds_and_no_restart():
    state=guard(target_yaw=-1.)
    ready(state)
    refresh(state,1.2,yaw=-.99)
    reached=state.step(wall_s=1.2)
    assert reached.angular_z==0. and reached.reason=="target_yaw_reached"
    refresh(state,3.,yaw=-.99)
    assert state.step(wall_s=3.).state=="STOP"
    refresh(state,3.21,yaw=-.99)
    assert state.step(wall_s=3.21).state=="FINISHED"
    refresh(state,4.,yaw=0.)
    assert state.step(wall_s=4.).angular_z==0.


@pytest.mark.parametrize("fault",["displacement","angular_speed","clock_stale","scan_stale","odom_stale","subscriber"])
def test_active_turn_faults_latch_zero(fault):
    state=guard(); ready(state)
    refresh(state,1.2,yaw=-.1,x=.06 if fault=="displacement" else 0.,yaw_speed=.56 if fault=="angular_speed" else .5)
    wall=1.2
    if fault=="subscriber": state.update_endpoints([],matched_subscriptions=0)
    elif fault=="clock_stale": wall=1.41
    elif fault=="scan_stale":
        state.update_clock(11.41,wall_s=1.41); state.update_odom(posed_odom(11.41,yaw=-.1),wall_s=1.41); wall=1.41
    elif fault=="odom_stale":
        state.update_clock(11.41,wall_s=1.41); state.update_scan(scan(11.41),wall_s=1.41); wall=1.41
    decision=state.step(wall_s=wall)
    expected="PAUSE" if fault in ("clock_stale","scan_stale","odom_stale") else "STOP"
    assert decision.linear_x==decision.angular_z==0. and decision.state==expected
    refresh(state,1.5,yaw=-.1)
    assert state.step(wall_s=1.5).angular_z==0.


def test_turn_total_angle_and_wall_deadline_are_hard_budgets():
    state=guard(); ready(state)
    refresh(state,1.2,yaw=-1.81)
    assert state.step(wall_s=1.2).reason=="rotation_budget"
    state=guard(); ready(state)
    refresh(state,26.01,yaw=-.2)
    decision=state.step(wall_s=26.01)
    assert decision.angular_z==0. and decision.reason=="turn_wall_time_limit"


def test_request_outside_yaw_budget_refuses_start_and_limits_cannot_expand():
    state=guard(target_yaw=2.)
    assert ready(state).angular_z==0. and state.state in ("STOP","FINISHED")
    g=api()
    for changes in ({"yaw_rate":.501},{"max_turn_wall_s":45.1},{"max_rotation_rad":1.81}):
        with pytest.raises(ValueError):g.SurveyLimits(**changes)


def test_exposure_association_uses_only_prior_ground_odom_with_point_zero_five_skew():
    state=guard()
    state.update_odom(posed_odom(10.,yaw=-.1),wall_s=0.)
    state.update_odom(posed_odom(10.04,yaw=-.2),wall_s=.04)
    uav_payload=posed_odom(10.03,yaw=.7)
    uav_payload["child_frame_id"]="uav/base_link"
    state.update_uav_odom(uav_payload,wall_s=.03)
    for topic,stamp in (("/camera/image",10.03),("/camera/image",10.12),("/uav/camera/image",10.04)):
        sec=int(stamp)
        msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=round((stamp-sec)*1e9)),
            frame_id="camera_link" if topic=="/camera/image" else "uav/camera_link"))
        state.observe_image(topic,msg,wall_s=.15)
    exposures=state.survey_evidence()["exposure_records"]
    ground=[item for item in exposures if item["topic"]=="/camera/image"]
    assert ground[0]["odom"]["stamp_s"]==10. and ground[0]["odom"]["yaw_rad"]==pytest.approx(-.1)
    assert ground[0]["odom"]["association"]=="prior_ground_odom"
    ground_last=next(item for item in ground if item["phase"]=="last")
    assert ground_last["odom"] is None and ground_last["association_error"]=="prior_odom_too_old"
    aerial=[item for item in exposures if item["topic"]=="/uav/camera/image"]
    assert aerial[0]["odom"]["child_frame_id"]=="uav/base_link"
    assert aerial[0]["odom"]["association"]=="prior_uav_odom"


def test_survey_adapter_publishes_only_zero_linear_bounded_yaw_and_records_command_times(monkeypatch):
    g=api();callbacks,published,publishers=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_survey_node(wall_clock=lambda:wall[0])
    callbacks["/referee/ground_tasks"]({"data":"ground_1,ground_4,ground_0"})
    for i in range(25):
        wall[0]=i*.05
        stamp=10.+wall[0];sec=int(stamp)
        callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((stamp-sec)*1e9)}})
        callbacks["/odom"](posed_odom(stamp))
        callbacks["/scan"](scan(stamp))
        node.tick()
    assert any(angular<0. for _,_,angular in published)
    assert all(topic=="/cmd_vel" and linear==0. and abs(angular)<=.5 for topic,linear,angular in published)
    evidence=node.guard.survey_evidence()
    assert evidence["actual_commands"][-1]["angular_z"]<0.
    assert evidence["actual_commands"][-1]["wall_elapsed_s"]==pytest.approx(1.2)
    assert len(evidence["trajectory_10hz"])<=14
    node.stop("test_end");node.close_watchdog()


def test_survey_watchdog_zeroes_on_sensor_loss_without_ros_spin(monkeypatch):
    g=api();_,published,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_survey_node(wall_clock=lambda:wall[0])
    node.guard.update_clock(10.,wall_s=0.)
    node.guard.state="TURN";node.guard.pulse_started=0.;node.guard.pulse_origin=(0.,0.,.055)
    wall[0]=.3
    node.arm_watchdog()
    deadline=time.monotonic()+.15
    while time.monotonic()<deadline and not published:time.sleep(.002)
    node.close_watchdog()
    assert published and all(angular==linear==0. for _,linear,angular in published)
    assert node.guard.state=="PAUSE"


def test_native_800px_images_are_real_reassemblable_jpeg_and_each_event_fits(monkeypatch):
    import base64,hashlib,json
    from array import array
    from inspection_mission.telemetry import DiagnosticReporter
    cv2=pytest.importorskip("cv2");np=pytest.importorskip("numpy")
    state=guard()
    for stamp in (10.,10.02):state.update_odom(posed_odom(stamp,yaw=-.4),wall_s=stamp-10.)
    # Noise requires multiple 32k parts. This checks true native pixels rather
    # than a resized 360px thumbnail or a synthetic success screen.
    pixels=np.random.default_rng(23).integers(0,256,(800,800,3),dtype=np.uint8)
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=20_000_000),frame_id="camera_link"),
        width=800,height=800,step=2400,encoding="bgr8",data=array("B",pixels.tobytes()))
    state.observe_image("/camera/image",msg,wall_s=.02)
    events=state.additional_events()
    image_parts=[payload for event,payload in events if payload.get("section")=="survey_image_part"]
    assert image_parts and len(image_parts)>1
    assert {part["part_index"] for part in image_parts}==set(range(image_parts[0]["part_count"]))
    jpeg=base64.b64decode("".join(part["jpeg_base64_part"] for part in image_parts))
    assert hashlib.sha256(jpeg).hexdigest()==image_parts[0]["jpeg_sha256"]
    decoded=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    assert decoded.shape==(800,800,3)
    for event,payload in events:assert len(DiagnosticReporter._encode(event,payload))<65536


def test_bounded_series_and_metadata_survive_as_small_independent_events():
    from inspection_mission.telemetry import DiagnosticReporter
    state=guard()
    ready(state)
    for i in range(600):
        wall=1.2+i*.05
        state.update_odom(posed_odom(10.+wall,yaw=-.2),wall_s=wall)
        state.record_command(wall_s=wall,linear_x=0.,angular_z=-.15)
    events=state.additional_events()
    payloads=[payload for event,payload in events]
    assert payloads[0]["section"]=="survey_metadata"
    assert payloads[0]["survey"]["candidate_mapping"]["official_id_mapping_confirmed"] is False
    series=[payload for payload in payloads if payload.get("section")=="survey_motion_series"]
    assert {item["series"] for item in series}=={"trajectory_10hz","actual_commands"}
    commands=[item for item in series if item["series"]=="actual_commands"]
    assert sum(len(item["rows"]) for item in commands)==600
    assert [item["offset"] for item in commands]==[0,300]
    for event,payload in events:assert len(DiagnosticReporter._encode(event,payload))<65536


def test_survey_launcher_uses_only_survey_entry_and_no_simulator_or_referee_start():
    from pathlib import Path
    path=Path("scripts/run_ground_observation_survey.sh")
    assert path.exists()
    source=path.read_text(encoding="utf-8")
    assert "-m inspection_mission.ground_observation_survey --duration 45" in source
    assert "mode\": \"ground_observation_survey" in source
    assert "ros2 launch" not in source and "gazebo" not in source


@pytest.mark.parametrize("first",list(range(5)))
def test_each_public_candidate_id_is_selected_dynamically_without_fixed_first_id(first):
    from inspection_mission.ground_observation_survey import CANDIDATE_POINTS
    state=guard()
    order=[first,(first+1)%5,(first+2)%5]
    decision=ready(state,draw=",".join("ground_"+str(i) for i in order))
    point=CANDIDATE_POINTS["ground_"+str(first)]
    assert state.target_id=="ground_"+str(first)
    assert state.target_yaw==pytest.approx(math.atan2(point[1],point[0]))
    assert decision.linear_x==0. and abs(decision.angular_z)<=.5


def test_survey_watchdog_stops_at_global_deadline_even_before_turn_deadline():
    g=api()
    from inspection_mission.bounded_ground_probe import MotionLimits
    state=g.GroundSurveyGuard(MotionLimits(.447,.114,.02),started_wall_s=0.,
        survey_limits=g.SurveyLimits(global_wall_s=5.))
    ready(state)
    refresh(state,5.,yaw=-.1)
    assert state.watchdog_reason(5.)=="survey_global_wall_time_limit"


def test_epoch_reversal_invalidates_new_image_odometry_association():
    state=guard()
    state.update_clock(10.,wall_s=0.)
    state.update_odom(posed_odom(10.),wall_s=0.)
    state.update_clock(9.,wall_s=.1)
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0),frame_id="camera_link"))
    state.observe_image("/camera/image",msg,wall_s=.1)
    exposure=state.survey_evidence()["exposure_records"][-1]
    assert exposure["odom"] is None and exposure["association_error"]=="clock_epoch_invalid"


def test_complete_survey_final_report_keeps_metadata_series_and_native_images_independent(monkeypatch):
    import json
    from array import array
    from inspection_mission.telemetry import DiagnosticReporter
    pytest.importorskip("cv2");pytest.importorskip("numpy")
    g=api();callbacks,published,_=install_ros_transport(monkeypatch)
    events=[]
    class Reporter:
        def emit(self,event,payload):return True
        def send_now(self,event,payload,timeout_s):
            events.append(json.loads(DiagnosticReporter._encode(event,payload)));return True
    wall=[0.]
    node=g.create_survey_node(Reporter(),wall_clock=lambda:wall[0])
    node.report_started(45.)
    callbacks["/referee/ground_tasks"]({"data":"ground_0,ground_1,ground_2"})
    for i in range(50):
        wall[0]=i*.05;stamp=10.+wall[0];sec=int(stamp)
        callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((stamp-sec)*1e9)}})
        callbacks["/odom"](posed_odom(stamp,yaw=-min(.05*i,.8)))
        callbacks["/scan"](scan(stamp))
        node.tick()
    callbacks["/camera/image"](SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12,nanosec=450_000_000),
        frame_id="camera_link"),width=800,height=800,step=2400,encoding="rgb8",data=array("B",bytes((150,70,20))*640000)))
    node.stop("test_complete")
    node.report_final()
    sections=[event["data"].get("section") for event in events if event["event"]=="probe_snapshot"]
    assert sections[0]=="ground_motion_core" and "survey_metadata" in sections
    assert "survey_motion_series" in sections and "survey_image_part" in sections
    assert all(event["data"].get("mode","ground_observation_survey")=="ground_observation_survey" for event in events)
    assert all(topic=="/cmd_vel" and linear==0. for topic,linear,_ in published)
    node.close_watchdog()


def test_uav_odom_cannot_be_replaced_by_ground_odom_for_uav_exposure():
    state=guard()
    state.update_odom(posed_odom(10.),wall_s=0.)
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=10_000_000),frame_id="uav/camera_link"))
    state.observe_image("/uav/camera/image",msg,wall_s=.01)
    exposure=state.survey_evidence()["exposure_records"][-1]
    assert exposure["odom"] is None and exposure["association_error"]=="no_prior_uav_odom"


def test_failed_native_image_encoding_is_returned_as_bounded_error_event():
    state=guard()
    bad=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0),frame_id="camera_link"),
        width=100,height=100,step=100,encoding="16UC1",data=b"bad")
    state.observe_image("/camera/image",bad,wall_s=0.)
    events=state.additional_events()
    errors=[payload for event,payload in events if payload.get("section")=="survey_image_errors"]
    assert errors and errors[0]["errors"]


def test_missing_expected_native_cameras_are_explicit_in_error_evidence():
    state=guard()
    events=state.additional_events()
    errors=[payload for event,payload in events if payload.get("section")=="survey_image_errors"]
    assert errors and set(errors[0]["errors"])=={
        "native_image_missing:/camera/image","native_image_missing:/uav/camera/image"}


def test_uav_final_native_image_precedes_ground_and_first_images_and_thumbnails(monkeypatch):
    import json
    from array import array
    from inspection_mission.telemetry import DiagnosticReporter
    pytest.importorskip("cv2");pytest.importorskip("numpy")
    g=api();callbacks,_,_=install_ros_transport(monkeypatch)
    events=[]
    class Reporter:
        def emit(self,event,payload):return True
        def send_now(self,event,payload,timeout_s):
            events.append(json.loads(DiagnosticReporter._encode(event,payload)));return True
    node=g.create_survey_node(Reporter(),wall_clock=lambda:0.)
    node.report_started(45.)
    for topic in ("/camera/image","/uav/camera/image"):
        for stamp in (10,11):
            callbacks[topic](SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0),
                frame_id="camera_link" if topic=="/camera/image" else "uav/camera_link"),
                width=800,height=800,step=2400,encoding="rgb8",data=array("B",bytes((80,90,100))*640000)))
    node.stop("test_end");node.report_final()
    native=[event for event in events if event["data"].get("section")=="survey_image_part"]
    assert native[0]["data"]["topic"]=="/uav/camera/image" and native[0]["data"]["phase"]=="last"
    native_index=next(i for i,event in enumerate(events) if event["data"].get("section")=="survey_image_part")
    thumb_index=next(i for i,event in enumerate(events) if event["event"]=="image_sample")
    assert native_index<thumb_index
    assert node.guard.final_report_wall_s==60.
    node.close_watchdog()


@pytest.mark.parametrize("child_exit,setup_exit",[(0,0),(23,0),(0,31)])
def test_actual_survey_shell_launch_preserves_source_location_arguments_and_exit(tmp_path,child_exit,setup_exit):
    import json,os,shlex,shutil,subprocess,sys
    from test_ground_probe_launch import BASH,ROOT,shell_path
    if not BASH.is_file():pytest.skip("bash unavailable")
    workspace=tmp_path/"uploaded survey workspace"
    package=workspace/"src/inspection_mission/inspection_mission"
    package.mkdir(parents=True)
    (package/"__init__.py").write_text("",encoding="utf-8")
    (package/"ground_observation_survey.py").write_text("import os,sys,json\nassert os.environ['FAKE_ROS_READY']=='1'\n"
        "print('SURVEY_ARGS '+json.dumps(sys.argv[1:]))\n"+f"sys.exit({child_exit})\n",encoding="utf-8")
    launcher=workspace/"scripts/run_ground_observation_survey.sh"
    launcher.parent.mkdir()
    shutil.copyfile(ROOT/"scripts/run_ground_observation_survey.sh",launcher)
    setup=tmp_path/"fake ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
        +(f"return {setup_exit}\n" if setup_exit else "export FAKE_ROS_READY=1\n"),encoding="utf-8")
    env=dict(os.environ,INSPECTION_ROS_SETUP=shell_path(setup));env.pop("INSPECTION_DIAGNOSTIC_URL",None)
    result=subprocess.run([str(BASH),str(launcher)],cwd=tmp_path,env=env,capture_output=True,text=True,encoding="utf-8",timeout=10)
    assert result.returncode==(setup_exit or child_exit),result.stdout+result.stderr
    assert f"GROUND_OBSERVATION_SURVEY launch_exit exit_code={setup_exit or child_exit}" in result.stderr
    if not setup_exit:
        line=next(line for line in result.stdout.splitlines() if line.startswith("SURVEY_ARGS "))
        assert json.loads(line.removeprefix("SURVEY_ARGS "))==["--duration","45"]


@pytest.mark.parametrize("topic",["odom","scan"])
def test_turn_invalid_sensor_latches_stop_before_following_valid_sample(topic):
    state=guard();ready(state)
    broken=(posed_odom if topic=="odom" else scan)(11.11)
    if topic=="odom":broken["pose"]["pose"]["position"]["x"]=float("nan")
    else:broken["ranges"][0]=0.
    getattr(state,"update_"+topic)(broken,wall_s=1.11)
    assert state.state=="STOP"
    original_reason=state.reason
    refresh(state,1.12,yaw=-.01)
    decision=state.step(wall_s=1.12)
    assert decision.angular_z==decision.linear_x==0. and decision.reason==original_reason


@pytest.mark.parametrize("body",["ground","uav"])
def test_invalid_source_odom_clears_exposure_history_then_later_valid_source_can_recover(body):
    state=guard()
    topic="/camera/image" if body=="ground" else "/uav/camera/image"
    update=state.update_odom if body=="ground" else state.update_uav_odom
    def payload(stamp):
        message=posed_odom(stamp)
        if body=="uav":message["child_frame_id"]="uav/base_link"
        return message
    update(payload(10.),wall_s=0.)
    bad=payload(10.01);bad["child_frame_id"]="wrong_source_body"
    update(bad,wall_s=.01)
    image=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=20_000_000),
        frame_id="camera_link" if body=="ground" else "uav/camera_link"))
    state.observe_image(topic,image,wall_s=.02)
    exposure=state.survey_evidence()["exposure_records"][-1]
    assert exposure["odom"] is None and exposure["association_error"].startswith(body+"_odom_invalid:")
    update(payload(10.03),wall_s=.03)
    image.header.stamp.nanosec=40_000_000
    state.observe_image(topic,image,wall_s=.04)
    exposure=state.survey_evidence()["exposure_records"][-1]
    assert exposure["odom"]["source_topic"]==("/odom" if body=="ground" else "/uav/odom")
    assert exposure["odom"]["stamp_s"]==10.03
    assert exposure["association_error"] is None


def test_survey_calibration_half_rad_command_still_stops_by_actual_yaw_not_sent_integral():
    g=api();state=guard()
    assert g.SurveyLimits().yaw_rate==.5
    assert ready(state).angular_z==-.5
    # Four seconds at a nominal half radian/second would integrate to >1.8rad,
    # while the actual measured turn remains small. Only real odom can end it.
    for i in range(1,83):
        wall=1.1+i*.05
        refresh(state,wall,yaw=-min(.3,i*.004),yaw_speed=.1)
    decision=state.step(wall_s=wall)
    assert decision.state=="TURN" and decision.angular_z==-.5
    refresh(state,5.3,yaw=state.target_yaw,yaw_speed=.1)
    decision=state.step(wall_s=5.3)
    assert decision.angular_z==0. and decision.reason=="target_yaw_reached"
    assert state.accumulated_rotation_rad<1.8
    assert state.limits.robot_radius_m==.447 and .5*state.limits.robot_radius_m<.5


def test_final_bad_association_keeps_real_last_associated_frame_and_native_priority():
    from array import array
    state=guard()
    prior=posed_odom(10.);prior["child_frame_id"]="uav/base_link"
    state.update_uav_odom(prior,wall_s=0.)
    def image(nanos):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=nanos),frame_id="uav/camera_link"),
            width=8,height=8,step=24,encoding="bgr8",data=array("B",bytes((100,90,80))*64))
    state.observe_image("/uav/camera/image",image(20_000_000),wall_s=.02)
    state.observe_image("/uav/camera/image",image(120_000_000),wall_s=.12)
    records=state.survey_evidence()["exposure_records"]
    last=next(record for record in records if record["phase"]=="last")
    associated=next(record for record in records if record["phase"]=="last_associated")
    assert last["stamp_s"]==10.12 and last["odom"] is None
    assert associated["stamp_s"]==10.02 and associated["odom"]["stamp_s"]==10.
    assert associated["frame_id"]=="uav/camera_link"
    images=[payload for _,payload in state.additional_events() if payload.get("section")=="survey_image_part"]
    assert images[0]["phase"]=="last_associated" and images[0]["stamp_s"]==10.02
    assert any(item["phase"]=="last" and item["stamp_s"]==10.12 for item in images)


def test_snapshot_keeps_survey_metadata_counts_without_repeating_large_motion_series(monkeypatch):
    import json
    from inspection_mission.telemetry import DiagnosticReporter
    g=api();install_ros_transport(monkeypatch)
    events=[]
    class Reporter:
        def emit(self,event,payload):return True
        def send_now(self,event,payload,timeout_s):
            events.append(json.loads(DiagnosticReporter._encode(event,payload)));return True
    node=g.create_survey_node(Reporter(),wall_clock=lambda:0.)
    state=node.guard;ready(state)
    for i in range(900):
        wall=1.2+i*.05
        state.update_odom(posed_odom(10.+wall,yaw=-.3),wall_s=wall)
        state.record_command(wall_s=wall,linear_x=0.,angular_z=-.5)
    snapshot=node.evidence_snapshot()
    survey=snapshot["survey"]
    assert "trajectory_10hz" not in survey and "actual_commands" not in survey
    assert survey["trajectory_count"]>400 and survey["actual_command_count"]==900
    assert len(DiagnosticReporter._encode("probe_snapshot",snapshot))<65536
    node.report_final()
    full=[item["data"] for item in events if item["data"].get("section")=="ground_motion_full"]
    assert len(full)==1
    assert not any(item["data"].get("section")=="evidence_errors" for item in events)
    assert sum(len(item["data"]["rows"]) for item in events
        if item["data"].get("series")=="actual_commands")==900
    node.close_watchdog()


def test_observed_odom_period_summary_records_fifty_to_five_hz_without_changing_age_gate():
    state=guard()
    for i in range(51):state.update_odom(posed_odom(10.+i*.02),wall_s=i*.02)
    for i in range(1,7):state.update_odom(posed_odom(11.+i*.2),wall_s=1.+i*.2)
    summary=state.survey_evidence()["observed_odom_periods"]["/odom"]
    assert summary["interval_count"]==56
    assert summary["header_delta_s"]["median"]==pytest.approx(.02)
    assert summary["header_delta_s"]["last"]==pytest.approx(.2)
    assert summary["arrival_delta_s"]["last"]==pytest.approx(.2)
    assert summary["header_delta_s"]["recent_intervals"][-6:]==pytest.approx([.2]*6)
    assert summary["interpretation"]=="observed_intervals_only; source cause unverified"
    assert state.limits.sensor_age_s==.2


def pause_and_restore(state,*,paused_at=1.31,resume_yaw=-.1):
    original=(state.target_id,state.target_yaw,state.pulse_started,state.pulse_origin)
    assert state.step(wall_s=paused_at).state=="PAUSE"
    decisions=[]
    for i in range(23):
        wall=paused_at+.01+i*.05
        refresh(state,wall,yaw=resume_yaw)
        decisions.append(state.step(wall_s=wall))
    assert original==(state.target_id,state.target_yaw,state.pulse_started,state.pulse_origin)
    return decisions


def test_temporary_stale_pauses_then_resumes_same_target_only_after_full_fresh_stable_dwell():
    state=guard();ready(state)
    decisions=pause_and_restore(state)
    assert all(item.angular_z==0. and item.state=="PAUSE" for item in decisions[:20])
    assert any(item.state=="TURN" and item.angular_z<0. for item in decisions[20:])
    assert state.resume_count==1 and state.accumulated_rotation_rad>=.1-1e-9
    assert state.limits.sensor_age_s==.2


@pytest.mark.parametrize("hard_fault",["draw_changed","physical_limit","invalid_scan","clock_reversed"])
def test_hard_fault_during_pause_never_resumes(hard_fault):
    state=guard();ready(state)
    state.step(wall_s=1.31)
    if hard_fault=="draw_changed":state.observe_public("/referee/ground_tasks",{"data":"ground_0,ground_1,ground_2"},wall_s=1.32)
    elif hard_fault=="physical_limit":state.update_odom(posed_odom(11.32,x=.061),wall_s=1.32)
    elif hard_fault=="invalid_scan":state.update_scan(scan(11.32,[0.]*360),wall_s=1.32)
    else:state.update_clock(9.,wall_s=1.32)
    assert state.state=="STOP"
    for i in range(24):
        wall=1.33+i*.05;refresh(state,wall,yaw=-.1)
        assert state.step(wall_s=wall).angular_z==0.
    assert state.resume_count==0


def test_pause_time_counts_toward_original_turn_budget_without_reset():
    state=guard();ready(state)
    started=state.pulse_started
    state.step(wall_s=1.31)
    for i in range(10):
        wall=started+24.5+i*.05
        refresh(state,wall,yaw=-.1)
        decision=state.step(wall_s=wall)
        assert decision.angular_z==0.
    assert state.pulse_started==started
    refresh(state,started+25.,yaw=-.1)
    decision=state.step(wall_s=started+25.)
    assert decision.state=="STOP" and decision.reason=="turn_wall_time_limit"


def test_fourth_temporary_pause_cannot_resume_after_three_recoveries():
    state=guard();ready(state)
    for recovery in range(3):
        pause_at=state.last_wall+.21
        pause_and_restore(state,paused_at=pause_at,resume_yaw=-.1)
        assert state.resume_count==recovery+1 and state.state=="TURN"
    fourth=state.last_wall+.21
    state.step(wall_s=fourth)
    for i in range(23):
        wall=fourth+.01+i*.05;refresh(state,wall,yaw=-.1)
        decision=state.step(wall_s=wall)
    assert decision.state=="STOP" and decision.reason=="pause_resume_limit"


def test_stale_callback_between_ticks_restarts_pause_dwell_even_if_replaced_before_next_step():
    state=guard();ready(state);state.step(wall_s=1.31)
    for i in range(10):
        wall=1.32+i*.05;refresh(state,wall,yaw=-.1);state.step(wall_s=wall)
    assert state.stable_anchor is not None
    # Same public callback order with a missed .2-second laser interval must
    # erase the partial dwell before all data are refreshed at the next tick.
    refresh(state,2.01,yaw=-.1)
    state.step(wall_s=2.01)
    for i in range(1,20):
        wall=2.01+i*.05;refresh(state,wall,yaw=-.1)
        assert state.step(wall_s=wall).angular_z==0.
    refresh(state,3.01,yaw=-.1)
    assert state.step(wall_s=3.01).state=="TURN"


def test_adapter_stale_sensor_gap_outputs_only_zero_then_resumes_after_fresh_dwell(monkeypatch):
    g=api();callbacks,published,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_survey_node(wall_clock=lambda:wall[0])
    callbacks["/referee/ground_tasks"]({"data":"ground_1,ground_4,ground_0"})
    states=[]
    for i in range(71):
        wall[0]=i*.05;stamp=10.+wall[0];sec=int(stamp)
        callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((stamp-sec)*1e9)}})
        callbacks["/odom"](posed_odom(stamp,yaw=-min(.1,max(0.,wall[0]-1.)*.2)))
        if not 1.3<=wall[0]<1.7:callbacks["/scan"](scan(stamp))
        decision=node.tick()
        states.append((wall[0],decision.state,decision.angular_z))
    paused=[row for row in states if row[1]=="PAUSE"]
    assert paused and all(rate==0. for _,_,rate in paused)
    assert all(rate==0. for wall,_,rate in states if 1.5<=wall<2.7)
    assert any(state=="TURN" and rate<0. for wall,state,rate in states if wall>2.7)
    assert node.guard.resume_count==1 and node.guard.pulse_started<1.2
    assert all(topic=="/cmd_vel" and linear==0. for topic,linear,_ in published)
    node.stop("test_end");node.close_watchdog()


def test_pause_still_accounts_actual_back_and_forth_motion_in_total_rotation_budget():
    state=guard();ready(state);state.step(wall_s=1.31)
    for wall,yaw in ((1.32,-.5),(1.37,.3),(1.42,-.3)):
        refresh(state,wall,yaw=yaw)
    assert state.state=="STOP" and state.reason=="rotation_budget"
    assert state.accumulated_rotation_rad>=1.8


def test_pause_moving_body_never_establishes_resume_stability():
    state=guard();ready(state);state.step(wall_s=1.31)
    for i in range(32):
        wall=1.32+i*.05;refresh(state,wall,yaw=-.1,yaw_speed=.04)
        decision=state.step(wall_s=wall)
        assert decision.angular_z==0. and decision.state=="PAUSE"
    assert state.resume_count==0


def test_watchdog_pause_transition_cannot_overwrite_a_concurrent_hard_stop():
    import inspect
    import sys
    import threading
    state=guard();ready(state)
    entered,release,hard_stop_started,hard_stop_done=(threading.Event() for _ in range(4))
    method=state.temporal_fault
    source,first_line=inspect.getsourcelines(method)
    pause_line=next(first_line+i for i,line in enumerate(source) if 'self.state,self.reason="PAUSE",reason' in line)
    def trace(frame,event,arg):
        if event=="line" and frame.f_code is method.__func__.__code__ and frame.f_lineno==pause_line:
            entered.set();release.wait(.5)
        return trace
    def watchdog_transition():
        sys.settrace(trace)
        try:state.handle_watchdog("scan_stale_or_unsafe",1.39)
        finally:sys.settrace(None)
    def hard_stop():
        hard_stop_started.set()
        state.stop("unexpected_motion",1.39)
        hard_stop_done.set()
    watchdog=threading.Thread(target=watchdog_transition,daemon=True)
    stopper=threading.Thread(target=hard_stop,daemon=True)
    watchdog.start()
    try:
        assert entered.wait(.3)
        stopper.start();assert hard_stop_started.wait(.2)
        # On the old unlocked method this hard STOP finishes while the pause
        # write is suspended, and the later write resurrects the action.
        hard_stop_done.wait(.05)
    finally:
        release.set();watchdog.join(.5);stopper.join(.5)
    assert not watchdog.is_alive() and not stopper.is_alive()
    assert state.state=="STOP" and state.reason=="unexpected_motion"
    for i in range(23):
        wall=1.4+i*.05;refresh(state,wall,yaw=-.1)
        assert state.step(wall_s=wall).angular_z==0.


@pytest.mark.parametrize("fault",["speed","yaw_speed","displacement","tilt","rotation_budget","target_crossed"])
def test_turn_callback_latches_transient_measured_violation_before_normal_replacement(fault):
    state=guard();ready(state)
    yaw=-1.81 if fault=="rotation_budget" else (-1.65 if fault=="target_crossed" else -.01)
    message=posed_odom(11.11,yaw=yaw,x=.061 if fault=="displacement" else 0.,yaw_speed=.551 if fault=="yaw_speed" else .1)
    if fault=="speed":message["twist"]["twist"]["linear"]["x"]=.101
    if fault=="tilt":message["pose"]["pose"]["orientation"]={"x":math.sin(.031/2),"y":0.,"z":0.,"w":math.cos(.031/2)}
    state.update_odom(message,wall_s=1.11)
    assert state.state=="STOP"
    reason=state.reason
    refresh(state,1.12,yaw=-.01)
    assert state.step(wall_s=1.12).angular_z==0. and state.reason==reason
