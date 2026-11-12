"""Finite turn/approach observations keep local and global budgets distinct."""
import importlib
import importlib.util
import math
from array import array
from types import SimpleNamespace

import pytest

from test_bounded_ground_probe import endpoint,install_ros_transport,scan
from test_ground_observation_survey import posed_odom


def api():
    name="inspection_mission.ground_bounded_approach"
    assert importlib.util.find_spec(name) is not None,"independent bounded approach is missing"
    return importlib.import_module(name)


def guard():return api().GroundBoundedApproachGuard(started_wall_s=0.)


def refresh(state,wall,*,x=0.,y=0.,yaw=0.,speed=0.,yaw_speed=0.,laser=True):
    stamp=round(10.+wall,9)
    state.update_clock(stamp,wall_s=wall)
    payload=posed_odom(stamp,x=x,yaw=yaw,yaw_speed=yaw_speed)
    payload["pose"]["pose"]["position"]["y"]=y
    payload["twist"]["twist"]["linear"]["x"]=speed
    state.update_odom(payload,wall_s=wall)
    if laser:state.update_scan(scan(stamp),wall_s=wall)


def ready(state,first="ground_2"):
    state.update_endpoints([endpoint()],matched_subscriptions=1)
    other=[value for value in ("ground_0","ground_1","ground_2","ground_3","ground_4") if value!=first][:2]
    state.observe_public("/referee/ground_tasks",{"data":",".join([first,*other])},wall_s=0.)
    for i in range(23):
        wall=i*.05;refresh(state,wall)
        decision=state.step(wall_s=wall)
    return decision


def drive_ready(state,first="ground_2"):
    ready(state,first)
    target=state.target_yaw
    for i in range(25):
        wall=1.15+i*.05;refresh(state,wall,yaw=target)
        decision=state.step(wall_s=wall)
    assert decision.state=="DRIVE"
    return wall,target


def image(stamp,frame="uav/camera_link"):
    stamp=round(stamp,9)
    sec=int(stamp)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=round((stamp-sec)*1e9)),frame_id=frame),
        width=8,height=8,step=24,encoding="bgr8",data=array("B",bytes((100,90,80))*64))


def capture_ready(state):
    wall,yaw=drive_ready(state)
    for i in range(1,42):
        wall+=.05;d=min(.201,i*.005)
        refresh(state,wall,x=d*math.cos(yaw),y=d*math.sin(yaw),yaw=yaw)
        state.step(wall_s=wall)
    for _ in range(23):
        wall+=.05;refresh(state,wall,x=.201*math.cos(yaw),y=.201*math.sin(yaw),yaw=yaw)
        state.step(wall_s=wall)
    assert state.state=="CAPTURE"
    return wall,yaw


@pytest.mark.parametrize("first",["ground_0","ground_1","ground_2","ground_3","ground_4"])
def test_each_live_first_id_sets_only_its_candidate_turn(first):
    state=guard();decision=ready(state,first)
    assert state.target_id==first and decision.state=="TURN"
    assert decision.linear_x==0. and -.5<=decision.angular_z<=-.35
    assert state.approach_evidence()["task_completion"]=="unverified"


def test_turn_zeroes_on_real_target_then_waits_stationary_before_drive():
    state=guard();ready(state)
    refresh(state,1.15,yaw=state.target_yaw)
    assert state.state=="TURN_SETTLE" and state.step(wall_s=1.15).angular_z==0.
    for i in range(1,20):
        wall=1.15+i*.05;refresh(state,wall,yaw=state.target_yaw)
        assert state.step(wall_s=wall).linear_x==0.
    refresh(state,2.2,yaw=state.target_yaw)
    decision=state.step(wall_s=2.2)
    assert decision.state=="DRIVE" and decision.linear_x==.03 and decision.angular_z==0.


def test_turn_crossing_more_than_heading_contract_stops_instead_of_freezing_wrong_drive_heading():
    state=guard();ready(state)
    refresh(state,1.15,yaw=state.target_yaw-.051)
    assert state.state=="STOP" and state.reason=="approach_turn_overshoot_limit"


def test_twenty_cm_feedback_completion_keeps_local_gate_point_six_four_one():
    state=guard();wall,yaw=drive_ready(state)
    for i in range(1,42):
        wall+=.05;distance=min(.201,i*.005)
        refresh(state,wall,x=distance*math.cos(yaw),y=distance*math.sin(yaw),yaw=yaw)
        decision=state.step(wall_s=wall)
    assert state.state=="DRIVE_SETTLE" and decision.linear_x==0.
    assert state.measured_forward()>=.20
    assert state.scan["required_range_m"]==pytest.approx(.641)
    evidence=state.approach_evidence()
    assert evidence["total_path_m"]>.20 and evidence["max_origin_displacement_m"]<.30
    assert evidence["limits"]["local_swept_motion_budget_m"]==.06


@pytest.mark.parametrize("fault",["reverse","cross_track","heading","tilt","bad_scan","bad_odom"])
def test_drive_callback_hard_fault_survives_next_good_sample(fault):
    state=guard();wall,yaw=drive_ready(state)
    if fault=="bad_scan":state.update_scan(scan(10.+wall+.01,[0.]*360),wall_s=wall+.01)
    elif fault=="bad_odom":
        payload=posed_odom(10.+wall+.01);payload["child_frame_id"]="uav/base_link"
        state.update_odom(payload,wall_s=wall+.01)
    else:
        forward=-.0051 if fault=="reverse" else 0.
        cross=.0201 if fault=="cross_track" else 0.
        payload=posed_odom(10.+wall+.01,x=forward*math.cos(yaw)-cross*math.sin(yaw),yaw=yaw+(.051 if fault=="heading" else 0.))
        payload["pose"]["pose"]["position"]["y"]=forward*math.sin(yaw)+cross*math.cos(yaw)
        if fault=="tilt":payload["pose"]["pose"]["orientation"]={"x":math.sin(.031/2),"y":0.,"z":0.,"w":math.cos(.031/2)}
        state.update_odom(payload,wall_s=wall+.01)
    assert state.state=="STOP"
    reason=state.reason;refresh(state,wall+.02,yaw=yaw)
    assert state.step(wall_s=wall+.02).linear_x==0. and state.reason==reason


def test_local_scan_anchor_is_independent_and_exhaustion_cannot_be_erased_by_fresh_scan():
    state=guard();wall,yaw=drive_ready(state)
    refresh(state,wall+.05,x=.031*math.cos(yaw),y=.031*math.sin(yaw),yaw=yaw,laser=False)
    assert state.state=="STOP" and state.reason=="local_scan_motion_reserve_exhausted"
    state.update_scan(scan(10.+wall+.05),wall_s=wall+.05)
    assert state.step(wall_s=wall+.05).linear_x==0.


def test_fresh_scan_reanchoring_cannot_reset_total_path_or_origin():
    state=guard();ready(state)
    origin=state.approach_evidence()["global_origin"]
    for i in range(1,61):
        wall=1.1+i*.05
        # Small valid increments repeatedly move and return during TURN.
        x=.005 if i%2 else 0.
        refresh(state,wall,x=x)
        state.step(wall_s=wall)
        if state.state=="STOP":break
    assert state.state=="STOP" and state.reason=="approach_path_limit"
    assert state.approach_evidence()["global_origin"]==origin
    assert state.approach_evidence()["total_path_m"]>=.30-1e-9


def test_pause_keeps_stage_goal_all_origins_and_global_ninety_second_budget():
    state=guard();wall,yaw=drive_ready(state)
    before=state.approach_evidence()
    assert state.step(wall_s=wall+.21).state=="PAUSE"
    for i in range(23):
        t=wall+.22+i*.05;refresh(state,t,yaw=yaw)
        decision=state.step(wall_s=t)
    assert decision.state=="DRIVE" and state.resume_count==1
    after=state.approach_evidence()
    for key in ("global_origin","drive_origin","drive_started_wall_s","first_motion_wall_s"):
        assert after[key]==before[key]
    assert state.watchdog_reason(90.)=="approach_global_wall_time_limit"


def test_capture_requires_three_distinct_fresh_uav_source_frames_and_preserves_each_pose():
    state=guard();wall,yaw=capture_ready(state)
    for index in range(3):
        wall+=.1;refresh(state,wall,x=.201*math.cos(yaw),y=.201*math.sin(yaw),yaw=yaw)
        payload=posed_odom(round(10.+wall,9),x=.201*math.cos(yaw),yaw=yaw)
        payload["pose"]["pose"]["position"]["y"]=.201*math.sin(yaw);payload["child_frame_id"]="uav/base_link"
        state.observe_public("/uav/odom",payload,wall_s=wall)
        msg=image(10.+wall+.01)
        state.observe_image("/uav/camera/image",msg,wall_s=wall+.01)
        state.observe_image("/uav/camera/image",msg,wall_s=wall+.01)
        assert state.approach_evidence()["capture_count"]==index+1
    assert state.state=="STOP" and state.reason=="approach_observation_complete"
    evidence=state.survey_evidence()
    captures=[record for record in evidence["exposure_records"] if record["phase"].startswith("capture_")]
    assert len(captures)==3
    assert len({record["stamp_s"] for record in captures})==3
    assert all(record["odom"]["source_topic"]=="/uav/odom" for record in captures)
    assert {record["stage"] for record in evidence["scan_records"]}=={"start","after_turn","mid_drive","after_drive"}
    mid=next(record for record in evidence["scan_records"] if record["stage"]=="mid_drive")
    assert -mid["odom"]["position"][1]>=.10


@pytest.mark.parametrize("deadline",["global","capture"])
def test_capture_image_callback_cannot_hide_an_expired_wall_budget(deadline):
    state=guard();wall,yaw=capture_ready(state)
    wall=90. if deadline=="global" else state.capture_started_wall+5.
    state.observe_image("/uav/camera/image",image(10.+wall),wall_s=wall)
    assert state.state=="STOP"
    assert state.reason==("approach_global_wall_time_limit" if deadline=="global" else "approach_capture_wall_time_limit")


def test_capture_drift_clears_dwell_instead_of_accepting_an_immediately_fresh_frame():
    state=guard();wall,yaw=capture_ready(state)
    wall+=.1
    x=.201*math.cos(yaw)+.007;y=.201*math.sin(yaw)
    refresh(state,wall,x=x,y=y,yaw=yaw)
    payload=posed_odom(round(10.+wall,9),x=x,yaw=yaw)
    payload["pose"]["pose"]["position"]["y"]=y;payload["child_frame_id"]="uav/base_link"
    state.observe_public("/uav/odom",payload,wall_s=wall)
    state.observe_image("/uav/camera/image",image(10.+wall+.01),wall_s=wall+.01)
    assert state.approach_evidence()["capture_count"]==0


@pytest.mark.parametrize("invalid",["empty_pixels","bad_stride","bad_stamp"])
def test_invalid_current_source_image_cannot_advance_capture_or_borrow_old_record(invalid):
    state=guard();wall,yaw=capture_ready(state)
    wall+=.1;refresh(state,wall,x=.201*math.cos(yaw),y=.201*math.sin(yaw),yaw=yaw)
    payload=posed_odom(round(10.+wall,9),yaw=yaw);payload["child_frame_id"]="uav/base_link"
    state.observe_public("/uav/odom",payload,wall_s=wall)
    # A real prior observation can remain when the current input is rejected.
    state._observations.observe_image("/uav/camera/image",image(10.+wall+.005),wall_s=wall+.005)
    msg=image(10.+wall+.01)
    if invalid=="empty_pixels":msg.data=b""
    elif invalid=="bad_stride":msg.step=1
    else:msg.header.stamp.nanosec=1_000_000_000
    state.observe_image("/uav/camera/image",msg,wall_s=wall+.01)
    assert state.approach_evidence()["capture_count"]==0 and state.state=="CAPTURE"


def test_missing_image_dependency_still_delivers_core_metadata_and_control_series(monkeypatch):
    import sys
    g=api();install_ros_transport(monkeypatch)
    sent=[]
    class Reporter:
        def emit(self,*args):return True
        def send_now(self,event,payload,timeout_s):sent.append((event,payload));return True
    node=g.create_approach_node(Reporter(),wall_clock=lambda:0.)
    wall,yaw=capture_ready(node.guard)
    for _ in range(3):
        wall+=.1;refresh(node.guard,wall,x=.201*math.cos(yaw),y=.201*math.sin(yaw),yaw=yaw)
        payload=posed_odom(round(10.+wall,9),yaw=yaw);payload["child_frame_id"]="uav/base_link"
        node.guard.observe_public("/uav/odom",payload,wall_s=wall)
        node.guard.observe_image("/uav/camera/image",image(10.+wall+.01),wall_s=wall+.01)
    node.guard.record_command(wall_s=1.1,linear_x=0.,angular_z=-.5)
    monkeypatch.setitem(sys.modules,"cv2",None)
    node.report_final()
    assert sent[0][1]["section"]=="ground_motion_core"
    assert any(payload.get("section")=="survey_metadata" for _,payload in sent)
    assert any(payload.get("series")=="actual_commands" for _,payload in sent)
    assert any(payload.get("section")=="survey_image_errors" for _,payload in sent)
    node.close_watchdog()


def test_new_capacities_keep_ninety_second_trajectories_and_commands_without_silent_drop():
    state=guard()
    for i in range(901):
        wall=i*.1;state.update_odom(posed_odom(10.+wall),wall_s=wall)
        for j in range(4):state.record_command(wall_s=wall+j*.02,linear_x=0.,angular_z=0.)
    evidence=state.survey_evidence()
    assert len(evidence["trajectory_10hz"])==901 and evidence["trajectory_dropped"]==0
    assert len(evidence["actual_commands"])==3604 and evidence["actual_commands_dropped"]==0
    assert evidence["mode"]=="ground_bounded_approach"


def test_approach_limits_do_not_inflate_local_gate_or_allow_unbounded_commands():
    g=api()
    for changes in ({"target_distance_m":.201},{"global_wall_s":90.1},{"max_path_m":.301},
                    {"max_origin_displacement_m":.301},{"local_swept_motion_budget_m":.061},
                    {"linear_command_mps":.031},{"maximum_yaw_rate":.501},{"minimum_yaw_rate":.351}):
        with pytest.raises(ValueError):g.ApproachLimits(**changes)


def test_expired_phase_can_finish_zero_dwell_without_waiting_to_global_deadline():
    state=guard();ready(state)
    wall=state.turn_started_wall+45.
    decision=state.step(wall_s=wall)
    assert decision.state=="STOP" and decision.reason=="approach_turn_wall_time_limit"
    assert state.step(wall_s=wall+2.).state=="FINISHED"


def test_watchdog_keeps_global_deadline_after_turn_to_drive_switch(monkeypatch):
    import time
    g=api();_,published,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_approach_node(wall_clock=lambda:wall[0]);drive_ready(node.guard)
    wall[0]=46.
    assert node.guard._budget_fault(wall[0])==""
    node.arm_watchdog()
    wall[0]=90.
    deadline=time.monotonic()+.4
    while node.guard.state!="STOP" and time.monotonic()<deadline:time.sleep(.01)
    node.close_watchdog()
    assert node.guard.reason=="approach_global_wall_time_limit"
    assert published and all(row==("/cmd_vel",0.,0.) for row in published)


def test_all_three_captured_source_images_and_control_events_fit_diagnostic_transport():
    from inspection_mission.telemetry import DiagnosticReporter
    state=guard();wall,yaw=capture_ready(state)
    for index in range(3):
        wall+=.1;refresh(state,wall,x=.201*math.cos(yaw),y=.201*math.sin(yaw),yaw=yaw)
        payload=posed_odom(round(10.+wall,9),yaw=yaw);payload["child_frame_id"]="uav/base_link"
        state.observe_public("/uav/odom",payload,wall_s=wall)
        state.observe_image("/uav/camera/image",image(10.+wall+.01),wall_s=wall+.01)
    events=state.additional_events()
    assert events and all(len(DiagnosticReporter._encode(event,payload))<65536 for event,payload in events)
    assert all(payload["mode"]=="ground_bounded_approach" for _,payload in events)
    frames=[payload for _,payload in events if payload.get("section")=="survey_image_part"]
    assert {payload["phase"] for payload in frames if payload["phase"].startswith("capture_")}=={"capture_0","capture_1","capture_2"}
    metadata=events[0][1]["survey"]
    assert metadata["approach"]["capture_count"]==3 and len(metadata["scan_records"])==4
    assert set(metadata["module_availability"])=={"cv2","numpy","torch","onnxruntime"}


def test_fourth_fresh_resume_stops_and_keeps_drive_origin_and_deadline():
    state=guard();wall,yaw=drive_ready(state)
    origin=state.drive_origin;started=state.drive_started_wall
    for resume in range(1,5):
        wall+=.21
        assert state.step(wall_s=wall).state=="PAUSE"
        for i in range(23):
            t=wall+.01+i*.05;refresh(state,t,yaw=yaw)
            decision=state.step(wall_s=t)
        wall=t
        assert state.drive_origin==origin and state.drive_started_wall==started
        if resume<=3:assert decision.state=="DRIVE" and state.resume_count==resume
        else:assert decision.state=="STOP" and decision.reason=="pause_resume_limit"


@pytest.mark.parametrize("child_exit,setup_exit",[(0,0),(23,0),(0,31)])
def test_real_approach_script_starts_uploaded_module_and_preserves_exit(tmp_path,child_exit,setup_exit):
    import json
    import os
    import shlex
    import shutil
    import subprocess
    import sys
    from pathlib import Path
    from test_ground_probe_launch import BASH,shell_path
    if not BASH.is_file():pytest.skip("bash unavailable")
    workspace=tmp_path/"uploaded approach workspace"
    package=workspace/"src/inspection_mission/inspection_mission";package.mkdir(parents=True)
    (package/"__init__.py").write_text("",encoding="utf-8")
    (package/"ground_bounded_approach.py").write_text(
        "import json,os,sys\nassert os.environ['FAKE_ROS_READY']=='1'\n"
        "print('APPROACH_ARGS '+json.dumps(sys.argv[1:]))\n"
        f"sys.exit({child_exit})\n",encoding="utf-8")
    launcher=workspace/"scripts/run_ground_bounded_approach.sh";launcher.parent.mkdir()
    source=Path(__file__).resolve().parents[1]/"scripts/run_ground_bounded_approach.sh"
    assert source.exists(),"independent approach launcher is missing"
    shutil.copyfile(source,launcher)
    setup=tmp_path/"fake ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
        +(f"return {setup_exit}\n" if setup_exit else "export FAKE_ROS_READY=1\n"),encoding="utf-8")
    env=dict(os.environ,INSPECTION_ROS_SETUP=shell_path(setup));env.pop("INSPECTION_DIAGNOSTIC_URL",None)
    result=subprocess.run([str(BASH),str(launcher)],cwd=tmp_path,env=env,
        capture_output=True,text=True,encoding="utf-8",timeout=10)
    assert result.returncode==(setup_exit or child_exit),result.stdout+result.stderr
    assert f"GROUND_BOUNDED_APPROACH launch_exit exit_code={setup_exit or child_exit}" in result.stderr
    if not setup_exit:
        line=next(line for line in result.stdout.splitlines() if line.startswith("APPROACH_ARGS "))
        assert json.loads(line.removeprefix("APPROACH_ARGS "))==["--duration","90"]
