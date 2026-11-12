"""A separate two-centimetre observation drive closes on measured odometry."""
import importlib
import importlib.util
import math
from types import SimpleNamespace

import pytest

from test_bounded_ground_probe import endpoint,install_ros_transport,scan
from test_ground_observation_survey import posed_odom


def api():
    name="inspection_mission.ground_short_drive"
    assert importlib.util.find_spec(name) is not None,"separate short-drive entry is missing"
    return importlib.import_module(name)


def guard():
    from inspection_mission.bounded_ground_probe import MotionLimits
    return api().GroundShortDriveGuard(MotionLimits(.447,.114,.02),started_wall_s=0.)


def refresh(state,wall,*,x=0.,y=0.,yaw=0.,speed=0.,yaw_speed=0.,laser=True):
    stamp=10.+wall
    state.update_clock(stamp,wall_s=wall)
    payload=posed_odom(stamp,x=x,yaw=yaw,yaw_speed=yaw_speed)
    payload["pose"]["pose"]["position"]["y"]=y
    payload["twist"]["twist"]["linear"]["x"]=speed
    state.update_odom(payload,wall_s=wall)
    if laser:state.update_scan(scan(stamp),wall_s=wall)


def ready(state,*,yaw=0.):
    state.update_endpoints([endpoint()],matched_subscriptions=1)
    for i in range(23):
        wall=i*.05;refresh(state,wall,yaw=yaw)
        decision=state.step(wall_s=wall)
    return decision


def test_short_drive_starts_once_with_only_small_forward_velocity():
    state=guard();decision=ready(state)
    assert decision.state=="DRIVE" and decision.linear_x==.03 and decision.angular_z==0.
    assert state.drive_evidence()["target_distance_m"]==.02
    assert state.drive_evidence()["task_completion"]=="unverified"


def test_completion_depends_on_measured_forward_distance_not_command_integral():
    state=guard();ready(state)
    for i in range(1,61):
        wall=1.1+i*.05;refresh(state,wall,x=.001)
        decision=state.step(wall_s=wall)
    assert decision.state=="DRIVE" and decision.linear_x==.03
    refresh(state,4.15,x=.0201)
    assert state.state=="STOP" and state.reason=="short_drive_distance_reached"
    assert state.step(wall_s=4.15).linear_x==0.


def test_initial_heading_defines_forward_axis_and_stays_fixed():
    state=guard();ready(state,yaw=math.pi/2)
    refresh(state,1.2,x=0.,y=.0201,yaw=math.pi/2)
    assert state.state=="STOP" and state.reason=="short_drive_distance_reached"
    evidence=state.drive_evidence()
    assert evidence["measured_forward_m"]==pytest.approx(.0201)
    assert evidence["initial_yaw_rad"]==pytest.approx(math.pi/2)


@pytest.mark.parametrize("paused",[False,True])
@pytest.mark.parametrize("fault",["displacement","yaw_error","linear_speed","angular_speed","tilt","bad_frame","bad_scan"])
def test_transient_hard_fault_in_callback_stays_stopped_after_valid_replacement(fault,paused):
    state=guard();ready(state)
    if paused:state.temporal_fault("scan_stale",1.105)
    message=posed_odom(11.11,x=.061 if fault=="displacement" else 0.,
        yaw=.051 if fault=="yaw_error" else 0.,yaw_speed=.101 if fault=="angular_speed" else 0.)
    if fault=="linear_speed":message["twist"]["twist"]["linear"]["x"]=.101
    if fault=="tilt":message["pose"]["pose"]["orientation"]={"x":math.sin(.031/2),"y":0.,"z":0.,"w":math.cos(.031/2)}
    if fault=="bad_frame":message["child_frame_id"]="uav/base_link"
    if fault=="bad_scan":state.update_scan(scan(11.11,[0.]*360),wall_s=1.11)
    else:state.update_odom(message,wall_s=1.11)
    assert state.state=="STOP"
    reason=state.reason
    refresh(state,1.12)
    decision=state.step(wall_s=1.12)
    assert decision.linear_x==decision.angular_z==0. and state.reason==reason


def test_sideways_motion_cannot_count_as_forward_completion():
    state=guard();ready(state)
    refresh(state,1.2,x=0.,y=.0201)
    decision=state.step(wall_s=1.2)
    assert decision.state=="DRIVE" and state.drive_evidence()["measured_forward_m"]==0.


def test_reverse_motion_over_five_millimetres_latches_stop():
    state=guard();ready(state)
    refresh(state,1.2,x=-.0051)
    assert state.state=="STOP" and state.reason=="short_drive_reverse_limit"
    refresh(state,1.21,x=.01)
    assert state.step(wall_s=1.21).linear_x==0.


def test_reverse_drift_within_five_millimetres_does_not_count_as_progress():
    state=guard();ready(state)
    refresh(state,1.2,x=-.0049)
    assert state.step(wall_s=1.2).state=="DRIVE"
    assert state.drive_evidence()["measured_forward_m"]==pytest.approx(-.0049)


def test_stale_pause_requires_fresh_stable_dwell_and_keeps_original_origin_and_deadline():
    state=guard();ready(state)
    before=state.drive_evidence()
    decision=state.step(wall_s=1.31)
    assert decision.state=="PAUSE" and decision.linear_x==0.
    for i in range(23):
        wall=1.32+i*.05;refresh(state,wall,x=.004)
        decision=state.step(wall_s=wall)
        if i<20:assert decision.linear_x==0.
    assert decision.state=="DRIVE" and decision.linear_x==.03
    after=state.drive_evidence()
    assert after["origin_position"]==before["origin_position"]
    assert after["first_motion_wall_s"]==before["first_motion_wall_s"]
    assert after["resume_count"]==1


def test_paused_time_is_included_in_ten_second_total_motion_budget():
    state=guard();ready(state)
    started=state.drive_evidence()["first_motion_wall_s"]
    state.step(wall_s=1.31)
    refresh(state,started+10.,x=.005)
    decision=state.step(wall_s=started+10.)
    assert decision.linear_x==0. and decision.state=="STOP"
    assert decision.reason=="short_drive_wall_time_limit"


def test_budget_limits_are_separate_and_cannot_expand_existing_motion_limits():
    from inspection_mission.bounded_ground_probe import MotionLimits
    before=MotionLimits(.447,.114,.02)
    g=api();limits=g.ShortDriveLimits()
    assert limits.target_distance_m==.02 and limits.max_motion_wall_s==10. and limits.max_heading_error_rad==.05
    for changes in ({"target_distance_m":.0201},{"max_motion_wall_s":10.1},{"max_heading_error_rad":.051}):
        with pytest.raises(ValueError):g.ShortDriveLimits(**changes)
    assert before==MotionLimits(.447,.114,.02)


def test_new_adapter_cannot_invoke_survey_yaw_control_and_never_publishes_nonzero_yaw(monkeypatch):
    from inspection_mission.ground_observation_survey import GroundSurveyGuard
    def forbidden(*args,**kwargs):raise AssertionError("survey control must not run during short drive")
    monkeypatch.setattr(GroundSurveyGuard,"step",forbidden)
    monkeypatch.setattr(GroundSurveyGuard,"watchdog_reason",forbidden)
    g=api();callbacks,published,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_short_drive_node(wall_clock=lambda:wall[0])
    for i in range(25):
        wall[0]=i*.05;stamp=10.+wall[0];sec=int(stamp)
        callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((stamp-sec)*1e9)}})
        callbacks["/odom"](posed_odom(stamp));callbacks["/scan"](scan(stamp))
        node.tick()
    assert any(linear==.03 for _,linear,_ in published)
    assert all(topic=="/cmd_vel" and 0<=linear<=.03 and angular==0. for topic,linear,angular in published)
    node.stop("test_complete");node.close_watchdog()


@pytest.mark.parametrize("child_exit,setup_exit",[(0,0),(23,0),(0,31)])
def test_short_drive_launcher_runs_only_uploaded_entry_and_preserves_exit(tmp_path,child_exit,setup_exit):
    import json
    import os
    import shlex
    import shutil
    import subprocess
    import sys
    from pathlib import Path
    from test_ground_probe_launch import BASH,shell_path
    if not BASH.is_file():pytest.skip("bash unavailable")
    workspace=tmp_path/"uploaded short drive workspace"
    package=workspace/"src/inspection_mission/inspection_mission"
    package.mkdir(parents=True)
    (package/"__init__.py").write_text("",encoding="utf-8")
    (package/"ground_short_drive.py").write_text(
        "import json,os,sys\nassert os.environ['FAKE_ROS_READY']=='1'\n"
        "print('SHORT_DRIVE_ARGS '+json.dumps(sys.argv[1:]))\n"
        f"sys.exit({child_exit})\n",encoding="utf-8")
    launcher=workspace/"scripts/run_ground_short_drive.sh"
    launcher.parent.mkdir()
    shutil.copyfile(Path(__file__).resolve().parents[1]/"scripts/run_ground_short_drive.sh",launcher)
    setup=tmp_path/"fake ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
        +(f"return {setup_exit}\n" if setup_exit else "export FAKE_ROS_READY=1\n"),encoding="utf-8")
    env=dict(os.environ,INSPECTION_ROS_SETUP=shell_path(setup))
    env.pop("INSPECTION_DIAGNOSTIC_URL",None)
    result=subprocess.run([str(BASH),str(launcher)],cwd=tmp_path,env=env,
        capture_output=True,text=True,encoding="utf-8",timeout=10)
    assert result.returncode==(setup_exit or child_exit),result.stdout+result.stderr
    assert f"GROUND_SHORT_DRIVE launch_exit exit_code={setup_exit or child_exit}" in result.stderr
    if not setup_exit:
        line=next(line for line in result.stdout.splitlines() if line.startswith("SHORT_DRIVE_ARGS "))
        assert json.loads(line.removeprefix("SHORT_DRIVE_ARGS "))==["--duration","25"]
    else:assert "SHORT_DRIVE_ARGS" not in result.stdout


def test_three_fresh_recoveries_are_allowed_but_fourth_cannot_resume():
    state=guard();ready(state)
    first=state.pulse_started
    wall=1.1
    for resume in range(1,5):
        wall+=.21
        assert state.step(wall_s=wall).state=="PAUSE"
        for i in range(23):
            t=wall+.01+i*.05;refresh(state,t)
            decision=state.step(wall_s=t)
        wall=t
        if resume<=3:
            assert decision.state=="DRIVE" and state.resume_count==resume
        else:
            assert decision.state=="STOP" and decision.reason=="pause_resume_limit"
    assert state.pulse_started==first


@pytest.mark.parametrize("fault",["clock_reverse","odom_reverse","scan_reverse","wall_reverse","clock_nan"])
def test_bad_time_callback_is_latched_before_next_good_sensor(fault):
    state=guard();ready(state)
    if fault=="clock_reverse":state.update_clock(10.,wall_s=1.11)
    elif fault=="clock_nan":state.update_clock(float("nan"),wall_s=1.11)
    elif fault=="odom_reverse":state.update_odom(posed_odom(10.),wall_s=1.11)
    elif fault=="scan_reverse":state.update_scan(scan(10.),wall_s=1.11)
    else:state.update_odom(posed_odom(11.11),wall_s=1.09)
    assert state.state=="STOP"
    reason=state.reason;refresh(state,1.12)
    assert state.step(wall_s=1.12).linear_x==0. and state.reason==reason


def test_watchdog_pauses_and_repeatedly_zeroes_without_ros_spin(monkeypatch):
    import time
    g=api();_,published,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_short_drive_node(wall_clock=lambda:wall[0])
    ready(node.guard);wall[0]=1.1
    node.arm_watchdog();wall[0]=1.31
    deadline=time.monotonic()+.4
    while len(published)<2 and time.monotonic()<deadline:time.sleep(.01)
    node.close_watchdog()
    assert node.guard.state=="PAUSE"
    assert len(published)>=2 and all(row==("/cmd_vel",0.,0.) for row in published)
    node.guard.stop("hard_stop",1.32)
    node.guard.handle_watchdog("scan_stale",1.33)
    assert node.guard.state=="STOP" and node.guard.reason=="hard_stop"


@pytest.mark.parametrize("boundary",["step","watchdog","watchdog_handler"])
def test_wall_reversal_after_new_sensor_is_hard_stop_even_before_next_tick(boundary):
    state=guard();ready(state)
    state.update_clock(11.15,wall_s=1.15)
    if boundary=="step":
        state.step(wall_s=1.14)
    elif boundary=="watchdog":
        reason=state.watchdog_reason(1.14)
        assert reason=="wall_clock_reversed_or_invalid"
        state.handle_watchdog(reason,1.14)
    else:state.handle_watchdog("scan_stale",1.14)
    assert state.state=="STOP" and state.reason=="wall_clock_reversed_or_invalid"


def test_pause_write_cannot_overwrite_a_concurrent_hard_stop():
    import threading
    from inspection_mission.bounded_ground_probe import MotionLimits
    entered,release,stop_started=(threading.Event() for _ in range(3))
    class RaceGuard(api().GroundShortDriveGuard):
        def __setattr__(self,name,value):
            if name=="state" and value=="PAUSE" and getattr(self,"race_enabled",False):
                entered.set();release.wait(.5)
            super().__setattr__(name,value)
    state=RaceGuard(MotionLimits(.447,.114,.02),started_wall_s=0.)
    ready(state);state.race_enabled=True
    pauser=threading.Thread(target=lambda:state.handle_watchdog("scan_stale",1.31),daemon=True)
    def stop():
        stop_started.set();state.stop("hard_stop",1.31)
    stopper=threading.Thread(target=stop,daemon=True)
    pauser.start()
    try:
        assert entered.wait(.3)
        stopper.start();assert stop_started.wait(.3)
    finally:
        release.set();pauser.join(.5);stopper.join(.5)
    assert not pauser.is_alive() and not stopper.is_alive()
    assert state.state=="STOP" and state.reason=="hard_stop"
    refresh(state,1.32)
    assert state.step(wall_s=1.32).linear_x==0.


def test_watchdog_late_arming_keeps_original_ten_second_deadline(monkeypatch):
    import time
    g=api();_,published,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_short_drive_node(wall_clock=lambda:wall[0]);ready(node.guard)
    first=node.guard.pulse_started
    wall[0]=first+9.
    node.arm_watchdog()
    wall[0]=first+10.
    deadline=time.monotonic()+.4
    while node.guard.state!="STOP" and time.monotonic()<deadline:time.sleep(.01)
    node.close_watchdog()
    assert node.guard.state=="STOP" and node.guard.reason=="short_drive_wall_time_limit"
    assert published and all(row==("/cmd_vel",0.,0.) for row in published)


def test_metadata_double_images_and_actual_drive_series_keep_independent_identity():
    from array import array
    from inspection_mission.telemetry import DiagnosticReporter
    state=guard();ready(state)
    state.record_command(wall_s=1.1,linear_x=.03,angular_z=0.)
    uav=posed_odom(11.1,x=1.);uav["child_frame_id"]="uav/base_link"
    state.observe_public("/uav/odom",uav,wall_s=1.1)
    for topic,frame in (("/camera/image","camera_link"),("/uav/camera/image","uav/camera_link")):
        image=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=11,nanosec=120_000_000),frame_id=frame),
            width=8,height=8,step=24,encoding="bgr8",data=array("B",bytes((100,90,80))*64))
        state.observe_image(topic,image,wall_s=1.12)
    evidence=state.survey_evidence()
    exposures=[item for item in evidence["exposure_records"] if item["phase"]=="last_associated"]
    assert {item["odom"]["source_topic"] for item in exposures}=={"/odom","/uav/odom"}
    assert {item["odom"]["position"][0] for item in exposures}=={0.,1.}
    events=state.additional_events()
    assert all(payload["mode"]=="ground_short_drive" for _,payload in events)
    metadata=next(payload for _,payload in events if payload.get("section")=="survey_metadata")
    assert metadata["survey"]["mode"]=="ground_short_drive"
    assert metadata["survey"]["drive"]["state"]=="DRIVE"
    assert "target_yaw" not in metadata["survey"] and "candidate_mapping" not in metadata["survey"]
    commands=next(payload for _,payload in events if payload.get("series")=="actual_commands")
    assert commands["rows"]==[[1.1,11.1,.03,0.,"DRIVE"]]
    images=[payload for _,payload in events if payload.get("section")=="survey_image_part"]
    assert {payload["topic"] for payload in images}=={"/camera/image","/uav/camera/image"}
    assert all(len(DiagnosticReporter._encode(event,payload))<65536 for event,payload in events)
