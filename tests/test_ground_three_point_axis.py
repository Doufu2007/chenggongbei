"""Axis/reverse runtime contracts; synthetic motion is not platform evidence."""
from copy import deepcopy
import importlib
import importlib.util
import math

import pytest

from test_bounded_ground_probe import endpoint, install_ros_transport, scan
from test_ground_bounded_approach import refresh
from test_ground_three_point import ThreeRun
from test_ground_single_point import camera_info


def api():
    name = "inspection_mission.ground_three_point_axis"
    assert importlib.util.find_spec(name), "independent axis three-point runtime missing"
    return importlib.import_module(name)


class AxisRun(ThreeRun):
    def __init__(self, draw=("ground_0", "ground_3", "ground_2"), **limits):
        super().__init__(draw)
        self.guard = api().GroundThreePointAxisGuard(started_wall_s=0., session_id="test-current-session",
            route_limits=api().AxisThreePointLimits(**limits))
        self.guard.update_endpoints([endpoint()], matched_subscriptions=1)
        self.guard.observe_public("/referee/ground_tasks", {"data": ",".join(draw)}, wall_s=0.)
        self.guard.observe_public("/uav/camera/camera_info", camera_info(), wall_s=0.)


def reverse_guard(g=None):
    from inspection_mission.ground_three_point_axis_plan import plan_ground_leg
    g = g or api().GroundThreePointAxisGuard(started_wall_s=0.)
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    refresh(g, 0., x=0., y=-1.8, yaw=math.pi/2)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = (0., -1.8, .055), math.pi/2
    g.target_id = "ground_2"
    g.point_plan = plan_ground_leg(None, "ground_2", (0.,0.,0.))
    reverse = next(o for o in g.point_plan.entry_operations if o.kind == "DRIVE_REVERSE_TO_XY")
    g._load_operations((reverse,), "entry", 0.)
    return g


def test_axis_limits_new_rotation_contract_and_old_caps_unchanged():
    from inspection_mission.ground_three_point import ThreePointLimits
    assert ThreePointLimits().max_rotation_rad == 25.
    limits = api().AxisThreePointLimits()
    assert (limits.max_rotation_rad,limits.max_path_m,limits.global_wall_s) == (28.,12.5,1200.)
    assert (limits.turn_total_wall_s,limits.turn_wall_s,limits.drive_wall_s)==(700.,45.,500.)
    for field,value in (("max_rotation_rad",28.001),("max_path_m",12.5001),
                        ("global_wall_s",1200.001),("turn_total_wall_s",700.001),
                        ("drive_wall_s",500.001),("linear_command_mps",.1001)):
        with pytest.raises(ValueError): api().AxisThreePointLimits(**{field:value})


def test_first_ground4_real_runtime_plan_has_two_axes():
    run=AxisRun(("ground_4","ground_0","ground_2")).until(lambda r:r.guard.point_plan is not None)
    assert [o.target_xy_m for o in run.guard.operations if o.kind.startswith("DRIVE")]==[(.35,0.),(.35,-.1)]


def test_actual_v43_entry_turn_reaches_the_first_executable_drive_with_original_deadline():
    from inspection_mission.ground_three_point_axis_plan import plan_ground_entry, plan_ground_leg
    run = AxisRun(("ground_2", "ground_1", "ground_4"))
    run.x, run.y, run.yaw = (.5888341895790857, -1.7953274616929733, -2.1343145342713865)
    g = run.guard
    refresh(g, 0., x=run.x, y=run.y, yaw=run.yaw)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = g.odom["xyz"], run.yaw
    g.target_id = "ground_2"
    g.point_plan = plan_ground_leg(None, "ground_2", (0., 0., 0.))
    g._load_operations(plan_ground_entry("ground_2", (run.x, run.y, run.yaw)), "entry", 0.)
    run.last = g.step(wall_s=0.)
    assert run.last.angular_z < 0.  # West directly, instead of chasing the 4.67mm south point.
    run.until(lambda r: r.guard.state == "DRIVE", cap=900)
    assert run.wall < 45.
    assert g.leg_target_xy == (0., -1.7953274616929733)
    assert g.approach_limits.turn_wall_s == 45.
    assert g.scan["required_range_m"] == pytest.approx(.641)


def test_reverse_positive_progress_body_heading_and_negative_request():
    g=reverse_guard()
    assert g.reverse_segment and g.drive_yaw==pytest.approx(math.pi/2)
    assert g.leg_target_distance_m>0 and g.approach_limits.target_distance_m>0
    refresh(g,.05,x=0.,y=-1.801,yaw=math.pi/2,speed=-.02)
    assert g.measured_forward()==pytest.approx(.001)
    assert g.cross_track()==pytest.approx(0.)
    d=g.step(wall_s=.05)
    assert d.linear_x==-.03 and d.angular_z==0. and g.state=="DRIVE"
    assert g.limits.robot_radius_m==.447 and g.scan["required_range_m"]==pytest.approx(.641)


@pytest.mark.parametrize("fault,reason",[("wrong_direction","approach_reverse_limit"),
    ("cross","approach_cross_track_limit"),("heading","approach_heading_limit"),
    ("dds","no_external_matching_cmd_subscriber"),("danger","scan_invalid")])
def test_reverse_keeps_original_actual_feedback_and_matching_stops(fault,reason):
    g=reverse_guard()
    if fault=="dds":g.update_endpoints([endpoint()],matched_subscriptions=0)
    elif fault=="danger":
        msg=scan(10.05);msg["ranges"][0]=.5;g.update_scan(msg,wall_s=.05)
    else:
        refresh(g,.05,x=.0201 if fault=="cross" else 0.,
                y=-1.794 if fault=="wrong_direction" else -1.8,
                yaw=math.pi/2+(.051 if fault=="heading" else 0.))
    d=g.step(wall_s=.05)
    assert g.state=="STOP" and reason in g.reason
    assert d.linear_x==0.
    g.update_endpoints([endpoint()],matched_subscriptions=1)
    refresh(g,.10,x=0.,y=-1.8,yaw=math.pi/2)
    assert g.step(wall_s=.10).linear_x==0.


def test_reverse_flag_clears_before_any_next_operation():
    from inspection_mission.ground_three_point_plan import GroundOperation
    g=reverse_guard()
    following=GroundOperation("DRIVE_TO_XY",(0.,-1.7),(0.,-1.8),math.pi/2,math.pi/2,0.,"test",.7,.1)
    g._load_operations((following,),"observation",0.)
    assert not g.reverse_segment
    assert g._linear_request(0.)>0.
    g._load_operations((),"observation",0.)
    assert not g.reverse_segment


def test_reverse_turn_and_settle_aim_at_body_north_not_destination_south():
    from inspection_mission.ground_three_point_plan import GroundOperation
    from inspection_mission.ground_three_point_axis_plan import plan_ground_leg
    g=api().GroundThreePointAxisGuard(started_wall_s=0.)
    g.update_endpoints([endpoint()],matched_subscriptions=1)
    g.target_id="ground_2"
    g.point_plan=plan_ground_leg(None,"ground_2",(0.,0.,0.))
    reverse=next(o for o in g.point_plan.entry_operations if o.kind=="DRIVE_REVERSE_TO_XY")
    turn=GroundOperation("TURN_ONLY",None,(0.,-1.8),0.,math.pi/2,math.pi/2,"reverse_aim",.7)
    refresh(g,.05,x=0.,y=-1.8,yaw=0.)
    g._load_operations((turn,reverse),"entry",.05)
    assert g.target_yaw==pytest.approx(math.pi/2)
    assert g.step(wall_s=.05).angular_z>0


@pytest.mark.parametrize("draw",[("ground_2","ground_0","ground_4"),
                                 ("ground_4","ground_3","ground_2")])
def test_axis_entry_and_handoff_really_use_new_planner_and_keep_visual_epochs(draw):
    run=AxisRun(draw);epochs=[];sent=[];saw_negative=False
    for index,target in enumerate(draw):
        sent+=run.recognise();epochs.append(run.guard.stationary_epoch)
        old=deepcopy(run.guard.match_results[0])
        sent+=run.declare()
        saw_negative=saw_negative or any(row[1].linear_x<0 for row in run.commands)
        if target=="ground_2":
            assert any(o.kind=="DRIVE_REVERSE_TO_XY" for o in run.guard.operations)
        if index<2:
            before=(run.guard.global_origin,run.guard.started_wall,run.guard.total_path_m,
                    run.guard.accumulated_rotation_rad,run.guard.stationary_epoch)
            run.until(lambda r:r.guard.point_index==index+1)
            assert run.guard.global_origin==before[0] and run.guard.started_wall==before[1]
            assert run.guard.total_path_m>=before[2] and run.guard.accumulated_rotation_rad>=before[3]
            assert run.guard.stationary_epoch>before[4] and run.guard.recognition is None
            assert not run.guard.accept_match_result(old,wall_s=run.wall)
            for op in run.guard.point_plan.to_observation_operations+run.guard.point_plan.entry_operations:
                if op.target_xy_m is not None:
                    dx,dy=[op.target_xy_m[j]-op.anchor_xy_m[j] for j in (0,1)]
                    assert min(abs(dx),abs(dy))<1e-12
    run.until(lambda r:r.guard.state=="STOP")
    assert saw_negative and run.guard.reason=="three_ground_transport_complete_official_score_unverified"
    assert epochs==sorted(set(epochs)) and len(run.guard.completed_points)==3
    assert [s[1] for s in sent if s[0]=="/referee/current_target"]==list(draw)
    assert run.guard.accumulated_rotation_rad<=28. and run.guard.total_path_m<=12.5


def test_handoff_cannot_expand_or_reset_fixed_rotation_budget():
    run=AxisRun();run.recognise();run.declare()
    run.guard.accumulated_rotation_rad=27.95
    run.guard._handoff(run.wall)
    assert run.guard.state=="STOP" and run.guard.point_index==0
    assert run.guard.reason=="three_ground_remaining_rotation_budget_insufficient"
    assert run.guard.frozen_limits.max_rotation_rad==28.


def test_entry_without_current_visual_still_locks_stop():
    g=reverse_guard();g._begin_entry(.01)
    assert g.state=="STOP" and g.reason=="three_ground_entry_visual_gate_missing"


def test_actual_adapter_tick_publishes_signed_negative_and_rebinds_coordinator(monkeypatch):
    callbacks,_,_=install_ros_transport(monkeypatch)
    class Worker:
        def cancel_context(self):pass
        def poll(self):return []
    wall=[0.]
    node=api().create_axis_three_point_node(wall_clock=lambda:wall[0],worker=Worker())
    node.arm_watchdog=lambda:None
    assert node.single_point_coordinator.guard is node.guard
    assert isinstance(node.guard,api().GroundThreePointAxisGuard)
    reverse_guard(node.guard)
    rows=[]
    node.publisher_by_topic["/cmd_vel"].publish=lambda m:rows.append(m.linear.x)
    from test_ground_observation_survey import posed_odom
    wall[0]=.05
    callbacks["/clock"]({"clock":{"sec":10,"nanosec":50_000_000}})
    payload=posed_odom(10.05,x=0.,yaw=math.pi/2)
    payload["pose"]["pose"]["position"]["y"]=-1.801
    payload["twist"]["twist"]["linear"]["x"]=-.02
    callbacks["/odom"](payload)
    callbacks["/scan"](scan(10.05))
    decision=node.tick()
    assert decision.linear_x==-.03 and rows[-1]==-.03
    assert node.guard._observations.actual_commands[-1]["linear_x"]==-.03
    assert node.evidence_snapshot()["mode"]=="ground_three_point_axis"
    node.close_watchdog()


def adapter_reverse(monkeypatch):
    callbacks,_,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=api().create_axis_three_point_node(wall_clock=lambda:wall[0])
    node.arm_watchdog=lambda:None
    reverse_guard(node.guard)
    wall[0]=.05;refresh(node.guard,.05,x=0.,y=-1.801,yaw=math.pi/2,speed=-.02)
    sent=[]
    node.publisher_by_topic["/cmd_vel"].publish=lambda m:sent.append((m.linear.x,m.angular.z))
    return node,wall,sent


def test_later_graph_query_dds_loss_rechecked_at_actual_publish_boundary(monkeypatch):
    node,wall,sent=adapter_reverse(monkeypatch)
    match=[1];query=node.get_subscriptions_info_by_topic
    def subscriptions(topic):
        result=query(topic)
        if topic!="/cmd_vel":match[0]=0
        return result
    node.publisher_by_topic["/cmd_vel"].get_subscription_count=lambda:match[0]
    node.get_subscriptions_info_by_topic=subscriptions
    node.last_graph_refresh_wall=None
    d=node.tick()
    assert node.guard.state=="STOP" and d.linear_x==0.
    assert sent and all(x==w==0. for x,w in sent)
    node.close_watchdog()


def test_negative_publish_failure_locks_stop_and_independently_attempts_zero(monkeypatch):
    node,wall,sent=adapter_reverse(monkeypatch)
    def publish(msg):
        sent.append((msg.linear.x,msg.angular.z))
        if msg.linear.x<0:raise RuntimeError("negative transport failure")
    node.publisher_by_topic["/cmd_vel"].publish=publish
    d=node.tick()
    assert node.guard.state=="STOP" and d.linear_x==0.
    assert sent==[(-.03,0.),(0.,0.)]
    assert node.guard.published_nonzero_count==0
    node.publish_zero();assert sent[-1]==(0.,0.)
    node.close_watchdog()


@pytest.mark.parametrize("fault",["stale","budget","phase_stop"])
def test_final_actual_match_query_cannot_bypass_new_time_or_phase_fault(monkeypatch,fault):
    node,wall,sent=adapter_reverse(monkeypatch)
    node.last_graph_refresh_wall=wall[0]
    calls=[0]
    def match():
        calls[0]+=1
        if calls[0]>=2:
            if fault=="stale":wall[0]=.30
            elif fault=="budget":wall[0]=1200.
            else:node.guard.stop("phase_changed_during_match_query",wall[0])
        return 1
    node.publisher_by_topic["/cmd_vel"].get_subscription_count=match
    d=node.tick()
    assert calls[0]>=2 and d.linear_x==0. and all(x==w==0. for x,w in sent)
    assert node.guard.state in ("STOP","PAUSE")
    node.close_watchdog()


def test_zero_publish_failure_is_caught_and_next_zero_attempt_remains_possible(monkeypatch):
    node,wall,sent=adapter_reverse(monkeypatch)
    node.guard.stop("preexisting_stop",wall[0]);fail=[True]
    def publish(msg):
        sent.append((msg.linear.x,msg.angular.z))
        if fail[0]:raise RuntimeError("all output failed")
    node.publisher_by_topic["/cmd_vel"].publish=publish
    d=node.tick()
    assert d.linear_x==0. and node.guard.state=="STOP" and node.guard.reason=="preexisting_stop"
    fail[0]=False;node.publish_zero()
    assert sent[-1]==(0.,0.) and node.guard.published_zero_count==1
    node.close_watchdog()


def test_rebuilt_tick_polls_real_coordinator_dispatches_current_recognition_once_and_keeps_counts(monkeypatch):
    from test_ground_single_point import fixture_jobs
    run=AxisRun().until(lambda r:r.guard.state=="RECOGNIZE_WAIT",cap=25000)
    _,results=fixture_jobs(run)
    queued=list(results)
    class Worker:
        def cancel_context(self):pass
        def set_context(self,*args):pass
        def poll(self):
            answer=list(queued);queued.clear();return answer
    install_ros_transport(monkeypatch)
    node=api().create_axis_three_point_node(wall_clock=lambda:run.wall,worker=Worker())
    node.arm_watchdog=lambda:None
    node.guard=run.guard
    node.single_point_coordinator.guard=run.guard
    node.single_point_coordinator.prepare=lambda frames,**kwargs:[]
    sent=[]
    node.publisher_by_topic["/inspection/detections"].publish=lambda msg:sent.append(msg.data)
    node.tick()
    assert node.guard.attempts["recognition"]["sent"] and len(sent)==1
    assert node.guard.phase=="REPORT_WAIT"
    assert node.single_point_coordinator.context[1]=="ground_0"
    node.refresh_endpoints();node.tick()
    item=node.command_endpoints["/inspection/detections"]
    assert len(sent)==1 and item["actual_publish_attempt_count"]==item["actual_publish_return_count"]==1
    assert item["silent"] is False
    for topic in ("/uav/cmd_vel","/uav/laser_fire","/referee/cruise_event"):
        assert node.command_endpoints[topic]["silent"] is True
    node.close_watchdog()


def test_reverse_pause_and_stop_do_not_clear_permanent_budgets_or_allow_revival():
    g=reverse_guard();origin=g.global_origin
    g.total_path_m=.05;g.accumulated_rotation_rad=2.
    g.temporal_fault("scan_stale",.01)
    assert g.state=="PAUSE" and g.reverse_segment
    assert g.global_origin==origin and g.total_path_m==.05 and g.accumulated_rotation_rad==2.
    assert g._budget_fault(1200.)=="three_ground_global_wall_time_limit"
    g.turn_elapsed_wall_s=700.;g.phase="TURN"
    assert g._budget_fault(.01)=="route_turn_total_wall_time_limit"
    g.turn_elapsed_wall_s=0.;g.drive_elapsed_wall_s=500.;g.phase="DRIVE"
    assert g._budget_fault(.01)=="route_drive_total_wall_time_limit"
    g.stop("test_lock",.01)
    g._start_operation(.01)
    assert not g.reverse_segment and g.state=="STOP" and g.reason=="test_lock"


def test_reverse_zero_request_still_limits_accumulated_post_stop_path():
    g=reverse_guard();g.calibration_verified=True
    for i in range(1,90):
        refresh(g,i*.05,x=0.,y=-1.8-min(.889,i*.01),yaw=math.pi/2)
    assert g.phase=="DRIVE_SETTLE"
    base=g.total_path_m
    for i,dy in enumerate((.004,-.004,.004),90):
        current=g.odom["xyz"][1]
        refresh(g,i*.05,x=0.,y=current+dy,yaw=math.pi/2)
    assert g.total_path_m-base>.01
    assert g.state=="STOP" and g.reason=="route_drive_settle_path_limit"


def test_axis_evidence_receiver_contract_keeps_28rad_and_reverse_plan():
    from inspection_mission.telemetry import DiagnosticReporter
    from tools.diagnostic_receiver import validate_event
    import base64,gzip,json
    g=reverse_guard();events=g.additional_events()
    decoded=[validate_event(DiagnosticReporter._encode(e,p))["data"] for e,p in events]
    parts=[p for p in decoded if p.get("section")=="three_ground_evidence_gzip_part"]
    body=json.loads(gzip.decompress(base64.b64decode("".join(p["base64_part"] for p in parts))))
    assert body["survey"]["approach"]["three_point"]["target_id"]=="ground_2"
    core=next(p for p in decoded if p.get("section")=="three_point_core")
    assert core["mode"]=="ground_three_point_axis" and core["frozen_limits"]["max_rotation_rad"]==28.
    assert core["full_mission_complete"] is False


@pytest.mark.parametrize("exit_code",[0,17])
def test_axis_launcher_runs_new_source_and_preserves_real_exit(tmp_path,exit_code):
    import os,shlex,shutil,subprocess,sys
    from pathlib import Path
    from test_ground_probe_launch import BASH,shell_path
    if not BASH.is_file():pytest.skip("Git Bash unavailable")
    root=tmp_path/"axis uploaded workspace"
    package=root/"src/inspection_mission/inspection_mission";package.mkdir(parents=True)
    (package/"__init__.py").write_text("",encoding="utf-8")
    (package/"ground_three_point_axis.py").write_text(
        "import os,sys\nassert os.environ['AXIS_FAKE_ROS']=='1'\n"
        f"sys.exit({exit_code})\n",encoding="utf-8")
    source=Path(__file__).resolve().parents[1]/"scripts/run_ground_three_point_axis.sh"
    assert source.is_file(),"axis launcher missing"
    launcher=root/"scripts/run_ground_three_point_axis.sh";launcher.parent.mkdir()
    shutil.copyfile(source,launcher)
    setup=tmp_path/"ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
        +"export AXIS_FAKE_ROS=1\n",encoding="utf-8")
    env=dict(os.environ,INSPECTION_ROS_SETUP=shell_path(setup));env.pop("INSPECTION_DIAGNOSTIC_URL",None)
    done=subprocess.run([str(BASH),str(launcher)],cwd=tmp_path,env=env,capture_output=True,
        text=True,encoding="utf-8",timeout=10)
    assert done.returncode==exit_code,done.stdout+done.stderr
    assert f"GROUND_THREE_POINT_AXIS launch_exit exit_code={exit_code}" in done.stderr
