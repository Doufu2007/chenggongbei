"""Finite XY identification contracts; synthetic feedback is not platform proof."""
from copy import deepcopy
import gzip
import importlib
import importlib.util
import json
import math
from pathlib import Path

import pytest

from test_bounded_ground_probe import endpoint, scan, install_ros_transport
from test_ground_observation_survey import posed_odom


def api():
    name = "inspection_mission.ground_direction_response"
    assert importlib.util.find_spec(name), "independent direction-response runtime is missing"
    return importlib.import_module(name)


class Run:
    def __init__(self, **limits):
        self.guard = api().GroundDirectionResponseGuard(started_wall_s=0.,
            direction_limits=api().DirectionLimits(**limits), session_id="direction-test")
        self.guard.update_endpoints([endpoint()], matched_subscriptions=1)
        self.wall = self.x = self.y = self.yaw = 0.
        self.last = None

    def tick(self, *, dx=0., dy=0., yaw_delta=0., command_gain=.2):
        dt = .05
        vx = 0. if self.last is None else self.last.linear_x*command_gain
        vy = 0. if self.last is None else self.last.linear_y*command_gain
        wz = 0. if self.last is None else self.last.angular_z*.15
        self.x += dt*(vx*math.cos(self.yaw)-vy*math.sin(self.yaw))+dx
        self.y += dt*(vx*math.sin(self.yaw)+vy*math.cos(self.yaw))+dy
        self.yaw += dt*wz+yaw_delta
        self.wall = round(self.wall+dt, 9)
        self.feed(speed=math.hypot(vx,vy), yaw_speed=abs(wz))
        self.last = self.guard.step(wall_s=self.wall)
        self.guard.record_command(wall_s=self.wall, linear_x=self.last.linear_x,
            linear_y=self.last.linear_y, angular_z=self.last.angular_z, sent=True)
        return self.last

    def feed(self, *, speed=0., yaw_speed=0., source_stamp=None):
        stamp = round(10.+self.wall, 9) if source_stamp is None else source_stamp
        self.guard.update_clock(stamp, wall_s=self.wall)
        msg = posed_odom(stamp, x=self.x, yaw=self.yaw)
        msg["twist"]["twist"]["linear"]["x"] = speed
        msg["twist"]["twist"]["angular"]["z"] = yaw_speed
        msg["pose"]["pose"]["position"]["y"] = self.y
        msg["twist"]["twist"]["linear"]["y"] = 0.
        self.guard.update_odom(msg, wall_s=self.wall)
        self.guard.update_scan(scan(stamp), wall_s=self.wall)

    def until(self, predicate, cap=2500):
        for _ in range(cap):
            self.tick()
            if predicate(self):
                return self
            assert self.guard.state not in ("STOP","FINISHED"), self.guard.reason
        pytest.fail("bounded response phase not reached")


def pulse_run():
    return Run().until(lambda r:r.last.linear_x > 0.)


def test_missing_runtime_has_finite_spec_and_three_measured_pulses():
    run = Run()
    seen = []
    for _ in range(2500):
        decision = run.tick()
        if decision.linear_x or decision.linear_y:
            seen.append((run.guard.pulse_index,decision.linear_x,decision.linear_y))
        if run.guard.state == "STOP":
            break
    assert run.guard.reason == "direction_response_observed_official_score_unverified"
    assert set(seen) == {(0,.03,0.),(1,0.,.03),(2,.10,0.)}
    assert len(run.guard.pulse_records) == 3
    for record in run.guard.pulse_records:
        assert record["start"]["raw_odom"]["header"]["stamp"]
        assert record["response"]["source_dt_s"] > 0.
        assert record["settled"]["stable_wall_s"] >= 1.-1e-9
        assert record["settled"]["stable_source_s"] >= 1.-1e-9
        assert record["settled"]["stop_path_increment_m"] <= .01
    assert run.guard.total_path_m < .18
    assert run.guard.max_observed_displacement_m < .12


@pytest.mark.parametrize("field,value",[("global_wall_s",120.001),("max_path_m",.180001),
    ("max_origin_displacement_m",.120001),("max_rotation_rad",1.00001),
    ("pulse_wall_s",2.00001),("pulse_displacement_m",.040001),
    ("pulse_request_stop_m",.010001),("max_translation_speed_mps",.150001),
    ("max_translation_yaw_speed_rps",.150001),("max_pulse_yaw_drift_rad",.030001),
    ("stable_drift_m",.002001),("stop_drift_m",.010001),
    ("maximum_yaw_rate",.500001),("turn_wall_s",45.001),("stable_s",.99)])
def test_experiment_cannot_expand_spec_bounds(field,value):
    with pytest.raises(ValueError):
        api().DirectionLimits(**{field:value})


def test_turn_precedes_translation_and_every_handoff_publishes_zero():
    run = Run()
    rows=[]
    for _ in range(2500):
        d=run.tick(); rows.append((run.wall,run.guard.phase,run.guard.pulse_index,d))
        if run.guard.state == "STOP":break
    first=next(i for i,r in enumerate(rows) if r[3].linear_x)
    assert abs(run.guard.pulse_records[0]["start"]["yaw_rad"]+.6)<=.025
    assert all(r[3].linear_x==r[3].linear_y==0 for r in rows[:first])
    for index in (1,2):
        start=next(i for i,r in enumerate(rows) if r[2]==index and (r[3].linear_x or r[3].linear_y))
        previous=max(i for i,r in enumerate(rows[:start]) if r[3].linear_x or r[3].linear_y)
        assert rows[start][0]-rows[previous][0]>=1.


def test_same_stamp_scan_change_cannot_hide_near_or_invalid_returns():
    for value in (.50,float("nan"),float("inf")):
        run=pulse_run(); stamp=10.+run.wall
        msg=scan(stamp);msg["ranges"][2]=value
        run.guard.update_scan(msg,wall_s=run.wall)
        d=run.guard.step(wall_s=run.wall)
        assert d.linear_x==d.linear_y==0 and run.guard.state=="STOP"


@pytest.mark.parametrize("topic",["scan","odom"])
def test_identical_duplicate_does_not_refresh_source_liveness(topic):
    run=pulse_run();g=run.guard
    sample=deepcopy(g.scan if topic=="scan" else g.odom)
    original=deepcopy(g.last_raw_scan if topic=="scan" else g.last_raw_odom)
    run.wall=round(run.wall+.19,9)
    getattr(g,"update_"+topic)(original,wall_s=run.wall)
    assert (g.scan if topic=="scan" else g.odom)["wall"]==sample["wall"]
    d=g.step(wall_s=run.wall+.02)
    assert d.linear_x==d.linear_y==0 and g.state in ("PAUSE","STOP")


def test_changed_duplicate_odom_locks_stop_instead_of_replacing_pose():
    run=pulse_run();msg=deepcopy(run.guard.last_raw_odom)
    msg["pose"]["pose"]["position"]["x"]+=.001
    run.guard.update_odom(msg,wall_s=run.wall)
    assert run.guard.state=="STOP" and "duplicate_changed" in run.guard.reason


def test_pause_does_not_restart_or_expand_pulse_time():
    run=pulse_run();g=run.guard;start=g.pulse_started_wall
    run.wall=round(run.wall+.25,9);g.step(wall_s=run.wall)
    assert g.state=="PAUSE"
    for _ in range(55):
        run.tick()
        if g.pulse_index>0:break
        assert g.pulse_started_wall==start
        if run.wall>=start+2.:
            assert run.last.linear_x==run.last.linear_y==0.


@pytest.mark.parametrize("kind",["pulse_radius","pulse_yaw","speed","angular_speed","global_origin","path","rotation","global_wall"])
def test_motion_and_permanent_budget_breaches_latch_zero(kind):
    run=pulse_run();g=run.guard
    if kind=="pulse_radius":run.tick(dx=.041)
    elif kind=="pulse_yaw":run.tick(yaw_delta=.031)
    elif kind=="speed":
        run.wall+=.01;run.feed(speed=.151)
    elif kind=="angular_speed":
        run.wall+=.01;run.feed(yaw_speed=.151)
    elif kind=="global_origin":
        g.global_origin=(-.121,0.,.055);run.tick()
    elif kind=="path":g.total_path_m=.181;run.tick()
    elif kind=="rotation":g.accumulated_rotation_rad=1.001;run.tick()
    else:run.wall=120.;g.step(wall_s=run.wall)
    assert g.state=="STOP",g.reason
    for _ in range(4):
        d=run.tick()
        assert d.linear_x==d.linear_y==d.angular_z==0.


def test_measured_10mm_pulse_requests_zero_before_40mm_hard_bound():
    run=pulse_run();g=run.guard
    run.tick(dx=.010)
    assert g.phase=="PULSE_SETTLE" and g.state!="STOP"
    assert run.last.linear_x==run.last.linear_y==0.
    run.tick(dx=.011)
    assert g.state=="STOP"


def test_missing_feedback_or_lost_dds_cannot_advance_pulse():
    run=pulse_run();g=run.guard;idx=g.pulse_index
    g.update_endpoints([endpoint()],matched_subscriptions=0)
    d=g.step(wall_s=run.wall)
    assert g.state=="STOP" and d.linear_x==d.linear_y==0
    assert g.pulse_index==idx


def test_v26_actual_trace_preserves_direction_mismatch_and_old_cross_track_stop():
    bundle=Path(__file__).resolve().parents[1]/"runs/analysis/20261112_three_point_v26/bundle.json"
    data=json.loads(bundle.read_text(encoding="utf-8"));s=data["survey"];a=s["approach"]
    from inspection_mission.ground_bounded_approach_sync import GroundSynchronizedApproachGuard
    old=GroundSynchronizedApproachGuard(started_wall_s=0.)
    old.phase=old.state="DRIVE";old.drive_origin=a["drive_origin"];old.drive_yaw=a["drive_yaw_rad"]
    sample=next(r for r in s["trajectory_10hz"] if r["stamp_s"]>=17.16-1e-9)
    old.odom={"xyz":tuple(sample["position"]),"q":tuple(sample["quaternion"]),
              "tilt":0.,"speed":sample["linear_speed"],"yaw_speed":sample["angular_speed"]}
    assert old.observed_motion_fault()=="approach_cross_track_limit"
    segment=[r for r in s["trajectory_10hz"] if 14.6-1e-9<=r["stamp_s"]<=15.24+1e-9]
    def raw(r):
        msg=posed_odom(r["stamp_s"],x=r["position"][0],yaw=r["yaw_rad"])
        msg["pose"]["pose"]["position"]["y"]=r["position"][1]
        return msg
    measured=api().response_from_odom(raw(segment[0]),raw(segment[-1]))
    assert measured["displacement_bearing_rad"]==pytest.approx(-.06056367275)
    assert .18<measured["bearing_minus_start_yaw_rad"]<.21
    assert measured["interface_semantics_confirmed"] is False


def test_xy_publisher_preserves_y_and_no_other_output_is_used(monkeypatch):
    install_ros_transport(monkeypatch);wall=[0.]
    node=api().create_direction_response_node(wall_clock=lambda:wall[0])
    sent=[]
    node.publisher_by_topic["/cmd_vel"].publish=lambda m:sent.append((m.linear.x,m.linear.y,m.angular.z))
    other=[]
    for topic,publisher in node.publisher_by_topic.items():
        if topic!="/cmd_vel":publisher.publish=lambda m,t=topic:other.append(t)
    d=api().DirectionDecision(0.,.03,0.,"PULSE","test")
    node.publish_direction(d)
    assert sent==[(0.,.03,0.)] and not other
    assert node.guard.direction_commands[-1]["linear_y"]==.03
    node.close_watchdog()


def test_publisher_failure_still_attempts_zero_and_latches_stop(monkeypatch):
    install_ros_transport(monkeypatch);node=api().create_direction_response_node(wall_clock=lambda:0.)
    attempts=[]
    def publish(m):
        attempts.append((m.linear.x,m.linear.y,m.angular.z))
        if m.linear.y:raise RuntimeError("transport failed")
    node.publisher_by_topic["/cmd_vel"].publish=publish
    node.publish_direction(api().DirectionDecision(0.,.03,0.,"PULSE","test"))
    assert attempts[:2]==[(0.,.03,0.),(0.,0.,0.)]
    assert all(item==(0.,0.,0.) for item in attempts[2:])
    assert node.guard.state=="STOP"
    node.publish_zero();assert attempts[-1]==(0.,0.,0.)
    assert any(not item["sent"] and item["linear_y"]==.03 for item in node.guard.direction_commands)
    assert any(item["sent"] and item["linear_y"]==0. for item in node.guard.direction_commands)
    node.close_watchdog()


def test_core_and_gzip_parts_use_receiver_sized_envelopes_and_keep_raw_twist():
    from inspection_mission.telemetry import DiagnosticReporter
    run=pulse_run();events=run.guard.additional_events()
    parts=[p for _,p in events if p.get("section")=="ground_direction_evidence_gzip_part"]
    assert parts
    import base64
    body=json.loads(gzip.decompress(base64.b64decode("".join(p["base64_part"] for p in parts))))
    assert body["raw_odometry"][0]["payload"]["twist"]["twist"]["linear"]=={"x":0.,"y":0.,"z":0.}
    assert body["official_score"] is None and body["full_mission_complete"] is False
    for event,payload in events:
        assert len(DiagnosticReporter._encode(event,payload))<65536


def test_scaled_valid_quaternion_cannot_hide_real_yaw_drift():
    run=pulse_run();g=run.guard
    run.wall+=.01;stamp=10.+run.wall
    msg=posed_odom(stamp,x=run.x,yaw=run.yaw-.03005)
    msg["pose"]["pose"]["position"]["y"]=run.y
    for key in ("x","y","z","w"):
        msg["pose"]["pose"]["orientation"][key]*=.9991
    g.update_clock(stamp,wall_s=run.wall);g.update_odom(msg,wall_s=run.wall)
    assert g.state=="STOP" and g.reason=="direction_pulse_yaw_drift_limit"
    assert math.hypot(*[g.last_raw_odom["pose"]["pose"]["orientation"][k] for k in ("x","y","z","w")])==pytest.approx(.9991)


@pytest.mark.parametrize("initial",[(.051,0.,0.),(0.,0.,.051)])
def test_initial_pose_remains_near_original_origin_and_zero_yaw(initial):
    run=Run();run.x,run.y,run.yaw=initial
    for _ in range(40):
        run.tick()
        if run.guard.state=="STOP":break
    assert run.guard.reason=="direction_initial_pose_outside_contract"


def test_actual_node_tick_transmits_y_phase_without_losing_linear_y(monkeypatch):
    install_ros_transport(monkeypatch);wall=[0.]
    node=api().create_direction_response_node(wall_clock=lambda:wall[0])
    sent=[];other=[]
    node.publisher_by_topic["/cmd_vel"].publish=lambda m:sent.append((m.linear.x,m.linear.y,m.angular.z))
    for topic,publisher in node.publisher_by_topic.items():
        if topic!="/cmd_vel":publisher.publish=lambda m,t=topic:other.append(t)
    run=Run();run.guard=node.guard;last=None
    try:
        for _ in range(2500):
            vx=0. if last is None else last.linear_x*.2
            vy=0. if last is None else last.linear_y*.2
            wz=0. if last is None else last.angular_z*.15
            run.x+=.05*(vx*math.cos(run.yaw)-vy*math.sin(run.yaw))
            run.y+=.05*(vx*math.sin(run.yaw)+vy*math.cos(run.yaw));run.yaw+=.05*wz
            run.wall=wall[0]=round(run.wall+.05,9);run.feed(speed=math.hypot(vx,vy),yaw_speed=abs(wz))
            last=node.tick()
            if last.linear_y:
                assert sent[-1]==(0.,.03,0.)
                assert node.guard.direction_commands[-1]["linear_y"]==.03
                break
        else:pytest.fail("actual node tick never published Y phase")
        assert not other
    finally:node.close_watchdog()


def test_dds_disappearing_after_step_is_checked_again_before_nonzero_publish(monkeypatch):
    install_ros_transport(monkeypatch);run=pulse_run();wall=[run.wall]
    node=api().create_direction_response_node(wall_clock=lambda:wall[0]);node.guard=run.guard
    node.last_graph_refresh_wall=wall[0]
    matches=iter((1,0));sent=[]
    node.publisher_by_topic["/cmd_vel"].get_subscription_count=lambda:next(matches)
    node.publisher_by_topic["/cmd_vel"].publish=lambda m:sent.append((m.linear.x,m.linear.y,m.angular.z))
    try:
        decision=node.tick()
        assert sent==[(0.,0.,0.)] and decision.linear_x==decision.linear_y==0.
        assert node.guard.state=="STOP"
    finally:node.close_watchdog()


def test_moving_feedback_cannot_complete_stop_dwell_or_start_next_pulse():
    run=pulse_run();g=run.guard;run.tick(dx=.010)
    for _ in range(12):run.tick(dx=.0005)
    assert g.pulse_index==0 and g.phase=="PULSE_SETTLE"
    assert run.last.linear_x==run.last.linear_y==0.


def test_scan_pair_never_borrows_future_odom():
    g=api().GroundDirectionResponseGuard(started_wall_s=0.)
    g.update_endpoints([endpoint()],matched_subscriptions=1)
    g.update_clock(10.01,wall_s=0.)
    g.update_odom(posed_odom(10.001),wall_s=0.)
    g.update_scan(scan(10.),wall_s=0.)
    assert g.scan_anchor is None
    assert g.step(wall_s=0.).linear_x==0.


def test_turn_wall_budget_survives_pause_and_never_emits_after_45s():
    run=Run().until(lambda r:r.last.angular_z!=0.)
    start=run.guard.turn_started_wall
    run.wall=start+45.
    d=run.guard.step(wall_s=run.wall)
    assert run.guard.state=="STOP" and d.angular_z==0.
    assert "turn_wall_time" in run.guard.reason
