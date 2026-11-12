"""Synthetic contract probes only; these do not prove platform response."""
from copy import deepcopy
from dataclasses import asdict
import base64
import gzip
import hashlib
import importlib
import importlib.util
import json
import math

import pytest

from test_bounded_ground_probe import endpoint, install_ros_transport, scan
from test_ground_direction_response import Run
from test_ground_observation_survey import posed_odom


def api():
    name = "inspection_mission.ground_relocated_reverse"
    assert importlib.util.find_spec(name), "new relocated reverse runtime is missing"
    return importlib.import_module(name)


def fixture():
    run = Run()
    run.guard = api().GroundRelocatedReverseGuard(started_wall_s=0., session_id="relocated-test")
    run.guard.update_endpoints([endpoint()], matched_subscriptions=1)
    return run


def reach(phase):
    return fixture().until(lambda r: r.guard.phase == phase, cap=3600)


def test_full_sequence_uses_measured_south_preposition_then_three_negative_pulses():
    run = fixture(); stages = []; starts = []; origins = []
    for _ in range(3600):
        d = run.tick()
        stages.append((run.guard.stage_index, run.guard.phase, d.linear_x, d.angular_z))
        if run.guard.global_origin is not None:
            origins.append(run.guard.global_origin)
        if run.guard.state == "STOP":
            break
    g = run.guard
    assert g.reason == "relocated_reverse_response_observed_official_score_unverified"
    assert set((i, x) for i, p, x, w in stages if x < 0.) == {(3, -.03), (3, -.10)}
    assert any(i == 0 and w < 0. for i, p, x, w in stages)
    assert any(i == 1 and p == "PREPOSITION" and x == .03 for i, p, x, w in stages)
    assert any(i == 1 and w > 0. for i, p, x, w in stages)
    assert any(i == 2 and w > 0. for i, p, x, w in stages)
    assert len(g.turn_records) == 3 and len(g.pulse_records) == 3
    assert all(r["settled"]["stable_wall_s"] >= 1.-1e-9 and r["settled"]["stable_source_s"] >= 1.-1e-9
               for r in g.turn_records)
    assert [r["target_yaw_rad"] for r in g.turn_records] == [-math.pi/2, 0., math.pi/2]
    pre = g.preposition_record
    assert pre["settled"]["endpoint_error_m"] <= .02
    assert pre["settled"]["south_progress_m"] >= .148-1e-9
    assert pre["settled"]["stop_path_increment_m"] <= .01
    assert all(origin == origins[0] for origin in origins)
    assert g.started_wall == 0. and .15 < g.total_path_m < .35
    assert 4.5 < g.accumulated_rotation_rad < 5.
    assert g.total_turn_elapsed_s(run.wall) >= sum(r["elapsed_s"] for r in g.turn_records)
    assert all(abs(r["start"]["yaw_rad"]-math.pi/2) <= .025 for r in g.pulse_records)


def test_preposition_does_not_advance_from_elapsed_command_time_without_actual_motion():
    run = reach("PREPOSITION")
    initial = deepcopy(run.guard.preposition_origin)
    for _ in range(20):
        run.tick(command_gain=0.)
    assert run.guard.phase == "PREPOSITION" and run.guard.preposition_origin == initial
    assert run.guard.preposition_progress_m() < .001


def test_preposition_stops_two_mm_early_and_checks_actual_endpoint():
    run = reach("PREPOSITION")
    run.until(lambda r:r.guard.preposition_progress_m() >= .1475)
    run.tick(dy=-.0006)
    assert run.guard.phase == "PREPOSITION_SETTLE" and run.last.linear_x == 0.
    run.tick(dx=.021)
    assert run.guard.state == "STOP"


@pytest.mark.parametrize("field,value", [("global_wall_s",180.001),("max_path_m",.35001),
    ("max_origin_displacement_m",.25001),("max_rotation_rad",5.0001),
    ("turn_wall_s",45.001),("total_turn_wall_s",120.001),("preposition_wall_s",40.001),
    ("preposition_distance_m",.15001),("preposition_command_mps",.03001),
    ("pulse_wall_s",2.001),("pulse_displacement_m",.04001),("pulse_request_stop_m",.01001),
    ("scan_anchor_motion_limit_m",.03001),("local_stop_reserve_m",.03001),
    ("stable_drift_m",.00201),("stop_drift_m",.01001),("max_heading_error_rad",.05001)])
def test_no_experiment_limit_can_expand(field,value):
    with pytest.raises(ValueError):
        api().RelocatedReverseLimits(**{field:value})


@pytest.mark.parametrize("kind", ["path", "radius", "rotation", "global", "turn", "total_turn", "preposition"])
def test_all_permanent_and_phase_budgets_latch_zero(kind):
    run = reach("PREPOSITION") if kind == "preposition" else fixture().until(lambda r:r.last.angular_z < 0.)
    g = run.guard
    if kind == "path":g.total_path_m=.35
    elif kind == "radius":run.x += .251
    elif kind == "rotation":g.accumulated_rotation_rad=5.
    elif kind == "global":run.wall=180.
    elif kind == "turn":run.wall=g.turn_started_wall+45.
    elif kind == "total_turn":g.completed_turn_wall_s=120.
    else:run.wall=g.preposition_started_wall+40.
    if kind == "radius":run.tick()
    else:run.last=g.step(wall_s=run.wall)
    assert g.state == "STOP" and run.last.linear_x == run.last.angular_z == 0.
    assert g.step(wall_s=run.wall+.01).linear_x == 0.


def test_first_rejected_dangerous_scan_is_full_bounded_one_frame():
    run=fixture().until(lambda r:r.last.angular_z < 0.)
    msg=scan(10.+run.wall);msg["ranges"][17]=.6409
    run.guard.update_scan(msg,wall_s=run.wall)
    assert run.guard.state == "STOP"
    first=deepcopy(run.guard.rejected_scan)
    assert first["payload"] == msg and first["reason"]
    newer=deepcopy(msg);newer["ranges"][18]=.1
    run.guard.update_scan(newer,wall_s=run.wall)
    assert run.guard.rejected_scan == first
    assert run.guard.core_evidence()["rejected_scan"] == first


def test_default_contract_keeps_frozen_pulse_and_local_sensor_limits():
    from inspection_mission.ground_direction_response import DirectionLimits
    new=asdict(api().RelocatedReverseLimits());old=asdict(DirectionLimits())
    changed={k for k in old if old[k] != new[k]}
    assert changed == {"global_wall_s","max_path_m","max_origin_displacement_m","max_rotation_rad"}
    assert new["preposition_wall_s"] == 40.


def test_evidence_bundle_retains_raw_rejected_frame_and_all_integrity_fields():
    run=fixture().until(lambda r:r.last.angular_z<0.)
    msg=scan(10.+run.wall);msg["ranges"][17]=.6409
    run.guard.update_scan(msg,wall_s=run.wall)
    events=run.guard.additional_events()
    core=next(v["data"] for _,v in events if v["section"]=="ground_relocated_reverse_core")
    parts=[v for _,v in events if v["section"]=="ground_relocated_reverse_evidence_gzip_part"]
    assert len(parts)==parts[0]["part_count"]
    packed=base64.b64decode("".join(v["base64_part"] for v in sorted(parts,key=lambda v:v["part_index"])))
    raw=gzip.decompress(packed);body=json.loads(raw)
    assert hashlib.sha256(packed).hexdigest()==parts[0]["compressed_sha256"]
    assert hashlib.sha256(raw).hexdigest()==parts[0]["uncompressed_sha256"]
    assert len(packed)==parts[0]["compressed_bytes"] and len(raw)==parts[0]["uncompressed_bytes"]
    assert body["schema"]=="ground_relocated_reverse_evidence_v1"
    assert body["rejected_scan"]["payload"]==msg and body["rejected_scan"]==core["rejected_scan"]
    assert body["raw_odometry"] and body["raw_scans"] and body["commands"]


def test_callbacks_and_adapter_emit_only_real_guard_cmd_vel(monkeypatch):
    callbacks,_,_=install_ros_transport(monkeypatch)
    wall=[0.];node=api().create_relocated_reverse_node(wall_clock=lambda:wall[0]);node.arm_watchdog=lambda:None
    sent=[];other=[]
    node.publisher_by_topic["/cmd_vel"].publish=lambda m:sent.append((m.linear.x,m.linear.y,m.angular.z))
    for topic,publisher in node.publisher_by_topic.items():
        if topic != "/cmd_vel":publisher.publish=lambda m,t=topic:other.append(t)
    x=y=yaw=0.
    try:
        for i in range(1,3601):
            vx,vy,wz=sent[-1] if sent else (0.,0.,0.)
            vx*=.2;vy*=.2;wz*=.15
            x+=.05*(vx*math.cos(yaw)-vy*math.sin(yaw));y+=.05*(vx*math.sin(yaw)+vy*math.cos(yaw));yaw+=.05*wz
            wall[0]=round(.05*i,9);source=round(10.+wall[0],9);sec=int(source)
            callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((source-sec)*1e9)}})
            msg=posed_odom(source,x=x,yaw=yaw);msg["pose"]["pose"]["position"]["y"]=y
            msg["twist"]["twist"]["linear"]["x"]=abs(vx);msg["twist"]["twist"]["angular"]["z"]=abs(wz)
            callbacks["/odom"](msg);callbacks["/scan"](scan(source));node.tick()
            if node.guard.state == "STOP":break
        assert node.guard.reason == "relocated_reverse_response_observed_official_score_unverified"
        assert node.experiment_mode == "ground_relocated_reverse" and not other
        assert sent == [(r["linear_x"],r["linear_y"],r["angular_z"]) for r in node.guard.direction_commands if r["sent"]]
        assert {r["pulse_index"] for r in node.guard.direction_commands if r["linear_x"] < 0.} == {0,1,2}
    finally:node.close_watchdog()


@pytest.mark.parametrize("kind",["global","turn","preposition","pulse"])
def test_final_dds_query_crossing_deadline_publishes_only_zero(monkeypatch,kind):
    install_ros_transport(monkeypatch)
    if kind=="pulse":run=reach("PULSE")
    elif kind=="preposition":run=reach("PREPOSITION")
    else:run=fixture().until(lambda r:r.last.angular_z<0.)
    wall=[run.wall];node=api().create_relocated_reverse_node(wall_clock=lambda:wall[0]);node.guard=run.guard
    node.arm_watchdog=lambda:None;node.last_graph_refresh_wall=wall[0]
    g=node.guard
    deadline={"global":180.,"turn":(g.turn_started_wall or 0.)+45.,
              "preposition":(g.preposition_started_wall or 0.)+40.,
              "pulse":(g.pulse_started_wall or 0.)+2.}[kind]
    calls=[0];sent=[]
    def matches():
        calls[0]+=1
        if calls[0] == 2:wall[0]=deadline
        return 1
    node.publisher_by_topic["/cmd_vel"].get_subscription_count=matches
    node.publisher_by_topic["/cmd_vel"].publish=lambda m:sent.append((m.linear.x,m.linear.y,m.angular.z))
    try:
        d=node.tick()
        assert sent == [(0.,0.,0.)] and d.linear_x == d.angular_z == 0.
        assert g.state == "STOP" or g.phase == "PULSE_SETTLE"
    finally:node.close_watchdog()


def test_negative_publish_exception_independently_attempts_zero(monkeypatch):
    install_ros_transport(monkeypatch);run=reach("PULSE");wall=[run.wall]
    node=api().create_relocated_reverse_node(wall_clock=lambda:wall[0]);node.guard=run.guard
    node.arm_watchdog=lambda:None;node.last_graph_refresh_wall=wall[0];attempts=[]
    def publish(msg):
        attempts.append((msg.linear.x,msg.linear.y,msg.angular.z))
        if msg.linear.x < 0.:raise RuntimeError("negative transport fail")
    node.publisher_by_topic["/cmd_vel"].publish=publish
    try:
        node.tick()
        assert attempts == [(-.03,0.,0.),(0.,0.,0.)] and node.guard.state == "STOP"
    finally:node.close_watchdog()
