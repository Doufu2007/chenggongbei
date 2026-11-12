"""The v16 timing variant pairs real late priors without changing ages."""
import importlib
import importlib.util
import json
import math

import pytest

from test_bounded_ground_probe import install_ros_transport,scan
from test_ground_bounded_approach import capture_ready,drive_ready,image,refresh
from test_ground_observation_survey import posed_odom


def api():
    name="inspection_mission.ground_bounded_approach_sync"
    assert importlib.util.find_spec(name) is not None,"independent v16 timing entry is missing"
    return importlib.import_module(name)


def guard():return api().GroundSynchronizedApproachGuard(started_wall_s=0.)


def pending(state,wall,yaw):
    state.update_clock(round(10.+wall,9),wall_s=wall)
    state.update_scan(scan(round(10.+wall+.002,9)),wall_s=wall+.001)


def test_late_real_prior_promotes_original_scan_without_rewriting_stamp_or_age():
    state=guard();wall,yaw=drive_ready(state)
    wall+=.05;pending(state,wall,yaw)
    old=state.scan.copy()
    state.update_odom(posed_odom(round(10.+wall,9),yaw=yaw),wall_s=wall+.01)
    assert state.scan["stamp"]==pytest.approx(10.+wall+.002)
    assert state.scan["wall"]==pytest.approx(wall+.001)
    assert state.scan_anchor["odom"]["stamp_s"]==pytest.approx(10.+wall)
    assert state.scan_anchor["scan_minus_odom_s"]==pytest.approx(.002)
    assert state.scan["stamp"]>old["stamp"] and state.state=="DRIVE"
    assert state.sync_diagnostics()["late_prior_promotions"]==1


def test_new_unpaired_scan_does_not_erase_previous_still_fresh_paired_scan():
    state=guard();wall,yaw=drive_ready(state)
    old_scan,old_anchor=state.scan.copy(),state.scan_anchor.copy()
    wall+=.05;pending(state,wall,yaw)
    assert state.scan==old_scan and state.scan_anchor==old_anchor
    assert state.step(wall_s=wall+.01).state=="DRIVE"
    state.update_clock(round(10.+wall+.17,9),wall_s=wall+.17)
    assert state.step(wall_s=wall+.17).state=="PAUSE"


@pytest.mark.parametrize("fault",["future_pose","expired_scan","invalid_pose","reversed_pose"])
def test_ineligible_late_pose_cannot_promote_a_scan_or_revive_hard_stop(fault):
    state=guard();wall,yaw=drive_ready(state)
    old_stamp=state.scan["stamp"]
    wall+=.05;pending(state,wall,yaw)
    payload=posed_odom(round(10.+wall+(.02 if fault=="future_pose" else 0.),9),yaw=yaw)
    arrival=wall+.01
    if fault=="expired_scan":
        arrival=wall+.22;state.update_clock(round(10.+arrival,9),wall_s=arrival)
    elif fault=="invalid_pose":payload["child_frame_id"]="uav/base_link"
    elif fault=="reversed_pose":payload=posed_odom(round(10.+wall-.1,9),yaw=yaw)
    state.update_odom(payload,wall_s=arrival)
    assert state.scan["stamp"]==old_stamp and state.sync_diagnostics()["late_prior_promotions"]==0
    if fault in ("invalid_pose","reversed_pose"):
        assert state.state=="STOP"
        reason=state.reason
        state.update_odom(posed_odom(round(10.+wall+.03,9),yaw=yaw),wall_s=wall+.03)
        assert state.state=="STOP" and state.reason==reason


def test_new_unsafe_scan_latches_stop_even_while_previous_pair_is_fresh():
    state=guard();wall,yaw=drive_ready(state)
    state.update_scan(scan(round(10.+wall+.01,9),[0.]*360),wall_s=wall+.01)
    assert state.state=="STOP" and state.reason.startswith("scan_invalid:")


def test_repeated_scan_before_prior_order_can_complete_fresh_pause_dwell():
    state=guard();wall,yaw=drive_ready(state)
    state.temporal_fault("scan_anchor_unavailable",wall)
    original=(state.global_origin,state.drive_origin,state.drive_started_wall)
    for i in range(1,25):
        current=wall+i*.05
        state.update_clock(round(10.+current,9),wall_s=current)
        if i%2==0:state.update_scan(scan(round(10.+current+.002,9)),wall_s=current+.001)
        state.update_odom(posed_odom(round(10.+current,9),yaw=yaw),wall_s=current+.005)
        decision=state.step(wall_s=current+.01)
    assert decision.state=="DRIVE" and state.resume_count==1
    assert (state.global_origin,state.drive_origin,state.drive_started_wall)==original


def test_throttled_graph_still_checks_real_matches_every_tick(monkeypatch):
    from inspection_mission.bounded_ground_probe import MotionLimits,create_bounded_node
    install_ros_transport(monkeypatch)
    wall=[0.]
    try:node=create_bounded_node(MotionLimits(.447,.114,.02),wall_clock=lambda:wall[0],graph_refresh_interval_s=.5)
    except TypeError as exc:pytest.fail("optional graph interval is missing: "+str(exc))
    calls=[];original=node.get_subscriptions_info_by_topic
    node.get_subscriptions_info_by_topic=lambda topic:(calls.append(topic),original(topic))[1]
    for i in range(1,10):wall[0]=i*.05;node.tick()
    assert calls==[]
    wall[0]=.5;node.tick()
    assert len(calls)==6
    diagnostics=node.evidence_snapshot()["graph_refresh_diagnostics"]
    assert diagnostics["refresh_count"]==2 and diagnostics["match_check_count"]==10
    node.close_watchdog()


def test_throttled_query_does_not_delay_matching_subscriber_loss_stop(monkeypatch):
    g=api();_,published,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_sync_approach_node(wall_clock=lambda:wall[0])
    wall[0],yaw=drive_ready(node.guard)
    node.tick()
    node.publisher_by_topic["/cmd_vel"].get_subscription_count=lambda:0
    wall[0]+=.05
    decision=node.tick()
    assert decision.state=="STOP" and decision.linear_x==0.
    assert decision.reason=="no_external_matching_cmd_subscriber"
    assert published[-1]==("/cmd_vel",0.,0.)
    node.close_watchdog()


@pytest.mark.parametrize("fault",["scan_reversed","clock_epoch_reset"])
def test_pending_scan_cannot_revive_a_hard_timestamp_fault(fault):
    state=guard();wall,yaw=drive_ready(state)
    wall+=.05;pending(state,wall,yaw)
    if fault=="scan_reversed":
        state.update_scan(scan(round(10.+wall-.1,9)),wall_s=wall+.005)
    else:
        state.update_clock(1.,wall_s=wall+.005)
    assert state.state=="STOP"
    reason=state.reason
    state.update_odom(posed_odom(round(10.+wall,9),yaw=yaw),wall_s=wall+.01)
    assert state.state=="STOP" and state.reason==reason
    assert state.sync_diagnostics()["pending_count"]==0
    assert state.sync_diagnostics()["late_prior_promotions"]==0


def test_sync_variant_keeps_original_budgets_and_full_stationary_three_frame_capture():
    from inspection_mission.telemetry import DiagnosticReporter
    from inspection_mission.ground_bounded_approach import ApproachLimits
    state=guard();assert state.approach_limits==ApproachLimits()
    wall,yaw=capture_ready(state)
    for index in range(3):
        wall+=.1
        refresh(state,wall,x=.201*math.cos(yaw),y=.201*math.sin(yaw),yaw=yaw)
        payload=posed_odom(round(10.+wall,9),yaw=yaw)
        payload["child_frame_id"]="uav/base_link"
        state.observe_public("/uav/odom",payload,wall_s=wall)
        state.observe_image("/uav/camera/image",image(round(10.+wall+.01,9)),wall_s=wall+.01)
    assert state.state=="STOP" and state.reason=="approach_observation_complete"
    assert len(state.capture_frames)==3 and state.measured_forward()>=.20
    assert state.scan["required_range_m"]==pytest.approx(.641)
    assert state.approach_evidence()["task_completion"]=="unverified"
    assert len(state.survey_evidence()["scan_records"])==4
    events=state.additional_events()
    assert events and all(len(DiagnosticReporter._encode(event,payload))<65536 for event,payload in events)
    assert events[0][1]["survey"]["approach"]["sensor_sync"]["pending_count"]==0


def test_pending_and_diagnostic_storage_remain_bounded_without_exposing_raw_scans():
    state=guard()
    # Without a clock no valid ring can yet be promoted; the pending queue is
    # bounded rather than accumulating complete sensor messages indefinitely.
    for i in range(8):state.update_scan(scan(10.+i*.01),wall_s=i*.01)
    assert state.sync_diagnostics()["pending_count"]==4
    assert state.sync_diagnostics()["capacity_dropped"]==4
    state=guard();wall,yaw=drive_ready(state)
    for i in range(1,40):refresh(state,wall+i*.01,yaw=yaw)
    diag=state.sync_diagnostics()
    assert len(diag["promotion_trace"])==12 and diag["promotion_trace_dropped"]>0
    assert "ranges" not in json.dumps(diag) and len(json.dumps(diag))<8192


def test_clock_age_point_two_and_prior_skew_point_zero_five_are_not_relaxed():
    state=guard();wall,yaw=drive_ready(state)
    wall+=.05;pending(state,wall,yaw)
    state.update_clock(round(10.+wall+.203,9),wall_s=wall+.19)
    state.update_odom(posed_odom(round(10.+wall,9),yaw=yaw),wall_s=wall+.195)
    assert state.sync_diagnostics()["late_prior_promotions"]==0
    assert state.step(wall_s=wall+.195).state=="PAUSE"


def test_late_prior_counter_uses_actual_arrival_instead_of_callback_name():
    state=guard()
    state.update_clock(10.,wall_s=0.)
    state.update_odom(posed_odom(10.12),wall_s=0.)
    state.update_scan(scan(10.15),wall_s=.001)
    state.update_clock(10.1,wall_s=.05)
    state.update_odom(posed_odom(10.16),wall_s=.06)
    assert state.scan["stamp"]==10.15
    assert state.sync_diagnostics()["late_prior_promotions"]==0
    assert state.sync_diagnostics()["promotion_trace"][-1]["late_prior"] is False
