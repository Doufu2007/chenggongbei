"""Independent local boundary probes; never platform or scoring evidence."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/inspection_mission"))
sys.path.insert(0, str(ROOT / "tests"))

import pytest
from inspection_mission import ground_three_point_axis as api
from inspection_mission.ground_three_point_axis_plan import plan_ground_leg
from inspection_mission.ground_three_point_plan import GroundOperation
from test_bounded_ground_probe import endpoint, install_ros_transport, scan
from test_ground_observation_survey import posed_odom
from test_ground_bounded_approach import refresh


def prepare_reverse(g):
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    refresh(g, 0., x=0., y=-1.8, yaw=math.pi/2)
    g.global_origin = g.pulse_origin = (0., 0., .055)
    g.previous_path_xyz, g.previous_yaw = (0., -1.8, .055), math.pi/2
    g.target_id = "ground_2"
    g.point_plan = plan_ground_leg(None, "ground_2", (0., 0., 0.))
    operation = next(op for op in g.point_plan.entry_operations if op.kind == "DRIVE_REVERSE_TO_XY")
    g._load_operations((operation,), "entry", 0.)
    return g


def node_fixture():
    patch = pytest.MonkeyPatch()
    callbacks, _, _ = install_ros_transport(patch)
    class Worker:
        def cancel_context(self):
            pass
        def poll(self):
            return []
    wall = [0.]
    node = api.create_axis_three_point_node(wall_clock=lambda: wall[0], worker=Worker())
    node.arm_watchdog = lambda: None
    prepare_reverse(node.guard)
    sent = []
    node.publisher_by_topic["/cmd_vel"].publish = lambda msg: sent.append((msg.linear.x, msg.angular.z))
    wall[0] = .05
    callbacks["/clock"]({"clock": {"sec": 10, "nanosec": 50_000_000}})
    packet = posed_odom(10.05, x=0., yaw=math.pi/2)
    packet["pose"]["pose"]["position"]["y"] = -1.801
    packet["twist"]["twist"]["linear"]["x"] = -.02
    callbacks["/odom"](packet)
    callbacks["/scan"](scan(10.05))
    return node, wall, callbacks, sent, patch


def signed_factory_callback_tick():
    node, wall, callbacks, sent, patch = node_fixture()
    try:
        decision = node.tick()
        assert node.single_point_coordinator.guard is node.guard
        assert decision.linear_x == -.03 and sent[-1] == (-.03, 0.)
        assert abs(node.guard.measured_forward() - .001) < 1e-12
        assert abs(node.guard.drive_yaw - math.pi/2) < 1e-12
        assert node.guard._observations.actual_commands[-1]["linear_x"] == -.03
        return {"pass": True, "published": sent, "progress_m": node.guard.measured_forward(),
                "body_yaw_rad": node.guard.drive_yaw}
    finally:
        node.close_watchdog(); patch.undo()


def dds_lost_during_later_graph_queries():
    node, wall, callbacks, sent, patch = node_fixture()
    state = {"actual_match": 1, "armed": True}
    query = node.get_subscriptions_info_by_topic
    def subscriptions(topic):
        result = query(topic)
        if topic != "/cmd_vel" and state["armed"]:
            state["actual_match"] = 0
        return result
    node.publisher_by_topic["/cmd_vel"].get_subscription_count = lambda: state["actual_match"]
    node.get_subscriptions_info_by_topic = subscriptions
    node.last_graph_refresh_wall = None
    try:
        decision = node.tick()
        observed = {"actual_match_at_publish": state["actual_match"], "published": sent,
                    "guard_cached_match": node.guard.matched_subscriptions,
                    "state": node.guard.state, "reason": node.guard.reason}
        return {"pass": all(v == 0. for v, _ in sent) and node.guard.state == "STOP",
                "observed": observed,
                "requirement": "actual loss during graph refresh must be rechecked before nonzero output"}
    finally:
        node.close_watchdog(); patch.undo()


def negative_publish_exception_zero_attempt():
    node, wall, callbacks, sent, patch = node_fixture()
    def publish(msg):
        sent.append((msg.linear.x, msg.angular.z))
        if msg.linear.x < 0:
            raise RuntimeError("independent negative publish failure")
    node.publisher_by_topic["/cmd_vel"].publish = publish
    escaped = None
    try:
        try:
            node.tick()
        except Exception as exc:
            escaped = type(exc).__name__ + ":" + str(exc)
        return {"pass": node.guard.state == "STOP" and any(v == w == 0. for v, w in sent),
                "observed": {"escaped_exception": escaped, "published_attempts": sent,
                             "state": node.guard.state, "reason": node.guard.reason},
                "requirement": "negative publish failure must lock stop and independently attempt zero"}
    finally:
        node.close_watchdog(); patch.undo()


def reverse_settle_reaim_and_flag_clear():
    g = api.GroundThreePointAxisGuard(started_wall_s=0.)
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    g.target_id = "ground_2"
    g.point_plan = plan_ground_leg(None, "ground_2", (0., 0., 0.))
    reverse = next(op for op in g.point_plan.entry_operations if op.kind == "DRIVE_REVERSE_TO_XY")
    turn = GroundOperation("TURN_ONLY", None, (0.,-1.8), 0., math.pi/2,
                           math.pi/2, "independent_reverse_turn", .7)
    refresh(g, .05, x=0., y=-1.8, yaw=0.)
    g._load_operations((turn, reverse), "entry", .05)
    assert g.target_yaw == math.pi/2 and g.step(wall_s=.05).angular_z > 0
    refresh(g, .1, x=0., y=-1.8, yaw=math.pi/2)
    assert g.phase == "TURN_SETTLE"
    for i in range(3, 29):
        wall = i*.05
        refresh(g, wall, x=.008, y=-1.8, yaw=math.pi/2)
        decision = g.step(wall_s=wall)
        if g.phase == "TURN":
            break
    expected = math.pi/2 - math.atan2(.008, .89)
    assert g.phase == "TURN" and abs(g.target_yaw-expected) < 1e-12
    assert g.initial_turn_error < 0 and g.settle_reaim_count == 1
    prepare_reverse(g := api.GroundThreePointAxisGuard(started_wall_s=0.))
    assert g.reverse_segment
    forward = GroundOperation("DRIVE_TO_XY", (0.,-1.7), (0.,-1.8), math.pi/2,
                              math.pi/2, 0., "independent_forward", .7, .1)
    g._load_operations((forward,), "observation", 0.)
    assert not g.reverse_segment and g._linear_request(0.) > 0
    return {"pass": True, "reaim_target_rad": expected, "forward_request_positive": True}


def alternate_order_visual_epoch_and_ledger():
    from test_ground_three_point_axis import AxisRun
    run = AxisRun(("ground_1", "ground_2", "ground_4"))
    epochs, snapshots, sends = [], [], []
    for index, target in enumerate(("ground_1", "ground_2", "ground_4")):
        sends += run.recognise()
        epochs.append(run.guard.stationary_epoch)
        stale = deepcopy(run.guard.match_results[0])
        sends += run.declare()
        for operation in run.guard.point_plan.to_observation_operations + run.guard.operations:
            if operation.target_xy_m is not None:
                dx = operation.target_xy_m[0] - operation.anchor_xy_m[0]
                dy = operation.target_xy_m[1] - operation.anchor_xy_m[1]
                assert abs(dx) < 1e-12 or abs(dy) < 1e-12
        ledger = {"origin": run.guard.global_origin, "started": run.guard.started_wall,
                  "path": run.guard.total_path_m, "rotation": run.guard.accumulated_rotation_rad,
                  "turn_wall": run.guard._elapsed_group("turn", run.wall),
                  "drive_wall": run.guard._elapsed_group("drive", run.wall)}
        snapshots.append(ledger)
        if index < 2:
            run.until(lambda r: r.guard.point_index == index + 1)
            assert run.guard.global_origin == ledger["origin"] and run.guard.started_wall == ledger["started"]
            assert run.guard.total_path_m >= ledger["path"]
            assert run.guard.accumulated_rotation_rad >= ledger["rotation"]
            assert run.guard._elapsed_group("turn", run.wall) >= ledger["turn_wall"]
            assert run.guard._elapsed_group("drive", run.wall) >= ledger["drive_wall"]
            assert not run.guard.accept_match_result(stale, wall_s=run.wall)
            assert run.guard.recognition is None
    run.until(lambda r: r.guard.state == "STOP")
    assert [value for topic, value in sends if topic == "/referee/current_target"] == ["ground_1", "ground_2", "ground_4"]
    assert len(set(epochs)) == 3 and epochs == sorted(epochs)
    return {"pass": True, "epochs": epochs, "ledger_at_points": snapshots,
            "reason": run.guard.reason, "actual_reverse_on_platform": "unverified"}


def main():
    probes = {}
    for name, fn in (("signed_factory_callback_tick", signed_factory_callback_tick),
                     ("dds_lost_during_later_graph_queries", dds_lost_during_later_graph_queries),
                     ("negative_publish_exception_zero_attempt", negative_publish_exception_zero_attempt),
                     ("reverse_settle_reaim_and_flag_clear", reverse_settle_reaim_and_flag_clear),
                     ("alternate_order_visual_epoch_and_ledger", alternate_order_visual_epoch_and_ledger)):
        try:
            probes[name] = fn()
        except Exception as exc:
            probes[name] = {"pass": False, "probe_error": type(exc).__name__ + ":" + str(exc)}
    source = ROOT / "src/inspection_mission/inspection_mission/ground_three_point_axis.py"
    result = {"source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "scope": "independent local software probes only", "probes": probes}
    (Path(__file__).resolve().parent / "independent_metrics.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
