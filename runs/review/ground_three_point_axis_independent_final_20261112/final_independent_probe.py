"""Independent final adapter review; local fake ROS transport only."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[3]
ORIGINAL = ROOT / "runs/review/ground_three_point_axis_independent_20261112/independent_probe.py"
spec = importlib.util.spec_from_file_location("original_axis_independent_probe", ORIGINAL)
original = importlib.util.module_from_spec(spec)
spec.loader.exec_module(original)


def final_query_gate(kind):
    node, wall, callbacks, sent, patch = original.node_fixture()
    calls = []
    node.last_graph_refresh_wall = wall[0]

    def matches():
        calls.append(wall[0])
        if len(calls) == 2:
            if kind == "wall_stale":
                wall[0] = .30
            elif kind == "global_budget":
                wall[0] = 1200.
            elif kind == "phase_stop":
                node.guard.stop("independent_phase_stop_during_final_query", wall[0])
            elif kind == "source_age":
                callbacks["/clock"]({"clock": {"sec": 10, "nanosec": 400_000_000}})
        return 1

    node.publisher_by_topic["/cmd_vel"].get_subscription_count = matches
    try:
        decision = node.tick()
        timing = node.guard.timing_snapshot(wall_s=wall[0])
        assert len(calls) == 2
        assert sent and all(x == w == 0. for x, w in sent)
        assert decision.linear_x == decision.angular_z == 0.
        assert node.guard.state in ("STOP", "PAUSE")
        if kind == "source_age":
            assert timing["odom"]["wall_age_s"] == 0.
            assert timing["odom"]["clock_minus_stamp_s"] > node.guard.limits.sensor_age_s
            assert not timing["odom"]["fresh"]
        return {"pass": True, "query_calls": len(calls), "published": sent,
                "state": node.guard.state, "phase": node.guard.phase,
                "reason": node.guard.reason, "timing": timing}
    finally:
        node.close_watchdog()
        patch.undo()


def real_watchdog_survives_failed_zero():
    node, wall, callbacks, sent, patch = original.node_fixture()
    recovered = threading.Event()
    zero_attempts = []

    def publish(msg):
        sent.append((msg.linear.x, msg.angular.z))
        assert msg.linear.x == msg.angular.z == 0.
        zero_attempts.append(len(zero_attempts) + 1)
        if len(zero_attempts) == 1:
            raise RuntimeError("independent first watchdog zero failure")
        recovered.set()

    node.publisher_by_topic["/cmd_vel"].publish = publish
    node.guard.stop("independent_watchdog_stopped_fixture", wall[0])
    try:
        # The original probe disables watchdog for synchronous boundary checks.
        # Invoke the real inherited implementation explicitly in this probe.
        type(node).arm_watchdog(node)
        assert recovered.wait(timeout=1.)
        node.close_watchdog()
        assert len(zero_attempts) >= 2
        assert node.guard.published_zero_count >= 1
        assert node.axis_publish_attempts[0]["returned"] is False
        assert node.axis_publish_attempts[1]["returned"] is True
        return {"pass": True, "published_attempts": sent,
                "attempt_records": node.axis_publish_attempts,
                "state": node.guard.state, "reason": node.guard.reason}
    finally:
        node.close_watchdog()
        patch.undo()


def factory_guard_real_coordinator_dispatch():
    from test_ground_three_point_axis import AxisRun
    from test_ground_single_point import camera_info, fixture_jobs

    queued, contexts = [], []

    class Worker:
        def cancel_context(self):
            pass
        def set_context(self, *args):
            contexts.append(args)
        def poll(self):
            answer = list(queued)
            queued.clear()
            return answer

    patch = original.pytest.MonkeyPatch()
    original.install_ros_transport(patch)
    run = AxisRun()
    node = original.api.create_axis_three_point_node(wall_clock=lambda: run.wall, worker=Worker())
    factory_guard = node.guard
    node.arm_watchdog = lambda: None
    try:
        # Drive the factory-created guard with the established synthetic feedback
        # harness; the node/coordinator references themselves are never replaced.
        run.guard = factory_guard
        factory_guard.session_id = "test-current-session"
        factory_guard.update_endpoints([original.endpoint()], matched_subscriptions=1)
        factory_guard.observe_public("/referee/ground_tasks", {"data": "ground_0,ground_3,ground_2"}, wall_s=0.)
        factory_guard.observe_public("/uav/camera/camera_info", camera_info(), wall_s=0.)
        run.until(lambda r: r.guard.state == "RECOGNIZE_WAIT", cap=25000)
        _, results = fixture_jobs(run)
        queued.extend(results)
        node.single_point_coordinator.prepare = lambda frames, **kwargs: []
        sent = []
        node.publisher_by_topic["/inspection/detections"].publish = lambda msg: sent.append(msg.data)
        node.tick()
        assert node.guard is factory_guard
        assert node.single_point_coordinator.guard is factory_guard
        assert contexts == [(factory_guard.session_id, "ground_0", factory_guard.stationary_epoch)]
        assert factory_guard.attempts["recognition"]["sent"]
        assert factory_guard.phase == "REPORT_WAIT" and len(sent) == 1
        node.refresh_endpoints()
        node.tick()
        endpoint = node.command_endpoints["/inspection/detections"]
        assert len(sent) == 1
        assert endpoint["actual_publish_attempt_count"] == endpoint["actual_publish_return_count"] == 1
        for topic in ("/uav/cmd_vel", "/uav/laser_fire", "/referee/cruise_event"):
            assert node.command_endpoints[topic]["silent"] is True
        return {"pass": True, "same_factory_guard": True, "contexts": contexts,
                "published_strings": sent, "phase": factory_guard.phase,
                "attempted": endpoint["actual_publish_attempt_count"],
                "returned": endpoint["actual_publish_return_count"],
                "other_outputs_silent": True}
    finally:
        node.close_watchdog()
        if node.single_point_coordinator.preparer is not None:
            node.single_point_coordinator.preparer.join(timeout=1.)
        patch.undo()


def main():
    manifest_path = ROOT / "runs/review/ground_three_point_axis_runtime_20261112/freeze_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    frozen = [{"path": item["path"], "expected_sha256": item["sha256"],
               "actual_sha256": hashlib.sha256((ROOT / item["path"]).read_bytes()).hexdigest()}
              for item in manifest["files"]]
    assert all(row["expected_sha256"] == row["actual_sha256"] for row in frozen)
    cases = [(name, getattr(original, name)) for name in (
        "signed_factory_callback_tick", "dds_lost_during_later_graph_queries",
        "negative_publish_exception_zero_attempt", "reverse_settle_reaim_and_flag_clear",
        "alternate_order_visual_epoch_and_ledger")]
    cases += [("final_query_" + kind, lambda kind=kind: final_query_gate(kind))
              for kind in ("wall_stale", "global_budget", "phase_stop", "source_age")]
    cases += [("real_watchdog_survives_failed_zero", real_watchdog_survives_failed_zero),
              ("factory_guard_real_coordinator_dispatch", factory_guard_real_coordinator_dispatch)]
    probes = {}
    for name, fn in cases:
        try:
            probes[name] = fn()
        except Exception as exc:
            probes[name] = {"pass": False, "probe_error": type(exc).__name__ + ":" + str(exc)}
    result = {"reviewed_at_utc": datetime.now(timezone.utc).isoformat(),
              "source_sha256": frozen[0]["actual_sha256"],
              "launcher_sha256": frozen[1]["actual_sha256"],
              "original_probe_sha256": hashlib.sha256(ORIGINAL.read_bytes()).hexdigest(),
              "frozen_files": frozen, "scope": "独立本地软件审查；模拟 ROS transport；无平台倒车或官方计分证据",
              "original_probe_count": 5, "supplemental_probe_count": 6,
              "pass_count": sum(bool(value.get("pass")) for value in probes.values()), "probes": probes}
    (Path(__file__).parent / "final_independent_metrics.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["pass_count"] == len(cases) else 1


if __name__ == "__main__":
    sys.exit(main())
