"""Independent v33 local adapter probes; never real platform evidence."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import math
import sys
import threading

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/inspection_mission"))
sys.path.insert(0, str(ROOT / "tests"))
import pytest
from inspection_mission import ground_relocated_reverse as api
from test_bounded_ground_probe import endpoint, install_ros_transport, scan
from test_ground_observation_survey import posed_odom


class Adapter:
    def __init__(self):
        self.patch = pytest.MonkeyPatch()
        self.callbacks, _, _ = install_ros_transport(self.patch)
        self.wall = 0.
        self.node = api.create_relocated_reverse_node(wall_clock=lambda: self.wall)
        self.guard = self.node.guard
        self.node.arm_watchdog = lambda: None
        self.sent, self.other = [], []
        self.node.publisher_by_topic["/cmd_vel"].publish = lambda m: self.sent.append((m.linear.x, m.linear.y, m.angular.z))
        for topic, publisher in self.node.publisher_by_topic.items():
            if topic != "/cmd_vel":
                publisher.publish = lambda m, topic=topic: self.other.append(topic)
        self.x = self.y = self.yaw = 0.

    def feed(self, *, source=None, speed=0., angular_speed=0., ranges=None):
        source = round(10. + self.wall, 9) if source is None else source
        sec = int(source)
        self.callbacks["/clock"]({"clock": {"sec": sec, "nanosec": round((source-sec)*1e9)}})
        msg = posed_odom(source, x=self.x, yaw=self.yaw)
        msg["pose"]["pose"]["position"]["y"] = self.y
        msg["twist"]["twist"]["linear"]["x"] = speed
        msg["twist"]["twist"]["angular"]["z"] = angular_speed
        self.callbacks["/odom"](msg)
        laser = scan(source)
        if ranges is not None:
            laser["ranges"] = ranges
        self.callbacks["/scan"](laser)

    def close(self):
        self.node.close_watchdog()
        self.patch.undo()

    def active(self, phase):
        if phase == "PREPOSITION":
            self.yaw = -math.pi/2
        elif phase == "PULSE":
            self.yaw, self.y = math.pi/2, -.15
        self.feed()
        g = self.guard
        g.global_origin = g.pulse_origin = (0., 0., .055)
        g.previous_path_xyz, g.previous_yaw = g.odom["xyz"], self.yaw
        g._derived_previous_yaw = self.yaw
        if phase == "TURN":
            g._begin_turn(-math.pi/2, 0., 0)
        elif phase == "PREPOSITION":
            g.accumulated_rotation_rad = math.pi/2
            g._begin_preposition(0.)
        else:
            g.accumulated_rotation_rad, g.total_path_m, g.stage_index = 4.65, .15, 3
            g._begin_pulse(0.)
        self.wall = .05
        self.feed()
        self.node.last_graph_refresh_wall = self.wall
        return self


def full_factory_callback_sequence():
    a = Adapter()
    snapshots, last_snapshot = [], None
    try:
        for i in range(1, 3601):
            x, y, w = a.sent[-1] if a.sent else (0., 0., 0.)
            x, y, w = x*.2, y*.2, w*.15
            a.x += .05*(x*math.cos(a.yaw)-y*math.sin(a.yaw))
            a.y += .05*(x*math.sin(a.yaw)+y*math.cos(a.yaw))
            a.yaw += .05*w
            a.wall = round(i*.05, 9)
            a.feed(speed=abs(x), angular_speed=abs(w))
            a.node.tick()
            g = a.guard
            assert a.node.guard is g
            if g.global_origin is not None:
                assert g.global_origin == (0., 0., .055)
            assert g.started_wall == 0.
            current = (g.stage_index, g.phase)
            if current != last_snapshot:
                snapshots.append({"wall_s": a.wall, "stage": g.stage_index, "phase": g.phase,
                    "path_m": g.total_path_m, "rotation_rad": g.accumulated_rotation_rad,
                    "total_turn_wall_s": g.total_turn_elapsed_s(a.wall), "origin": g.global_origin})
                last_snapshot = current
            if g.state == "STOP":
                break
        assert g.reason == "relocated_reverse_response_observed_official_score_unverified"
        assert not a.other
        assert all(later["path_m"] >= earlier["path_m"] and later["rotation_rad"] >= earlier["rotation_rad"]
                   and later["total_turn_wall_s"] >= earlier["total_turn_wall_s"]
                   for earlier, later in zip(snapshots, snapshots[1:]))
        assert [r["target_yaw_rad"] for r in g.turn_records] == [-math.pi/2, 0., math.pi/2]
        assert len(g.pulse_records) == 3
        assert all(min(r["settled"]["stable_wall_s"], r["settled"]["stable_source_s"]) >= 1.-1e-9 for r in g.turn_records)
        assert all(min(r["settled"]["stable_wall_s"], r["settled"]["stable_source_s"]) >= 1.-1e-9 for r in g.pulse_records)
        assert .148-1e-9 <= g.preposition_record["settled"]["south_progress_m"] <= .17
        assert g.preposition_record["settled"]["endpoint_error_m"] <= .02
        assert min(g.preposition_record["settled"]["stable_wall_s"], g.preposition_record["settled"]["stable_source_s"]) >= 1.-1e-9
        assert a.sent == [(r["linear_x"], r["linear_y"], r["angular_z"]) for r in g.direction_commands if r["sent"]]
        assert {r["pulse_index"] for r in g.direction_commands if r["linear_x"] < 0.} == {0, 1, 2}
        assert a.node.experiment_mode == g.mode == "ground_relocated_reverse"
        return {"pass": True, "same_factory_guard": True, "other_publishers": a.other,
            "stages": snapshots, "preposition": g.preposition_record,
            "turn_targets_rad": [r["target_yaw_rad"] for r in g.turn_records],
            "pulse_requests": [r["requested_body_xy_mps"] for r in g.pulse_records],
            "final_wall_s": a.wall, "path_m": g.total_path_m, "rotation_rad": g.accumulated_rotation_rad,
            "total_turn_wall_s": g.total_turn_elapsed_s(a.wall), "reason": g.reason}
    finally:
        a.close()


def final_query_gate(kind):
    phase = "PREPOSITION" if kind == "preposition" else "PULSE" if kind == "pulse" else "TURN"
    a = Adapter().active(phase)
    g, calls = a.guard, []
    if kind == "total_turn":
        g.completed_turn_wall_s = 119.90
    deadlines = {"global": 180., "turn": 45., "total_turn": .10, "preposition": 40., "pulse": 2.}
    def matches():
        calls.append(a.wall)
        if len(calls) == 2:
            if kind in deadlines:
                a.wall = deadlines[kind]
            elif kind == "source_age":
                a.callbacks["/clock"]({"clock": {"sec": 10, "nanosec": 400_000_000}})
            elif kind == "arrival_age":
                a.wall = .30
            elif kind == "stop":
                g.stop("independent_stop_during_dds", a.wall)
            elif kind == "dds_loss":
                return 0
        return 1
    a.node.publisher_by_topic["/cmd_vel"].get_subscription_count = matches
    try:
        d = a.node.tick()
        assert len(calls) == 2
        assert a.sent == [(0., 0., 0.)]
        assert d.linear_x == d.linear_y == d.angular_z == 0.
        assert g.state in ("STOP", "PAUSE") or g.phase == "PULSE_SETTLE"
        return {"pass": True, "query_count": len(calls), "published": a.sent,
            "state": g.state, "phase": g.phase, "reason": g.reason,
            "timing": g.timing_snapshot(wall_s=a.wall)}
    finally:
        a.close()


def preposition_endpoint_geometry():
    a = Adapter().active("PREPOSITION")
    try:
        g = a.guard
        target, start = g.preposition_target, g.preposition_origin
        # No motion for one second must leave the forward request active.
        for i in range(2, 23):
            a.wall = round(i*.05, 9)
            a.feed()
            d = a.node.tick()
            assert g.phase == "PREPOSITION" and d.linear_x == .03
        # Feed actual south progress in <=1cm increments with new scans.
        for i in range(1, 16):
            a.wall = round(a.wall+.05, 9)
            a.y = start[1] - min(i*.01, .148)
            a.x = start[0] + .01995
            a.feed(speed=.006)
            d = a.node.tick()
        assert g.phase == "PREPOSITION_SETTLE" and d.linear_x == 0.
        assert abs(g.preposition_progress_m()-.148) < 1e-12
        # Lateral error stays individually within .02m, but the actual endpoint
        # Euclidean error exceeds .02m when combined with the 2mm shortfall.
        for _ in range(24):
            a.wall = round(a.wall+.05, 9)
            a.feed()
            a.node.tick()
            if g.state == "STOP":
                break
        assert math.dist(g.odom["xyz"], target) > .02
        assert g.state == "STOP" and g.reason == "relocated_preposition_endpoint_error"
        return {"pass": True, "target": target, "progress_m": g.preposition_progress_m(),
            "endpoint_error_m": math.dist(g.odom["xyz"], target), "reason": g.reason}
    finally:
        a.close()


def negative_publish_failure_and_real_watchdog():
    a = Adapter().active("PULSE")
    attempts, recovered = [], threading.Event()
    def publish(msg):
        attempts.append((msg.linear.x, msg.linear.y, msg.angular.z))
        if len(attempts) <= 2:
            raise RuntimeError("independent first nonzero and zero transport failures")
        recovered.set()
    a.node.publisher_by_topic["/cmd_vel"].publish = publish
    try:
        a.node.tick()
        assert attempts == [(-.03, 0., 0.), (0., 0., 0.)]
        assert a.guard.state == "STOP" and a.guard.published_nonzero_count == 0
        # Explicitly arm the actual inherited watchdog, disabled only while
        # checking the synchronous tick's independent fallback above.
        type(a.node).arm_watchdog(a.node)
        assert recovered.wait(timeout=1.)
        a.node.close_watchdog()
        assert attempts[2] == (0., 0., 0.) and a.guard.published_zero_count >= 1
        return {"pass": True, "attempts": attempts, "state": a.guard.state,
            "command_records": [{k:r[k] for k in ("linear_x", "linear_y", "angular_z", "sent", "error")} for r in a.guard.direction_commands]}
    finally:
        a.close()


def main():
    source = ROOT / "src/inspection_mission/inspection_mission/ground_relocated_reverse.py"
    launcher = ROOT / "scripts/run_ground_relocated_reverse.sh"
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    cases = [("full_factory_callback_sequence", full_factory_callback_sequence),
        ("preposition_endpoint_geometry", preposition_endpoint_geometry),
        ("negative_publish_failure_and_real_watchdog", negative_publish_failure_and_real_watchdog)]
    cases += [("final_query_"+kind, lambda kind=kind: final_query_gate(kind)) for kind in
        ("global", "turn", "total_turn", "preposition", "pulse", "source_age", "arrival_age", "stop", "dds_loss")]
    probes = {}
    for name, fn in cases:
        try:
            probes[name] = fn()
        except Exception as exc:
            probes[name] = {"pass": False, "error": type(exc).__name__+":"+str(exc)}
    after = hashlib.sha256(source.read_bytes()).hexdigest()
    manifest_path = ROOT / "runs/review/ground_relocated_reverse_20261112/freeze_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    frozen = [{"path": name, "expected_sha256": row["sha256"],
               "actual_sha256": hashlib.sha256((ROOT/name).read_bytes()).hexdigest()}
              for section in ("new_files", "frozen_files") for name, row in manifest[section].items()]
    frozen_match = all(row["actual_sha256"] == row["expected_sha256"] for row in frozen)
    result = {"reviewed_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "独立本地软件 adapter 探针；无平台运动、地图或官方计分证据",
        "source_sha256": before, "source_unchanged_during_review": before == after,
        "launcher_sha256": hashlib.sha256(launcher.read_bytes()).hexdigest(),
        "manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "frozen_files": frozen, "all_manifest_files_match": frozen_match,
        "pass_count": sum(bool(r.get("pass")) for r in probes.values()), "probe_count": len(cases), "probes": probes}
    (Path(__file__).parent / "independent_metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"source_sha256": before, "pass_count": result["pass_count"],
        "probe_count": len(cases), "probes": {name: {k:v for k,v in row.items() if k in ("pass", "error", "reason")} for name,row in probes.items()}}, indent=2))
    return 0 if result["pass_count"] == len(cases) and before == after and frozen_match else 1


if __name__ == "__main__":
    sys.exit(main())
