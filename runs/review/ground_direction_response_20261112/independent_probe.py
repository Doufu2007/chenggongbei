"""Independent counterexamples, not an author-suite rerun or platform proof."""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "inspection_mission"))
sys.path.insert(0, str(ROOT / "tests"))

import pytest
from inspection_mission import ground_direction_response as api
from test_bounded_ground_probe import endpoint, install_ros_transport, scan
from test_ground_observation_survey import posed_odom


def packet(stamp, x, y, yaw, speed=0., yaw_speed=0., scale=1.):
    msg = posed_odom(stamp, x=x, yaw=yaw)
    msg["pose"]["pose"]["position"]["y"] = y
    msg["twist"]["twist"]["linear"]["x"] = speed
    msg["twist"]["twist"]["angular"]["z"] = yaw_speed
    for k in "xyzw":
        msg["pose"]["pose"]["orientation"][k] *= scale
    return msg


class Fixture:
    def __init__(self):
        self.g = api.GroundDirectionResponseGuard(started_wall_s=0., session_id="independent-review")
        self.g.update_endpoints([endpoint()], matched_subscriptions=1)
        self.t = self.x = self.y = self.yaw = 0.
        self.d = None

    def feed(self, *, dx=0., dy=0., yaw_delta=0., scale=1., integrate=True):
        dt = .05
        vx = self.d.linear_x * .2 if self.d and integrate else 0.
        vy = self.d.linear_y * .2 if self.d and integrate else 0.
        wz = self.d.angular_z * .15 if self.d and integrate else 0.
        self.x += dt * (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) + dx
        self.y += dt * (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) + dy
        self.yaw += dt * wz + yaw_delta
        self.t = round(self.t + dt, 9)
        stamp = round(10. + self.t, 9)
        self.g.update_clock(stamp, wall_s=self.t)
        self.g.update_odom(packet(stamp, self.x, self.y, self.yaw,
            math.hypot(vx, vy), max(abs(wz), abs(yaw_delta) / dt), scale), wall_s=self.t)
        self.g.update_scan(scan(stamp), wall_s=self.t)
        self.d = self.g.step(wall_s=self.t)
        return self.d

    def first_pulse(self):
        for _ in range(2000):
            self.feed()
            if self.d.linear_x:
                return self
            assert self.g.state not in ("STOP", "FINISHED"), self.g.reason
        raise AssertionError("independent fixture did not reach first pulse")


def actual_callback_tick_xy():
    patch = pytest.MonkeyPatch()
    callbacks, _, _ = install_ros_transport(patch)
    now = [0.]
    node = api.create_direction_response_node(wall_clock=lambda: now[0])
    # Advance a deterministic fake clock synchronously; watchdog timing has a
    # separate real-clock transport test and is not simulated by this probe.
    node.arm_watchdog = lambda: None
    rows = []
    node.publisher_by_topic["/cmd_vel"].publish = lambda m: rows.append((m.linear.x, m.linear.y, m.angular.z))
    x = y = yaw = 0.
    try:
        for i in range(1, 2401):
            vx, vy, wz = rows[-1] if rows else (0., 0., 0.)
            vx *= .2
            vy *= .2
            wz *= .15
            x += .05 * (vx * math.cos(yaw) - vy * math.sin(yaw))
            y += .05 * (vx * math.sin(yaw) + vy * math.cos(yaw))
            yaw += .05 * wz
            now[0] = round(.05 * i, 9)
            stamp = round(10. + now[0], 9)
            sec = int(stamp)
            callbacks["/clock"]({"clock": {"sec": sec, "nanosec": round((stamp-sec)*1e9)}})
            callbacks["/odom"](packet(stamp, x, y, yaw, math.hypot(vx, vy), abs(wz)))
            callbacks["/scan"](scan(stamp))
            node.tick()
            if node.guard.state == "STOP":
                break
        recorded = [(r["linear_x"], r["linear_y"], r["angular_z"])
                    for r in node.guard.direction_commands if r["sent"]]
        assert any(y == .03 for _, y, _ in rows), "real tick never emitted the Y pulse"
        assert rows == recorded, "actual transport and evidence diverge"
        assert node.guard.reason == "direction_response_observed_official_score_unverified", node.guard.reason
        return {"commands": len(rows), "y_commands": sum(y != 0. for _, y, _ in rows),
                "reason": node.guard.reason}
    finally:
        node.close_watchdog()
        patch.undo()


def harmless_changed_duplicate():
    f = Fixture().first_pulse()
    msg = copy.deepcopy(f.g.last_raw_scan)
    msg["ranges"][4] += .01
    f.g.update_scan(msg, wall_s=f.t)
    d = f.g.step(wall_s=f.t)
    assert f.g.state == "STOP" and d.linear_x == d.linear_y == 0.
    return {"reason": f.g.reason}


def watchdog_zero_without_tick():
    f = Fixture().first_pulse()
    patch = pytest.MonkeyPatch()
    install_ros_transport(patch)
    now = [f.t]
    node = api.create_direction_response_node(wall_clock=lambda: now[0])
    node.guard = f.g
    rows = []
    stopped = threading.Event()
    def publish(msg):
        rows.append((msg.linear.x, msg.linear.y, msg.angular.z))
        if len(rows) >= 2 and rows[-1] == (0., 0., 0.):
            stopped.set()
    node.publisher_by_topic["/cmd_vel"].publish = publish
    try:
        node.publish_direction(f.d)
        now[0] = round(f.t + .21, 9)
        assert stopped.wait(.3), "watchdog did not independently publish zero"
        assert rows[0][0] == .03 and rows[-1] == (0., 0., 0.)
        assert node.guard.state in ("STOP", "PAUSE"), node.guard.state
        return {"reason": node.guard.reason, "state": node.guard.state, "commands": rows}
    finally:
        node.close_watchdog()
        patch.undo()


def dangerous_duplicate_odom_vector():
    f = Fixture().first_pulse()
    msg = copy.deepcopy(f.g.last_raw_odom)
    msg["twist"]["twist"]["linear"]["y"] = .151
    f.g.update_odom(msg, wall_s=f.t)
    d = f.g.step(wall_s=f.t)
    assert f.g.state == "STOP" and d.linear_x == d.linear_y == 0.
    return {"reason": f.g.reason}


def zero_stop_oscillation():
    f = Fixture().first_pulse()
    f.feed(dx=.0101, integrate=False)
    assert f.g.phase == "PULSE_SETTLE", f.g.reason
    origin = f.g.total_path_m
    net_start = (f.x, f.y)
    for delta in (.004, -.004, .004):
        f.feed(dx=delta, integrate=False)
    assert math.dist(net_start, (f.x, f.y)) < .01
    assert f.g.state == "STOP", "net displacement hid >1cm post-zero accumulated motion"
    return {"reason": f.g.reason, "net_m": math.dist(net_start, (f.x, f.y)),
            "path_increment_m": f.g.total_path_m-origin}


def quaternion_scale_cannot_hide_yaw_drift():
    f = Fixture().first_pulse()
    start_yaw = f.yaw
    for _ in range(5):
        f.feed(yaw_delta=.03001/5, scale=1.0009, integrate=False)
    true_drift = f.yaw-start_yaw
    assert .03 < true_drift < .03002
    assert f.g.state == "STOP", "accepted quaternion scale hid true yaw drift above .03rad"
    return {"reason": f.g.reason, "true_yaw_drift_rad": true_drift}


def response_scale_and_translation_invariance():
    a = packet(10., 0., 0., -.6)
    b = packet(11., .01, .02, -.58)
    baseline = api.response_from_odom(a, b)
    c, d = copy.deepcopy(a), copy.deepcopy(b)
    for raw, scale in ((c, .9991), (d, 1.0009)):
        raw["pose"]["pose"]["position"]["x"] += 10.
        raw["pose"]["pose"]["position"]["y"] -= 5.
        for key in "xyzw":
            raw["pose"]["pose"]["orientation"][key] *= scale
    measured = api.response_from_odom(c, d)
    for key in ("horizontal_displacement_m", "start_yaw_rad", "yaw_delta_rad",
                "displacement_bearing_rad", "bearing_minus_start_yaw_rad"):
        assert math.isclose(baseline[key], measured[key], rel_tol=0., abs_tol=1e-12), key
    return {"yaw_delta_rad": measured["yaw_delta_rad"], "displacement_m": measured["horizontal_displacement_m"]}


def main():
    source = ROOT/"src/inspection_mission/inspection_mission/ground_direction_response.py"
    report = {"source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "probes": {}}
    for test in (actual_callback_tick_xy, watchdog_zero_without_tick,
                 harmless_changed_duplicate, dangerous_duplicate_odom_vector,
                 zero_stop_oscillation, quaternion_scale_cannot_hide_yaw_drift,
                 response_scale_and_translation_invariance):
        try:
            report["probes"][test.__name__] = {"pass": True, "observed": test()}
        except Exception as exc:
            report["probes"][test.__name__] = {"pass": False,
                "error": f"{type(exc).__name__}: {exc}"}
    out = Path(__file__).with_name("independent_metrics.json")
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return int(any(not value["pass"] for value in report["probes"].values()))


if __name__ == "__main__":
    raise SystemExit(main())
