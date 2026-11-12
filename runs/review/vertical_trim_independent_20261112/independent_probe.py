"""Independent simulated-transport counterexamples; no official motion claim."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src/inspection_mission"), str(ROOT / "tests")]
import pytest
from inspection_mission import bounded_vertical_trim as api
from test_bounded_ground_probe import install_ros_transport, scan
from test_bounded_vertical_response import odom, tf_static


class Transport:
    def __init__(self):
        self.patch = pytest.MonkeyPatch()
        self.callbacks, _, _ = install_ros_transport(self.patch)
        class Twist:
            def __init__(self):
                from types import SimpleNamespace
                self.linear = SimpleNamespace(x=0., y=0., z=0.)
                self.angular = SimpleNamespace(x=0., y=0., z=0.)
        self.patch.setattr(sys.modules["rosidl_runtime_py.utilities"], "get_message", lambda _: Twist)
        self.now = [0.]
        self.node = api.create_trim_node(wall_clock=lambda: self.now[0])
        self.rows = []
        for topic, pub in self.node.publisher_by_topic.items():
            pub.publish = lambda m, topic=topic: self.rows.append((topic,
                (m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z)))
        self.callbacks["/tf_static"](tf_static())

    def feed(self, t, z=.055, vz=0., tick=True):
        self.now[0] = t
        stamp = round(10.+t, 9)
        sec = int(stamp)
        self.callbacks["/clock"]({"clock": {"sec": sec, "nanosec": round((stamp-sec)*1e9)}})
        self.callbacks["/uav/odom"](odom(stamp, z=z, vz=vz))
        self.callbacks["/odom"](odom(stamp, child="base_footprint"))
        self.callbacks["/scan"](scan(stamp))
        return self.node.tick() if tick else None

    def enter(self):
        for i in range(23): self.feed(i*.05)
        self.feed(1.15, .155)
        assert self.node.guard.phase == "TRIM_POS_025"
        assert self.rows[-1][1][2] == .025
        return self

    def negative(self):
        self.enter()
        for i in range(24, 64): self.feed(i*.05, .155)
        assert self.node.guard.phase == "TRIM_NEG_010", self.node.guard.phase
        assert self.rows[-1][1][2] == -.010
        return self

    def close(self):
        self.node.close_watchdog()
        self.patch.undo()


def actual_full_protocol():
    f = Transport()
    z = .055
    try:
        for i in range(221):
            # This is an independently selected synthetic actuator, NOT a fit
            # or claim about the official simulator. Use last actual request.
            cmd = next((v[2] for t, v in reversed(f.rows) if t == "/uav/cmd_vel"), 0.)
            vz = {0.: -.054, .05: .046, .025: -.004, .03: .006, -.01: -.064}[cmd]
            z_next = max(.055, z+vz*.05)
            if z_next == .055: vz = 0.
            z = z_next
            f.feed(round(i*.05, 9), z, vz)
            if f.node.guard.state in ("STOP", "FINISHED"): break
        g = f.node.guard
        assert g.state == "FINISHED" and g.protocol_complete and g.support_return_verified, (g.state, g.reason)
        values = [v[2] for t, v in f.rows if t == "/uav/cmd_vel"]
        assert all(v == (0.,)*6 for t, v in f.rows if t == "/cmd_vel")
        assert all(v[:2]+v[3:] == (0.,)*5 for t, v in f.rows if t == "/uav/cmd_vel")
        assert values == [r["linear_z"] for r in g.commands]
        assert set(values) == {0., .05, .025, .03, -.01}
        for phase, _, _ in api.PROTOCOL:
            assert g.phase_samples[phase]["requested_command_count"] > 0
            assert g.phase_samples[phase]["advancing_odom_count"] > 0
        return {"commands": len(values), "negative": values.count(-.01), "state": g.state,
                "reason": g.reason, "wall_s": f.now[0], "motion_s": f.now[0]-g.motion_started_wall}
    finally: f.close()


def gate_nan_absorbing(topic):
    f = Transport().enter()
    try:
        for i in range(24, 43): f.feed(round(i*.05, 9), .155)
        f.now[0] = 2.15
        f.callbacks["/clock"]({"clock": {"sec": 12, "nanosec": 150000000}})
        assert f.node.guard.state == "PHASE_GATE", f.node.guard.state
        message = odom(12.15, z=math.nan) if topic == "/uav/odom" else scan(12.15)
        if topic == "/scan": message["ranges"][5] = math.nan
        f.callbacks[topic](message)
        assert f.node.guard.state == "STOP"
        before = len(f.rows)
        f.feed(2.16, .155)
        assert f.node.guard.state == "STOP"
        assert all(v == (0.,)*6 for _, v in f.rows[before:])
        return {"topic": topic, "reason": f.node.guard.reason}
    finally: f.close()


def negative_low_callback_absorbing():
    f = Transport().negative()
    try:
        f.now[0] = 3.16
        f.callbacks["/uav/odom"](odom(13.16, z=.089))
        assert f.node.guard.phase == "ZERO_RETURN" and f.rows[-1][1] == (0.,)*6
        assert not f.node.guard.protocol_complete
        before = len(f.rows)
        for i in range(64, 89): f.feed(round(i*.05, 9), .155)
        assert all(v == (0.,)*6 for _, v in f.rows[before:])
        return {"phase": f.node.guard.phase, "reason": f.node.guard.reason}
    finally: f.close()


def permanent_budget_all_states():
    results = []
    for inject in ("PAUSE", "PHASE_GATE", "ZERO_RETURN"):
        f = Transport().enter()
        try:
            g = f.node.guard
            start, origin = g.motion_started_wall, g.origin
            if inject == "ZERO_RETURN": g.pause("independent_abort", 1.16)
            elif inject == "PHASE_GATE":
                for i in range(24, 43): f.feed(round(i*.05, 9), .155)
                f.now[0] = 2.15
                f.callbacks["/clock"]({"clock": {"sec": 12, "nanosec": 150000000}})
                assert g.state == "PHASE_GATE"
            else:
                # Legitimate ascent PAUSE is separately exercised from the fixed
                # trim, whose interruption is deliberately permanent.
                g.phase = "ASCEND_TO_010"
                g.pause("independent_ascent_pause", 1.16)
                assert g.state == "PAUSE"
            f.now[0] = start+10.
            f.node.tick()
            assert g.state == "STOP" and g.reason == "motion_wall_time_limit", (inject, g.state, g.reason)
            assert g.motion_started_wall == start and g.origin == origin
            assert f.rows[-1][1] == (0.,)*6
            results.append({"injected_state": inject, "reason": g.reason})
        finally: f.close()
    return results


def no_successful_request_no_phase_advance():
    f = Transport()
    try:
        for i in range(23): f.feed(i*.05)
        f.feed(1.15, .155, tick=False)
        d = f.node.guard.step(wall_s=1.15)
        assert d.linear_z == .025
        # Keep all source data fresh, but only send explicit zero outputs.
        for i in range(24, 44):
            f.feed(round(i*.05, 9), .155, tick=False)
            f.node.publish_zero()
        f.node.tick()
        g = f.node.guard
        assert g.phase == "ZERO_RETURN" and not g.protocol_complete
        assert g.phase_samples["TRIM_POS_025"]["requested_command_count"] == 0
        return {"reason": g.reason, "odom_count": g.phase_samples["TRIM_POS_025"]["advancing_odom_count"]}
    finally: f.close()


def watchdog_negative_no_tick():
    f = Transport().negative()
    observed = threading.Event()
    original = f.node.publisher_by_topic["/uav/cmd_vel"].publish
    def capture(msg):
        original(msg)
        if msg.linear.z == 0.: observed.set()
    f.node.publisher_by_topic["/uav/cmd_vel"].publish = capture
    try:
        f.now[0] = 3.32
        f.node.arm_watchdog()
        assert observed.wait(.4)
        assert f.node.guard.phase == "ZERO_RETURN"
        assert not f.node.guard.protocol_complete
        assert f.rows[-1][1] == (0.,)*6
        return {"reason": f.node.guard.reason}
    finally: f.close()


if __name__ == "__main__":
    result = {"actual_full_protocol": actual_full_protocol(),
              "gate_nan": [gate_nan_absorbing(t) for t in ("/uav/odom", "/scan")],
              "negative_low_callback": negative_low_callback_absorbing(),
              "permanent_budget": permanent_budget_all_states(),
              "no_successful_request": no_successful_request_no_phase_advance(),
              "independent_watchdog": watchdog_negative_no_tick()}
    result["sha256"] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT/"src/inspection_mission/inspection_mission/bounded_vertical_trim.py",
                  ROOT/"scripts/run_bounded_vertical_trim.sh")}
    dest = Path(__file__).with_name("independent_metrics.json")
    dest.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
