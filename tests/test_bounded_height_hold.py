"""Finite local height feedback; synthetic trajectories are not platform evidence."""
import base64
import gzip
import hashlib
import importlib
import importlib.util
import json
import math
import sys
from types import SimpleNamespace

import pytest

from test_bounded_vertical_response import odom, tf_static, update, ready, feed_transport
from test_bounded_ground_probe import scan, install_ros_transport


def api():
    name = "inspection_mission.bounded_height_hold"
    assert importlib.util.find_spec(name) is not None, "bounded height hold module missing"
    return importlib.import_module(name)


def guard(): return api().HeightHoldGuard(started_wall_s=0.)


def step(g, wall):
    d = g.step(wall_s=wall)
    g.record_command(wall_s=wall, linear_z=d.linear_z)
    return d


def enter_hold(g):
    assert ready(g).linear_z == .05
    g.record_command(wall_s=1.1, linear_z=.05)
    update(g, 1.2, z=.155)
    assert step(g, 1.2).linear_z == 0.
    assert g.state == "PHASE_GATE"
    update(g, 1.3, z=.155)
    assert step(g, 1.3).linear_z == pytest.approx(.02695)
    assert g.phase == "HOLD_010"


def feed_steps(g, start, end, *, z=.155, vz=0.):
    result = None
    for i in range(round(start*10), round(end*10)+1):
        wall = i/10
        update(g, wall, z=z, vz=vz)
        result = step(g, wall)
    return result


@pytest.mark.parametrize("target,z,want", [(.155, .155, .02695), (.155, .145, .02945),
    (.075, .155, .01195), (.155, .075, .04195)])
def test_feedback_uses_bounded_local_trim_and_error_response(target, z, want):
    assert api().feedback_command(target, z) == pytest.approx(want)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_feedback_inputs_are_rejected(bad):
    with pytest.raises(ValueError): api().feedback_command(.155, bad)


def test_hold_needs_three_continuous_actual_seconds_then_gates_to_positive_trim_descent():
    g = guard(); enter_hold(g)
    assert feed_steps(g, 1.4, 4.2).linear_z > 0.
    assert not g.hold_verified
    assert feed_steps(g, 4.3, 4.3).linear_z == 0.
    assert g.hold_verified and g.state == "PHASE_GATE"
    d = feed_steps(g, 4.4, 4.4)
    assert g.phase == "DESCEND_002" and d.linear_z == pytest.approx(.01195)
    assert g.motion_started_wall == 1.0 and g.origin == pytest.approx((0., 0., .055))


@pytest.mark.parametrize("change", [{"z": .161}, {"vz": .011}])
def test_hold_height_or_speed_break_restarts_dwell(change):
    g = guard(); enter_hold(g)
    feed_steps(g, 1.4, 3.0)
    update(g, 3.1, z=change.get("z", .155), vz=change.get("vz", 0.))
    step(g, 3.1)
    feed_steps(g, 3.2, 6.1)
    assert not g.hold_verified
    feed_steps(g, 6.2, 6.2)
    assert g.hold_verified
    assert g.hold_dwell_resets


def test_outside_hold_sample_cannot_be_hidden_by_a_good_replacement_before_tick():
    g = guard(); enter_hold(g); feed_steps(g, 1.4, 3.0)
    update(g, 3.05, z=.161)
    update(g, 3.1, z=.155)
    step(g, 3.1)
    feed_steps(g, 3.2, 4.3)
    assert not g.hold_verified and g.phase == "HOLD_010"
    assert g.hold_dwell_resets[-1]["odom_stamp"] == pytest.approx(13.05)


def test_descent_gate_cannot_send_a_first_nonzero_below_the_near_support_cutoff():
    g = guard(); enter_hold(g); feed_steps(g, 1.4, 4.3)
    update(g, 4.4, z=.079, vz=-.01)
    assert step(g, 4.4).linear_z == 0.
    assert g.phase == "ZERO_RETURN" and g.descent_verified


def test_hold_eight_second_timeout_cannot_be_extended_by_dwell_resets():
    g = guard(); enter_hold(g)
    feed_steps(g, 1.4, 9.3, z=.162)
    assert g.phase == "ZERO_RETURN" and not g.hold_verified
    assert "hold_wall_time_limit" in g.protocol_abort_reason
    assert g.commands[-1]["linear_z"] == 0.


def test_replaying_one_odom_stamp_cannot_satisfy_hold():
    g = guard(); enter_hold(g)
    for i in range(14, 50):
        wall = i/10
        g.update_clock(10.+wall, wall_s=wall)
        g.update_scan(scan(10.+wall), wall_s=wall)
        g.update_odom(odom(11.3, z=.155), wall_s=wall)
        d = step(g, wall)
        if wall > 1.5: assert d.linear_z == 0.
    assert not g.hold_verified and g.phase == "ZERO_RETURN"


@pytest.mark.parametrize("phase", ["ASCEND_TO_010", "HOLD_010", "DESCEND_002"])
def test_any_nonzero_phase_interruption_is_permanent_zero_return(phase):
    g = guard()
    if phase == "ASCEND_TO_010": ready(g)
    else:
        enter_hold(g)
        if phase == "DESCEND_002": feed_steps(g, 1.4, 4.4)
    start, origin = g.motion_started_wall, g.origin
    wall = g.last_observation_wall+.01
    g.pause("injected_gap", wall)
    assert g.phase == "ZERO_RETURN" and g.state == "OBSERVE"
    feed_steps(g, math.ceil(wall*10)/10, math.ceil(wall*10)/10+.5)
    assert all(c["linear_z"] == 0. for c in g.commands if c["wall_elapsed_s"] >= wall)
    assert g.motion_started_wall == start and g.origin == origin


@pytest.mark.parametrize("vz,completed", [(-.01, True), (-.021, False)])
def test_descent_near_support_needs_height_and_speed_and_always_stops_nonzero(vz, completed):
    g = guard(); enter_hold(g); feed_steps(g, 1.4, 4.4)
    update(g, 4.5, z=.079, vz=vz)
    d = step(g, 4.5)
    assert d.linear_z == 0. and g.phase == "ZERO_RETURN"
    assert g.descent_verified is completed and g.protocol_complete is completed
    feed_steps(g, 4.6, 4.7, z=.079, vz=0.)
    assert not g.support_return_verified
    feed_steps(g, 4.8, 5.9, z=.055, vz=0.)
    assert g.state == "FINISHED" and g.support_return_verified
    assert g.evidence()["official_score"] is None


def test_permanent_twenty_five_second_motion_budget_includes_zero_return():
    g = guard(); enter_hold(g)
    g.pause("abort_hold", 1.31)
    feed_steps(g, 1.4, 26.1)
    assert g.state == "STOP" and "motion_wall_time_limit" in g.reason
    assert g.motion_started_wall == 1.0
    assert g.limits.motion_wall_s == 25.
    from inspection_mission.bounded_vertical_response import VerticalLimits
    with pytest.raises(ValueError): VerticalLimits(motion_wall_s=25.)


@pytest.mark.parametrize("change,reason", [({"z": .206}, "rise_limit"), ({"x": .041}, "horizontal_drift"),
    ({"pitch": .051}, "tilt_limit"), ({"vz": .151}, "speed_limit"), ({"angular_x": .101}, "angular_speed_limit")])
def test_original_hard_bounds_remain_absorbing(change, reason):
    g = guard(); enter_hold(g)
    args = {k:v for k,v in change.items() if k != "angular_x"}
    payload = odom(11.4, **args)
    if "angular_x" in change: payload["twist"]["twist"]["angular"]["x"] = change["angular_x"]
    g.update_odom(payload, wall_s=1.4)
    assert g.state == "STOP" and reason in g.reason
    update(g, 1.5, z=.155)
    assert step(g, 1.5).linear_z == 0.


def test_global_and_path_hard_limits_are_not_relaxed():
    g = guard(); assert g.step(wall_s=45.).state == "STOP"
    g = guard(); enter_hold(g)
    for i, z in enumerate((.195, .095, .195, .095), 14):
        update(g, i/10, z=z); step(g, i/10)
    assert g.state == "STOP" and "total_path_limit" in g.reason


@pytest.mark.parametrize("topic", ["odom", "scan"])
def test_invalid_input_in_stage_zero_gate_is_absorbing(topic):
    g = guard(); ready(g); g.record_command(wall_s=1.1, linear_z=.05)
    update(g, 1.2, z=.155); step(g, 1.2)
    if topic == "odom": g.update_odom(odom(11.21, z=math.nan), wall_s=1.21)
    else:
        payload = scan(11.21); payload["ranges"][0] = math.nan
        g.update_scan(payload, wall_s=1.21)
    assert g.state == "STOP"
    update(g, 1.3, z=.155)
    assert step(g, 1.3).linear_z == 0. and g.phase == "ASCEND_TO_010"


def test_evidence_roundtrip_preserves_feedback_and_dwell_without_score_claim():
    from inspection_mission.telemetry import DiagnosticReporter
    from tools.diagnostic_receiver import validate_event
    g = guard(); enter_hold(g); feed_steps(g, 1.4, 4.4)
    rows = [validate_event(DiagnosticReporter._encode(k,v))["data"] for k,v in g.diagnostic_events()]
    parts = [x for x in rows if x["section"] == "bounded_height_hold_gzip_part"]
    packed = base64.b64decode("".join(x["base64_part"] for x in parts), validate=True)
    raw = gzip.decompress(packed)
    assert len(raw) == parts[0]["uncompressed_bytes"]
    assert hashlib.sha256(raw).hexdigest() == parts[0]["uncompressed_sha256"]
    body = json.loads(raw)
    assert body == g.evidence() and body["hold_verified"]
    assert body["commands"][-1]["target_z"] == pytest.approx(.075)
    assert body["commands"][-1]["measured_vz"] == 0.
    assert body["assumptions"]["static_map_verified_on_platform"] is False
    assert body["official_score"] is None and not body["full_mission_complete"]


def transport(monkeypatch):
    callbacks, _, _ = install_ros_transport(monkeypatch)
    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0., y=0., z=0.)
            self.angular = SimpleNamespace(x=0., y=0., z=0.)
    monkeypatch.setattr(sys.modules["rosidl_runtime_py.utilities"], "get_message", lambda _: Twist)
    wall = [0.]
    node = api().create_height_node(wall_clock=lambda: wall[0])
    outputs = []
    for topic, pub in node.publisher_by_topic.items():
        pub.publish = lambda m, topic=topic: outputs.append((topic,
            (m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)))
    callbacks["/tf_static"](tf_static())
    for i in range(14):
        wall[0] = i/10
        feed_transport(node, callbacks, wall, z=.055 if i < 12 else .155)
        node.tick()
    assert node.guard.phase == "HOLD_010"
    return node, callbacks, outputs, wall


def test_adapter_only_emits_two_channels_and_feedback_z(monkeypatch):
    node, _, outputs, _ = transport(monkeypatch)
    assert set(node.publisher_by_topic) == {"/cmd_vel", "/uav/cmd_vel"}
    assert outputs[-1][1][2] == pytest.approx(.02695)
    assert all(v == (0.,)*6 for t,v in outputs if t == "/cmd_vel")
    assert all(v[:2]+v[3:] == (0.,)*5 and 0. <= v[2] <= .05 for t,v in outputs if t == "/uav/cmd_vel")


@pytest.mark.parametrize("fault", ["sensor_age", "motion_budget", "hold_deadline", "dds_match", "dds_query_failure", "dds_query_delay", "ground_failure", "feedback_changed"])
def test_blocking_ground_or_match_fault_cannot_send_stale_nonzero_and_zero_is_independent(monkeypatch, fault):
    node, _, outputs, wall = transport(monkeypatch)
    before = len(outputs)
    original = node.publisher_by_topic["/cmd_vel"].publish
    send_wall = {"sensor_age":1.51,"motion_budget":26.11,"hold_deadline":9.31,
                 "dds_match":1.32,"dds_query_failure":1.32,"dds_query_delay":1.51,"ground_failure":1.32,"feedback_changed":1.32}[fault]
    wall[0] = 1.31
    def delayed(m):
        original(m)
        wall[0] = 1.32 if fault == "dds_query_delay" else send_wall
        if fault == "dds_match": node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = lambda:0
        elif fault == "dds_query_failure":
            def bad(): raise RuntimeError("DDS query failed")
            node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = bad
        elif fault == "dds_query_delay":
            def slow(): wall[0] = send_wall; return 1
            node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = slow
        elif fault == "ground_failure": raise RuntimeError("ground failed")
        elif fault == "feedback_changed": node.guard.update_odom(odom(11.32,z=.156),wall_s=1.32)
    node.publisher_by_topic["/cmd_vel"].publish = delayed
    d = node.tick()
    following = [v for t,v in outputs[before:] if t == "/uav/cmd_vel"]
    assert following and all(v == (0.,)*6 for v in following)
    assert d.linear_z == 0. and node.guard.commands[-1]["linear_z"] == 0.
    assert node.guard.commands[-1]["wall_elapsed_s"] == send_wall


def test_failed_positive_publish_is_not_counted_and_still_attempts_uav_zero(monkeypatch):
    node, _, outputs, wall = transport(monkeypatch)
    original = node.publisher_by_topic["/uav/cmd_vel"].publish
    positive_before = node.publish_counts["uav_positive_z"]
    def fail(m):
        if m.linear.z: raise RuntimeError("positive failed")
        original(m)
    node.publisher_by_topic["/uav/cmd_vel"].publish = fail
    wall[0] = 1.31
    assert node.tick().linear_z == 0. and node.guard.state == "STOP"
    assert outputs[-1] == ("/uav/cmd_vel", (0.,)*6)
    assert node.publish_counts["uav_positive_z"] == positive_before


@pytest.mark.parametrize("child_exit,setup_exit", [(0,0),(17,0),(0,29)])
def test_launcher_executes_uploaded_module_and_preserves_real_exit(tmp_path, child_exit, setup_exit):
    import os
    from pathlib import Path
    import shlex
    import shutil
    import subprocess
    from test_ground_probe_launch import BASH, shell_path
    if not BASH.is_file(): pytest.skip("bash unavailable")
    workspace = tmp_path/"uploaded height workspace"
    package = workspace/"src/inspection_mission/inspection_mission"
    package.mkdir(parents=True)
    (package/"__init__.py").write_text("", encoding="utf-8")
    (package/"bounded_height_hold.py").write_text("import os,sys\nassert os.environ['FAKE_ROS_READY']=='1'\n"+f"sys.exit({child_exit})\n", encoding="utf-8")
    source = Path(__file__).resolve().parents[1]/"scripts/run_bounded_height_hold.sh"
    assert source.is_file(), "height launcher missing"
    launcher = workspace/"scripts/run_bounded_height_hold.sh"; launcher.parent.mkdir()
    shutil.copyfile(source, launcher)
    setup = tmp_path/"fake ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'+(f"return {setup_exit}\n" if setup_exit else "export FAKE_ROS_READY=1\n"), encoding="utf-8")
    env = dict(os.environ, INSPECTION_ROS_SETUP=shell_path(setup)); env.pop("INSPECTION_DIAGNOSTIC_URL",None)
    r = subprocess.run([str(BASH), str(launcher)], cwd=tmp_path, env=env, capture_output=True, text=True, encoding="utf-8", timeout=10)
    assert r.returncode == (setup_exit or child_exit), r.stdout+r.stderr
    assert f"BOUNDED_HEIGHT_HOLD launch_exit exit_code={setup_exit or child_exit}" in r.stderr
