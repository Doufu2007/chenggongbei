"""The low altitude trim protocol must stay bounded and record actual signed requests."""
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

from test_bounded_vertical_response import odom, tf_static, update, ready
from test_bounded_ground_probe import scan, install_ros_transport


def api():
    name = "inspection_mission.bounded_vertical_trim"
    assert importlib.util.find_spec(name) is not None, "bounded signed trim module missing"
    return importlib.import_module(name)


def guard():
    return api().VerticalTrimGuard(started_wall_s=0.)


def enter_trim(g):
    ready(g)
    update(g, 1.2, z=.155)
    return step(g, 1.2)


def step(g, wall):
    decision = g.step(wall_s=wall)
    g.record_command(wall_s=wall, linear_z=decision.linear_z)
    return decision


def progress(g, start, end, *, z=.155):
    decision = None
    for i in range(round(start*10), round(end*10)+1):
        wall = i/10
        update(g, wall, z=z)
        decision = step(g, wall)
    return decision


def test_fixed_signed_protocol_and_actual_support_return_required_for_normal_finish():
    g = guard()
    assert ready(g).linear_z == .05
    update(g, 1.2, z=.155)
    assert step(g, 1.2).linear_z == .025 and g.phase == "TRIM_POS_025"
    assert progress(g, 1.3, 2.2).linear_z == .030 and g.phase == "TRIM_POS_030"
    assert progress(g, 2.3, 3.2).linear_z == -.010 and g.phase == "TRIM_NEG_010"
    assert progress(g, 3.3, 3.5).linear_z == 0. and g.phase == "ZERO_RETURN"
    assert g.state == "OBSERVE" and g.protocol_complete
    progress(g, 3.6, 4.8)
    assert g.state == "OBSERVE" and not g.support_return_verified
    for i in range(49, 68):
        wall = i/10
        update(g, wall, z=max(.055, .155-(wall-4.8)*.10))
        g.step(wall_s=wall)
    assert g.state == "OBSERVE"
    progress(g, 6.8, 6.9, z=.055)
    assert g.state == "FINISHED" and g.support_return_verified
    assert g.evidence()["official_score"] is None


@pytest.mark.parametrize("phase_end", [1.2, 2.2, 3.2])
def test_low_height_immediately_aborts_every_nonzero_trim_stage(phase_end):
    g = guard(); enter_trim(g)
    if phase_end > 1.2: progress(g, 1.3, phase_end)
    wall = phase_end+.1
    g.update_odom(odom(10.+wall, z=.089), wall_s=wall)
    assert g.state == "OBSERVE" and g.phase == "ZERO_RETURN"
    assert not g.protocol_complete and "low_height" in g.reason
    assert g.step(wall_s=wall).linear_z == 0.
    progress(g, wall+.1, wall+.3, z=.155)
    assert g.phase == "ZERO_RETURN" and g.state != "ASCEND"


def test_no_actual_new_odom_cannot_complete_or_advance_a_trim_stage():
    g = guard(); enter_trim(g)
    for i in range(13, 26):
        wall = i/10
        g.update_clock(10.+wall, wall_s=wall)
        g.update_scan(scan(10.+wall), wall_s=wall)
        g.update_odom(odom(11.2, z=.155), wall_s=wall)
        decision = g.step(wall_s=wall)
        if wall > 1.4: assert decision.linear_z == 0.
        else: assert g.state == "ASCEND"
    assert g.phase == "ZERO_RETURN" and not g.protocol_complete
    assert not any(x["to"] == "TRIM_POS_030" for x in g.phase_transitions)


def test_trim_pause_becomes_permanent_zero_return_and_never_retries_nonzero():
    g = guard(); enter_trim(g)
    origin, started = g.origin, g.motion_started_wall
    g.pause("injected_match_loss", 1.21)
    assert g.state == "OBSERVE" and g.phase == "ZERO_RETURN"
    assert not g.protocol_complete
    progress(g, 1.3, 3., z=.155)
    assert g.step(wall_s=3.).linear_z == 0.
    assert g.origin == origin and g.motion_started_wall == started


def test_ascend_pause_and_zero_return_cannot_reset_the_permanent_motion_budget():
    g = guard(); ready(g)
    origin, started = g.origin, g.motion_started_wall
    g.pause("initial_gap", 1.11)
    progress(g, 1.2, 2.3, z=.055)
    assert g.state == "ASCEND" and g.motion_started_wall == started
    update(g, 2.4, z=.155); g.step(wall_s=2.4)
    g.pause("trim_gap", 2.41)
    assert g.phase == "ZERO_RETURN"
    progress(g, 2.5, 11.1, z=.155)
    assert g.state == "STOP" and "motion_wall_time_limit" in g.reason
    assert g.origin == origin and g.motion_started_wall == started


@pytest.mark.parametrize("change,reason", [({"z": .206}, "rise_limit"), ({"x": .041}, "horizontal_drift"),
    ({"pitch": .051}, "tilt_limit"), ({"vz": .151}, "speed_limit")])
def test_inherited_hard_bounds_latch_during_trim_and_never_resume(change, reason):
    g = guard(); enter_trim(g)
    g.update_odom(odom(11.3, **change), wall_s=1.3)
    assert g.state == "STOP" and reason in g.reason
    update(g, 1.4, z=.155)
    assert g.step(wall_s=1.4).linear_z == 0.


def test_total_wall_and_total_path_limits_remain_absorbing():
    g = guard()
    assert g.step(wall_s=45.).state == "STOP"
    g = guard(); enter_trim(g)
    for i, z in enumerate((.195, .095, .195, .095), 13):
        update(g, i/10, z=z); g.step(wall_s=i/10)
    assert g.state == "STOP" and "total_path_limit" in g.reason


def test_signed_recording_rejects_out_of_phase_and_unbounded_commands():
    g = guard(); enter_trim(g)
    for value in (-.010, -.011, .030, .051, math.nan, True):
        with pytest.raises(ValueError): g.record_command(wall_s=1.21, linear_z=value)
    progress(g, 1.3, 3.2)
    g.record_command(wall_s=3.21, linear_z=-.010)
    assert g.commands[-1]["linear_z"] == -.010 and g.commands[-1]["phase"] == "TRIM_NEG_010"


def test_signed_evidence_roundtrip_contains_phase_samples_and_transitions():
    from tools.diagnostic_receiver import validate_event
    from inspection_mission.telemetry import DiagnosticReporter
    g = guard(); enter_trim(g)
    progress(g, 1.3, 3.2)
    g.record_command(wall_s=3.21, linear_z=-.010)
    events = g.diagnostic_events()
    decoded = [validate_event(DiagnosticReporter._encode(event, payload)) for event, payload in events]
    parts = [x["data"] for x in decoded if x["data"].get("section") == "bounded_vertical_trim_gzip_part"]
    packed = base64.b64decode("".join(p["base64_part"] for p in parts), validate=True)
    raw = gzip.decompress(packed)
    assert hashlib.sha256(raw).hexdigest() == parts[0]["uncompressed_sha256"]
    assert len(raw) == parts[0]["uncompressed_bytes"]
    body = json.loads(raw)
    assert body == g.evidence()
    assert body["commands"][-1]["linear_z"] == -.010
    assert body["phase_samples"]["TRIM_POS_025"]["advancing_odom_count"] >= 1
    assert body["phase_transitions"][-1]["to"] == "TRIM_NEG_010"
    assert body["assumptions"]["static_map_verified_on_platform"] is False


def test_return_stability_cannot_be_satisfied_by_replaying_one_old_odom_stamp():
    g = guard(); enter_trim(g)
    g.pause("trim_interruption", 1.21)
    update(g, 1.3, z=.055); g.step(wall_s=1.3)
    for i in range(14, 25):
        wall = i/10
        g.update_clock(10.+wall, wall_s=wall)
        g.update_scan(scan(10.+wall), wall_s=wall)
        g.update_odom(odom(11.3, z=.055), wall_s=wall)
        g.step(wall_s=wall)
    assert not g.support_return_verified
    assert g.state in ("STOP", "FINISHED") and "return_sensor_lost" in g.reason


@pytest.mark.parametrize("topic", ["odom", "scan"])
def test_invalid_input_during_zero_phase_gate_is_absorbing_before_any_replacement(topic):
    g = guard(); enter_trim(g); progress(g, 1.3, 2.1)
    g.update_clock(12.2, wall_s=2.2)
    assert g.state == "PHASE_GATE"
    if topic == "odom": g.update_odom(odom(12.2, z=math.nan), wall_s=2.2)
    else:
        message = scan(12.2); message["ranges"][0] = math.nan
        g.update_scan(message, wall_s=2.2)
    assert g.state == "STOP"
    update(g, 2.3, z=.155)
    assert g.step(wall_s=2.3).linear_z == 0. and g.phase == "TRIM_POS_025"


def transport(monkeypatch, wall):
    callbacks, _, _ = install_ros_transport(monkeypatch)
    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0., y=0., z=0.)
            self.angular = SimpleNamespace(x=0., y=0., z=0.)
    monkeypatch.setattr(sys.modules["rosidl_runtime_py.utilities"], "get_message", lambda _: Twist)
    node = api().create_trim_node(wall_clock=lambda: wall[0])
    published = []
    for topic, publisher in node.publisher_by_topic.items():
        publisher.publish = lambda msg, topic=topic: published.append((topic,
            (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)))
    return node, callbacks, published


def feed(node, callbacks, wall, *, z=.155):
    from test_bounded_vertical_response import feed_transport
    feed_transport(node, callbacks, wall, z=z)
    return node.tick()


def negative_transport(monkeypatch):
    wall = [0.]
    node, callbacks, published = transport(monkeypatch, wall)
    callbacks["/tf_static"](tf_static())
    for i in range(12):
        wall[0] = i/10
        feed(node, callbacks, wall, z=.055)
    for i in range(12, 33):
        wall[0] = i/10
        feed(node, callbacks, wall)
    assert published[-1] == ("/uav/cmd_vel", (0., 0., -.010, 0., 0., 0.))
    return node, callbacks, published, wall


def test_adapter_keeps_only_two_publishers_and_counts_positive_negative_zero_distinctly(monkeypatch):
    node, _, published, _ = negative_transport(monkeypatch)
    assert set(node.publisher_by_topic) == {"/cmd_vel", "/uav/cmd_vel"}
    assert node.publish_counts["uav_negative_z"] == 1
    assert node.publish_counts["uav_positive_z"] == sum(t == "/uav/cmd_vel" and v[2] > 0. for t, v in published)
    assert all(v == (0.,)*6 for t, v in published if t == "/cmd_vel")
    assert all(v[:2]+v[3:] == (0.,)*5 for t, v in published if t == "/uav/cmd_vel")


@pytest.mark.parametrize("action", ["tick", "stop", "low_callback", "watchdog"])
def test_negative_phase_fault_still_independently_attempts_uav_zero_when_ground_publish_fails(monkeypatch, action):
    import threading
    node, callbacks, published, wall = negative_transport(monkeypatch)
    before = len(published)
    def fail(_): raise RuntimeError("ground channel failed")
    node.publisher_by_topic["/cmd_vel"].publish = fail
    if action == "tick":
        wall[0] = 3.21
        assert node.tick().linear_z == 0.
    elif action == "stop": node.stop("requested_stop")
    elif action == "low_callback":
        wall[0] = 3.21
        callbacks["/uav/odom"](odom(13.21, z=.089))
    else:
        event = threading.Event()
        original = node.publisher_by_topic["/uav/cmd_vel"].publish
        def observe(msg):
            original(msg)
            if msg.linear.z == 0.: event.set()
        node.publisher_by_topic["/uav/cmd_vel"].publish = observe
        wall[0] = 3.5
        node.arm_watchdog()
        try: assert event.wait(.3)
        finally: node.close_watchdog()
    assert node.guard.state == "STOP"
    assert published[before:] and all(t == "/uav/cmd_vel" and v == (0.,)*6 for t, v in published[before:])


def test_negative_phase_live_dds_loss_sends_zero_and_cannot_rearm(monkeypatch):
    node, _, published, wall = negative_transport(monkeypatch)
    node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = lambda: 0
    wall[0] = 3.21
    assert node.tick().linear_z == 0.
    assert node.guard.phase == "ZERO_RETURN" and not node.guard.protocol_complete
    assert published[-1] == ("/uav/cmd_vel", (0.,)*6)


def test_failed_negative_uav_publish_tries_zero_and_keeps_signed_success_counts_honest(monkeypatch):
    node, _, published, wall = negative_transport(monkeypatch)
    previous_negative = node.publish_counts["uav_negative_z"]
    original = node.publisher_by_topic["/uav/cmd_vel"].publish
    attempted = []
    def fail_negative(msg):
        attempted.append(msg.linear.z)
        if msg.linear.z < 0.: raise RuntimeError("negative channel failure")
        original(msg)
    node.publisher_by_topic["/uav/cmd_vel"].publish = fail_negative
    wall[0] = 3.21
    decision = node.tick()
    assert attempted == [-.010, 0.]
    assert decision.state == "STOP" and decision.linear_z == 0.
    assert node.publish_counts["uav_negative_z"] == previous_negative
    assert published[-1] == ("/uav/cmd_vel", (0.,)*6)


def test_expired_negative_phase_is_zeroed_at_publish_boundary_before_next_tick(monkeypatch):
    node, callbacks, published, wall = negative_transport(monkeypatch)
    for elapsed in (3.3, 3.4):
        wall[0] = elapsed
        feed(node, callbacks, wall)
    wall[0] = 3.5
    node._publish(-.010)
    assert node.guard.state == "PHASE_GATE"
    assert published[-1] == ("/uav/cmd_vel", (0.,)*6)


@pytest.mark.parametrize("child_exit,setup_exit", [(0, 0), (17, 0), (0, 29)])
def test_trim_launcher_runs_uploaded_source_and_preserves_real_exit(tmp_path, child_exit, setup_exit):
    import os
    from pathlib import Path
    import shlex
    import shutil
    import subprocess
    from test_ground_probe_launch import BASH, shell_path
    if not BASH.is_file(): pytest.skip("bash unavailable")
    workspace = tmp_path/"uploaded trim workspace"
    package = workspace/"src/inspection_mission/inspection_mission"
    package.mkdir(parents=True)
    (package/"__init__.py").write_text("", encoding="utf-8")
    (package/"bounded_vertical_trim.py").write_text(
        "import os,sys\nassert os.environ['FAKE_ROS_READY']=='1'\n"
        f"sys.exit({child_exit})\n", encoding="utf-8")
    source = Path(__file__).resolve().parents[1]/"scripts/run_bounded_vertical_trim.sh"
    assert source.is_file(), "trim launcher missing"
    launcher = workspace/"scripts/run_bounded_vertical_trim.sh"
    launcher.parent.mkdir()
    shutil.copyfile(source, launcher)
    setup = tmp_path/"fake ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
        +(f"return {setup_exit}\n" if setup_exit else "export FAKE_ROS_READY=1\n"), encoding="utf-8")
    env = dict(os.environ, INSPECTION_ROS_SETUP=shell_path(setup))
    env.pop("INSPECTION_DIAGNOSTIC_URL", None)
    result = subprocess.run([str(BASH), str(launcher)], cwd=tmp_path, env=env,
        capture_output=True, text=True, encoding="utf-8", timeout=10)
    assert result.returncode == (setup_exit or child_exit), result.stdout+result.stderr
    assert f"BOUNDED_VERTICAL_TRIM launch_exit exit_code={setup_exit or child_exit}" in result.stderr
