"""A small positive-Z experiment must remain bounded by live observations."""
import importlib
import importlib.util
import math
import sys
from types import SimpleNamespace

import pytest

from test_bounded_ground_probe import scan, endpoint


def api():
    name = "inspection_mission.bounded_vertical_response"
    assert importlib.util.find_spec(name) is not None, "finite vertical response module is missing"
    return importlib.import_module(name)


def odom(stamp, *, x=0., y=0., z=.055, vz=0., child="uav/base_link", pitch=0.):
    sec = int(stamp)
    return {"header": {"stamp": {"sec": sec, "nanosec": round((stamp-sec)*1e9)}, "frame_id": "odom"},
            "child_frame_id": child,
            "pose": {"pose": {"position": {"x": x, "y": y, "z": z},
                              "orientation": {"x": 0., "y": math.sin(pitch/2), "z": 0., "w": math.cos(pitch/2)}}},
            "twist": {"twist": {"linear": {"x": 0., "y": 0., "z": vz},
                                "angular": {"x": 0., "y": 0., "z": 0.}}}}


def tf_static():
    return {"transforms": [{"header": {"frame_id": "uav/base_link", "stamp": {"sec": 0, "nanosec": 0}},
                            "child_frame_id": "uav/laser_link",
                            "transform": {"translation": {"x": 0., "y": 0., "z": -.07},
                                          "rotation": {"x": 0., "y": math.sqrt(.5), "z": 0., "w": math.sqrt(.5)}}}]}


def state():
    return api().VerticalResponseGuard(started_wall_s=0.)


def update(g, wall, **kwargs):
    stamp = 10.+wall
    g.update_clock(stamp, wall_s=wall)
    g.update_odom(odom(stamp, **kwargs), wall_s=wall)
    g.update_ground_odom(odom(stamp, child="base_footprint", **kwargs), wall_s=wall)
    g.update_scan(scan(stamp), wall_s=wall)


def ready(g):
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    g.update_tf(tf_static(), wall_s=0.)
    for i in range(12):
        update(g, i/10)
        decision = g.step(wall_s=i/10)
    return decision


def test_fresh_stable_initial_column_allows_only_bounded_positive_z():
    g = state()
    d = ready(g)
    assert d.linear_z == .05 and d.state == "ASCEND"
    assert g.origin == pytest.approx((0., 0., .055))
    assert g.evidence()["assumptions"]["static_map_verified_on_platform"] is False


def test_reaching_measured_height_requests_zero_and_records_five_seconds_of_response():
    g = state(); ready(g)
    update(g, 1.2, z=.205)
    d = g.step(wall_s=1.2)
    assert d.linear_z == 0. and d.state == "OBSERVE"
    for i in range(13, 64):
        update(g, i/10, z=.202)
        d = g.step(wall_s=i/10)
    assert d.state == "FINISHED"
    assert g.evidence()["target_reached"] is True
    assert g.evidence()["official_score"] is None


@pytest.mark.parametrize("change,reason", [
    ({"z": .256}, "rise_limit"), ({"x": .041}, "horizontal_drift"),
    ({"pitch": .051}, "tilt_limit"), ({"vz": .151}, "speed_limit"),
    ({"z": .034}, "unexpected_descent"),
])
def test_observed_hard_fault_latches_before_another_control_tick(change, reason):
    g = state(); ready(g)
    g.update_odom(odom(11.2, **change), wall_s=1.2)
    assert g.state == "STOP" and reason in g.reason
    update(g, 1.3)
    assert g.step(wall_s=1.3).linear_z == 0.


def test_staleness_pauses_then_rechecks_stability_without_extending_motion_budget():
    g = state(); ready(g)
    g.step(wall_s=1.5)
    assert g.state == "PAUSE"
    start, origin = g.motion_started_wall, g.origin
    for i in range(16, 28):
        update(g, i/10)
        g.step(wall_s=i/10)
    assert g.state == "ASCEND"
    assert g.motion_started_wall == start and g.origin == origin
    update(g, 11.1)
    d = g.step(wall_s=11.1)
    assert d.linear_z == 0. and "motion_wall_time_limit" in d.reason


def test_callback_gap_cannot_be_hidden_by_fresh_replacements_before_next_tick():
    g = state(); ready(g)
    update(g, 1.5)
    assert g.state == "PAUSE"
    assert g.step(wall_s=1.5).linear_z == 0.


@pytest.mark.parametrize("kind", ["no_tf", "no_ground", "wrong_frame", "not_matched", "bad_scan"])
def test_missing_contract_never_opens_flight(kind):
    g = state()
    g.update_endpoints([endpoint()], matched_subscriptions=0 if kind == "not_matched" else 1)
    if kind != "no_tf": g.update_tf(tf_static(), wall_s=0.)
    for i in range(12):
        wall = i/10; stamp = 10.+wall
        g.update_clock(stamp, wall_s=wall)
        g.update_odom(odom(stamp, child="wrong" if kind == "wrong_frame" else "uav/base_link"), wall_s=wall)
        if kind != "no_ground": g.update_ground_odom(odom(stamp, child="base_footprint"), wall_s=wall)
        message = scan(stamp)
        if kind == "bad_scan": message["ranges"][0] = .6
        g.update_scan(message, wall_s=wall)
        assert g.step(wall_s=wall).linear_z == 0.


def test_changed_static_laser_transform_is_not_ignored_after_start():
    g = state(); ready(g)
    changed = tf_static()
    changed["transforms"][0]["transform"]["translation"]["z"] = -.08
    g.update_tf(changed, wall_s=1.2)
    assert g.state == "STOP" and "static_laser_tf" in g.reason


@pytest.mark.parametrize("kind", ["clock", "odom", "wall"])
def test_time_reversal_is_an_absorbing_hard_stop(kind):
    g = state(); ready(g)
    if kind == "clock": g.update_clock(9., wall_s=1.2)
    elif kind == "odom": g.update_odom(odom(9.), wall_s=1.2)
    else: g.step(wall_s=.5)
    assert g.state == "STOP" and "reversed" in g.reason


def test_initial_pose_must_remain_inside_the_explicit_candidate_start_column():
    g = state()
    g.update_endpoints([endpoint()], matched_subscriptions=1)
    g.update_tf(tf_static(), wall_s=0.)
    for i in range(12):
        update(g, i/10, x=.06)
        assert g.step(wall_s=i/10).linear_z == 0.


def test_candidate_column_checks_real_static_boxes_and_source_hash():
    m = api()
    assert m.candidate_column_clear((0., 0., .055), m.VerticalLimits())
    assert not m.candidate_column_clear((0., .2, .055), m.VerticalLimits())
    assert len(m.WORLD_SHA256) == 64


def test_limits_do_not_allow_larger_actions_or_nonfinite_values():
    m = api()
    for kwargs in ({"command_mps": .051}, {"hard_rise_m": .201}, {"target_rise_m": .2},
                   {"max_horizontal_drift_m": .041}, {"motion_wall_s": 10.1},
                   {"global_wall_s": 46.}, {"max_tilt_rad": float("nan")}, {"command_mps": True}):
        with pytest.raises(ValueError): m.VerticalLimits(**kwargs)


def transport(monkeypatch, wall):
    from test_bounded_ground_probe import install_ros_transport
    callbacks, _, publishers = install_ros_transport(monkeypatch)
    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0., y=0., z=0.)
            self.angular = SimpleNamespace(x=0., y=0., z=0.)
    monkeypatch.setattr(sys.modules["rosidl_runtime_py.utilities"], "get_message", lambda _: Twist)
    node = api().create_vertical_node(wall_clock=lambda: wall[0])
    published = []
    for topic, publisher in node.publisher_by_topic.items():
        publisher.publish = lambda msg, topic=topic: published.append((topic,
            (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)))
    return node, callbacks, published, publishers


def feed_transport(node, callbacks, wall, **kwargs):
    stamp = 10.+wall[0]; sec = int(stamp)
    callbacks["/clock"]({"clock": {"sec": sec, "nanosec": round((stamp-sec)*1e9)}})
    callbacks["/uav/odom"](odom(stamp, **kwargs))
    callbacks["/odom"](odom(stamp, child="base_footprint", **kwargs))
    callbacks["/scan"](scan(stamp))


def test_ros_adapter_outputs_ground_zero_and_only_positive_uav_z_then_live_match_loss_zero(monkeypatch):
    wall = [0.]
    node, callbacks, published, publishers = transport(monkeypatch, wall)
    assert set(publishers) == {"/cmd_vel", "/uav/cmd_vel"}
    callbacks["/tf_static"](tf_static())
    for i in range(12):
        wall[0] = i/10
        feed_transport(node, callbacks, wall)
        node.tick()
    assert any(topic == "/uav/cmd_vel" and values[2] == .05 for topic, values in published)
    assert all(values == (0., 0., 0., 0., 0., 0.) for topic, values in published if topic == "/cmd_vel")
    assert all(values[:2]+values[3:] == (0.,)*5 for topic, values in published if topic == "/uav/cmd_vel")
    node.publisher_by_topic["/uav/cmd_vel"].get_subscription_count = lambda: 0
    wall[0] = 1.11
    node.tick()
    assert node.guard.state == "PAUSE" and published[-1] == ("/uav/cmd_vel", (0.,)*6)


def test_control_watchdog_can_publish_zero_while_descriptive_graph_refresh_blocks(monkeypatch):
    import threading
    import time
    wall = [0.]
    node, _, published, _ = transport(monkeypatch, wall)
    entered, release = threading.Event(), threading.Event()
    def blocked(_):
        entered.set()
        release.wait(1.)
        return []
    node.get_subscriptions_info_by_topic = blocked
    node.arm_watchdog()
    runner = threading.Thread(target=node.tick, daemon=True)
    runner.start()
    assert entered.wait(.2)
    wall[0] = .3
    try:
        deadline = time.monotonic()+.25
        while time.monotonic() < deadline and not published:
            time.sleep(.005)
        assert published and all(values == (0.,)*6 for _, values in published)
    finally:
        release.set()
        runner.join(timeout=1.)
        node.close_watchdog()


def test_production_compressed_evidence_passes_receiver_and_roundtrips_full_public_samples():
    import base64
    import gzip
    import hashlib
    import json
    from tools.diagnostic_receiver import validate_event
    from inspection_mission.telemetry import DiagnosticReporter
    g = state(); ready(g)
    g.update_downscan({"ranges": [math.inf, math.inf, math.inf]}, wall_s=1.11)
    g.record_command(wall_s=1.11, linear_z=.05)
    events = g.diagnostic_events()
    envelopes = [validate_event(DiagnosticReporter._encode(event, data)) for event, data in events]
    parts = [e["data"] for e in envelopes if e["data"].get("section") == "bounded_vertical_gzip_part"]
    packed = base64.b64decode("".join(p["base64_part"] for p in parts), validate=True)
    raw = gzip.decompress(packed)
    assert hashlib.sha256(packed).hexdigest() == parts[0]["compressed_sha256"]
    assert hashlib.sha256(raw).hexdigest() == parts[0]["uncompressed_sha256"]
    body = json.loads(raw)
    assert body == g.evidence()
    assert body["downscans"][0]["message"]["ranges"] == ["inf"]*3
    assert body["official_score"] is None


def test_static_candidate_collision_box_is_actually_checked(monkeypatch):
    m = api()
    monkeypatch.setattr(m, "STATIC_BOXES", (("test_overhead", (-.1, -.1, .3), (.1, .1, .4)),))
    assert not m.candidate_column_clear((0., 0., .055), m.VerticalLimits())


def test_frozen_public_world_hash_and_every_static_obstacle_match_baked_candidate():
    import hashlib
    from pathlib import Path
    import xml.etree.ElementTree as ET
    m = api()
    root = Path(__file__).resolve().parents[1]
    path = root/"external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == m.WORLD_SHA256
    world = ET.fromstring(path.read_text("utf-8")).find("world")
    def pose(element): return tuple(float(x) for x in (element.findtext("pose") or "0 0 0 0 0 0").split())
    expected = {}
    for model in world.findall("model"):
        if model.findtext("static") != "true" or model.get("name") == "inspection_floor": continue
        a = pose(model)
        for link in model.findall("link"):
            b = pose(link)
            for collision in link.findall("collision"):
                c = pose(collision)
                assert not any(a[3:]+b[3:]+c[3:])
                size = tuple(float(x) for x in collision.findtext("geometry/box/size").split())
                center = tuple(a[i]+b[i]+c[i] for i in range(3))
                name = model.get("name")
                if collision.get("name") != "collision": name += "/"+collision.get("name")
                expected[name] = (tuple(center[i]-size[i]/2 for i in range(3)), tuple(center[i]+size[i]/2 for i in range(3)))
    assert len(expected) == len(m.STATIC_BOXES) == 12
    for name, low, high in m.STATIC_BOXES:
        assert low == pytest.approx(expected[name][0], abs=1e-12)
        assert high == pytest.approx(expected[name][1], abs=1e-12)


@pytest.mark.parametrize("child_exit,setup_exit", [(0, 0), (23, 0), (0, 31)])
def test_vertical_launcher_runs_uploaded_source_and_preserves_actual_exit(tmp_path, child_exit, setup_exit):
    import json
    import os
    from pathlib import Path
    import shlex
    import shutil
    import subprocess
    from test_ground_probe_launch import BASH, shell_path
    if not BASH.is_file(): pytest.skip("bash unavailable")
    workspace = tmp_path/"uploaded vertical workspace"
    package = workspace/"src/inspection_mission/inspection_mission"
    package.mkdir(parents=True)
    (package/"__init__.py").write_text("", encoding="utf-8")
    (package/"bounded_vertical_response.py").write_text(
        "import json,os,sys\nassert os.environ['FAKE_ROS_READY']=='1'\n"
        "print('VERTICAL_ARGS '+json.dumps(sys.argv[1:]))\n"
        f"sys.exit({child_exit})\n", encoding="utf-8")
    source = Path(__file__).resolve().parents[1]/"scripts/run_bounded_vertical_response.sh"
    assert source.is_file(), "independent vertical launcher is missing"
    launcher = workspace/"scripts/run_bounded_vertical_response.sh"
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
    assert f"BOUNDED_VERTICAL_RESPONSE launch_exit exit_code={setup_exit or child_exit}" in result.stderr
    if not setup_exit:
        line = next(line for line in result.stdout.splitlines() if line.startswith("VERTICAL_ARGS "))
        assert json.loads(line.removeprefix("VERTICAL_ARGS ")) == []
