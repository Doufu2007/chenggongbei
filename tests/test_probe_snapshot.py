"""Final evidence bypasses a slow per-event queue without claiming a score."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from inspection_mission import probe, telemetry
from test_probe_images import install_ros_boundary, raw_image


def test_send_now_reaches_receiver_while_older_queue_request_is_still_blocked():
    pending, release, final_received = threading.Event(), threading.Event(), threading.Event()
    received = []

    class Receiver(BaseHTTPRequestHandler):
        def do_POST(self):
            payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            received.append(payload)
            if payload["event"] == "sample":
                pending.set()
                release.wait(2)
            elif payload["event"] == "probe_snapshot":
                final_received.set()
            self.send_response(202)
            self.end_headers()

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Receiver)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    reporter = telemetry.DiagnosticReporter(f"http://127.0.0.1:{server.server_port}/session")
    try:
        reporter.emit("sample", {"topic": "/odom"})
        assert pending.wait(1)
        assert hasattr(reporter, "send_now"), "final callback cannot bypass the slow queue"
        started = time.monotonic()
        assert reporter.send_now("probe_snapshot", {"read_only": True, "sample_counts": {"/odom": 20}}, timeout_s=.8)
        assert time.monotonic() - started < .8
        assert final_received.is_set() and not release.is_set()
        assert received[-1]["event"] == "probe_snapshot"
    finally:
        release.set()
        reporter.close()
        server.shutdown()
        server.server_close()
        thread.join()


def test_send_now_network_failure_is_bounded_and_returns_false():
    from socket import socket
    with socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        unused_port = reservation.getsockname()[1]
    with telemetry.DiagnosticReporter(f"http://127.0.0.1:{unused_port}/session") as reporter:
        assert hasattr(reporter, "send_now"), "synchronous final send is missing"
        started = time.monotonic()
        assert reporter.send_now("probe_snapshot", {"read_only": True}, timeout_s=.2) is False
        assert time.monotonic() - started < 1
        assert reporter.failed == 1


def test_camera_summary_keeps_full_calibration_but_laser_and_image_data_stay_bounded():
    assert hasattr(probe, "summarize_sensor_payload"), "typed calibration-aware summary is missing"
    camera = {"k": list(range(9)), "r": list(range(9)), "p": list(range(12)), "d": list(range(14))}
    summary = probe.summarize_sensor_payload(camera, "sensor_msgs/msg/CameraInfo")
    assert summary == camera
    scan = probe.summarize_sensor_payload({"ranges": list(range(1000)), "intensities": list(range(1000))},
                                         "sensor_msgs/msg/LaserScan")
    assert scan["ranges"] == list(range(512))
    assert scan["intensities"] == list(range(512))
    image = probe.summarize_sensor_payload({"width": 64, "data": [100] * 6000}, "sensor_msgs/msg/Image")
    assert image["data"] == {"byte_length": 6000}


def test_final_snapshot_collects_first_samples_counts_schemas_and_separate_images(monkeypatch):
    node, callbacks, _ = install_ros_boundary(monkeypatch, [("/camera/image", "sensor_msgs/msg/Image")])
    callbacks["/camera/image"](raw_image())
    callbacks["/camera/image"](raw_image())
    assert hasattr(node, "probe_snapshot"), "final read-only evidence snapshot is missing"
    snapshot = node.probe_snapshot()
    assert snapshot["read_only"] is True
    assert snapshot["official_scoring"] == "unverified"
    assert snapshot["sample_counts"] == {"/camera/image": 2}
    assert snapshot["samples"]["/camera/image"]["sample_index"] == 1
    assert snapshot["subscriptions"]["/camera/image"]["fields"]["data"] == "sequence<uint8>"
    assert snapshot["topics"][0]["name"] == "/camera/image"
    assert "jpeg_base64" not in json.dumps(snapshot)
    assert set(node.image_snapshots) == {"/camera/image"}


def test_compact_callback_retains_all_logs_and_only_queues_probe_started(monkeypatch):
    node, callbacks, logs = install_ros_boundary(monkeypatch, [("/camera/image", "sensor_msgs/msg/Image")])
    # The public constructor owns the compact option; rebuild after the ROS
    # boundary has been installed, without replacing any diagnostic methods.
    queued = []
    reporter = SimpleNamespace(emit=lambda event, data: queued.append((event, data)))
    compact = probe.create_probe_node(reporter=reporter, compact_callback=True)
    callbacks["/camera/image"](raw_image())
    compact.heartbeat()
    assert [event for event, _ in queued] == ["probe_started"]
    assert any('"event":"subscription"' in line for line in logs)
    assert any('"event":"sample"' in line for line in logs)
    assert any('"event":"image_sample"' in line for line in logs)


@pytest.mark.parametrize("infos,want", [
    ([], (1, 1)),
    ([(2, 1)], (2, 1)),
    ([(2, 2), (2, 2)], (2, 2)),
    ([(1, 1), (2, 2)], (1, 1)),
])
def test_probe_qos_uses_publisher_compatibility_instead_of_always_best_effort(monkeypatch, infos, want):
    node, _, _ = install_ros_boundary(monkeypatch, [])
    from sys import modules
    rosnode = modules["rclpy.node"].Node
    modules["rclpy.qos"].ReliabilityPolicy.RELIABLE = 2
    captured = []
    monkeypatch.setattr(rosnode, "get_topic_names_and_types", lambda _: [("/scan", ["sensor_msgs/msg/LaserScan"])])
    monkeypatch.setattr(rosnode, "get_publishers_info_by_topic", lambda *_: [
        SimpleNamespace(node_namespace="/", node_name="sensor", qos_profile=SimpleNamespace(reliability=r, durability=d))
        for r, d in infos])
    monkeypatch.setattr(rosnode, "create_subscription", lambda self, cls, name, callback, qos: captured.append(qos))
    probe.create_probe_node()
    assert (captured[0].reliability, captured[0].durability) == want


def test_snapshot_discovers_subscriber_only_control_endpoints_without_publishing(monkeypatch):
    node, callbacks, _ = install_ros_boundary(monkeypatch, [])
    from sys import modules
    rosnode = modules["rclpy.node"].Node
    monkeypatch.setattr(rosnode, "count_publishers", lambda self, name: 0)
    monkeypatch.setattr(rosnode, "count_subscribers", lambda self, name: 1 if name == "/cmd_vel" else 0)
    endpoint = SimpleNamespace(node_name="drive", node_namespace="/robot", topic_type="geometry_msgs/msg/Twist",
                               qos_profile=SimpleNamespace(reliability="RELIABLE", durability="VOLATILE"))
    monkeypatch.setattr(rosnode, "get_subscriptions_info_by_topic",
                        lambda self, name: [endpoint] if name == "/cmd_vel" else [], raising=False)
    snapshot = node.probe_snapshot()
    assert "public_endpoints" in snapshot, "subscriber-only public interfaces are not inspected"
    assert set(snapshot["public_endpoints"]) == set(probe.PUBLIC_EXACT_TOPICS)
    control = snapshot["public_endpoints"]["/cmd_vel"]
    assert (control["publishers"], control["subscribers"]) == (0, 1)
    assert control["subscription_details_available"] is True
    assert control["subscriber_endpoints"] == [{"node": "/robot/drive", "type": "geometry_msgs/msg/Twist",
                                                "reliability": "RELIABLE", "durability": "VOLATILE"}]
    assert callbacks == {}


def test_snapshot_marks_missing_endpoint_api_and_bounds_endpoint_metadata(monkeypatch):
    node, _, _ = install_ros_boundary(monkeypatch, [])
    snapshot = node.probe_snapshot()
    assert "public_endpoints" in snapshot, "public endpoint snapshot is missing"
    assert snapshot["public_endpoints"]["/cmd_vel"]["subscription_details_available"] is False
    assert snapshot["public_endpoints"]["/cmd_vel"]["subscriber_endpoints"] == []
    from sys import modules
    endpoint = SimpleNamespace(node_name="n" * 4096, node_namespace="/" + "s" * 4096,
                               topic_type="t" * 4096,
                               qos_profile=SimpleNamespace(reliability="r" * 4096, durability="d" * 4096))
    monkeypatch.setattr(modules["rclpy.node"].Node, "get_subscriptions_info_by_topic",
                        lambda *_: [endpoint] * 100, raising=False)
    bounded = node.probe_snapshot()
    assert len(bounded["public_endpoints"]["/cmd_vel"]["subscriber_endpoints"]) == 4
    assert bounded["public_endpoints"]["/cmd_vel"]["subscriber_endpoints_omitted"] == 96
    assert len(telemetry.DiagnosticReporter._encode("probe_snapshot", bounded)) < 65536


@pytest.mark.parametrize("cause", ["size", "depth", "topic_key"])
def test_final_snapshot_falls_back_to_bounded_partial_sections_and_still_sends_image(monkeypatch, tmp_path, cause):
    from tools import diagnostic_receiver
    node, callbacks, _ = install_ros_boundary(monkeypatch, [("/camera/image", "sensor_msgs/msg/Image")])
    callbacks["/camera/image"](raw_image())
    if cause == "size":
        node.first_samples.update({f"/camera/{i}": {"topic": f"/camera/{i}", "message": {"data": "x" * 4096}}
                                   for i in range(17)})
    elif cause == "depth":
        message = {"detections": [{"results": [{"pose": {"pose": {"position": {"x": 1.0}}}}]}]}
        node.first_samples["/inspection/detections"] = {"topic": "/inspection/detections", "sample_index": 1,
                                                        "message": probe.summarize_payload(message)}
    else:
        node.first_samples["/camera/token_overlay"] = {"topic": "/camera/token_overlay", "message": {"data": "ok"}}
    server = diagnostic_receiver.create_receiver(tmp_path / "callback.jsonl")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    reporter = telemetry.DiagnosticReporter(server.local_url)
    # Keep all ROS/image processing real, replacing only unavailable ROS transport.
    try:
        instrumented = probe.create_probe_node(reporter=reporter, compact_callback=True)
        instrumented.first_samples = node.first_samples
        instrumented.image_snapshots = node.image_snapshots
        instrumented.report_final()
        lines = server.output_path.read_text(encoding="utf-8").splitlines() if server.output_path.exists() else []
        records = [json.loads(line)["payload"] for line in lines]
        assert any(item["event"] == "image_sample" for item in records), "invalid metadata suppresses independent images"
        snapshots = [item["data"] for item in records if item["event"] == "probe_snapshot"]
        assert snapshots and all(item.get("partial") is True for item in snapshots)
        assert all(len(telemetry.DiagnosticReporter._encode("probe_snapshot", item)) <= 65536 for item in snapshots)
        if cause == "depth":
            assert any(item.get("topic") == "/inspection/detections" for item in snapshots)
        elif cause == "topic_key":
            assert any(item.get("topic") == "/camera/token_overlay" for item in snapshots)
        else:
            assert len(snapshots) <= 16
            assert any(item["event"] == "sample_error" and "limit" in item["data"]["error"].lower() for item in records)
    finally:
        reporter.close()
        server.shutdown()
        server.server_close()
        thread.join()
