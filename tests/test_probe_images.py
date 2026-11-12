"""Real image adapters retain sensor content while bounding diagnostic snapshots."""
import base64
import builtins
import json
import sys
from array import array
from types import ModuleType, SimpleNamespace

import pytest

from inspection_mission import probe, telemetry


def header():
    return SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=250_000_000), frame_id="camera_optical")


def raw_image(encoding="bgr8", *, width=64, height=32, padding=5):
    pixels = {"bgr8": bytes((0, 0, 255)), "rgb8": bytes((255, 0, 0)), "mono8": bytes((150,))}[encoding]
    row = pixels * width + b"\x01" * padding
    return SimpleNamespace(header=header(), width=width, height=height,
                           step=len(row), encoding=encoding, is_bigendian=0,
                           data=array("B", row * height))


def image_adapter():
    assert hasattr(probe, "build_image_sample"), "public image snapshot adapter is missing"
    return probe.build_image_sample


@pytest.mark.parametrize("encoding", ["rgb8", "bgr8", "mono8"])
def test_raw_snapshot_obeys_row_step_color_order_and_real_header(encoding):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    data = image_adapter()(raw_image(encoding), "sensor_msgs/msg/Image", "/camera/image")
    assert set(data) == {"topic", "stamp", "frame_id", "width", "height", "encoding", "jpeg_base64"}
    assert (data["topic"], data["stamp"], data["frame_id"]) == ("/camera/image", 7.25, "camera_optical")
    assert (data["width"], data["height"], data["encoding"]) == (64, 32, "jpeg")
    decoded = cv2.imdecode(np.frombuffer(base64.b64decode(data["jpeg_base64"]), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (32, 64, 3)
    if encoding == "mono8":
        assert decoded.mean() == pytest.approx(150, abs=4)
    else:
        assert decoded[:, :, 2].mean() > 245
        assert decoded[:, :, :2].mean() < 8


@pytest.mark.parametrize("shape", [(1080, 1920), (1920, 1080)])
def test_noisy_compressed_snapshot_is_scaled_and_event_stays_strictly_below_64k(shape):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    original = np.random.default_rng(12).integers(0, 256, (*shape, 3), dtype=np.uint8)
    ok, png = cv2.imencode(".png", original)
    assert ok
    msg = SimpleNamespace(header=header(), format="png", data=array("B", png.tobytes()))
    data = image_adapter()(msg, "sensor_msgs/msg/CompressedImage", "/uav/camera/image/compressed")
    assert data["width"] <= 480 and data["height"] <= 360
    assert abs(data["width"] / data["height"] - shape[1] / shape[0]) < 0.02
    decoded = cv2.imdecode(np.frombuffer(base64.b64decode(data["jpeg_base64"]), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[:2] == (data["height"], data["width"])
    event = {"event": "image_sample", "timestamp": "2026-11-11T00:00:00.123456+00:00", "data": data}
    assert len(json.dumps(event, ensure_ascii=False).encode()) < 65536


@pytest.mark.parametrize("changes", [
    {"step": 1}, {"height": 0}, {"width": True}, {"data": b"\0"},
    {"encoding": "16UC1"}, {"step": 64 * 3 - 1},
])
def test_malformed_raw_snapshot_is_rejected_instead_of_reshaped_or_relabelled(changes):
    msg = raw_image()
    vars(msg).update(changes)
    with pytest.raises((ValueError, ImportError)):
        image_adapter()(msg, "sensor_msgs/msg/Image", "/camera/image")


def test_bad_compressed_snapshot_is_rejected():
    with pytest.raises((ValueError, ImportError)):
        image_adapter()(SimpleNamespace(header=header(), format="jpeg", data=b"not a jpeg"),
                        "sensor_msgs/msg/CompressedImage", "/camera/image/compressed")


def test_image_dependency_failure_is_explicit_and_import_is_optional(monkeypatch):
    original = builtins.__import__

    def unavailable(name, *args, **kwargs):
        if name in ("cv2", "numpy"):
            raise ImportError("image dependency unavailable")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", unavailable)
    with pytest.raises(ImportError, match="image dependency"):
        image_adapter()(raw_image(), "sensor_msgs/msg/Image", "/camera/image")
    assert probe.select_public_topics([("/scan", "sensor_msgs/msg/LaserScan")])


def install_ros_boundary(monkeypatch, topics):
    """Only replace unavailable ROS transport; image encode/decode remains real."""
    callbacks, logs = {}, []

    class Node:
        def __init__(self, name):
            self.name = name

        def create_timer(self, *_):
            pass

        def get_topic_names_and_types(self):
            return [(topic, [ros_type]) for topic, ros_type in topics]

        def count_publishers(self, _):
            return 1

        def count_subscribers(self, _):
            return 0

        def get_publishers_info_by_topic(self, _):
            return []

        def create_subscription(self, cls, name, callback, qos):
            callbacks[name] = callback
            return callback

        def get_logger(self):
            return SimpleNamespace(info=logs.append)

    class Message:
        @classmethod
        def get_fields_and_field_types(cls):
            return {"header": "std_msgs/Header", "data": "sequence<uint8>"}

    modules = {
        "rclpy": ModuleType("rclpy"), "rclpy.node": ModuleType("rclpy.node"),
        "rclpy.qos": ModuleType("rclpy.qos"),
        "rosidl_runtime_py": ModuleType("rosidl_runtime_py"),
        "rosidl_runtime_py.convert": ModuleType("rosidl_runtime_py.convert"),
        "rosidl_runtime_py.utilities": ModuleType("rosidl_runtime_py.utilities"),
    }
    modules["rclpy.node"].Node = Node
    modules["rclpy.qos"].QoSProfile = lambda **kw: SimpleNamespace(**kw)
    modules["rclpy.qos"].ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
    modules["rclpy.qos"].DurabilityPolicy = SimpleNamespace(VOLATILE=1, TRANSIENT_LOCAL=2)
    modules["rosidl_runtime_py.convert"].message_to_ordereddict = lambda msg: vars(msg)
    modules["rosidl_runtime_py.utilities"].get_message = lambda _: Message
    for name, value in modules.items():
        monkeypatch.setitem(sys.modules, name, value)
    node = probe.create_probe_node()
    return node, callbacks, logs


def test_probe_emits_at_most_one_real_snapshot_per_topic_without_publishing(monkeypatch):
    topics = [("/camera/image", "sensor_msgs/msg/Image"), ("/uav/camera/image", "sensor_msgs/msg/Image")]
    node, callbacks, logs = install_ros_boundary(monkeypatch, topics)
    for _ in range(4):
        for callback in callbacks.values():
            callback(raw_image())
    images = [json.loads(line.removeprefix("INTERFACE_PROBE ")) for line in logs if '"event":"image_sample"' in line]
    assert len(images) == 2
    assert {item["topic"] for item in images} == {"/camera/image", "/uav/camera/image"}
    assert node.samples == {"/camera/image": 4, "/uav/camera/image": 4}


def test_probe_image_dependency_error_does_not_interrupt_counts_or_other_topics(monkeypatch):
    node, callbacks, logs = install_ros_boundary(monkeypatch, [("/camera/image", "sensor_msgs/msg/Image")])
    original = builtins.__import__

    def unavailable(name, *args, **kwargs):
        if name in ("cv2", "numpy"):
            raise ImportError("image dependency unavailable")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", unavailable)
    for _ in range(4):
        callbacks["/camera/image"](raw_image())
    errors = [line for line in logs if '"event":"sample_error"' in line]
    assert len(errors) == 1
    assert "image dependency unavailable" in errors[0]
    assert node.samples["/camera/image"] == 4
    node.heartbeat()


def test_public_event_topics_use_discovered_types_but_never_allow_other_referee_topics():
    topics = [("/inspection/detections", "std_msgs/msg/String"),
              ("/referee/cruise_event", "std_msgs/msg/String"),
              ("/uav/laser_fire", "std_msgs/msg/Bool"),
              ("/uav/laser_state", "std_msgs/msg/Bool"),
              ("/referee/texture_assignment", "std_msgs/msg/String")]
    assert probe.select_public_topics(topics) == sorted(topics[:-1])


def test_sensor_type_does_not_make_private_or_truth_namespaces_public():
    topics = [("/ican_referee_private/camera", "sensor_msgs/msg/Image"),
              ("/robot/ground_truth/image", "sensor_msgs/msg/Image"),
              ("/robot/hidden_targets/image", "sensor_msgs/msg/CompressedImage")]
    assert probe.select_public_topics(topics) == []


def test_reporter_accepts_bounded_image_snapshot_and_rejects_extra_fields():
    data = image_adapter()(raw_image(), "sensor_msgs/msg/Image", "/camera/image")
    with telemetry.DiagnosticReporter("") as reporter:
        assert reporter.emit("image_sample", data) is False
        with pytest.raises(ValueError):
            reporter.emit("image_sample", {**data, "score": 100})
