"""Read-only ROS graph and public sensor/interface probe.

This diagnostic never publishes actuator commands and never reads referee-private
answers. Discovered ROS type names are used directly instead of a profile.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import time
from collections.abc import Mapping


PUBLIC_EXACT_TOPICS = (
    "/cmd_vel", "/uav/cmd_vel", "/odom", "/uav/odom",
    "/referee/current_target", "/referee/ground_tasks",
    "/referee/recognition_status", "/referee/events_public",
    "/inspection/detections", "/referee/cruise_event",
    "/uav/laser_fire", "/uav/laser_state", "/clock",
)
PUBLIC_SENSOR_TYPES = {
    "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/LaserScan",
    "sensor_msgs/msg/PointCloud2",
}
CONTROL_TOPICS = {"/cmd_vel", "/uav/cmd_vel"}
IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}


def select_public_topics(topics):
    """Filter (name, type) pairs before creating any subscriptions."""
    result = []
    for name, ros_type in topics:
        if (name.startswith("/gazebo/") or any(
                part in name.lower() for part in ("private", "truth", "hidden", "answer"))):
            continue
        if name.startswith("/referee/") and name not in PUBLIC_EXACT_TOPICS:
            continue
        if (name in PUBLIC_EXACT_TOPICS or name in ("/tf", "/tf_static")
                or ros_type in PUBLIC_SENSOR_TYPES):
            result.append((name, ros_type))
    return sorted(set(result))


def summarize_payload(payload, max_items=8, depth=0):
    """Preserve real field names while bounding arrays and binary sensor data."""
    if depth >= 8:
        return "<depth limit>"
    if isinstance(payload, Mapping):
        return {str(k): summarize_payload(v, max_items, depth + 1)
                for k, v in list(payload.items())[:64]}
    if isinstance(payload, (bytes, bytearray, memoryview)):
        return {"byte_length": len(payload)}
    if isinstance(payload, (list, tuple)) or hasattr(payload, "tolist"):
        values = payload[:max_items].tolist() if hasattr(payload, "tolist") else payload
        return [summarize_payload(v, max_items, depth + 1) for v in values[:max_items]]
    if isinstance(payload, float) and not math.isfinite(payload):
        return str(payload)
    if isinstance(payload, str):
        return payload[:4096]
    if payload is None or isinstance(payload, (bool, int, float)):
        return payload
    return str(payload)[:512]


def summarize_sensor_payload(payload, ros_type):
    """Keep calibration matrices whole while bounding lidar and binary data."""
    summary = summarize_payload(payload)
    if not isinstance(payload, Mapping):
        return summary
    if ros_type == "sensor_msgs/msg/CameraInfo":
        for name in ("k", "r", "p", "d"):
            if name in payload:
                summary[name] = summarize_payload(payload[name], max_items=128)
    elif ros_type == "sensor_msgs/msg/LaserScan":
        for name in ("ranges", "intensities"):
            if name in payload:
                summary[name] = summarize_payload(payload[name], max_items=512)
    if ros_type in IMAGE_TYPES and "data" in payload:
        summary["data"] = {"byte_length": len(payload["data"])}
    return summary


def build_image_sample(msg, ros_type, topic):
    """Encode a real sensor frame, respecting ROS row stride and bounded size.

    OpenCV and NumPy are optional for the probe. Import or decode errors are
    handled by receive() as diagnostics; no placeholder image is generated.
    """
    import cv2
    import numpy as np

    if ros_type not in IMAGE_TYPES or not select_public_topics([(topic, ros_type)]):
        raise ValueError("image topic is not a public sensor")
    header = msg.header
    sec, nanos = header.stamp.sec, header.stamp.nanosec
    if (type(sec) is not int or type(nanos) is not int or sec < 0
            or not 0 <= nanos < 1_000_000_000 or not isinstance(header.frame_id, str)
            or len(header.frame_id) > 256 or not isinstance(topic, str) or len(topic) > 256):
        raise ValueError("invalid image header")
    raw = memoryview(msg.data).cast("B")
    if len(raw) > 64 * 1024 * 1024:
        raise ValueError("sensor image buffer exceeds diagnostic decode limit")
    if ros_type == "sensor_msgs/msg/Image":
        width, height, step = msg.width, msg.height, msg.step
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
        if (channels is None or type(width) is not int or type(height) is not int
                or type(step) is not int or min(width, height) <= 0
                or width * height > 16_777_216 or step < width * channels
                or len(raw) != step * height):
            raise ValueError("unsupported encoding or invalid image dimensions/step/data")
        # Strip padding row by row; flattening the full buffer would shift rows.
        image = np.frombuffer(raw, dtype=np.uint8).reshape(height, step)[:, :width * channels]
        image = image.reshape(height, width, channels) if channels == 3 else image.reshape(height, width)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    else:
        image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.size == 0 or image.shape[0] * image.shape[1] > 16_777_216:
            raise ValueError("compressed sensor image cannot be decoded within diagnostic limits")
        height, width = image.shape[:2]
    scale = min(1.0, 480 / width, 360 / height)
    out_width, out_height = max(1, int(width * scale)), max(1, int(height * scale))
    image = cv2.resize(image, (out_width, out_height), interpolation=cv2.INTER_AREA)
    data = {"topic": topic, "stamp": sec + nanos / 1e9, "frame_id": header.frame_id,
            "width": out_width, "height": out_height, "encoding": "jpeg"}
    # Reserve space for event/timestamp framing and JSON separators. Reduce JPEG
    # quality first, then resolution; never truncate the encoded byte stream.
    while True:
        for quality in (80, 65, 50, 35, 20):
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                raise ValueError("diagnostic JPEG encoding failed")
            data["jpeg_base64"] = base64.b64encode(encoded.tobytes()).decode("ascii")
            if len(json.dumps(data, ensure_ascii=False).encode("utf-8")) < 64000:
                return data
        if max(image.shape[:2]) <= 32:
            raise ValueError("diagnostic JPEG cannot fit event size limit")
        out_width, out_height = max(1, int(out_width * .75)), max(1, int(out_height * .75))
        image = cv2.resize(image, (out_width, out_height), interpolation=cv2.INTER_AREA)
        data.update(width=out_width, height=out_height)


def snapshot_fragments(snapshot, max_parts=16):
    """Keep ordinary snapshots whole; explicitly mark bounded fallback parts.

    Topic keys become values in fallback records, so legitimate sensor names
    cannot be mistaken for credential field names. No part is claimed complete.
    """
    from .telemetry import DiagnosticReporter

    def fits(data):
        try:
            DiagnosticReporter._encode("probe_snapshot", data)
            return True
        except ValueError:
            return False

    if fits(snapshot):
        return [snapshot], []
    common = {"read_only": True, "official_scoring": "unverified", "partial": True}
    metadata = {**snapshot, **common, "section": "metadata", "samples": {}}
    for section in ("subscriptions", "sample_counts", "errors", "public_endpoints"):
        if section in metadata:
            metadata[section] = [{"topic": name, "value": value}
                                 for name, value in metadata[section].items()]
    candidates = []
    if fits(metadata):
        candidates.append(metadata)
    else:
        for section, value in snapshot.items():
            if section in ("read_only", "official_scoring", "samples"):
                continue
            candidate = {**common, "section": section, "value": value}
            if fits(candidate) or not isinstance(value, Mapping):
                candidates.append(candidate)
            else:
                candidates.extend({**common, "section": section, "topic": name, "value": item}
                                  for name, item in value.items())
    candidates.extend({**common, "section": "samples", "topic": name, "sample": sample}
                      for name, sample in snapshot.get("samples", {}).items())
    parts, errors = [], []
    for candidate in candidates:
        if not fits(candidate):
            errors.append("snapshot section could not fit size/depth limits: " + str(candidate.get("section")))
        elif len(parts) < max_parts:
            parts.append(candidate)
        else:
            errors.append("snapshot fragment limit reached; remaining sections omitted")
            break
    return parts, errors


def create_probe_node(duration_s=30.0, reporter=None, compact_callback=False):
    """Construct a diagnostic node after importing the installed ROS runtime."""
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    class InterfaceProbe(Node):
        def __init__(self):
            super().__init__("inspection_interface_probe")
            self.started = time.monotonic()
            self.duration_s = duration_s
            self.finished = False
            self.subscriptions_by_topic = {}
            self.samples = {}
            self.first_samples = {}
            self.subscription_schemas = {}
            self.public_graph = []
            self.errors = {}
            self.image_snapshots = {}
            self.image_attempted = set()
            self.observed_graph = None
            self.create_timer(1.0, self.inspect_graph)
            self.create_timer(5.0, self.heartbeat)
            self.emit("probe_started", {"read_only": True, "duration_s": duration_s})
            self.inspect_graph()

        def emit(self, event, value, *, network=True):
            self.get_logger().info("INTERFACE_PROBE " + json.dumps(
                {"event": event, **value}, ensure_ascii=False, allow_nan=False,
                separators=(",", ":")))
            if event in ("sample_error", "subscription_error"):
                self.errors[value.get("topic", "probe")] = value.get("error", "")[:512]
            if reporter is not None and network and (not compact_callback or event == "probe_started"):
                reporter.emit(event, value)

        def inspect_graph(self):
            if time.monotonic() - self.started >= self.duration_s:
                self.finished = True
                return
            pairs = [(name, ros_type) for name, types in self.get_topic_names_and_types()
                     for ros_type in types]
            public = select_public_topics(pairs)
            if public != self.observed_graph:
                self.observed_graph = public
                self.public_graph = [
                    {"name": name, "type": ros_type,
                     "publishers": self.count_publishers(name),
                     "subscribers": self.count_subscribers(name)}
                    for name, ros_type in public]
                self.emit("public_graph", {"topics": self.public_graph})
            for name, ros_type in public:
                if name in self.subscriptions_by_topic or name in CONTROL_TOPICS:
                    continue
                try:
                    cls = get_message(ros_type)
                    infos = self.get_publishers_info_by_topic(name)
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
                    if infos and all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos):
                        qos.reliability = ReliabilityPolicy.RELIABLE
                    if infos and all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos):
                        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                    schema = {"topic": name, "type": ros_type,
                        "fields": cls.get_fields_and_field_types(),
                        "publishers": [{"node": i.node_namespace + "/" + i.node_name,
                            "reliability": str(i.qos_profile.reliability),
                            "durability": str(i.qos_profile.durability)} for i in infos]}
                    self.subscription_schemas[name] = schema
                    self.emit("subscription", schema)
                    self.subscriptions_by_topic[name] = self.create_subscription(
                        cls, name, lambda msg, name=name, ros_type=ros_type: self.receive(name, msg, ros_type), qos)
                except Exception as exc:
                    self.emit("subscription_error", {"topic": name, "error": str(exc)})

        def receive(self, name, msg, ros_type=""):
            self.samples[name] = self.samples.get(name, 0) + 1
            if ros_type in IMAGE_TYPES and name not in self.image_attempted:
                self.image_attempted.add(name)
                try:
                    image = build_image_sample(msg, ros_type, name)
                    self.image_snapshots[name] = image
                    self.emit("image_sample", image, network=False)
                except Exception as exc:
                    self.emit("sample_error", {"topic": name, "error": str(exc)[:512]})
            if self.samples[name] > 2:
                return
            try:
                sample = {"topic": name, "sample_index": self.samples[name],
                          "message": summarize_sensor_payload(message_to_ordereddict(msg), ros_type)}
                self.first_samples.setdefault(name, sample)
                self.emit("sample", sample)
            except Exception as exc:
                self.emit("sample_error", {"topic": name, "error": str(exc)})

        def heartbeat(self):
            self.emit("heartbeat", {"elapsed_s": round(time.monotonic() - self.started, 2),
                                    "sample_counts": self.samples})

        def public_endpoints_snapshot(self):
            """Inspect even subscriber-only public controls without publishing."""
            detail_method = getattr(self, "get_subscriptions_info_by_topic", None)
            result = {}
            for name in PUBLIC_EXACT_TOPICS:
                entry = {"subscription_details_available": callable(detail_method), "subscriber_endpoints": []}
                try:
                    entry.update(publishers=self.count_publishers(name), subscribers=self.count_subscribers(name))
                    if callable(detail_method):
                        infos = detail_method(name)
                        entry["subscriber_endpoints"] = [
                            {"node": (i.node_namespace.rstrip("/") + "/" + i.node_name)[:128],
                             "type": str(i.topic_type)[:128],
                             "reliability": str(i.qos_profile.reliability)[:48],
                             "durability": str(i.qos_profile.durability)[:48]} for i in infos[:4]]
                        if len(infos) > 4:
                            entry["subscriber_endpoints_omitted"] = len(infos) - 4
                except Exception as exc:
                    entry["error"] = str(exc)[:512]
                result[name] = entry
            return result

        def probe_snapshot(self):
            return {"read_only": True, "official_scoring": "unverified",
                    "elapsed_s": round(time.monotonic() - self.started, 2),
                    "topics": self.public_graph, "subscriptions": self.subscription_schemas,
                    "samples": self.first_samples, "sample_counts": dict(self.samples),
                    "errors": dict(self.errors), "public_endpoints": self.public_endpoints_snapshot()}

        def report_final(self):
            """Call only after spin has ended; images remain independent events."""
            snapshot = self.probe_snapshot()
            self.emit("probe_snapshot", snapshot, network=False)
            if reporter is None:
                return
            parts, errors = snapshot_fragments(snapshot)
            events = [("probe_snapshot", part) for part in parts]
            events += [("sample_error", {"topic": "probe_final", "error": error}) for error in errors]
            events += [("image_sample", image) for image in self.image_snapshots.values()]
            deadline = time.monotonic() + 30
            for event, value in events:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.emit("sample_error", {"topic": "probe_final", "error": "final callback time limit reached"},
                              network=False)
                    break
                try:
                    reporter.send_now(event, value, timeout_s=min(8, remaining))
                except ValueError as exc:
                    # One rejected section must never suppress later images.
                    self.emit("sample_error", {"topic": "probe_final", "error": str(exc)[:512]}, network=False)

    return InterfaceProbe()


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="Read-only public ROS interface diagnostic")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parser.add_argument("--compact-callback", action="store_true",
                        help="Send only start and final read-only evidence over the optional network callback")
    parsed, ros_args = parser.parse_known_args(args)
    if not math.isfinite(parsed.duration) or not 1 <= parsed.duration <= 180:
        parser.error("duration must lie in [1,180] seconds")
    rclpy.init(args=ros_args)
    node = None
    reporter = DiagnosticReporter(parsed.report_url)
    try:
        node = create_probe_node(parsed.duration, reporter=reporter, compact_callback=parsed.compact_callback)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
        node.emit("probe_finished", {"read_only": True, "sample_counts": node.samples})
        node.report_final()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
