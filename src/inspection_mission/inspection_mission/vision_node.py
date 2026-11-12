"""Lightweight illumination-tolerant QR/ArUco detector."""

from __future__ import annotations

import json

import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .perception import camera_target_position
from .vision_core import TemplateMatcher, detect_targets, normalise_template_paths


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("inspection_vision")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ground_image_topic", "")
        self.declare_parameter("uav_image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("ground_camera_info_topic", "")
        self.declare_parameter("uav_camera_info_topic", "")
        self.declare_parameter("detection_topic", "/inspection_mission/observations")
        self.declare_parameter("min_detection_interval_s", 0.25)
        self.declare_parameter("enable_qr", True)
        self.declare_parameter("enable_aruco", True)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("template_paths_json", "{}")
        self.declare_parameter("target_sizes_json", "{}")
        self._last_processed_at: dict[str, float] = {}
        self._camera_calibrations: dict[str, tuple] = {}
        paths = json.loads(str(self.get_parameter("template_paths_json").value))
        sizes = json.loads(str(self.get_parameter("target_sizes_json").value))
        if not isinstance(paths, dict) or not isinstance(sizes, dict):
            raise ValueError("template_paths_json and target_sizes_json must be JSON objects")
        paths = normalise_template_paths(paths)
        self.template_matcher = TemplateMatcher(paths) if paths else None
        self.target_sizes = {}
        for target_id, dimensions in sizes.items():
            size = np.asarray(dimensions, dtype=float)
            if size.shape != (2,) or not np.isfinite(size).all() or (size <= 0).any():
                raise ValueError(f"invalid physical width/height for target {target_id}")
            self.target_sizes[str(target_id)] = tuple(size.tolist())
        info_topics = [self.get_parameter(name).value for name in (
            "camera_info_topic", "ground_camera_info_topic", "uav_camera_info_topic")]
        if any(not isinstance(topic, str) or (topic and not topic.startswith("/")) for topic in info_topics):
            raise ValueError("CameraInfo topics must be empty or verified absolute topic names")
        if self.target_sizes and not any(info_topics):
            raise ValueError("physical target sizes require at least one verified CameraInfo topic")
        self.bridge = None
        self.cv2 = None
        try:
            import cv2
            from cv_bridge import CvBridge

            self.cv2 = cv2
            self.bridge = CvBridge()
        except Exception as exc:  # pragma: no cover - depends on platform image
            self.get_logger().error(f"OpenCV/cv_bridge unavailable: {exc}")
        self.pub = self.create_publisher(String, str(self.get_parameter("detection_topic").value), 10)
        legacy_topic = str(self.get_parameter("image_topic").value).strip()
        ground_topic = str(self.get_parameter("ground_image_topic").value).strip() or legacy_topic
        uav_topic = str(self.get_parameter("uav_image_topic").value).strip()
        self._image_subscriptions = []
        self._camera_info_subscriptions = []
        seen_topics: set[str] = set()
        for topic, source in ((ground_topic, "ground"), (uav_topic, "uav")):
            if not topic or topic in seen_topics:
                continue
            seen_topics.add(topic)
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, source=source: self._image_cb(msg, source),
                    qos_profile_sensor_data,
                )
            )
        legacy_info = str(self.get_parameter("camera_info_topic").value).strip()
        ground_info = str(self.get_parameter("ground_camera_info_topic").value).strip() or legacy_info
        uav_info = str(self.get_parameter("uav_camera_info_topic").value).strip()
        for topic, source in ((ground_info, "ground"), (uav_info, "uav")):
            if topic:
                self._camera_info_subscriptions.append(self.create_subscription(
                    CameraInfo, topic,
                    lambda msg, source=source: self._camera_info_cb(msg, source),
                    qos_profile_sensor_data,
                ))
        self.get_logger().info("inspection vision node ready")

    def _camera_info_cb(self, msg: CameraInfo, source: str = "ground") -> None:
        """Use only calibrated, supported raw-image intrinsics for this source."""
        try:
            matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
            distortion = np.asarray(msg.d, dtype=float)
            if (msg.width <= 0 or msg.height <= 0 or not np.isfinite(matrix).all()
                    or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                    or msg.distortion_model not in ("plumb_bob", "rational_polynomial")
                    or not np.isfinite(distortion).all()):
                raise ValueError("uncalibrated or unsupported CameraInfo")
            self._camera_calibrations[source] = (int(msg.width), int(msg.height), matrix,
                                                 distortion, str(msg.header.frame_id))
        except (TypeError, ValueError) as exc:
            self._camera_calibrations.pop(source, None)
            self.get_logger().warning(f"{source} calibration skipped: {exc}")

    def _image_cb(self, msg: Image, source: str = "ground") -> None:
        if self.bridge is None or self.cv2 is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        previous = self._last_processed_at.get(source)
        if previous is not None and 0 <= now - previous < float(self.get_parameter("min_detection_interval_s").value):
            return
        self._last_processed_at[source] = now
        detections = []
        error = ""
        width = height = 0
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = image.shape[:2]
            detections = detect_targets(
                image,
                enable_qr=bool(self.get_parameter("enable_qr").value),
                enable_aruco=bool(self.get_parameter("enable_aruco").value),
                aruco_dictionary=str(self.get_parameter("aruco_dictionary").value),
                template_matcher=self.template_matcher,
            )
            calibration = self._camera_calibrations.get(source)
            if calibration is not None:
                calib_width, calib_height, matrix, distortion, frame = calibration
                # Cropping/binning cannot be inferred from image dimensions.
                if ((width, height) == (calib_width, calib_height)
                        and frame and frame == msg.header.frame_id):
                    for detection in detections:
                        size = self.target_sizes.get(detection["target_id"])
                        if size is None:
                            continue
                        position = camera_target_position(detection["corners_px"], matrix, distortion,
                                                          target_size_m=size)
                        if position is not None:
                            detection["position_camera_m"] = list(position)
        except Exception as exc:  # keep the mission alive on a bad frame
            self.get_logger().warning(f"vision frame skipped: {exc}")
            detections = []
            error = str(exc)[:200]
        out = String()
        out.data = json.dumps({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "received_stamp": now,
            "source": source,
            "frame_id": str(msg.header.frame_id),
            "image_width": width,
            "image_height": height,
            "detections": detections,
            "error": error,
        }, ensure_ascii=False, allow_nan=False)
        # An empty result is an observation and must reset recognition dwell.
        self.pub.publish(out)


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

