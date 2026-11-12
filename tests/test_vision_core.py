import cv2
import numpy as np
import pytest
import importlib.util
import json
import sys
import types
from pathlib import Path

from inspection_mission import vision_core
from inspection_mission.vision_core import detect_targets


def test_qr_detection_survives_contrast_change():
    encoder = cv2.QRCodeEncoder_create()
    qr = encoder.encode("inspection-target-1")
    qr = cv2.resize(qr, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    qr = cv2.copyMakeBorder(qr, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    image = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    image = np.clip(image.astype(np.float32) * 0.55 + 18.0, 0, 255).astype(np.uint8)
    detections = detect_targets(image, enable_qr=True, enable_aruco=False)
    assert any(item["kind"] == "qr" and item["text"] == "inspection-target-1" for item in detections)


def test_blank_frame_returns_no_guess():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    assert detect_targets(image, enable_qr=True, enable_aruco=True) == []


def test_aruco_detection_uses_supported_opencv_api():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 7, 160)
    marker = cv2.copyMakeBorder(marker, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    image = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    detections = detect_targets(image, enable_qr=False, enable_aruco=True)
    assert any(item["kind"] == "aruco" and item["id"] == 7 for item in detections)


@pytest.mark.parametrize("scale,offset,rotation", [(0.18, 3.0, 0), (0.50, 95.0, 90), (1.0, 0.0, 180)])
def test_qr_identity_and_normalized_geometry_survive_lighting_and_rotation(scale, offset, rotation):
    qr = cv2.QRCodeEncoder_create().encode("team-provided-reference")
    qr = cv2.resize(qr, (232, 232), interpolation=cv2.INTER_NEAREST)
    qr = cv2.copyMakeBorder(qr, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    if rotation:
        qr = np.rot90(qr, rotation // 90).copy()
    image = np.clip(qr.astype(np.float32) * scale + offset, 0, 255).astype(np.uint8)
    results = detect_targets(image, enable_aruco=False)
    assert len(results) == 1
    result = results[0]
    assert result["target_id"] == "team-provided-reference"
    assert result["center"] == pytest.approx([0.5, 0.5], abs=0.015)
    assert 0.0 < result["confidence"] <= 1.0
    assert np.asarray(result["corners_px"]).shape == (4, 2)
    xmin, ymin, xmax, ymax = result["bbox"]
    assert 0 <= xmin < xmax < image.shape[1]
    assert 0 <= ymin < ymax < image.shape[0]


def test_aruco_has_shared_identity_and_geometry_contract():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 19, 160)
    image = cv2.copyMakeBorder(marker, 40, 40, 80, 80, cv2.BORDER_CONSTANT, value=255)
    result = detect_targets(image, enable_qr=False)[0]
    assert result["target_id"] == "aruco:19"
    assert result["center"] == pytest.approx([0.5, 0.5], abs=0.005)
    assert result["bbox"] == pytest.approx([80.0, 40.0, 239.0, 199.0], abs=1.0)


def test_perspective_quad_center_is_the_projected_surface_center():
    quad = np.asarray([[0.0, 0.0], [4.0, 0.0], [6.0, 4.0], [0.0, 2.0]])

    result = vision_core._detection("template", "reference", quad, (8, 8))

    assert result["center_px"] == pytest.approx([12 / 7, 8 / 7])
    assert result["center"] == pytest.approx([3 / 14, 1 / 7])


def _reference_image(seed):
    """Test-only synthetic artwork; these IDs are not competition answers."""
    rng = np.random.default_rng(seed)
    image = np.full((300, 380), 225, dtype=np.uint8)
    for _ in range(160):
        x, y = rng.integers([10, 10], [370, 290])
        cv2.circle(image, (int(x), int(y)), int(rng.integers(2, 9)), int(rng.integers(0, 200)), -1)
    cv2.putText(image, "TEST REFERENCE", (24, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 15, 2)
    return image


@pytest.mark.parametrize("scale,offset", [(0.3, 5.0), (0.55, 90.0)])
def test_reference_template_matches_rotated_perspective_artwork(tmp_path, scale, offset):
    reference = _reference_image(41)
    template_path = tmp_path / "reference.png"
    template_path.write_bytes(cv2.imencode(".png", reference)[1].tobytes())
    assert hasattr(vision_core, "TemplateMatcher"), "configured reference matcher is missing"
    matcher = vision_core.TemplateMatcher({"user-reference": str(template_path)})
    # Corners are hand-selected, with rotation, scale and perspective distortion.
    source = np.float32([[0, 0], [379, 0], [379, 299], [0, 299]])
    expected = np.float32([[180, 70], [520, 170], [430, 440], [80, 310]])
    transform = cv2.getPerspectiveTransform(source, expected)
    image = cv2.warpPerspective(reference, transform, (640, 480), borderValue=225)
    image = np.clip(image.astype(np.float32) * scale + offset, 0, 255).astype(np.uint8)
    results = detect_targets(image, enable_qr=False, enable_aruco=False, template_matcher=matcher)
    assert len(results) == 1
    assert results[0]["target_id"] == "user-reference"
    assert np.asarray(results[0]["corners_px"]) == pytest.approx(expected, abs=7.0)
    assert results[0]["confidence"] >= 0.5


def test_reference_matcher_rejects_unrelated_and_blank_frames(tmp_path):
    path = tmp_path / "reference.png"
    path.write_bytes(cv2.imencode(".png", _reference_image(41))[1].tobytes())
    assert hasattr(vision_core, "TemplateMatcher"), "configured reference matcher is missing"
    matcher = vision_core.TemplateMatcher({"user-reference": str(path)})
    rng = np.random.default_rng(92)
    for image in [np.zeros((480, 640), np.uint8), rng.integers(0, 255, (480, 640), np.uint8)]:
        assert detect_targets(image, enable_qr=False, enable_aruco=False, template_matcher=matcher) == []


def test_missing_or_featureless_reference_is_explicitly_rejected(tmp_path):
    assert hasattr(vision_core, "TemplateMatcher"), "configured reference matcher is missing"
    with pytest.raises(ValueError):
        vision_core.TemplateMatcher({"user-reference": str(tmp_path / "missing.png")})
    path = tmp_path / "blank.png"
    path.write_bytes(cv2.imencode(".png", np.full((200, 200), 128, np.uint8))[1].tobytes())
    with pytest.raises(ValueError):
        vision_core.TemplateMatcher({"user-reference": str(path)})


def _vision_node_boundary(monkeypatch, overrides=None):
    """Only ROS transport is substituted; image recognition and geometry are real.

    The read-only subscriptions property mirrors rclpy.Node's public API.
    This fixture does not claim a ROS executor, DDS or real camera has run.
    """
    parameters = overrides or {}
    sensor_qos = object()

    class FakeNode:
        def __init__(self, name):
            self.params = {}
            self._clock_ns = 10_000_000_000
            self._subscription_refs = []
            self.messages = []

        @property
        def subscriptions(self):
            return iter(self._subscription_refs)

        def declare_parameter(self, name, default):
            self.params[name] = parameters.get(name, default)

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.params[name])

        def create_subscription(self, message_type, topic, callback, qos):
            ref = types.SimpleNamespace(message_type=message_type, topic=topic, callback=callback, qos=qos)
            self._subscription_refs.append(ref)
            return ref

        def create_publisher(self, message_type, topic, qos):
            return types.SimpleNamespace(publish=lambda message: self.messages.append(json.loads(message.data)))

        def get_logger(self):
            return types.SimpleNamespace(info=lambda message: None, warning=lambda message: None, error=lambda message: None)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=self._clock_ns))

    class Image:
        pass

    class CameraInfo:
        pass

    class String:
        data = ""

    modules = {
        "rclpy": {}, "rclpy.node": {"Node": FakeNode},
        "rclpy.qos": {"qos_profile_sensor_data": sensor_qos},
        "sensor_msgs": {}, "sensor_msgs.msg": {"Image": Image, "CameraInfo": CameraInfo},
        "std_msgs": {}, "std_msgs.msg": {"String": String},
        "cv_bridge": {"CvBridge": lambda: types.SimpleNamespace(imgmsg_to_cv2=lambda msg, desired_encoding: msg.image)},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(vision_core.__file__).with_name("vision_node.py")
    spec = importlib.util.spec_from_file_location("inspection_mission._vision_node_boundary", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.VisionNode(), sensor_qos


def _image_message(image, stamp=10.0):
    seconds = int(stamp)
    return types.SimpleNamespace(image=image, header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=seconds, nanosec=int(round((stamp - seconds) * 1e9))),
        frame_id="camera_optical"))


def test_vision_node_initializes_with_ros_readonly_subscriptions_and_sensor_qos(monkeypatch):
    node, sensor_qos = _vision_node_boundary(monkeypatch, {
        "ground_image_topic": "/ground/image", "uav_image_topic": "/uav/image",
        "ground_camera_info_topic": "/ground/info", "uav_camera_info_topic": "/uav/info"})
    configured = {subscription.topic: subscription.qos for subscription in node.subscriptions}
    assert set(configured) == {"/ground/image", "/uav/image", "/ground/info", "/uav/info"}
    assert all(qos is sensor_qos for qos in configured.values())


def test_each_camera_throttles_independently_and_empty_frames_reset_observation(monkeypatch):
    node, _ = _vision_node_boundary(monkeypatch)
    blank = np.zeros((240, 320, 3), np.uint8)
    node._image_cb(_image_message(blank), "ground")
    node._image_cb(_image_message(blank), "uav")
    node._image_cb(_image_message(blank), "ground")
    assert len(node.messages) == 2
    assert [item["source"] for item in node.messages] == ["ground", "uav"]
    assert all(item["stamp"] == 10.0 and item["detections"] == [] for item in node.messages)
    node._clock_ns = 10_300_000_000
    node._image_cb(_image_message(blank, 10.3), "ground")
    assert node.messages[-1]["stamp"] == pytest.approx(10.3)
    assert len(node.messages) == 3


def test_vision_node_consumes_source_camera_info_only_with_known_target_size(monkeypatch):
    node, _ = _vision_node_boundary(monkeypatch, {
        "target_sizes_json": '{"aruco:19":[0.2,0.2]}',
        "ground_camera_info_topic": "/ground/info",
    })
    calibration = types.SimpleNamespace(width=320, height=240,
        k=[400.0, 0.0, 160.0, 0.0, 400.0, 120.0, 0.0, 0.0, 1.0], d=[0.0] * 5,
        distortion_model="plumb_bob", header=types.SimpleNamespace(frame_id="camera_optical"))
    node._camera_info_cb(calibration, "ground")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 19, 160)
    image = cv2.copyMakeBorder(marker, 40, 40, 80, 80, cv2.BORDER_CONSTANT, value=255)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    node._image_cb(_image_message(image), "ground")
    detection = node.messages[-1]["detections"][0]
    assert detection["position_camera_m"] == pytest.approx([-0.00063, -0.00063, 0.50314], abs=0.002)
    node._image_cb(_image_message(image), "uav")
    assert "position_camera_m" not in node.messages[-1]["detections"][0]


def test_vision_node_rejects_metric_sizes_without_any_camera_info_subscription(monkeypatch):
    with pytest.raises(ValueError, match="CameraInfo"):
        _vision_node_boundary(monkeypatch, {
            "camera_info_topic": "",
            "target_sizes_json": '{"aruco:19":[0.2,0.2]}',
        })


def test_unconfigured_physical_size_never_adds_metric_range(monkeypatch):
    node, _ = _vision_node_boundary(monkeypatch)
    calibration = types.SimpleNamespace(width=320, height=240,
        k=[400.0, 0.0, 160.0, 0.0, 400.0, 120.0, 0.0, 0.0, 1.0], d=[0.0] * 5,
        distortion_model="plumb_bob", header=types.SimpleNamespace(frame_id="camera_optical"))
    node._camera_info_cb(calibration, "ground")
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 19, 160)
    image = cv2.copyMakeBorder(marker, 40, 40, 80, 80, cv2.BORDER_CONSTANT, value=255)
    node._image_cb(_image_message(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)), "ground")
    assert "position_camera_m" not in node.messages[-1]["detections"][0]


def test_failed_frame_still_publishes_empty_observation_and_original_stamp(monkeypatch):
    node, _ = _vision_node_boundary(monkeypatch)
    node._image_cb(_image_message(None, 9.75), "ground")
    assert len(node.messages) == 1
    assert node.messages[0]["stamp"] == 9.75
    assert node.messages[0]["detections"] == []
    assert node.messages[0]["error"]


def test_reference_matcher_rejects_two_ids_with_identical_visual_reference(tmp_path):
    path = tmp_path / "ambiguous.png"
    reference = _reference_image(41)
    path.write_bytes(cv2.imencode(".png", reference)[1].tobytes())
    matcher = vision_core.TemplateMatcher({"id-one": str(path), "id-two": str(path)})
    assert detect_targets(reference, enable_qr=False, enable_aruco=False, template_matcher=matcher) == []


@pytest.mark.parametrize("reverse_order", [False, True])
def test_reference_matcher_rejects_shifted_centers_from_padded_duplicate_identity(tmp_path, reverse_order):
    reference = _reference_image(41)
    padded = cv2.copyMakeBorder(reference, 0, 0, 0, 120, cv2.BORDER_CONSTANT, value=225)
    references = {}
    for target_id, artwork in (("original", reference), ("padded-other-identity", padded)):
        path = tmp_path / f"{target_id}.png"
        path.write_bytes(cv2.imencode(".png", artwork)[1].tobytes())
        references[target_id] = str(path)
    if reverse_order:
        references = dict(reversed(list(references.items())))
    frame = np.full((600, 800), 225, np.uint8)
    frame[100:400, 100:480] = reference
    matcher = vision_core.TemplateMatcher(references)
    # The extra blank border shifts the projected box center by ~60 px. Both
    # identities still derive their evidence from the single physical artwork.
    assert detect_targets(frame, enable_qr=False, enable_aruco=False, template_matcher=matcher) == []


def test_reference_matcher_preserves_two_spatially_separated_real_targets(tmp_path):
    first, second = _reference_image(41), _reference_image(92)
    references = {}
    for target_id, artwork in (("left-target", first), ("right-target", second)):
        path = tmp_path / f"{target_id}.png"
        path.write_bytes(cv2.imencode(".png", artwork)[1].tobytes())
        references[target_id] = str(path)
    frame = np.full((600, 1000), 225, np.uint8)
    frame[130:430, 40:420] = first
    frame[130:430, 580:960] = second
    matcher = vision_core.TemplateMatcher(references)
    results = detect_targets(frame, enable_qr=False, enable_aruco=False, template_matcher=matcher)
    by_id = {item["target_id"]: item for item in results}
    assert set(by_id) == {"left-target", "right-target"}
    assert by_id["left-target"]["center"] == pytest.approx([0.23, 280 / 600], abs=0.015)
    assert by_id["right-target"]["center"] == pytest.approx([0.77, 280 / 600], abs=0.015)

