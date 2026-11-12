"""One-variable requested reliability contrast at the real adapter boundary."""
import importlib
import importlib.util
import json
from types import SimpleNamespace

import pytest

from test_bounded_ground_probe import install_ros_transport, odom


def api():
    name = "inspection_mission.ground_odom_qos"
    assert importlib.util.find_spec(name) is not None, "independent v18 odom QoS entry missing"
    return importlib.import_module(name)


def capture_transport(monkeypatch, *, profiles=None):
    install_ros_transport(monkeypatch)
    from rclpy.node import Node
    subscriptions, publications = {}, {}
    subscribe, publish = Node.create_subscription, Node.create_publisher

    def capture_sub(self, cls, topic, callback, qos):
        subscriptions[topic] = (qos.depth, qos.reliability, qos.durability)
        return subscribe(self, cls, topic, callback, qos)

    def capture_pub(self, cls, topic, qos):
        publications[topic] = (qos.depth, qos.reliability, qos.durability)
        return publish(self, cls, topic, qos)

    def public_sources(self, topic):
        offered = (["BEST_EFFORT"] if topic == "/clock" else ["RELIABLE"]) if profiles is None else profiles
        return [SimpleNamespace(node_name="public_source", node_namespace="/", topic_type="public_type",
            qos_profile=SimpleNamespace(reliability=value,
                durability="TRANSIENT_LOCAL" if topic in ("/tf_static", "/referee/ground_tasks") else "VOLATILE"))
            for value in offered]

    monkeypatch.setattr(Node, "create_subscription", capture_sub)
    monkeypatch.setattr(Node, "create_publisher", capture_pub)
    monkeypatch.setattr(Node, "get_publishers_info_by_topic", public_sources)
    return subscriptions, publications


@pytest.mark.parametrize("offered,expected", [
    (["RELIABLE"], "RELIABLE"),
    (["RELIABLE", "BEST_EFFORT"], "BEST_EFFORT"),
    ([], "BEST_EFFORT"),
])
def test_default_adapter_keeps_creation_time_reliability_policy(monkeypatch, offered, expected):
    from inspection_mission.bounded_ground_probe import MotionLimits, create_bounded_node
    subscriptions, _ = capture_transport(monkeypatch, profiles=offered)
    node = create_bounded_node(MotionLimits(.447, .114, .02), wall_clock=lambda: 0.)
    try:
        assert subscriptions["/odom"] == (1, expected, "VOLATILE")
        assert subscriptions["/tf_static"] == (10, expected, "TRANSIENT_LOCAL")
    finally:
        node.close_watchdog()


def test_explicit_override_changes_only_odom_subscription_not_outputs(monkeypatch):
    from inspection_mission.bounded_ground_probe import MotionLimits, create_bounded_node
    subscriptions, publications = capture_transport(monkeypatch)
    node = create_bounded_node(MotionLimits(.447, .114, .02), wall_clock=lambda: 0.,
        input_reliability_overrides={"/odom": "BEST_EFFORT"})
    try:
        assert subscriptions == {
            "/clock": (1, "BEST_EFFORT", "VOLATILE"),
            "/odom": (1, "BEST_EFFORT", "VOLATILE"),
            "/uav/odom": (1, "RELIABLE", "VOLATILE"),
            "/scan": (1, "RELIABLE", "VOLATILE"),
            "/tf": (10, "RELIABLE", "VOLATILE"),
            "/tf_static": (10, "RELIABLE", "TRANSIENT_LOCAL"),
            "/camera/image": (1, "RELIABLE", "VOLATILE"),
            "/uav/camera/image": (1, "RELIABLE", "VOLATILE"),
            "/camera/camera_info": (1, "RELIABLE", "VOLATILE"),
            "/uav/camera/camera_info": (1, "RELIABLE", "VOLATILE"),
            "/referee/ground_tasks": (1, "RELIABLE", "TRANSIENT_LOCAL"),
            "/referee/recognition_status": (1, "RELIABLE", "VOLATILE"),
        }
        assert set(publications) == {"/cmd_vel", "/uav/cmd_vel", "/referee/current_target",
            "/inspection/detections", "/referee/cruise_event", "/uav/laser_fire"}
        assert set(publications.values()) == {(1, "RELIABLE", "VOLATILE")}
    finally:
        node.close_watchdog()


@pytest.mark.parametrize("overrides", [{"/uav/odom": "BEST_EFFORT"},
    {"/odom": "BEST_EFFORT", "/uav/odom": "BEST_EFFORT"}])
def test_explicit_uav_odom_override_keeps_other_inputs_and_all_outputs_unchanged(monkeypatch, overrides):
    from inspection_mission.bounded_ground_probe import MotionLimits, create_bounded_node
    subscriptions, publications = capture_transport(monkeypatch)
    node = create_bounded_node(MotionLimits(.447, .114, .02), wall_clock=lambda: 0.,
        input_reliability_overrides=overrides)
    try:
        for topic in ("/odom", "/uav/odom"):
            assert subscriptions[topic] == (1, "BEST_EFFORT" if topic in overrides else "RELIABLE", "VOLATILE")
        assert subscriptions["/scan"] == (1, "RELIABLE", "VOLATILE")
        assert subscriptions["/tf_static"] == (10, "RELIABLE", "TRANSIENT_LOCAL")
        assert set(publications.values()) == {(1, "RELIABLE", "VOLATILE")}
    finally:
        node.close_watchdog()


def test_caller_mutation_cannot_change_frozen_override_during_node_construction(monkeypatch):
    from inspection_mission.bounded_ground_probe import MotionLimits, create_bounded_node
    subscriptions, _ = capture_transport(monkeypatch)
    from rclpy.node import Node
    overrides = {"/odom": "BEST_EFFORT"}
    initialize = Node.__init__

    def mutate_caller(self, name):
        overrides["/odom"] = "RELIABLE"
        initialize(self, name)

    monkeypatch.setattr(Node, "__init__", mutate_caller)
    node = create_bounded_node(MotionLimits(.447, .114, .02), wall_clock=lambda: 0.,
        input_reliability_overrides=overrides)
    try:
        assert subscriptions["/odom"] == (1, "BEST_EFFORT", "VOLATILE")
    finally:
        node.close_watchdog()


@pytest.mark.parametrize("invalid", [
    [], "/odom", {"/scan": "BEST_EFFORT"}, {"/cmd_vel": "BEST_EFFORT"},
    {"/unknown": "BEST_EFFORT"}, {"/odom": "RELIABLE"}, {"/odom": "best_effort"},
    {"/odom": None}, {"/odom": True}, {"/odom": 1},
    {"/uav/odom": "RELIABLE"}, {"/uav/odom": None},
])
def test_invalid_override_is_rejected_before_ros_side_effects(monkeypatch, invalid):
    from inspection_mission.bounded_ground_probe import MotionLimits, create_bounded_node
    subscriptions, publications = capture_transport(monkeypatch)
    with pytest.raises(ValueError, match="input reliability override"):
        create_bounded_node(MotionLimits(.447, .114, .02), input_reliability_overrides=invalid)
    assert subscriptions == {} and publications == {}


def test_v18_factory_preserves_guard_limits_and_records_actual_override(monkeypatch):
    from inspection_mission.ground_bounded_approach import ApproachLimits
    from inspection_mission.ground_bounded_approach_sync import GroundSynchronizedApproachGuard
    subscriptions, _ = capture_transport(monkeypatch)
    node = api().create_odom_qos_approach_node(wall_clock=lambda: 0.)
    try:
        node.report_started(90.)
        assert isinstance(node.guard, GroundSynchronizedApproachGuard)
        assert node.guard.approach_limits == ApproachLimits()
        assert node.guard.limits.sensor_age_s == .2 and node.guard.limits.clock_sampling_lag_s == .12
        assert node.experiment_mode == "ground_bounded_approach"
        assert node.startup["revision"] == "v18_odom_best_effort"
        timing = node.evidence_snapshot()["callback_timing"]
        assert timing["revision"] == "v18_odom_best_effort"
        assert timing["input_qos"]["/odom"]["requested"] == {
            "depth": 1, "reliability": "BEST_EFFORT", "durability": "VOLATILE"}
        assert timing["input_qos"]["/odom"]["observed_publisher_profiles"][0]["reliability"] == "RELIABLE"
        assert subscriptions["/scan"] == (1, "RELIABLE", "VOLATILE")
    finally:
        node.close_watchdog()


def test_v18_keeps_full_bounded_timing_in_final_report_under_payload_limit(monkeypatch):
    from inspection_mission.bounded_ground_probe import INPUT_TYPES
    from inspection_mission.telemetry import DiagnosticReporter
    capture_transport(monkeypatch)
    events = []

    class Reporter:
        def send_now(self, event, payload, timeout_s):
            encoded = DiagnosticReporter._encode(event, payload)
            assert len(encoded) < 65536
            events.append(json.loads(encoded))

        def emit(self, event, payload): return True

    node = api().create_odom_qos_approach_node(Reporter(), wall_clock=lambda: 0.)
    try:
        rec = node.callback_observer
        perf = [0.]
        rec.perf_counter = lambda: perf[0]
        for topic in INPUT_TYPES:
            for i in range(25):
                token = rec.begin_receive(topic, odom(10.+i), (10.2+i, 0.), "PAUSE")
                perf[0] += .03
                rec.end(token)
        node.report_final()
        core = next(row["data"] for row in events if row["data"].get("section") == "ground_motion_core")
        assert core["callback_timing"]["revision"] == "v18_odom_best_effort"
        assert len(core["callback_timing"]["slow_events"]) == 12
        assert len(core["callback_timing"]["spans"]) == 12
        assert core["callback_timing"]["guard_lock_wait"]["status"] == "not_measured"
    finally:
        node.close_watchdog()
