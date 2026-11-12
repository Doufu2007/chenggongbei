"""One bounded ground-only control experiment, never a scoring/mission client.

The only ROS publisher permitted to send messages here is /cmd_vel. The other
documented output publishers are declared for endpoint discovery and stay silent.
Robot radius and scan-origin
offset bounds must be supplied explicitly for the experimental candidate model;
they do not verify a platform profile. A complete scan ring gives a conservative
direction-independent guard without inventing a scan-to-body yaw transform.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class MotionLimits:
    robot_radius_m: float
    sensor_offset_bound_m: float
    clearance_m: float
    speed_mps: float = .03
    pulse_wall_s: float = 1.
    max_displacement_m: float = .06
    sensor_age_s: float = .2
    clock_sampling_lag_s: float = .12
    stable_s: float = 1.
    wait_wall_s: float = 20.
    stop_wall_s: float = 2.
    bounds_source: str = "explicit public candidate model; current official dimensions unverified"

    def __post_init__(self):
        for name in ("robot_radius_m", "sensor_offset_bound_m", "clearance_m", "speed_mps", "pulse_wall_s",
                     "max_displacement_m", "sensor_age_s", "clock_sampling_lag_s", "stable_s", "wait_wall_s", "stop_wall_s"):
            value = getattr(self, name)
            if not finite(value) or value <= 0:
                raise ValueError(name + " must be explicitly positive and finite")
        if (self.speed_mps > .03 or self.pulse_wall_s > 1.5 or self.max_displacement_m > .06
                or self.sensor_age_s > .2 or self.clock_sampling_lag_s > .12
                or self.stable_s < 1. or self.wait_wall_s > 60
                or not isinstance(self.bounds_source, str) or not self.bounds_source.strip()):
            raise ValueError("experimental motion bounds exceeded or source missing")


@dataclass(frozen=True)
class MotionDecision:
    linear_x: float
    state: str
    reason: str
    angular_z: float = 0.


class GroundMotionGuard:
    """Pure state machine; time/transport evidence supplied at each boundary.

    Any failed condition during PULSE permanently stops this one-shot run.
    Runtime ages use both received monotonic wall time and /clock sim time.
    A paused sim clock therefore cannot prolong nonzero velocity indefinitely.
    """
    OWN_NODES = frozenset(("inspection_bounded_ground_probe", "inspection_interface_probe",
                           "inspection_mission", "inspection_vision"))

    def __init__(self, limits, *, started_wall_s):
        if not isinstance(limits, MotionLimits) or not finite(started_wall_s):
            raise ValueError("invalid motion guard arguments")
        self.limits = limits
        self.started_wall = self.last_wall = started_wall_s
        self.state, self.reason = "WAIT", "waiting_for_evidence"
        self.clock = self.odom = self.scan = None
        self.last_observation_wall = started_wall_s
        self.last_clock_progress_wall = None
        self.last_sensor_stamp = {}
        self.last_sensor_arrival = {}
        self.endpoints = []
        self.matched_subscriptions = 0
        self.first_odom = self.last_odom = None
        self.stable_anchor = None
        self.pulse_started = self.pulse_origin = self.stopped_wall = None
        self.max_observed_displacement_m = 0.
        self.published_nonzero_count = self.published_zero_count = 0
        self._lock = threading.RLock()

    def stop(self, reason, wall_s):
        with self._lock:
            if self.state not in ("STOP", "FINISHED"):
                safe_wall = wall_s if finite(wall_s) else self.last_observation_wall
                self.state, self.reason, self.stopped_wall = "STOP", str(reason)[:256], max(self.last_wall,safe_wall)
            return MotionDecision(0., self.state, self.reason)

    def update_endpoints(self, endpoints, *, matched_subscriptions):
        with self._lock:
            self.endpoints = [dict(i) for i in endpoints if isinstance(i, dict)]
            self.matched_subscriptions = matched_subscriptions if type(matched_subscriptions) is int else 0

    def update_clock(self, stamp_s, *, wall_s):
        with self._lock:
            if not self._observation_wall("clock",wall_s):
                return
            if not finite(stamp_s) or stamp_s < 0 or not finite(wall_s):
                self.stop("clock_invalid", wall_s)
            elif self.clock is not None and stamp_s < self.clock[0]:
                self.stop("clock_reversed", wall_s)
            else:
                if self.clock is None or stamp_s>self.clock[0]:
                    self.last_clock_progress_wall=wall_s
                self.clock = (stamp_s, wall_s)
                self._check_active_timing(wall_s)

    def _observation_wall(self,topic,wall):
        if not finite(wall) or wall<max(self.last_wall,self.last_observation_wall):
            self.stop(topic+"_arrival_wall_reversed_or_invalid",wall)
            return False
        if (self.state=="PULSE" and self.pulse_started is not None
                and wall-self.pulse_started>=self.limits.pulse_wall_s-1e-9):
            self.stop("pulse_time_limit",wall)
        previous=self.last_sensor_arrival.get(topic)
        self.last_observation_wall=wall
        self.last_sensor_arrival[topic]=wall
        if self.state in ("PULSE","TURN","PAUSE") and previous is not None and wall-previous>self.limits.sensor_age_s+1e-9:
            self.temporal_fault(topic+"_stale",wall)
        return True

    def temporal_fault(self,reason,wall):
        self.stop(reason,wall)

    def _check_active_timing(self,wall):
        if self.state not in ("PULSE","TURN","PAUSE"):
            return
        if (self.clock is None or wall-self.clock[1]>self.limits.sensor_age_s+1e-9
                or self.last_clock_progress_wall is None
                or wall-self.last_clock_progress_wall>self.limits.sensor_age_s+1e-9):
            self.temporal_fault("clock_stale",wall)
            return
        for name,sample in (("odom",self.odom),("scan",self.scan)):
            if not self._fresh(sample,wall):
                self.temporal_fault(name+"_stale_or_invalid" if name=="odom" else "scan_stale_or_unsafe",wall)
                return

    def observed_motion_fault(self):
        if self.odom is None:
            return ""
        if self.odom["tilt"]>.03:
            return "ground_tilt"
        if self.pulse_origin is not None and self.max_observed_displacement_m>=self.limits.max_displacement_m-1e-9:
            return "displacement_limit"
        if self.odom["speed"]>.10 or self.odom["yaw_speed"]>getattr(self,"observed_yaw_speed_limit",.10):
            return "unexpected_motion"
        return ""

    @staticmethod
    def _stamp(payload, frame):
        header = payload["header"]
        sec, nano = header["stamp"]["sec"], header["stamp"]["nanosec"]
        if (header["frame_id"] != frame or type(sec) is not int or type(nano) is not int
                or sec < 0 or not 0 <= nano < 1_000_000_000):
            raise ValueError("sensor_frame_or_timestamp")
        return sec + nano / 1e9

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            previous = self.odom
            self.odom = None
            try:
                if not self._observation_wall("odom",wall_s):
                    return
                stamp = self._stamp(payload, "odom")
                if payload["child_frame_id"] != "base_footprint":
                    raise ValueError("odom_child_frame")
                pose, twist = payload["pose"]["pose"], payload["twist"]["twist"]
                xyz = tuple(pose["position"][k] for k in ("x", "y", "z"))
                q = tuple(pose["orientation"][k] for k in ("x", "y", "z", "w"))
                v = tuple(twist["linear"][k] for k in ("x", "y", "z"))
                w = tuple(twist["angular"][k] for k in ("x", "y", "z"))
                if not all(finite(x) for x in xyz + q + v + w) or abs(math.hypot(*q)-1) > .001:
                    raise ValueError("odom_numeric_or_quaternion")
                if previous is not None and stamp < previous["stamp"]:
                    self.stop("odom_stamp_reversed", wall_s)
                    return
                tilt = math.acos(max(-1., min(1., 1-2*(q[0]*q[0]+q[1]*q[1]))))
                self.odom = {"stamp": stamp, "wall": wall_s, "xyz": xyz, "q": q,
                             "speed": math.hypot(*v), "yaw_speed": math.hypot(*w), "tilt": tilt}
                if self.first_odom is None:
                    self.first_odom = copy.deepcopy(payload)
                self.last_odom = copy.deepcopy(payload)
                if self.pulse_origin is not None:
                    self.max_observed_displacement_m = max(self.max_observed_displacement_m,
                                                          math.dist(xyz, self.pulse_origin))
                if self.state in ("PULSE","TURN","PAUSE"):
                    fault=self.observed_motion_fault()
                    if fault:self.stop(fault,wall_s)
                    self._check_active_timing(wall_s)
            except (KeyError, ValueError, TypeError, OverflowError) as exc:
                if self.state in ("PULSE", "TURN", "PAUSE"):
                    self.stop("odom_invalid:" + str(exc)[:128], wall_s)
                elif self.state not in ("STOP", "FINISHED"):
                    self.reason = str(exc)[:256]
                self.stable_anchor = None

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            self.scan = None
            try:
                if not self._observation_wall("scan",wall_s):
                    return
                stamp = self._stamp(payload, "laser_link")
                if "scan" in self.last_sensor_stamp and stamp<self.last_sensor_stamp["scan"]:
                    self.stop("scan_stamp_reversed",wall_s)
                    return
                self.last_sensor_stamp["scan"]=stamp
                amin, amax, step, rmin, rmax = (payload[k] for k in
                    ("angle_min", "angle_max", "angle_increment", "range_min", "range_max"))
                values = payload["ranges"]
                if (not all(finite(x) for x in (amin, amax, step, rmin, rmax)) or step == 0
                        or not isinstance(values, (list, tuple)) or not 12 <= len(values) <= 8192
                        or abs(step) > .02
                        or not 0 <= rmin < rmax or abs(amax - (amin+(len(values)-1)*step)) > abs(step)*.05
                        or abs(step)*len(values) < 2*math.pi - 1e-5):
                    raise ValueError("scan_ring_coverage")
                threshold = (self.limits.robot_radius_m + self.limits.sensor_offset_bound_m
                             + self.limits.max_displacement_m + self.limits.clearance_m)
                if rmax <= threshold:
                    raise ValueError("scan_max_range")
                distances = []
                for value in values:
                    if type(value) not in (int, float) or math.isnan(value) or value <= 0:
                        raise ValueError("scan_invalid_return")
                    if math.isinf(value):
                        distances.append(rmax)  # Positive infinity only, declared max covers the whole guard.
                    elif not rmin <= value <= rmax:
                        raise ValueError("scan_return_out_of_range")
                    else:
                        distances.append(value)
                if min(distances) <= threshold:
                    raise ValueError("scan_too_close")
                self.scan = {"stamp": stamp, "wall": wall_s, "min_range_m": min(distances),
                             "required_range_m": threshold, "beam_count": len(values)}
                self._check_active_timing(wall_s)
            except (KeyError, ValueError, TypeError, OverflowError) as exc:
                if self.state in ("PULSE", "TURN", "PAUSE"):
                    self.stop("scan_invalid:" + str(exc)[:128], wall_s)
                elif self.state not in ("STOP", "FINISHED"):
                    self.reason = str(exc)[:256]

    def _fresh(self, sample, wall):
        # /clock is sampled at 10 Hz in the observed public bridge, while odom
        # arrives at 50 Hz. A sensor may precede the next clock sample by the
        # explicitly bounded phase lag; this does not extend either age limit.
        return (sample is not None and self.clock is not None
                and 0 <= wall - sample["wall"] <= self.limits.sensor_age_s + 1e-9
                and -self.limits.clock_sampling_lag_s - 1e-9 <= self.clock[0] - sample["stamp"]
                <= self.limits.sensor_age_s + 1e-9)

    def timing_snapshot(self, *, wall_s):
        """Observed stamps/arrival ages at a decision, never a substitute clock."""
        with self._lock:
            result = {"clock_sampling_lag_s": self.limits.clock_sampling_lag_s,
                      "sensor_age_s": self.limits.sensor_age_s,
                      "clock": None if self.clock is None else
                          {"stamp_s": self.clock[0], "wall_age_s": wall_s-self.clock[1]}}
            for topic, sample in (("odom",self.odom),("scan",self.scan)):
                result[topic] = None if sample is None else {
                    "stamp_s": sample["stamp"], "wall_age_s": wall_s-sample["wall"],
                    "clock_minus_stamp_s": None if self.clock is None else self.clock[0]-sample["stamp"],
                    "fresh": self._fresh(sample,wall_s)}
            return result

    def _condition(self, wall):
        external = [e for e in self.endpoints
                    if str(e.get("node", "")).split("/")[-1] not in self.OWN_NODES
                    and e.get("type") == "geometry_msgs/msg/Twist"]
        if not external or self.matched_subscriptions <= 0:
            return "no_external_matching_cmd_subscriber"
        if (self.clock is None or not 0 <= wall - self.clock[1] <= self.limits.sensor_age_s + 1e-9
                or self.last_clock_progress_wall is None
                or wall-self.last_clock_progress_wall>self.limits.sensor_age_s+1e-9):
            return "clock_stale"
        if not self._fresh(self.odom, wall):
            return "odom_stale_or_invalid"
        if not self._fresh(self.scan, wall):
            return "scan_stale_or_unsafe"
        if self.odom["tilt"] > .03:
            return "ground_tilt"
        return ""

    def step(self, *, wall_s):
        with self._lock:
            if not finite(wall_s) or wall_s < self.last_wall:
                return self.stop("wall_clock_reversed_or_invalid", self.last_wall)
            self.last_wall = wall_s
            if self.state in ("STOP", "FINISHED"):
                if wall_s - self.stopped_wall >= self.limits.stop_wall_s:
                    self.state = "FINISHED"
                return MotionDecision(0., self.state, self.reason)
            if self.state == "PULSE" and wall_s - self.pulse_started >= self.limits.pulse_wall_s - 1e-9:
                return self.stop("pulse_time_limit", wall_s)
            if self.state == "WAIT" and wall_s - self.started_wall >= self.limits.wait_wall_s:
                return self.stop("wait_timeout:" + self.reason, wall_s)
            reason = self._condition(wall_s)
            if reason:
                self.stable_anchor = None
                if self.state == "PULSE":
                    return self.stop(reason, wall_s)
                if wall_s - self.started_wall >= self.limits.wait_wall_s:
                    return self.stop("wait_timeout:" + reason, wall_s)
                self.reason = reason
                return MotionDecision(0., self.state, reason)
            pose = self.odom
            if self.state == "PULSE":
                distance = math.dist(pose["xyz"], self.pulse_origin)
                self.max_observed_displacement_m = max(distance, self.max_observed_displacement_m)
                if distance >= self.limits.max_displacement_m - 1e-9:
                    return self.stop("displacement_limit", wall_s)
                if pose["speed"] > .10 or pose["yaw_speed"] > .10:
                    return self.stop("unexpected_motion", wall_s)
            else:
                stable = pose["speed"] <= .01 and pose["yaw_speed"] <= .03
                if self.stable_anchor is not None:
                    a = self.stable_anchor
                    stable = stable and math.dist(pose["xyz"], a[2]) <= .005
                    stable = stable and abs(sum(x*y for x,y in zip(pose["q"], a[3]))) >= math.cos(.02/2)
                if not stable:
                    self.stable_anchor = None
                    return MotionDecision(0., self.state, "start_not_stable")
                if self.stable_anchor is None:
                    self.stable_anchor = (wall_s, pose["stamp"], pose["xyz"], pose["q"])
                a = self.stable_anchor
                if min(wall_s-a[0], pose["stamp"]-a[1]) < self.limits.stable_s - 1e-9:
                    return MotionDecision(0., self.state, "start_stability_dwell")
                self.state, self.reason = "PULSE", "bounded_ground_pulse"
                self.pulse_started, self.pulse_origin = wall_s, pose["xyz"]
            return MotionDecision(self.limits.speed_mps, self.state, self.reason)


OUTPUT_TYPES = {
    "/cmd_vel": "geometry_msgs/msg/Twist", "/uav/cmd_vel": "geometry_msgs/msg/Twist",
    "/referee/current_target": "std_msgs/msg/String", "/inspection/detections": "std_msgs/msg/String",
    "/referee/cruise_event": "std_msgs/msg/String", "/uav/laser_fire": "std_msgs/msg/Bool",
}
INPUT_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock", "/odom": "nav_msgs/msg/Odometry", "/uav/odom": "nav_msgs/msg/Odometry",
    "/scan": "sensor_msgs/msg/LaserScan", "/tf": "tf2_msgs/msg/TFMessage", "/tf_static": "tf2_msgs/msg/TFMessage",
    "/camera/image": "sensor_msgs/msg/Image", "/uav/camera/image": "sensor_msgs/msg/Image",
    "/camera/camera_info": "sensor_msgs/msg/CameraInfo", "/uav/camera/camera_info": "sensor_msgs/msg/CameraInfo",
    "/referee/ground_tasks": "std_msgs/msg/String", "/referee/recognition_status": "std_msgs/msg/String",
}


def _json_safe(value):
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, (bytes, bytearray)):
        return {"byte_length": len(value)}
    return value


def create_bounded_node(limits, reporter=None, *, wall_clock=time.monotonic, guard_factory=None,
                        graph_refresh_interval_s=0., callback_observer=None,
                        input_reliability_overrides=None):
    """ROS adapter: other documented outputs are declared but strictly silent."""
    if not finite(graph_refresh_interval_s) or graph_refresh_interval_s < 0:
        raise ValueError("graph refresh interval must be finite and nonnegative")
    if input_reliability_overrides is None:
        input_reliability_overrides = {}
    if not isinstance(input_reliability_overrides, Mapping):
        raise ValueError("input reliability override must be a mapping")
    frozen_input_reliability = MappingProxyType(dict(input_reliability_overrides))
    if any(type(topic) is not str or topic not in ("/odom", "/uav/odom")
           or type(value) is not str or value != "BEST_EFFORT"
           for topic, value in frozen_input_reliability.items()):
        raise ValueError("input reliability override only supports /odom and /uav/odom BEST_EFFORT")
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from .probe import build_image_sample
    from .telemetry import DiagnosticReporter

    class BoundedGroundProbe(Node):
        def __init__(self):
            super().__init__("inspection_bounded_ground_probe")
            self.wall_clock = wall_clock
            self.callback_observer = callback_observer
            self.guard = (GroundMotionGuard if guard_factory is None else guard_factory)(
                limits, started_wall_s=wall_clock())
            self.experiment_mode = getattr(self.guard, "mode", "bounded_ground_motion")
            self.command_lock = threading.RLock()
            self.watchdog_closed = threading.Event()
            self.watchdog_thread = None
            self.publisher_by_topic, self.command_endpoints, self.input_subscriptions = {}, {}, []
            self.command_endpoint_history = []
            self.command_endpoint_history_dropped = 0
            self.last_graph_refresh_wall = None
            self.graph_refresh_diagnostics = {"interval_s": graph_refresh_interval_s,
                "refresh_count": 0, "graph_query_count": 0, "graph_query_total_wall_s": 0.,
                "graph_query_max_wall_s": 0., "match_check_count": 0}
            self.startup = None
            self.sample_counts, self.public_messages, self.tf_edges, self.image_frames = {}, {}, {}, {}
            self.errors, self.odom_records = [], {}
            self.motion_log = []
            self.motion_log_dropped = 0
            graph_method = getattr(self, "get_topic_names_and_types", None)
            graph = dict(graph_method()) if callable(graph_method) else {}
            for topic, kind in OUTPUT_TYPES.items():
                if topic in graph and any(t != kind for t in graph[topic]):
                    self.errors.append("output_type_conflict:" + topic)
                    continue
                try:
                    self.publisher_by_topic[topic] = self.create_publisher(get_message(kind), topic,
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
                except Exception as exc:
                    self.errors.append("publisher_creation:" + topic + ":" + str(exc)[:128])
            for topic, kind in INPUT_TYPES.items():
                if topic in graph and any(t != kind for t in graph[topic]):
                    self.errors.append("input_type_conflict:" + topic)
                    continue
                try:
                    reliability = ReliabilityPolicy.BEST_EFFORT
                    durability = DurabilityPolicy.TRANSIENT_LOCAL if topic in ("/tf_static", "/referee/ground_tasks") else DurabilityPolicy.VOLATILE
                    infos = self.get_publishers_info_by_topic(topic)
                    if infos and all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos):
                        reliability = ReliabilityPolicy.RELIABLE
                    if topic in frozen_input_reliability:
                        reliability = ReliabilityPolicy.BEST_EFFORT
                    qos = QoSProfile(depth=10 if topic.startswith("/tf") else 1,
                                     reliability=reliability, durability=durability)
                    self.input_subscriptions.append(self.create_subscription(get_message(kind), topic,
                        lambda msg, topic=topic: self.receive(topic, msg), qos))
                    if callback_observer is not None:
                        callback_observer.record_input_qos(topic,
                            {"depth": 10 if topic.startswith("/tf") else 1,
                             "reliability": str(reliability), "durability": str(durability)},
                            [{"reliability": str(i.qos_profile.reliability),
                              "durability": str(i.qos_profile.durability)} for i in infos[:4]])
                except Exception as exc:
                    self.errors.append("subscription_creation:" + topic + ":" + str(exc)[:128])
            self.refresh_endpoints()

        def refresh_endpoints(self):
            self.graph_refresh_diagnostics["refresh_count"] += 1
            for topic in OUTPUT_TYPES:
                try:
                    query_started = time.monotonic()
                    try:
                        infos = self.get_subscriptions_info_by_topic(topic)
                    finally:
                        elapsed = time.monotonic()-query_started
                        self.graph_refresh_diagnostics["graph_query_count"] += 1
                        self.graph_refresh_diagnostics["graph_query_total_wall_s"] += elapsed
                        self.graph_refresh_diagnostics["graph_query_max_wall_s"] = max(
                            self.graph_refresh_diagnostics["graph_query_max_wall_s"], elapsed)
                    all_endpoints = [{"node": (i.node_namespace.rstrip("/")+"/"+i.node_name)[:256], "type": i.topic_type,
                                  "reliability": str(i.qos_profile.reliability), "durability": str(i.qos_profile.durability)}
                                 for i in infos if i.node_name not in self.guard.OWN_NODES]
                    endpoints = all_endpoints[:4]
                    publisher = self.publisher_by_topic.get(topic)
                    matches = publisher.get_subscription_count() if publisher is not None else 0
                    self.command_endpoints[topic] = {"subscriber_endpoints": endpoints, "matched_subscriptions": matches,
                                                       "subscriber_endpoints_omitted": max(0, len(all_endpoints)-4),
                                                       "declared_type": OUTPUT_TYPES[topic], "silent": topic != "/cmd_vel"}
                    if topic == "/cmd_vel":
                        self.guard.update_endpoints(endpoints, matched_subscriptions=matches)
                except Exception as exc:
                    self.command_endpoints[topic] = {"error": str(exc)[:128]}
                    if topic == "/cmd_vel":
                        self.guard.update_endpoints([], matched_subscriptions=0)
            self.last_graph_refresh_wall = self.wall_clock()
            self.record_endpoint_history()

        def record_endpoint_history(self):
            if not self.command_endpoint_history or self.command_endpoint_history[-1]["endpoints"] != self.command_endpoints:
                self.command_endpoint_history.append({
                    "wall_elapsed_s": round(self.wall_clock()-self.guard.started_wall, 4),
                    "endpoints": copy.deepcopy(self.command_endpoints)})
                if len(self.command_endpoint_history) > 16:
                    # Preserve creation-time graph and the most recent changes.
                    self.command_endpoint_history.pop(1)
                    self.command_endpoint_history_dropped += 1

        def check_command_match(self):
            # Actual DDS matching is lightweight and never throttled with the
            # descriptive graph. A cached positive match cannot authorize motion.
            self.graph_refresh_diagnostics["match_check_count"] += 1
            publisher = self.publisher_by_topic.get("/cmd_vel")
            try:
                matches = publisher.get_subscription_count() if publisher is not None else 0
            except Exception as exc:
                matches = 0
                error = "command_match_error:"+str(exc)[:128]
                if error not in self.errors and len(self.errors) < 64:
                    self.errors.append(error)
            item = self.command_endpoints.setdefault("/cmd_vel", {})
            item["matched_subscriptions"] = matches
            self.guard.update_endpoints(item.get("subscriber_endpoints", []), matched_subscriptions=matches)
            self.record_endpoint_history()
            return type(matches) is int and matches > 0

        def report_started(self, duration_s):
            self.startup = {"mode": self.experiment_mode, "read_only": False,
                "duration_s": duration_s, "official_scoring": "unverified",
                "node": "inspection_bounded_ground_probe", "limits": vars(limits)}
            if self.callback_observer is not None:
                self.startup["revision"] = self.callback_observer.revision
            if reporter is not None:
                reporter.emit("probe_started", self.startup)

        def receive(self, topic, msg):
            observer = self.callback_observer
            if observer is None:
                return self._receive_impl(topic, msg)
            token = observer.begin_receive(topic, msg, self.guard.clock, self.guard.state)
            try:
                return self._receive_impl(topic, msg)
            finally:
                observer.end(token)

        def _receive_impl(self, topic, msg):
            self.sample_counts[topic] = self.sample_counts.get(topic, 0) + 1
            wall = self.wall_clock()
            if INPUT_TYPES[topic] == "sensor_msgs/msg/Image":
                frames = self.image_frames.setdefault(topic, {"first": msg, "last": msg})
                frames["last"] = msg
                image_hook = getattr(self.guard, "observe_image", None)
                if callable(image_hook):
                    image_hook(topic, msg, wall_s=wall)
                return
            try:
                payload = message_to_ordereddict(msg)
                if topic == "/clock":
                    c = payload["clock"]
                    sec, nano = c["sec"], c["nanosec"]
                    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 1_000_000_000:
                        raise ValueError("invalid_clock_header")
                    self.guard.update_clock(sec+nano/1e9, wall_s=wall)
                elif topic == "/odom":
                    self.guard.update_odom(payload, wall_s=wall)
                elif topic == "/scan":
                    self.guard.update_scan(payload, wall_s=wall)
                public_hook = getattr(self.guard, "observe_public", None)
                if callable(public_hook):
                    public_hook(topic, payload, wall_s=wall)
                if topic in ("/odom", "/uav/odom"):
                    record = self.odom_records.setdefault(topic, {"first": payload, "last": payload})
                    record["last"] = payload
                elif topic in ("/tf", "/tf_static"):
                    for transform in payload["transforms"]:
                        parent, child = transform["header"]["frame_id"], transform["child_frame_id"]
                        key = (topic, parent, child)
                        if key not in self.tf_edges and len(self.tf_edges) >= 128:
                            if "tf_unique_transform_limit" not in self.errors:
                                self.errors.append("tf_unique_transform_limit")
                            continue
                        record = self.tf_edges.setdefault(key, {"topic": topic, "parent_frame": parent,
                            "child_frame_id": child, "first": transform, "last": transform, "count": 0})
                        record["last"], record["count"] = transform, record["count"]+1
                else:
                    record = self.public_messages.setdefault(topic, {"first": payload, "last": payload})
                    record["last"] = payload
            except Exception as exc:
                error = "input_error:" + topic + ":" + str(exc)[:128]
                if error not in self.errors and len(self.errors) < 64:
                    self.errors.append(error)
                if topic in ("/clock", "/odom", "/scan"):
                    self.guard.stop(error, wall)

        def publish_zero(self):
            publisher = self.publisher_by_topic.get("/cmd_vel")
            if publisher is not None:
                publisher.publish(Twist())
                self.guard.published_zero_count += 1
                self.record_command(0., 0.)

        def record_command(self, linear_x, angular_z):
            hook = getattr(self.guard, "record_command", None)
            if callable(hook):
                hook(wall_s=self.wall_clock(), linear_x=linear_x, angular_z=angular_z)

        def arm_watchdog(self):
            if self.watchdog_thread is not None:
                return
            started = self.guard.pulse_started
            deadline = (self.wall_clock() if started is None else started) + getattr(
                self.guard, "motion_wall_limit_s", limits.pulse_wall_s)

            def watchdog():
                while not self.watchdog_closed.is_set():
                    now = self.wall_clock()
                    hook = getattr(self.guard, "watchdog_reason", None)
                    reason = hook(now) if callable(hook) else ""
                    if now >= deadline:
                        reason=getattr(self.guard,"wall_limit_reason","pulse_time_limit")
                    if reason or self.guard.state in ("STOP", "FINISHED", "PAUSE"):
                        with self.command_lock:
                            if reason:
                                handler=getattr(self.guard,"handle_watchdog",self.guard.stop)
                                handler(reason,self.wall_clock())
                            self.publish_zero()
                        if self.watchdog_closed.wait(.05):
                            return
                    else:
                        self.watchdog_closed.wait(min(.02, max(0., deadline-self.wall_clock())))
            self.watchdog_thread = threading.Thread(target=watchdog, daemon=True)
            self.watchdog_thread.start()

        def close_watchdog(self):
            self.watchdog_closed.set()
            if self.watchdog_thread is not None:
                self.watchdog_thread.join(timeout=.5)

        def stop(self, reason):
            with self.command_lock:
                self.guard.stop(reason, self.wall_clock())
                self.publish_zero()

        def tick(self):
            # Graph queries may wait on ROS internals. Never hold the publish
            # lock here: the independent watchdog must still send zero while
            # these queries stall. Guard endpoint updates use their own lock.
            try:
                matched = self.check_command_match()
                if not matched and self.guard.state not in ("WAIT", "STOP", "FINISHED"):
                    # Send zero before any potentially blocking graph query.
                    self.stop("no_external_matching_cmd_subscriber")
                elif (self.last_graph_refresh_wall is None or graph_refresh_interval_s == 0.
                        or self.wall_clock()-self.last_graph_refresh_wall >= graph_refresh_interval_s-1e-9):
                    self.refresh_endpoints()
            except Exception as exc:
                error = "endpoint_refresh_error:" + str(exc)[:128]
                if error not in self.errors and len(self.errors) < 64:
                    self.errors.append(error)
                with self.command_lock:
                    self.guard.stop(error, self.wall_clock())
            with self.command_lock:
                decision = self.guard.step(wall_s=self.wall_clock())
                publisher = self.publisher_by_topic.get("/cmd_vel")
                if publisher is not None:
                    message = Twist()
                    message.linear.x = decision.linear_x
                    message.angular.z = decision.angular_z
                    publisher.publish(message)
                    self.record_command(decision.linear_x, decision.angular_z)
                    if decision.linear_x or decision.angular_z:
                        self.guard.published_nonzero_count += 1
                        self.arm_watchdog()
                    else:
                        self.guard.published_zero_count += 1
                if not self.motion_log or self.motion_log[-1]["state"] != decision.state or self.motion_log[-1]["reason"] != decision.reason:
                    self.motion_log.append({"wall_elapsed_s": round(self.wall_clock()-self.guard.started_wall, 4),
                        "state": decision.state, "reason": decision.reason, "linear_x": decision.linear_x,
                        "angular_z": decision.angular_z,
                        "timing": self.guard.timing_snapshot(wall_s=self.wall_clock())})
                    if len(self.motion_log) > 64:
                        self.motion_log.pop(1)
                        self.motion_log_dropped += 1
                return decision

        def evidence_snapshot(self):
            snapshot = {"mode": self.experiment_mode, "read_only": False, "official_scoring": "unverified",
                "elapsed_s": round(self.wall_clock()-self.guard.started_wall, 3), "state": self.guard.state,
                "reason": self.guard.reason, "limits": vars(limits), "command_endpoints": self.command_endpoints,
                "startup": self.startup, "command_endpoint_history": self.command_endpoint_history,
                "graph_refresh_diagnostics": dict(self.graph_refresh_diagnostics),
                "command_endpoint_history_dropped": self.command_endpoint_history_dropped,
                "sample_counts": self.sample_counts, "odom_first": self.guard.first_odom, "odom_last": self.guard.last_odom,
                "odometry": self.odom_records, "public_messages": self.public_messages,
                "tf_edges": {str(i): record for i,record in enumerate(self.tf_edges.values())},
                "max_observed_displacement_m": self.guard.max_observed_displacement_m,
                "published_nonzero_count": self.guard.published_nonzero_count,
                "published_zero_count": self.guard.published_zero_count,
                "timing": self.guard.timing_snapshot(wall_s=self.wall_clock()),
                "motion_log": self.motion_log, "motion_log_dropped": self.motion_log_dropped, "errors": self.errors}
            hook = getattr(self.guard, "snapshot_survey_evidence", None)
            if not callable(hook):
                hook = getattr(self.guard, "survey_evidence", None)
            if callable(hook):
                snapshot["survey"] = hook()
            if self.callback_observer is not None:
                snapshot["callback_timing"] = self.callback_observer.snapshot()
            return _json_safe(snapshot)

        def report_final(self):
            snapshot = self.evidence_snapshot()
            images = []
            for topic, frames in self.image_frames.items():
                seen = set()
                for phase, msg in frames.items():
                    try:
                        image = build_image_sample(msg, "sensor_msgs/msg/Image", topic)
                        snapshot.setdefault("image_records", []).append({"topic": topic, "phase": phase, "stamp": image["stamp"]})
                        if image["stamp"] not in seen:
                            images.append(image)
                            seen.add(image["stamp"])
                    except Exception as exc:
                        snapshot["errors"].append("image_error:"+topic+":"+str(exc)[:128])
            self.get_logger().info("BOUNDED_GROUND_PROBE " + json.dumps(snapshot, ensure_ascii=False, allow_nan=False))
            if reporter is None:
                return
            core_fields = ("mode", "read_only", "official_scoring", "startup", "elapsed_s", "state", "reason",
                "limits", "command_endpoints", "sample_counts", "odom_first", "odom_last",
                "max_observed_displacement_m", "published_nonzero_count", "published_zero_count", "errors",
                "motion_log_dropped", "command_endpoint_history_dropped", "timing", "graph_refresh_diagnostics")
            core = {key: snapshot[key] for key in core_fields}
            if "callback_timing" in snapshot:
                core["callback_timing"] = snapshot["callback_timing"]
            core.update(section="ground_motion_core", partial=True)
            # Always deliver the small motion summary independently. A large
            # sensor/TF section may not suppress this summary or real images.
            events = [("probe_snapshot", core)]
            hook = getattr(self.guard, "additional_events", None)
            if callable(hook):
                events.extend(hook())
            events.extend(("image_sample", data) for data in images)
            complete = {**snapshot, "section": "ground_motion_full", "partial": False}
            try:
                DiagnosticReporter._encode("probe_snapshot", complete)
                events.append(("probe_snapshot", complete))
            except ValueError:
                common = {"mode":self.experiment_mode, "read_only":False,
                          "official_scoring":"unverified", "partial":True}
                rejected = []
                for section in ("command_endpoint_history", "odometry", "public_messages", "tf_edges",
                                "motion_log", "image_records", "survey"):
                    value = snapshot.get(section, [])
                    if section == "tf_edges":
                        values = list(value.values())
                        parts = [{"offset":i, "transforms":values[i:i+8]} for i in range(0,len(values),8)]
                    elif isinstance(value, dict):
                        parts = [{"topic": key, "value": item} for key,item in value.items()]
                    else:
                        size = 1 if section == "command_endpoint_history" else 8
                        parts = [{"offset":i, "value":value[i:i+size]} for i in range(0,len(value),size)]
                    for part in parts:
                        payload = {**common, "section":section, **part}
                        try:
                            DiagnosticReporter._encode("probe_snapshot", payload)
                            events.append(("probe_snapshot", payload))
                        except ValueError as exc:
                            rejected.append({"section":section, "topic":part.get("topic"),
                                             "offset":part.get("offset"), "error":str(exc)[:128]})
                if rejected:
                    events.append(("probe_snapshot", {**common, "section":"evidence_errors", "errors":rejected}))
            deadline = time.monotonic()+getattr(self.guard, "final_report_wall_s", 30.)
            for event, payload in events:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    self.get_logger().warning("bounded probe final callback time limit; evidence incomplete")
                    break
                try:
                    reporter.send_now(event, payload, timeout_s=min(5., remaining))
                except ValueError as exc:
                    self.get_logger().warning("bounded probe callback rejected: "+str(exc)[:128])

    return BoundedGroundProbe()


def spin_control_loop(node, ros, *, duration_s, wall_clock=time.monotonic):
    """Spin incoming data continuously, independently cap commands to 20 Hz.

    Slow iterations skip missed command slots; queued sensor callbacks never
    cause catch-up bursts. The separate watchdog still stops a blocked spin.
    """
    started = next_command = wall_clock()
    observer = getattr(node, "callback_observer", None)
    while ros.ok():
        now = wall_clock()
        if now-started >= duration_s:
            break
        token = None if observer is None else observer.begin_span("spin_once")
        try:
            ros.spin_once(node, timeout_sec=min(.02, max(0., next_command-now)))
        finally:
            if observer is not None:
                observer.end(token)
        now = wall_clock()
        if now >= next_command:
            token = None if observer is None else observer.begin_span("tick")
            try:
                decision = node.tick()
            finally:
                if observer is not None:
                    observer.end(token)
            if decision.state == "FINISHED":
                break
            # A slow query/tick consumes its command slot. Schedule from its
            # completion, so the next callback cannot cause an output burst.
            next_command = wall_clock()+.05


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="One bounded ground-only pulse; no scoring or task messages")
    parser.add_argument("--duration", type=float, default=45.)
    parser.add_argument("--body-radius", type=float, required=True)
    parser.add_argument("--scan-offset-bound", type=float, required=True)
    parser.add_argument("--clearance", type=float, default=.02)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    if not finite(parsed.duration) or not 5 <= parsed.duration <= 60:
        parser.error("duration must lie in [5,60] wall seconds")
    limits = MotionLimits(parsed.body_radius, parsed.scan_offset_bound, parsed.clearance)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_bounded_node(limits, reporter)
        node.report_started(parsed.duration)
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("experiment_wall_time_limit")
        # Final zero messages continue independent of simulator time.
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("experiment_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
