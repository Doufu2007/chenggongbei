"""Finite positive-Z response experiment under an explicit static-column assumption.

This is separate from mission control. No recognition, score, laser, window,
landing or completion messages are published. Unknown upper space is not sensed.
"""
from __future__ import annotations

import argparse
import base64
import copy
from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
import math
import os
import threading
import time
import uuid

from .bounded_ground_probe import finite, spin_control_loop


WORLD_SHA256 = "595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1"
WORLD_SOURCE = "ranmengjia/ICAN@82b041043849a5d86e085390bf0df787366d98cd:binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
# All static collision boxes, excluding only the support floor. The model is
# publicly available geometry, not current Gazebo truth or texture assignments.
STATIC_BOXES = (
    ("north_wall", (-.8, .65, 0.), (4.25, .70, .4)),
    ("south_wall", (-.8, -3.4, 0.), (4.25, -3.35, .4)),
    ("west_wall", (-.825, -3.375, 0.), (-.775, .675, .4)),
    ("east_wall", (4.225, -3.375, 0.), (4.275, .675, .4)),
    ("window_1/bottom", (1.845, -.8275, 0.), (2.005, .1725, .4)),
    ("window_1/top", (1.845, -.8275, 1.15), (2.005, .1725, 1.8)),
    ("window_1/side_negative", (1.845, -1.0275, 0.), (2.005, -.8275, 1.8)),
    ("window_1/side_positive", (1.845, .1725, 0.), (2.005, .3725, 1.8)),
    ("window_2/bottom", (1.845, -2.8725, 0.), (2.005, -1.8725, .4)),
    ("window_2/top", (1.845, -2.8725, 1.15), (2.005, -1.8725, 1.8)),
    ("window_2/side_negative", (1.845, -3.0725, 0.), (2.005, -2.8725, 1.8)),
    ("window_2/side_positive", (1.845, -1.8725, 0.), (2.005, -1.6725, 1.8)),
)


@dataclass(frozen=True)
class VerticalLimits:
    command_mps: float = .05
    target_rise_m: float = .15
    hard_rise_m: float = .20
    max_horizontal_drift_m: float = .04
    max_tilt_rad: float = .05
    max_speed_mps: float = .15
    max_angular_speed_rps: float = .10
    max_total_path_m: float = .30
    motion_wall_s: float = 10.
    global_wall_s: float = 45.
    sensor_age_s: float = .20
    clock_sampling_lag_s: float = .12
    stable_s: float = 1.
    observe_s: float = 5.

    def __post_init__(self):
        caps = {"command_mps": .05, "target_rise_m": .15, "hard_rise_m": .20,
                "max_horizontal_drift_m": .04, "max_tilt_rad": .05,
                "max_speed_mps": .15, "max_angular_speed_rps": .10,
                "max_total_path_m": .30, "motion_wall_s": 10., "global_wall_s": 45.,
                "sensor_age_s": .20, "clock_sampling_lag_s": .12, "stable_s": 2., "observe_s": 10.}
        for key, cap in caps.items():
            if not finite(getattr(self, key)) or not 0 < getattr(self, key) <= cap:
                raise ValueError("invalid finite vertical bound: " + key)
        if self.stable_s < 1. or self.observe_s < 5. or self.target_rise_m >= self.hard_rise_m:
            raise ValueError("inconsistent vertical response limits")


@dataclass(frozen=True)
class VerticalDecision:
    linear_z: float
    state: str
    reason: str


def candidate_column_clear(origin, limits):
    """Conservative static candidate cylinder; floor contact is departure-only."""
    if (len(origin) != 3 or not all(finite(v) for v in origin)
            or math.hypot(*origin[:2]) > .05+1e-9 or not .035 <= origin[2] <= .075):
        return False
    radius = .447 + .166*math.sin(limits.max_tilt_rad) + .02 + limits.max_horizontal_drift_m
    bottom = origin[2] - (.068 + .447*math.sin(limits.max_tilt_rad) + .02)
    top = origin[2] + limits.hard_rise_m + .166 + .447*math.sin(limits.max_tilt_rad) + .02
    for _, low, high in STATIC_BOXES:
        if high[2] < bottom or low[2] > top:
            continue
        dx = max(low[0]-origin[0], 0., origin[0]-high[0])
        dy = max(low[1]-origin[1], 0., origin[1]-high[1])
        if math.hypot(dx, dy) <= radius:
            return False
    return True


def _stamp(payload, frame):
    header = payload["header"]
    sec, nano = header["stamp"]["sec"], header["stamp"]["nanosec"]
    if (header["frame_id"] != frame or type(sec) is not int or type(nano) is not int
            or sec < 0 or not 0 <= nano < 1_000_000_000):
        raise ValueError("frame_or_stamp")
    return sec+nano/1e9


def _pose(payload, child):
    stamp = _stamp(payload, "odom")
    if payload["child_frame_id"] != child:
        raise ValueError("child_frame")
    pose, twist = payload["pose"]["pose"], payload["twist"]["twist"]
    xyz = tuple(pose["position"][k] for k in ("x", "y", "z"))
    q = tuple(pose["orientation"][k] for k in ("x", "y", "z", "w"))
    v = tuple(twist["linear"][k] for k in ("x", "y", "z"))
    omega = tuple(twist["angular"][k] for k in ("x", "y", "z"))
    if not all(finite(v) for v in (*xyz, *q, *v, *omega)) or abs(math.hypot(*q)-1.) > .001:
        raise ValueError("numeric_or_quaternion")
    tilt = math.acos(max(-1., min(1., 1-2*(q[0]**2+q[1]**2))))
    return {"stamp": stamp, "xyz": xyz, "q": q, "linear": v, "angular": omega,
            "speed": math.hypot(*v), "angular_speed": math.hypot(*omega), "tilt": tilt}


def _safe(value):
    if isinstance(value, dict): return {str(k): _safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return "inf" if value > 0 else "-inf" if value < 0 else "nan"
    return value


class VerticalResponseGuard:
    mode = "bounded_vertical_response"
    OWN_NODES = frozenset(("inspection_bounded_vertical_response", "inspection_interface_probe"))

    def __init__(self, *, started_wall_s, limits=None):
        if not finite(started_wall_s): raise ValueError("finite start time required")
        self.limits = limits or VerticalLimits()
        self.started_wall = self.last_wall = self.last_observation_wall = started_wall_s
        self.session_id = uuid.uuid4().hex
        self.state, self.reason = "WAIT", "waiting_for_fresh_initial_column"
        self.clock = self.odom = self.ground_odom = self.scan = None
        self.last_clock_progress_wall = None
        self.static_laser_tf = None
        self.endpoints, self.matched_subscriptions = [], 0
        self.origin = self.motion_started_wall = self.observe_started_wall = None
        self.stopped_wall = self.stable_anchor = None
        self.target_reached = False
        self.total_path_m = self.max_rise_m = self.max_horizontal_drift_m = 0.
        self.trajectory, self.commands, self.scans, self.downscans, self.tf_records = [], [], [], [], []
        self.dropped = {"trajectory": 0, "commands": 0, "scans": 0, "downscans": 0, "tf_records": 0}
        self.post_stop_faults = []
        self.sensor_stamps = {}
        self._lock = threading.RLock()

    def _append(self, name, value, cap):
        values = getattr(self, name)
        values.append(_safe(value))
        if len(values) > cap:
            values.pop(1)
            self.dropped[name] += 1

    def stop(self, reason, wall_s):
        with self._lock:
            if self.state not in ("STOP", "FINISHED"):
                self.state, self.reason = "STOP", str(reason)[:256]
                self.stopped_wall = max(self.last_wall, self.last_observation_wall, wall_s if finite(wall_s) else self.last_wall)
            return VerticalDecision(0., self.state, self.reason)

    def pause(self, reason, wall_s):
        with self._lock:
            if not finite(wall_s): return self.stop("pause_wall_invalid", wall_s)
            self._budget(max(wall_s, self.last_observation_wall))
            if self.state == "ASCEND":
                self.state, self.reason = "PAUSE", str(reason)[:256]
                self.stable_anchor = None
            return VerticalDecision(0., self.state, self.reason)

    def _wall(self, wall):
        if not finite(wall) or wall < max(self.last_wall, self.last_observation_wall):
            self.stop("arrival_wall_reversed_or_invalid", wall)
            return False
        self.last_observation_wall = wall
        self._budget(wall)
        if self.state == "ASCEND":
            condition = self._condition(wall)
            if condition: self.pause(condition, wall)
        return True

    def _check_current_sources(self, wall):
        if self.state == "ASCEND":
            condition = self._condition(wall)
            if condition: self.pause(condition, wall)

    def _budget(self, wall):
        if wall-self.started_wall >= self.limits.global_wall_s-1e-9:
            self.stop("global_wall_time_limit", wall)
        elif (self.state in ("ASCEND", "PAUSE") and self.motion_started_wall is not None
              and wall-self.motion_started_wall >= self.limits.motion_wall_s-1e-9):
            self.stop("motion_wall_time_limit", wall)

    def update_endpoints(self, endpoints, *, matched_subscriptions):
        with self._lock:
            self.endpoints = [dict(x) for x in endpoints if isinstance(x, dict)]
            self.matched_subscriptions = matched_subscriptions if type(matched_subscriptions) is int else 0

    def update_clock(self, stamp_s, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return
            if not finite(stamp_s) or stamp_s < 0:
                return self.stop("clock_invalid", wall_s)
            if self.clock is not None and stamp_s < self.clock[0]:
                return self.stop("clock_reversed", wall_s)
            if self.clock is None or stamp_s > self.clock[0]:
                self.clock = (stamp_s, wall_s)
                self.last_clock_progress_wall = wall_s
                self._check_current_sources(wall_s)

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return
            try:
                sample = _pose(payload, "uav/base_link")
                previous = self.odom
                if previous is not None and sample["stamp"] < previous["stamp"]:
                    return self.stop("uav_odom_stamp_reversed", wall_s)
                if previous is not None and sample["stamp"] == previous["stamp"]:
                    if {k:v for k,v in previous.items() if k != "wall"} != sample:
                        return self.stop("uav_odom_duplicate_changed", wall_s)
                    return
                sample["wall"] = wall_s
                self.odom = sample
                self._append("trajectory", {"wall_elapsed_s": wall_s-self.started_wall, **sample}, 4096)
                if self.origin is not None:
                    rise = sample["xyz"][2]-self.origin[2]
                    drift = math.dist(sample["xyz"][:2], self.origin[:2])
                    self.max_rise_m, self.max_horizontal_drift_m = max(self.max_rise_m, rise), max(self.max_horizontal_drift_m, drift)
                    if previous is not None: self.total_path_m += math.dist(previous["xyz"], sample["xyz"])
                    fault = ("rise_limit" if rise >= self.limits.hard_rise_m-1e-9 else
                             "horizontal_drift_limit" if drift >= self.limits.max_horizontal_drift_m-1e-9 else
                             "unexpected_descent" if rise < -.02-1e-9 else
                             "total_path_limit" if self.total_path_m >= self.limits.max_total_path_m-1e-9 else "")
                else: fault = ""
                fault = fault or ("tilt_limit" if sample["tilt"] > self.limits.max_tilt_rad+1e-9 else
                                  "speed_limit" if sample["speed"] > self.limits.max_speed_mps+1e-9 else
                                  "angular_speed_limit" if sample["angular_speed"] > self.limits.max_angular_speed_rps+1e-9 else "")
                if fault:
                    if self.state in ("STOP", "FINISHED"):
                        if fault not in self.post_stop_faults: self.post_stop_faults.append(fault)
                    else: self.stop(fault, wall_s)
                self._check_current_sources(wall_s)
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                self.odom = None
                self.stable_anchor = None
                if self.state in ("ASCEND", "PAUSE", "OBSERVE"): self.stop("uav_odom_invalid:"+str(exc), wall_s)
                elif self.state == "WAIT": self.reason = "uav_odom_invalid:"+str(exc)

    def update_ground_odom(self, payload, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return
            try:
                sample = _pose(payload, "base_footprint")
                if self.ground_odom is not None and sample["stamp"] < self.ground_odom["stamp"]:
                    return self.stop("ground_odom_stamp_reversed", wall_s)
                if self.ground_odom is None or sample["stamp"] > self.ground_odom["stamp"]:
                    self.ground_odom = {**sample, "wall": wall_s}
            except (KeyError, TypeError, ValueError, OverflowError):
                self.ground_odom = None

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return
            try:
                stamp = _stamp(payload, "laser_link")
                previous = self.sensor_stamps.get("scan")
                if previous is not None and stamp < previous: return self.stop("scan_stamp_reversed", wall_s)
                if previous is not None and stamp == previous: return
                self.sensor_stamps["scan"] = stamp
                amin, amax, step, rmin, rmax = (payload[k] for k in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max"))
                ranges = payload["ranges"]
                if (not all(finite(v) for v in (amin, amax, step, rmin, rmax)) or not 0 < abs(step) <= .02
                        or not isinstance(ranges, (list, tuple)) or not 12 <= len(ranges) <= 8192
                        or not 0 <= rmin < rmax or abs(amax-(amin+(len(ranges)-1)*step)) > abs(step)*.05
                        or abs(step)*len(ranges) < 2*math.pi-1e-5):
                    raise ValueError("ring_coverage")
                if any(not finite(v) or not rmin <= v <= rmax for v in ranges):
                    raise ValueError("finite_complete_ring_required")
                if min(ranges) <= .641+1e-9: raise ValueError("horizontal_ring_too_close")
                self.scan = {"stamp": stamp, "wall": wall_s, "minimum_m": min(ranges)}
                self._append("scans", {"wall_elapsed_s": wall_s-self.started_wall, "message": copy.deepcopy(payload)}, 512)
                self._check_current_sources(wall_s)
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                self.scan = None
                self.stable_anchor = None
                if self.state in ("ASCEND", "PAUSE", "OBSERVE"): self.stop("scan_invalid:"+str(exc), wall_s)
                elif self.state == "WAIT": self.reason = "scan_invalid:"+str(exc)

    def update_downscan(self, payload, *, wall_s):
        with self._lock:
            if self._wall(wall_s):
                self._append("downscans", {"wall_elapsed_s": wall_s-self.started_wall, "message": copy.deepcopy(payload),
                                          "used_for_motion_permission": False}, 1000)

    def update_tf(self, payload, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return
            self._append("tf_records", {"wall_elapsed_s": wall_s-self.started_wall, "message": copy.deepcopy(payload)}, 4096)
            try:
                for transform in payload["transforms"]:
                    if transform["header"]["frame_id"] != "uav/base_link" or transform["child_frame_id"] != "uav/laser_link": continue
                    t, q = transform["transform"]["translation"], transform["transform"]["rotation"]
                    xyz, quat = tuple(t[k] for k in ("x", "y", "z")), tuple(q[k] for k in ("x", "y", "z", "w"))
                    if (not all(finite(v) for v in (*xyz, *quat)) or math.dist(xyz, (0., 0., -.07)) > .002
                            or abs(math.hypot(*quat)-1.) > .001
                            or abs(sum(a*b for a,b in zip(quat, (0., math.sqrt(.5), 0., math.sqrt(.5))))) < math.cos(.005/2)):
                        return self.stop("static_laser_tf_mismatch", wall_s)
                    self.static_laser_tf = copy.deepcopy(transform)
            except (KeyError, TypeError, ValueError, OverflowError):
                self.stop("tf_payload_invalid", wall_s)

    def _fresh(self, sample, wall):
        return (sample is not None and self.clock is not None
                and wall-sample["wall"] <= self.limits.sensor_age_s+1e-9
                and -self.limits.clock_sampling_lag_s-1e-9 <= self.clock[0]-sample["stamp"] <= self.limits.sensor_age_s+1e-9)

    def _condition(self, wall):
        if (self.matched_subscriptions <= 0 or not any(x.get("type") == "geometry_msgs/msg/Twist"
                and x.get("node", "").rsplit("/", 1)[-1] not in self.OWN_NODES for x in self.endpoints)):
            return "no_matching_external_uav_subscriber"
        if self.clock is None or wall-self.clock[1] > self.limits.sensor_age_s+1e-9: return "clock_stale"
        if self.last_clock_progress_wall is None or wall-self.last_clock_progress_wall > self.limits.sensor_age_s+1e-9: return "clock_not_progressing"
        if not self._fresh(self.odom, wall): return "uav_odom_stale"
        if not self._fresh(self.scan, wall): return "horizontal_scan_stale"
        if self.static_laser_tf is None: return "static_laser_tf_not_confirmed"
        if self.origin is None:
            if not self._fresh(self.ground_odom, wall): return "ground_odom_stale"
            if abs(self.ground_odom["stamp"]-self.odom["stamp"]) > .05+1e-9: return "initial_odom_pair_skew"
            if math.dist(self.ground_odom["xyz"], self.odom["xyz"]) > .02+1e-9: return "initial_odom_origins_disagree"
            if not candidate_column_clear(self.odom["xyz"], self.limits): return "outside_candidate_initial_column"
        return ""

    def _stable(self, wall):
        if self.odom["speed"] > .005 or self.odom["angular_speed"] > .01:
            self.stable_anchor = None
            return False
        if self.stable_anchor is None or math.dist(self.odom["xyz"], self.stable_anchor[1]) > .002:
            self.stable_anchor = (wall, self.odom["xyz"])
        return wall-self.stable_anchor[0] >= self.limits.stable_s-1e-9

    def step(self, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return VerticalDecision(0., self.state, self.reason)
            self.last_wall = wall_s
            if self.state == "STOP":
                if wall_s-self.stopped_wall >= 2.-1e-9: self.state = "FINISHED"
                return VerticalDecision(0., self.state, self.reason)
            if self.state == "FINISHED": return VerticalDecision(0., self.state, self.reason)
            condition = self._condition(wall_s)
            if condition:
                if self.state in ("ASCEND", "PAUSE"): self.pause(condition, wall_s)
                elif self.state == "WAIT": self.reason = condition
                elif self.state == "OBSERVE": self.stop("observation_sensor_lost:"+condition, wall_s)
                self.stable_anchor = None
                return VerticalDecision(0., self.state, self.reason)
            if self.state in ("WAIT", "PAUSE"):
                if not self._stable(wall_s): return VerticalDecision(0., self.state, "waiting_for_stationary_dwell")
                if self.origin is None:
                    self.origin = self.odom["xyz"]
                    self.motion_started_wall = wall_s
                self.state, self.reason = "ASCEND", "bounded_positive_z_response"
            if self.state == "ASCEND":
                if self.odom["xyz"][2]-self.origin[2] >= self.limits.target_rise_m-1e-9:
                    self.target_reached = True
                    self.state, self.reason, self.observe_started_wall = "OBSERVE", "zero_command_height_response", wall_s
                else: return VerticalDecision(self.limits.command_mps, self.state, self.reason)
            if self.state == "OBSERVE" and wall_s-self.observe_started_wall >= self.limits.observe_s-1e-9:
                self.state, self.reason = "FINISHED", "finite_vertical_response_observed_not_mission_complete"
            return VerticalDecision(0., self.state, self.reason)

    def record_command(self, *, wall_s, linear_z):
        with self._lock:
            if not finite(linear_z) or linear_z < 0 or linear_z > self.limits.command_mps+1e-9:
                raise ValueError("unbounded vertical command")
            if linear_z and self.state != "ASCEND": raise ValueError("vertical command outside ASCEND")
            self._append("commands", {"wall_elapsed_s": wall_s-self.started_wall, "linear_z": linear_z,
                                      "ground_linear_x": 0., "ground_angular_z": 0., "state": self.state}, 1200)

    def evidence(self):
        with self._lock:
            return _safe({"schema": "bounded_vertical_response_v1", "mode": self.mode, "session_id": self.session_id,
                "read_only": False, "official_score": None, "full_mission_complete": False,
                "limits": asdict(self.limits), "state": self.state, "reason": self.reason,
                "origin": self.origin, "motion_started_wall": self.motion_started_wall,
                "observe_started_wall": self.observe_started_wall, "target_reached": self.target_reached,
                "max_rise_m": self.max_rise_m, "max_horizontal_drift_m": self.max_horizontal_drift_m,
                "total_path_m": self.total_path_m, "post_stop_faults": self.post_stop_faults,
                "trajectory": self.trajectory, "commands": self.commands, "scans": self.scans,
                "downscans": self.downscans, "tf_records": self.tf_records, "dropped": self.dropped,
                "static_laser_tf": self.static_laser_tf, "assumptions": {
                    "world_source": WORLD_SOURCE, "world_sha256": WORLD_SHA256,
                    "static_map_verified_on_platform": False,
                    "space_evidence": "fixed public static candidate departure column plus live horizontal scan consistency",
                    "initial_support_floor": "excluded only as known initial support; positive-Z departure; no descent requested",
                    "unknown_upper_obstacles_sensed": False, "downscan_used_as_upper_clearance": False,
                    "body_world_positive_z_contract": "unverified; limited tilt and displacement measured during experiment"}})

    def diagnostic_events(self):
        evidence = self.evidence()
        raw = json.dumps(evidence, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        packed = gzip.compress(raw, mtime=0)
        encoded = base64.b64encode(packed).decode("ascii")
        parts = [encoded[i:i+48000] for i in range(0, len(encoded), 48000)]
        core = {key: evidence[key] for key in ("mode", "session_id", "state", "reason", "limits", "origin", "target_reached",
                    "max_rise_m", "max_horizontal_drift_m", "total_path_m", "dropped", "post_stop_faults", "assumptions")}
        core.update(section="bounded_vertical_core", read_only=False, official_scoring="unverified", full_mission_complete=False)
        return [("probe_snapshot", core)]+[("probe_snapshot", {
            "mode": self.mode, "read_only": False, "official_scoring": "unverified", "section": "bounded_vertical_gzip_part",
            "session_id": self.session_id, "encoding": "gzip+base64", "uncompressed_bytes": len(raw), "compressed_bytes": len(packed),
            "uncompressed_sha256": hashlib.sha256(raw).hexdigest(), "compressed_sha256": hashlib.sha256(packed).hexdigest(),
            "part_index": i, "part_count": len(parts), "base64_part": part}) for i,part in enumerate(parts)]


def create_vertical_node(reporter=None, *, wall_clock=time.monotonic, limits=None):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    inputs = {"/clock": "rosgraph_msgs/msg/Clock", "/uav/odom": "nav_msgs/msg/Odometry", "/odom": "nav_msgs/msg/Odometry",
              "/scan": "sensor_msgs/msg/LaserScan", "/uav/laser/scan": "sensor_msgs/msg/LaserScan",
              "/tf": "tf2_msgs/msg/TFMessage", "/tf_static": "tf2_msgs/msg/TFMessage"}
    Twist = get_message("geometry_msgs/msg/Twist")

    class VerticalNode(Node):
        def __init__(self):
            super().__init__("inspection_bounded_vertical_response")
            self.guard = VerticalResponseGuard(started_wall_s=wall_clock(), limits=limits)
            self.command_lock = threading.RLock()
            self.watchdog_closed = threading.Event()
            self.last_tick_wall = wall_clock()
            self.publisher_by_topic = {topic: self.create_publisher(Twist, topic, QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)) for topic in ("/cmd_vel", "/uav/cmd_vel")}
            self.subscriptions_keep = []
            self.endpoint_descriptions, self.last_graph_wall = [], None
            self.publish_counts = {"ground_zero": 0, "uav_zero": 0, "uav_positive_z": 0}
            self.errors = []
            for topic, kind in inputs.items():
                qos = QoSProfile(depth=10 if topic.startswith("/tf") else 1,
                    reliability=ReliabilityPolicy.BEST_EFFORT if topic in ("/clock", "/odom", "/uav/odom") else ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == "/tf_static" else DurabilityPolicy.VOLATILE)
                self.subscriptions_keep.append(self.create_subscription(get_message(kind), topic, lambda msg, topic=topic:self.receive(topic, msg), qos))

        def receive(self, topic, msg):
            wall = wall_clock()
            old_state = self.guard.state
            try:
                data = message_to_ordereddict(msg)
                if topic == "/clock":
                    sec, nano = data["clock"]["sec"], data["clock"]["nanosec"]
                    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 1_000_000_000: raise ValueError("clock stamp")
                    self.guard.update_clock(sec+nano/1e9, wall_s=wall)
                elif topic == "/uav/odom": self.guard.update_odom(data, wall_s=wall)
                elif topic == "/odom": self.guard.update_ground_odom(data, wall_s=wall)
                elif topic == "/scan": self.guard.update_scan(data, wall_s=wall)
                elif topic == "/uav/laser/scan": self.guard.update_downscan(data, wall_s=wall)
                else: self.guard.update_tf(data, wall_s=wall)
                if old_state == "ASCEND" and self.guard.state != "ASCEND": self.publish_zero()
            except Exception as exc:
                self.stop("input_callback_failed:"+topic+":"+type(exc).__name__)

        def _publish(self, speed):
            ground, uav = Twist(), Twist()
            uav.linear.z = float(speed)
            self.publisher_by_topic["/cmd_vel"].publish(ground)
            self.publish_counts["ground_zero"] += 1
            self.publisher_by_topic["/uav/cmd_vel"].publish(uav)
            self.publish_counts["uav_positive_z" if speed else "uav_zero"] += 1
            self.guard.record_command(wall_s=wall_clock(), linear_z=speed)

        def publish_zero(self):
            with self.command_lock: self._publish(0.)

        def stop(self, reason):
            with self.command_lock:
                decision = self.guard.stop(reason, wall_clock())
                self._publish(0.)
                return decision

        def tick(self):
            # Descriptive DDS graph queries can block. Keep them outside the
            # command lock so the independent watchdog can still send zero.
            try:
                wall = wall_clock()
                if self.last_graph_wall is None or wall-self.last_graph_wall >= .5:
                    descriptions = [{"node": (i.node_namespace.rstrip("/")+"/"+i.node_name), "type":i.topic_type}
                        for i in self.get_subscriptions_info_by_topic("/uav/cmd_vel") if i.node_name not in self.guard.OWN_NODES]
                    self.endpoint_descriptions = descriptions
                    self.last_graph_wall = wall_clock()
            except Exception as exc:
                self.errors.append(type(exc).__name__+":"+str(exc)[:128])
                return self.stop("uav_graph_refresh_failed:"+type(exc).__name__)
            with self.command_lock:
                publisher = self.publisher_by_topic["/uav/cmd_vel"]
                try:
                    matches = publisher.get_subscription_count()
                    self.guard.update_endpoints(self.endpoint_descriptions, matched_subscriptions=matches)
                    decision = self.guard.step(wall_s=wall_clock())
                    # Query actual DDS matching again at the publish boundary.
                    if decision.linear_z and publisher.get_subscription_count() <= 0:
                        decision = self.guard.pause("uav_match_lost_at_publish", wall_clock())
                    self._publish(decision.linear_z)
                    self.last_tick_wall = wall_clock()
                    return decision
                except Exception as exc:
                    self.errors.append(type(exc).__name__+":"+str(exc)[:128])
                    return self.stop("control_tick_failed:"+type(exc).__name__)

        def arm_watchdog(self):
            def watch():
                while not self.watchdog_closed.wait(.05):
                    with self.command_lock:
                        if wall_clock()-self.last_tick_wall > .15:
                            self.guard.pause("control_watchdog_stale", wall_clock())
                            try: self._publish(0.)
                            except Exception: self.guard.stop("watchdog_zero_publish_failed", wall_clock())
            self.watchdog_thread = threading.Thread(target=watch, name="bounded-vertical-watchdog", daemon=True)
            self.watchdog_thread.start()

        def close_watchdog(self):
            self.watchdog_closed.set()
            if hasattr(self, "watchdog_thread"): self.watchdog_thread.join(timeout=.5)

        def report_final(self):
            if reporter is None: return
            deadline = wall_clock()+60.
            for event, payload in self.guard.diagnostic_events():
                remaining = deadline-wall_clock()
                if remaining <= 0: break
                reporter.send_now(event, payload, timeout_s=min(5., remaining))
            reporter.send_now("probe_finished", {"mode":self.guard.mode, "read_only":False, "state":self.guard.state,
                "reason":self.guard.reason, "publish_counts":self.publish_counts, "errors":self.errors,
                "full_mission_complete":False, "official_scoring":"unverified"}, timeout_s=3.)

    return VerticalNode()


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    reporter = DiagnosticReporter(parsed.report_url)
    limits = VerticalLimits()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_vertical_node(reporter, limits=limits)
        reporter.emit("probe_started", {"mode":node.guard.mode, "read_only":False, "limits":asdict(limits),
            "world_sha256":WORLD_SHA256, "official_scoring":"unverified", "full_mission_complete":False})
        node.arm_watchdog()
        spin_control_loop(node, rclpy, duration_s=limits.global_wall_s)
        node.stop("vertical_experiment_cleanup")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.close_watchdog()
        node.report_final()
    finally:
        if node is not None:
            node.stop("vertical_experiment_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
