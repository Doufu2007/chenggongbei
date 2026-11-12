"""ROS adapter; interfaces are validated before actuator publishers exist.

ROS imports stay in create_node(), allowing real configuration validation on
non-ROS hosts. No platform interface, map or recognition answer is guessed.
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import replace
from pathlib import Path

from .mission import MissionController
from .protocol import (build_payload, planar_velocity, recognition_confirmation_mode,
                       validate_profile, uav_obstacle_sensor)
from .runtime import RuntimeInputs, flight_motion_intent, number


def completion_status_payload() -> str:
    """Completion payload reported in the platform forum; transport is unverified."""
    return "mission_completed"


def required_input_channels(profile, *, require_ground_order=True, active_vehicle=None):
    required = []
    if active_vehicle in (None, "ground"):
        required += ["ground_odom", "ground_scan"]
    if active_vehicle in (None, "uav"):
        required += ["uav_odom", "light_feedback"]
        required += ["uav_cloud" if uav_obstacle_sensor(profile) == "pointcloud" else "uav_scan"]
    if recognition_confirmation_mode(profile) == "event_ack":
        required.append("recognition_ack")
    if require_ground_order:
        required.append("ground_tasks")
    if active_vehicle in (None, "uav") and "uav_cloud" in profile["channels"] and "uav_cloud" not in required:
        required.append("uav_cloud")
    return required


def publish_recognition_report(controller, report, publish, now):
    """Mark local transport only after publish returns without an exception."""
    publish(report)
    controller.mark_recognition_published(report["event_id"], now)


def motion_path_clear(controller, inputs, vehicle, velocity, now):
    """Use the same validated body envelope at prelaunch and the final gate."""
    radius = controller.plan["robot_radius_m"] + controller.plan["clearance_m"]
    if vehicle == "uav" and controller.uav_envelope is not None:
        if inputs.uav_envelope != controller.uav_envelope:
            return False
        radius = controller.uav_envelope.horizontal_radius_m
    speed = abs(velocity[0]) if vehicle == "ground" else math.sqrt(sum(v*v for v in velocity[:3]))
    distance = radius + speed * max(inputs.sensor_timeout_s, inputs.scan_timeout_s)
    return inputs.motion_clear(vehicle, velocity, now, stop_distance=distance, corridor_radius=radius)


def completion_grace_elapsed(start_monotonic: float, now_monotonic: float, grace_s: float = 0.5) -> bool:
    """Use wall time for shutdown grace so a paused ROS clock cannot deadlock exit."""
    return now_monotonic - start_monotonic >= grace_s


def completion_shutdown_due(start_monotonic: float, now_monotonic: float, grace_s: float = 0.5) -> bool:
    """Return whether the independent wall-clock completion timer may shut down."""
    return completion_grace_elapsed(start_monotonic, now_monotonic, grace_s)


class SimulationClockWatchdog:
    """Detect a frozen simulation clock using monotonic wall time."""

    def __init__(self, timeout_s: float, startup_timeout_s: float | None = None):
        self.timeout_s = number(timeout_s, "simulation_clock_timeout_s")
        self.startup_timeout_s = number(
            self.timeout_s if startup_timeout_s is None else startup_timeout_s,
            "simulation_clock_startup_timeout_s")
        if self.timeout_s <= 0 or self.startup_timeout_s <= 0:
            raise ValueError("simulation clock timeouts must be positive")
        self.last_sim_time = None
        self.last_progress_monotonic = None
        self.started = False

    def observe(self, sim_time: float, now_monotonic: float) -> bool:
        sim_time = number(sim_time, "simulation_time")
        now_monotonic = number(now_monotonic, "monotonic_time")
        if self.last_sim_time is None or sim_time != self.last_sim_time:
            self.last_sim_time = sim_time
            self.last_progress_monotonic = now_monotonic
            self.started = self.started or sim_time > 0
            return False
        timeout = self.timeout_s if self.started else self.startup_timeout_s
        return (self.last_progress_monotonic is not None
                and now_monotonic - self.last_progress_monotonic >= timeout)


class RunOutcome:
    """Terminal result shared by watchdog, control loop and process exit."""

    def __init__(self, grace_s: float = 0.5):
        self.grace_s = number(grace_s, "termination_grace_s")
        if self.grace_s < 0:
            raise ValueError("termination_grace_s must not be negative")
        self.status = None
        self.detail = ""
        self.started_at = None

    def fail(self, detail: str, now_monotonic: float) -> None:
        now_monotonic = number(now_monotonic, "monotonic_time")
        if self.status != "failed":
            self.started_at = now_monotonic
        self.status = "failed"
        self.detail = str(detail)

    def complete_after_stop(self, stop_succeeded: bool, now_monotonic: float, publish_completion) -> bool:
        if self.status == "failed":
            return False
        if not stop_succeeded:
            self.fail("safe_stop_failed", now_monotonic)
            return False
        try:
            publish_completion()
        except Exception as exc:
            self.fail(f"completion_publish_failed:{type(exc).__name__}:{exc}", now_monotonic)
            return False
        self.status = "succeeded"
        self.detail = ""
        self.started_at = number(now_monotonic, "monotonic_time")
        return True

    def shutdown_due(self, now_monotonic: float) -> bool:
        return (self.status is not None and self.started_at is not None
                and completion_grace_elapsed(self.started_at, now_monotonic, self.grace_s))

    @property
    def exit_code(self) -> int:
        return 0 if self.status == "succeeded" else 1


def missing_stop_subscribers(publishers, channels):
    """List stop outputs without a discovered consumer; this is not an execution ack."""
    missing = []
    for channel in channels:
        publisher = publishers.get(channel)
        if publisher is None:
            missing.append(channel)
            continue
        try:
            if publisher.get_subscription_count() < 1:
                missing.append(channel)
        except Exception:
            missing.append(channel)
    return missing


def _json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate configuration key: {key}")
        result[key] = value
    return result


def outbound_payload(key, channel, values):
    """Encode observed reports using an explicitly selected public contract."""
    if channel.get("encoding") == "inspection_phrase_v1":
        if key != "recognition":
            raise ValueError("inspection phrases are only valid for recognition")
        from .official_protocol import recognition_phrase
        values = {**values, "phrase": recognition_phrase(values["kind"], values["target_id"])}
    return build_payload(channel["fields"], values)


def validate_outbound_fields(profile):
    """Validate templates against fields supplied by that channel at runtime."""
    recognition = {"event_id": "validation:0", "id": "validation:0", "point_id": "measured-point", "target_id": "被困人员", "kind": "ground",
                   "source": "ground", "camera_source": "ground", "vehicle": "ground", "image_stamp": 0.0,
                   "stamp": 0.0, "state": "VALIDATION", "pose": [0.0]*4}
    samples = {"recognition": recognition, "light": {"enabled": False, "stamp": 0.0, "state": "STOP"}}
    for key in ("current_target", "cruise_event"):
        if key in profile["channels"]:
            samples[key] = {"value": "ground_1" if key == "current_target" else "window_1",
                            "stamp": 0.0, "state": "VALIDATION", "point_id": "measured-point"}
    if profile["flight_mode"] == "trigger":
        if profile.get("trigger_abort_stops_motion") is not True or "flight_abort" not in profile["channels"]:
            raise ValueError("trigger flight requires a confirmed emergency abort contract")
        flight_motion_intent((0, 0, 0, 0), "TAKEOFF", profile)
        for key in ("takeoff", "land", "flight_abort"):
            samples[key] = {"enabled": True, "stamp": 0.0, "state": "VALIDATION"}
    result = {}
    for key, sample in samples.items():
        fields = profile["channels"][key].get("fields")
        if not isinstance(fields, dict):
            raise ValueError(f"missing confirmed message template: {key}")
        result[key] = outbound_payload(key, profile["channels"][key], sample)
    return result


def resolve_runtime_path(value, default):
    if not value:
        return Path(default)
    path = Path(value)
    if not path.is_absolute():
        raise ValueError("explicit plan/profile paths must be absolute")
    return path


def load_configuration(plan_path, interface_profile_path, settings=None):
    """Load strict JSON and validate critical settings without importing ROS."""
    def load(path):
        def nonfinite(value):
            raise ValueError(f"nonfinite JSON value: {value}")
        payload = json.loads(Path(path).read_text(encoding="utf-8"),
                             object_pairs_hook=_json_object, parse_constant=nonfinite)
        if not isinstance(payload, dict):
            raise ValueError("configuration root must be an object")
        return payload

    profile = validate_profile(load(interface_profile_path))
    validate_outbound_fields(profile)
    plan = load(plan_path)
    if plan.get("protocol") == "inspection_v1":
        required = {
            "ground_tasks": ("/referee/ground_tasks", "std_msgs/msg/String"),
            "recognition": ("/inspection/detections", "std_msgs/msg/String"),
            "current_target": ("/referee/current_target", "std_msgs/msg/String"),
            "cruise_event": ("/referee/cruise_event", "std_msgs/msg/String"),
            "light": ("/uav/laser_fire", "std_msgs/msg/Bool"),
        }
        if any(key not in profile["channels"] or
               (profile["channels"][key]["topic"], profile["channels"][key]["type"]) != binding
               for key, binding in required.items()):
            raise ValueError("inspection_v1 requires official output channels from referee rules table 1")
        recognition = profile["channels"]["recognition"]
        if recognition.get("encoding") != "inspection_phrase_v1" or recognition.get("fields") != {"data": "$phrase"}:
            raise ValueError("inspection_v1 requires the official table-5 recognition phrase encoder")
        for key, fields in {
            "current_target": {"data": "$value"},
            "cruise_event": {"data": "$value"},
            "light": {"data": "$enabled"},
        }.items():
            if profile["channels"][key].get("fields") != fields:
                raise ValueError(f"inspection_v1 requires the official runtime field binding for {key}")
        order = profile["channels"]["ground_tasks"]
        if (order.get("encoding") != "csv" or order.get("path") != "data"
                or order.get("id_field") != "" or order.get("selection_count") != 3):
            raise ValueError("inspection_v1 requires the official String.data CSV order selecting three ground points")
        ground_points = plan.get("ground_points")
        if (not isinstance(ground_points, list) or len(ground_points) != 5
                or any(not isinstance(point, dict) or not isinstance(point.get("id"), str)
                       or re.fullmatch(r"ground_[0-9]+", point["id"]) is None for point in ground_points)
                or len({point["id"] for point in ground_points}) != 5):
            raise ValueError("inspection_v1 ground_points must contain exactly five unique ground_<nonnegative integer> IDs")
        if profile["flight_mode"] == "trigger" and profile["trigger_max_speed_mps"] > 0.5:
            raise ValueError("inspection_v1 automatic flight speed must not exceed the official 0.5 m/s limit")
    supplied = plan.get("settings", {})
    if not isinstance(supplied, dict):
        raise ValueError("mission settings must be an object")
    merged = {**supplied, **(settings or {})}
    mode = recognition_confirmation_mode(profile)
    if merged.get("recognition_confirmation_mode", mode) != mode:
        raise ValueError("recognition_confirmation_mode must match the explicit interface profile")
    controller = MissionController(plan, {**merged, "recognition_confirmation_mode": mode})
    RuntimeInputs(profile, {point["id"] for point in plan["ground_points"]},
                  sensor_timeout_s=controller.settings["pose_timeout_s"],
                  detection_timeout_s=controller.settings["detection_timeout_s"],
                  uav_envelope=controller.uav_envelope)
    return plan, profile, controller


def create_node():
    """Construct the node with actual ROS2 Humble dependencies."""
    from ament_index_python.packages import get_package_share_directory
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields
    from std_msgs.msg import String

    class InspectionMissionNode(Node):
        def __init__(self):
            super().__init__("inspection_mission")
            config_dir = Path(get_package_share_directory("inspection_mission")) / "config"
            for key, default in {
                "plan_path": str(config_dir / "mission_plan.json"),
                "interface_profile_path": str(config_dir / "interface_profile.json"),
                "detection_topic": "/inspection_mission/observations",
                "mission_status_topic": "/inspection/mission_status",
                "runtime_status_topic": "/inspection_mission/status",
                "control_rate_hz": 20.0,
            }.items():
                self.declare_parameter(key, default)
            controller_defaults = {
                "phase_timeout_s": 90.0, "pose_timeout_s": 0.5, "detection_timeout_s": 0.35,
                "recognition_stable_s": 0.5, "illumination_s": 2.0, "no_progress_timeout_s": 8.0,
                "recognition_transport_wait_s": 0.2,
                "obstacle_timeout_s": 4.0, "require_ground_order": True, "require_detection": True,
            }
            for key, default in controller_defaults.items():
                self.declare_parameter(key, default)
            self.declare_parameter("scan_timeout_s", 0.5)
            parameter = lambda key: self.get_parameter(key).value
            plan, self.profile, self.controller = load_configuration(
                resolve_runtime_path(parameter("plan_path"), config_dir / "mission_plan.json"),
                resolve_runtime_path(parameter("interface_profile_path"), config_dir / "interface_profile.json"),
                settings={key: parameter(key) for key in controller_defaults})
            rate = number(parameter("control_rate_hz"), "control_rate_hz")
            if not 1 <= rate <= 100:
                raise ValueError("control_rate_hz must lie in [1,100]")
            self.channels = self.profile["channels"]
            self.inputs = RuntimeInputs(self.profile, {p["id"] for p in plan["ground_points"]},
                                        sensor_timeout_s=self.controller.settings["pose_timeout_s"],
                                        detection_timeout_s=self.controller.settings["detection_timeout_s"],
                                        scan_timeout_s=parameter("scan_timeout_s"),
                                        uav_envelope=self.controller.uav_envelope)
            self.runtime_fault = ""
            self._blocked_since = None
            self._flight_sent = set()
            self._flight_cancelled = False
            self._logged_errors = {}
            self._pubs, self._subs = {}, []
            self._types = {key: get_message(spec["type"]) for key, spec in self.channels.items()}
            outbound = ["ground_cmd", "uav_cmd", "recognition", "light"]
            outbound += [key for key in ("current_target", "cruise_event") if key in self.channels]
            if self.profile["flight_mode"] == "trigger":
                outbound += ["takeoff", "land", "flight_abort"]
            # Validate type and field bindings before creating actuator publishers.
            for key, fields in validate_outbound_fields(self.profile).items():
                set_message_fields(self._types[key](), fields)
            for key in outbound:
                self._pubs[key] = self.create_publisher(self._types[key], self.channels[key]["topic"],
                                                       self._qos(self.channels[key]))
            self._status = self.create_publisher(String, str(parameter("mission_status_topic")), 10)
            self._runtime_status = self.create_publisher(String, str(parameter("runtime_status_topic")), 10)
            self._outcome = RunOutcome()
            self._clock_watchdog = SimulationClockWatchdog(
                max(2.0, 4 * max(self.inputs.sensor_timeout_s, self.inputs.scan_timeout_s)),
                startup_timeout_s=10.0)
            for source in ("ground", "uav"):
                for kind in ("odom", "scan"):
                    if source + "_" + kind not in self.channels:
                        continue
                    self._subscribe(source + "_" + kind,
                                    lambda msg, source=source, kind=kind: self._sensor_callback(source, kind, msg),
                                    sensor=True)
            self._subscribe("ground_tasks", lambda msg: self._receive("ground_tasks", self.inputs.update_order, msg))
            if "recognition_ack" in required_input_channels(self.profile):
                self._subscribe("recognition_ack", lambda msg: self._receive("recognition_ack", self.inputs.update_ack, msg))
            self._subscribe("light_feedback", lambda msg: self._receive(
                "light_feedback", lambda value: self.inputs.update_light(value, now=self._now()), msg))
            if "uav_cloud" in self.channels:
                self._subscribe("uav_cloud", self._cloud_callback, sensor=True)
            self._subs.append(self.create_subscription(String, str(parameter("detection_topic")), self._detections_callback, 10))
            self.create_timer(1.0 / rate, self._tick)
            steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1.0, self._heartbeat, clock=steady_clock)
            self.create_timer(0.1, self._watchdog_tick, clock=steady_clock)
            self._termination_timer = self.create_timer(
                0.05, self._completion_shutdown_tick, clock=steady_clock)
            self.get_logger().info("Complete inspection controller ready; waiting for confirmed interfaces and observations")
            if recognition_confirmation_mode(self.profile) == "transport_only":
                self.get_logger().warning("Recognition uses local transport only; referee acceptance and official scoring remain unverified")
            if "uav_cloud" not in self.channels:
                self.get_logger().warning("UAV scan guard is planar; vertical clearance relies on the measured mission plan")

        def _now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def _completion_shutdown_tick(self):
            if self._outcome.shutdown_due(time.monotonic()):
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()

        def _watchdog_tick(self):
            if self._outcome.status == "succeeded":
                return
            try:
                if self._clock_watchdog.observe(self._now(), time.monotonic()):
                    self._fail_runtime("simulation_clock_stalled")
            except Exception as exc:
                self._fail_runtime(f"clock_watchdog_exception:{type(exc).__name__}:{exc}")

        def _fail_runtime(self, detail):
            if self._outcome.status == "failed":
                return
            self.runtime_fault = self.runtime_fault or str(detail)
            self.get_logger().error(self.runtime_fault)
            self._stop(require_subscribers=True)
            terminal_detail = (self.runtime_fault or self.inputs.fault or
                               self.controller.fault_reason or str(detail))
            self._outcome.fail(terminal_detail, time.monotonic())

        def _qos(self, spec, sensor=False):
            config = spec.get("qos", {})
            reliability = config.get("reliability", "best_effort" if sensor else "reliable")
            durability = config.get("durability", "volatile")
            if reliability not in ("best_effort", "reliable") or durability not in ("volatile", "transient_local"):
                raise ValueError("unsupported QoS reliability/durability")
            return QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT if reliability == "best_effort" else ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL if durability == "transient_local" else DurabilityPolicy.VOLATILE)

        def _subscribe(self, key, callback, sensor=False):
            self._subs.append(self.create_subscription(self._types[key], self.channels[key]["topic"], callback,
                                                       self._qos(self.channels[key], sensor)))

        def _error(self, key, exc):
            detail = f"{key}:{type(exc).__name__}:{exc}"
            self.inputs.errors[key] = detail
            if self._logged_errors.get(key) != detail:
                self.get_logger().error(detail)
                self._logged_errors[key] = detail
            self._stop()

        def _receive(self, key, parser, msg):
            try:
                parser(message_to_ordereddict(msg))
                self.inputs.errors.pop(key, None)
            except Exception as exc:
                self._error(key, exc)

        def _sensor_callback(self, source, kind, msg):
            key = source + "_" + kind
            try:
                function = self.inputs.update_pose if kind == "odom" else self.inputs.update_scan
                function(source, message_to_ordereddict(msg), now=self._now())
            except Exception as exc:
                if kind == "odom":
                    self.inputs.poses[source] = None
                else:
                    self.inputs.scans[source] = None
                self._error(key, exc)

        def _cloud_callback(self, msg):
            try:
                from sensor_msgs_py import point_cloud2
                points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=False)
                self.inputs.update_cloud(message_to_ordereddict(msg.header), points, now=self._now())
            except Exception as exc:
                self.inputs.cloud = None
                self._error("uav_cloud", exc)

        def _detections_callback(self, msg):
            try:
                self.inputs.update_detections(json.loads(msg.data), now=self._now())
                self.inputs.errors.pop("detections", None)
            except Exception as exc:
                self.inputs.frames = {"ground": None, "uav": None}
                self._error("detections", exc)

        def _connected(self):
            vehicle = self.controller.active_vehicle
            vehicle = "ground" if vehicle == "none" else vehicle
            outputs = (["ground_cmd", "recognition", "current_target"] if vehicle == "ground"
                       else ["uav_cmd", "recognition", "light", "cruise_event"])
            if vehicle == "uav" and self.profile["flight_mode"] == "trigger":
                outputs += ["takeoff", "land", "flight_abort"]
            if any(self._pubs[key].get_subscription_count() == 0 for key in outputs if key in self._pubs):
                return False
            required = required_input_channels(self.profile,
                require_ground_order=self.controller.settings["require_ground_order"], active_vehicle=vehicle)
            return all(self.count_publishers(self.channels[key]["topic"]) > 0 for key in required)

        def _message(self, key, values):
            msg = self._types[key]()
            set_message_fields(msg, outbound_payload(key, self.channels[key], values))
            return msg

        def _stop(self, *, require_subscribers=False):
            errors = []
            if require_subscribers:
                required = ["ground_cmd", "uav_cmd", "light"]
                if (self.profile["flight_mode"] == "trigger" and self._flight_sent
                        and not self._flight_cancelled and not self.controller.completed):
                    required.append("flight_abort")
                errors.extend("no_stop_subscriber:" + channel
                              for channel in missing_stop_subscribers(self._pubs, required))
            for key in ("ground_cmd", "uav_cmd"):
                try:
                    self._pubs[key].publish(self._types[key]())
                except Exception as exc:
                    errors.append(key + ":" + str(exc))
            try:
                self._pubs["light"].publish(self._message("light", {"enabled": False, "stamp": self._now(), "state": "STOP"}))
            except Exception as exc:
                errors.append("light_off:" + str(exc))
            if (self.profile["flight_mode"] == "trigger" and self._flight_sent
                    and not self._flight_cancelled and not self.controller.completed):
                try:
                    self._pubs["flight_abort"].publish(self._message(
                        "flight_abort", {"enabled": True, "stamp": self._now(), "state": "ABORT"}))
                    self._flight_cancelled = True
                    self.runtime_fault = self.runtime_fault or "trigger_flight_aborted"
                except Exception as exc:
                    errors.append("flight_abort:" + str(exc))
            if errors:
                detail = "stop_delivery_failed:" + ";".join(errors)
                self.runtime_fault = self.runtime_fault or detail
                self.inputs.errors["stop"] = detail
                self.get_logger().error(detail)
            return not errors

        def _publish(self, command):
            if command.declaration is not None:
                key = command.declaration["channel"]
                if key not in ("current_target", "cruise_event") or key not in self._pubs:
                    raise ValueError("missing confirmed declaration channel: " + str(key))
                self._pubs[key].publish(self._message(key, command.declaration))
            ground = self._types["ground_cmd"]()
            ground.linear.x, ground.angular.z = command.ground
            aerial = self._types["uav_cmd"]()
            vx, vy, vz, yaw_rate = command.uav
            pose = self.inputs.poses["uav"]
            if any(abs(value) > 1e-12 for value in (vx, vy, vz, yaw_rate)) and pose is None:
                raise ValueError("cannot transform UAV command without pose")
            bx, by = planar_velocity(vx, vy, pose.yaw if pose else 0.0, self.profile["uav_velocity_frame"])
            aerial.linear.x, aerial.linear.y, aerial.linear.z, aerial.angular.z = bx, by, vz, yaw_rate
            if self.profile["flight_mode"] == "trigger" and command.state in ("TAKEOFF", "LAND"):
                aerial = self._types["uav_cmd"]()
                if (command.state not in self._flight_sent and not command.fault
                        and self._blocked_since is None and not self.runtime_fault):
                    key = command.state.lower()
                    self._pubs[key].publish(self._message(key, {"enabled": True, "stamp": self._now(), "state": command.state}))
                    self._flight_sent.add(command.state)
            self._pubs["ground_cmd"].publish(ground)
            self._pubs["uav_cmd"].publish(aerial)
            self._pubs["light"].publish(self._message("light", {"enabled": command.light, "stamp": self._now(), "state": command.state}))
            if command.recognition is not None:
                publish_recognition_report(self.controller, command.recognition,
                    lambda report: self._pubs["recognition"].publish(self._message("recognition", report)), self._now())

        def _tick(self):
            try:
                now = self._now()
                if self.runtime_fault or self.inputs.fault:
                    self._fail_runtime(self.runtime_fault or self.inputs.fault)
                    return
                command = self.controller.step(
                    now, self.inputs.poses["ground"], self.inputs.poses["uav"], self.inputs.fresh_detections(now),
                    ground_clear=self.inputs.obstacle_fresh("ground", now), uav_clear=self.inputs.obstacle_fresh("uav", now),
                    light_ready=self.inputs.light_ready(now), controls_ready=self._connected(),
                    recognition_ack=self.inputs.acknowledgements, ground_order=self.inputs.order)
                vehicle = self.controller.active_vehicle
                if vehicle != "none":
                    velocity = (command.ground if vehicle == "ground" else
                                flight_motion_intent(command.uav, command.state, self.profile))
                    clear = motion_path_clear(self.controller, self.inputs, vehicle, velocity, now)
                    if not clear:
                        if self._blocked_since is None:
                            self._blocked_since = now
                        if now - self._blocked_since >= self.controller.settings["obstacle_timeout_s"]:
                            self.runtime_fault = "motion_path_blocked_or_unobserved:" + vehicle
                        if self.profile["flight_mode"] == "trigger" and self._flight_sent:
                            self.runtime_fault = "trigger_flight_path_blocked"
                        command = replace(command, ground=(0.0, 0.0), uav=(0.0, 0.0, 0.0, 0.0), light=False, recognition=None)
                    else:
                        self._blocked_since = None
                self._publish(command)
                if command.done and self._outcome.status is None:
                    stop_succeeded = self._stop(require_subscribers=True)
                    if self.runtime_fault or self.inputs.fault:
                        self._outcome.fail(self.runtime_fault or self.inputs.fault, time.monotonic())
                    else:
                        completion = String()
                        completion.data = completion_status_payload()
                        completed = self._outcome.complete_after_stop(
                            stop_succeeded, time.monotonic(), lambda: self._status.publish(completion))
                        if completed:
                            self.get_logger().info("主任务本地流程已完成；官方计分仍待平台核实")
                        elif self._outcome.detail:
                            self.runtime_fault = self.runtime_fault or self._outcome.detail
                elif command.fault or self.runtime_fault:
                    self._fail_runtime(self.runtime_fault or self.controller.fault_reason or "mission_failed")
            except Exception as exc:
                self.runtime_fault = f"runtime_exception:{type(exc).__name__}:{exc}"
                self.get_logger().error(self.runtime_fault)
                self._fail_runtime(self.runtime_fault)

        def _heartbeat(self):
            try:
                now = self._now()
                message = String()
                message.data = json.dumps({"stamp": now, "session_id": self.controller.session_id,
                                           **self.controller.recognition_status(),
                                           "state": "FAILSAFE" if self.runtime_fault or self.inputs.fault else self.controller.state,
                                           "done": self._outcome.status == "succeeded" and not (self.runtime_fault or self.inputs.fault),
                                           "fault": self.runtime_fault or self.inputs.fault or self.controller.fault_reason,
                                           "errors": self.inputs.errors,
                                           "pose_fresh": {s: self.inputs.pose_fresh(s, now) for s in ("ground", "uav")},
                                           "scan_fresh": {s: self.inputs.scan_fresh(s, now) for s in ("ground", "uav")}},
                                          ensure_ascii=False, allow_nan=False)
                self._runtime_status.publish(message)
            except Exception as exc:
                self.runtime_fault = "status_exception:" + str(exc)
                self._fail_runtime(self.runtime_fault)

    return InspectionMissionNode()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = None
    try:
        node = create_node()
        rclpy.spin(node)
        if node._outcome.status is None:
            node._fail_runtime("spin_ended_without_terminal_result")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node._outcome.exit_code
