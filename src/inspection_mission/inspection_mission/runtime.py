"""Sensor/message validation and final motion gates, with no ROS imports."""
from __future__ import annotations

import math

from .control import Pose2D
from .envelope import UAVEnvelope
from .geometry_runtime import (PoseHistory, camera_metadata, confirmed_text,
                               detection_with_metadata, parse_odometry_evidence)
from .perception import (pointcloud_path_clear, scan_path_clear, yaw_level_rotation,
                         pointcloud_envelope_path_clear, pointcloud_sensor_envelope_path_clear)
from .protocol import (decode_ack, decode_bool, decode_order, planar_velocity,
                       recognition_confirmation_mode, sensor_time, validate_profile,
                       uav_obstacle_sensor)


def number(value, label="number") -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return float(value)


def flight_motion_intent(command_uav, state, profile):
    """The motion to guard, including a zero-Twist automatic-flight transition.

    This is only safety geometry; the ROS adapter still publishes the confirmed
    trigger message, never this synthesized Twist. The platform must confirm
    the automatic vertical speed upper bound.
    """
    if not isinstance(command_uav, (list, tuple)) or len(command_uav) != 4:
        raise ValueError("UAV command must contain four values")
    intent = tuple(number(value, "UAV command") for value in command_uav)
    mode = profile.get("flight_mode")
    if mode not in ("velocity", "trigger"):
        raise ValueError("unconfirmed flight mode")
    if mode == "trigger" and state in ("TAKEOFF", "LAND"):
        speed = number(profile.get("trigger_max_speed_mps"), "trigger_max_speed_mps")
        if speed <= 0:
            raise ValueError("trigger_max_speed_mps must be a confirmed positive upper bound")
        return (0.0, 0.0, speed if state == "TAKEOFF" else -speed, 0.0)
    return intent


def header_time(payload: dict, expected_frame: str, now: float) -> float:
    header = payload["header"]
    if header["frame_id"] != expected_frame:
        raise ValueError(f"unconfirmed frame: {header['frame_id']!r}, expected {expected_frame!r}")
    stamp = header["stamp"]
    sec, nano = stamp["sec"], stamp["nanosec"]
    if type(sec) is not int or type(nano) is not int or not 0 <= nano < 1_000_000_000:
        raise ValueError("invalid header timestamp")
    return sensor_time(sec, nano, now)


def parse_pose(payload: dict, expected_frame: str, now: float, *, max_tilt_rad=0.05) -> Pose2D:
    stamp = header_time(payload, expected_frame, now)
    pose = payload["pose"]["pose"]
    position, q = pose["position"], pose["orientation"]
    xyz = [number(position[key], key) for key in ("x", "y", "z")]
    qx, qy, qz, qw = (number(q[key], "quaternion") for key in ("x", "y", "z", "w"))
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if abs(norm - 1.0) > 0.01:
        raise ValueError("quaternion must be normalized")
    # All following scan/control geometry is planar. Refuse tilted poses rather
    # than discarding roll/pitch and applying a wrong yaw-only cloud transform.
    tilt = math.acos(max(-1.0, min(1.0, 1 - 2 * (qx*qx + qy*qy) / (norm*norm))))
    if tilt > max_tilt_rad:
        raise ValueError("pose tilt exceeds the confirmed planar-motion contract")
    yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
    return Pose2D(*xyz, yaw, stamp)


class RuntimeInputs:
    """Invalid updates clear previous evidence; public-order errors latch a fault."""

    def __init__(self, profile: dict, known_ground_ids: set[str], *, sensor_timeout_s=0.5,
                 detection_timeout_s=0.35, scan_timeout_s=None,
                 pose_history_max_samples=128, pose_history_span_s=2.0, uav_envelope=None):
        self.profile = validate_profile(profile)
        self.channels = profile["channels"]
        self.known_ground_ids = set(known_ground_ids)
        self.sensor_timeout_s = number(sensor_timeout_s)
        self.detection_timeout_s = number(detection_timeout_s)
        self.scan_timeout_s = number(sensor_timeout_s if scan_timeout_s is None else scan_timeout_s)
        if min(self.sensor_timeout_s, self.detection_timeout_s, self.scan_timeout_s) <= 0:
            raise ValueError("runtime timeouts must be positive")
        self.max_tilt_rad = number(profile.get("max_planar_tilt_rad", 0.05), "max_planar_tilt_rad")
        if not 0 <= self.max_tilt_rad <= 0.15:
            raise ValueError("planar-motion tilt tolerance must lie in [0,0.15] radians")
        if uav_envelope is not None and not isinstance(uav_envelope, UAVEnvelope):
            raise ValueError("uav_envelope must be a validated UAVEnvelope")
        self.uav_envelope = uav_envelope
        for source in ("ground", "uav"):
            if source == "uav" and uav_obstacle_sensor(profile) == "pointcloud":
                continue
            spec = self.channels[source + "_scan"]
            if not isinstance(spec.get("frame_id"), str) or not spec["frame_id"].strip():
                raise ValueError(f"{source}_scan.frame_id must be confirmed")
            number(spec.get("yaw_offset_rad"), source + "_scan.yaw_offset_rad")
        if "uav_cloud" in self.channels:
            spec = self.channels["uav_cloud"]
            if spec["type"] != "sensor_msgs/msg/PointCloud2" or not spec.get("frame_id"):
                raise ValueError("uav_cloud requires PointCloud2 and a confirmed body frame_id")
        self.poses = {"ground": None, "uav": None}
        # This raw evidence sidecar does not relax the planar control contract.
        self.odometry_evidence = {"ground": None, "uav": None}
        self._pose_histories = {source: PoseHistory(max_samples=pose_history_max_samples,
                                                   max_span_s=pose_history_span_s)
                                for source in self.poses}
        self._geometry_last_now = None
        self._expected_child_frames = {}
        for source in self.poses:
            spec = self.channels[source + "_odom"]
            if "child_frame_id" in spec:
                self._expected_child_frames[source] = confirmed_text(spec["child_frame_id"], source + " expected child frame")
        self.scans = {"ground": None, "uav": None}
        self.cloud = None
        self.frames = {"ground": None, "uav": None}
        self.order = None
        self.acknowledgements: set[str] = set()
        self.light = False
        self.light_stamp = None
        self.fault = ""
        self.errors = {}

    @staticmethod
    def _fresh(stamp, now, timeout):
        return stamp is not None and math.isfinite(now) and -0.05 <= now - stamp <= timeout

    def update_pose(self, source, payload, *, now):
        self.poses[source] = None
        self.odometry_evidence[source] = None
        try:
            self._observe_geometry_time(now)
            full = parse_odometry_evidence(payload, self.profile[source + "_frame"], now,
                                            expected_child_frame=self._expected_child_frames.get(source))
            planar = parse_pose(payload, self.profile[source + "_frame"], now,
                                 max_tilt_rad=(min(self.max_tilt_rad, self.uav_envelope.max_tilt_rad)
                                     if source == "uav" and self.uav_envelope is not None else self.max_tilt_rad))
            self._pose_histories[source].append(full)
            self.odometry_evidence[source], self.poses[source] = full, planar
            self.errors.pop(source + "_odom", None)
        except (KeyError, TypeError, ValueError) as exc:
            self._pose_histories[source].clear()
            self.frames[source] = None
            self.errors[source + "_odom"] = str(exc)
            raise ValueError(str(exc)) from exc

    def _observe_geometry_time(self, now):
        now = number(now, "runtime time")
        if now < 0:
            raise ValueError("negative runtime time")
        previous, self._geometry_last_now = self._geometry_last_now, now
        if previous is not None and now < previous:
            for source, history in self._pose_histories.items():
                history.clear()
                self.odometry_evidence[source] = None
                self.frames[source] = None
            raise ValueError("geometry clock reversed; all old geometry evidence cleared")

    def pose_at_exposure(self, source, stamp, *, expected_child_frame, max_skew_s, now, max_age_s):
        """Return a preceding raw sample; no calibration, clock ID or fix is invented."""
        self._observe_geometry_time(now)
        if source not in self._pose_histories:
            raise ValueError("unknown odometry source")
        return self._pose_histories[source].at_exposure(
            stamp, expected_child_frame=expected_child_frame, max_skew_s=max_skew_s,
            now=now, max_age_s=max_age_s)

    def pose_fresh(self, source, now):
        pose = self.poses[source]
        return pose is not None and self._fresh(pose.stamp, now, self.sensor_timeout_s)

    def update_scan(self, source, payload, *, now):
        self.scans[source] = None
        try:
            spec = self.channels[source + "_scan"]
            stamp = header_time(payload, spec["frame_id"], now)
            values = {key: number(payload[key], key) for key in
                      ("angle_min", "angle_increment", "range_min", "range_max")}
            if values["angle_increment"] == 0 or values["range_min"] < 0 or values["range_max"] <= values["range_min"]:
                raise ValueError("invalid scan limits")
            ranges = payload["ranges"]
            if not isinstance(ranges, (list, tuple)) or len(ranges) < 3:
                raise ValueError("scan has insufficient beams")
            ranges = [float(value) for value in ranges]
            self.scans[source] = {**values, "ranges": ranges, "stamp": stamp}
            self.errors.pop(source + "_scan", None)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.errors[source + "_scan"] = str(exc)
            raise ValueError(str(exc)) from exc

    def scan_fresh(self, source, now):
        scan = self.scans[source]
        return scan is not None and self._fresh(scan["stamp"], now, self.scan_timeout_s)

    def obstacle_fresh(self, source, now):
        if source == "uav" and uav_obstacle_sensor(self.profile) == "pointcloud":
            return self.cloud is not None and self._fresh(self.cloud["stamp"], now, self.scan_timeout_s)
        return self.scan_fresh(source, now)

    def update_cloud(self, header, points, *, now):
        self.cloud = None
        try:
            spec = self.channels["uav_cloud"]
            stamp = header_time({"header": header}, spec["frame_id"], now)
            self.cloud = {"stamp": stamp, "points": list(points)}
            self.errors.pop("uav_cloud", None)
        except (KeyError, TypeError, ValueError) as exc:
            self.errors["uav_cloud"] = str(exc)
            raise ValueError(str(exc)) from exc

    def update_detections(self, payload, *, now):
        source = payload.get("source") if isinstance(payload, dict) else None
        if source not in self.frames:
            self.frames = {"ground": None, "uav": None}
            raise ValueError("invalid camera source")
        previous = self.frames[source]
        self.frames[source] = None
        try:
            self._observe_geometry_time(now)
            stamp = number(payload["stamp"], "camera stamp")
            if stamp < 0 or stamp > now + 0.05:
                raise ValueError("invalid/future camera stamp")
            if previous is not None and stamp < previous["stamp"]:
                raise ValueError("camera stamp reversed")
            if payload.get("error"):
                raise ValueError("camera processing error: " + str(payload["error"]))
            detections = payload["detections"]
            if not isinstance(detections, list):
                raise ValueError("detections must be a list")
            metadata = camera_metadata(payload, now=now)
            parsed = []
            for item in detections:
                target = item["target_id"]
                center = item["center"]
                confidence = number(item["confidence"], "confidence")
                if (not isinstance(target, str) or not target or not 0 <= confidence <= 1
                        or not isinstance(center, (list, tuple)) or len(center) != 2
                        or any(not 0 <= number(value, "center") <= 1 for value in center)
                        or item.get("source", source) != source):
                    raise ValueError("malformed detection")
                parsed.append(detection_with_metadata(item, metadata, source=source, stamp=stamp, now=now))
            self.frames[source] = {**metadata, "stamp": stamp, "detections": parsed}
            self.errors.pop(source + "_camera", None)
        except (KeyError, TypeError, ValueError) as exc:
            self.errors[source + "_camera"] = str(exc)
            raise ValueError(str(exc)) from exc

    def fresh_detections(self, now):
        return [item for frame in self.frames.values()
                if frame is not None and self._fresh(frame["stamp"], now, self.detection_timeout_s)
                for item in frame["detections"]]

    def update_order(self, payload):
        try:
            order = decode_order(payload, self.channels["ground_tasks"], self.known_ground_ids)
            if self.order is not None and order != self.order:
                raise ValueError("ground order changed")
            self.order = order
        except (KeyError, TypeError, ValueError) as exc:
            self.order = None
            self.fault = "ground_order_invalid:" + str(exc)
            raise ValueError(str(exc)) from exc

    def update_ack(self, payload):
        if recognition_confirmation_mode(self.profile) == "transport_only":
            self.acknowledgements.clear()
            raise ValueError("transport_only has no referee acknowledgement contract")
        try:
            self.acknowledgements |= decode_ack(payload, self.channels["recognition_ack"])
        except (KeyError, TypeError, ValueError) as exc:
            self.acknowledgements.clear()
            raise ValueError(str(exc)) from exc

    def update_light(self, payload, *, now):
        self.light, self.light_stamp = False, None
        self.light = decode_bool(payload, self.channels["light_feedback"])
        self.light_stamp = number(now)

    def light_ready(self, now):
        return self.light and self._fresh(self.light_stamp, now, self.sensor_timeout_s)

    def motion_clear(self, source, velocity, now, *, stop_distance, corridor_radius=0.2):
        """Check the proposed command direction, never a previous tick's direction.

        LaserScan guards horizontal travel only. Without configured PointCloud2,
        vertical clearance relies on the measured/validated mission plan.
        """
        if not self.pose_fresh(source, now) or not self.obstacle_fresh(source, now):
            return False
        pose, scan = self.poses[source], self.scans[source]
        if source == "ground":
            linear, omega = velocity
            vx, vy, vz = linear * math.cos(pose.yaw), linear * math.sin(pose.yaw), 0.0
        else:
            vx, vy, vz, omega = velocity
        if not all(math.isfinite(value) for value in (vx, vy, vz, omega)):
            return False
        envelope = self.uav_envelope if source == "uav" else None
        if envelope is not None and corridor_radius < envelope.horizontal_radius_m:
            return False  # A caller cannot silently reduce the rotor/tilt bound.
        if source == "uav" and uav_obstacle_sensor(self.profile) == "pointcloud":
            # A front-facing aperture cannot prove the volume swept by a yaw.
            if abs(omega) > 1e-8:
                return False
            if math.sqrt(vx*vx + vy*vy + vz*vz) <= 1e-8:
                return True  # No requested motion; no clearance claim is made.
            bx, by = planar_velocity(vx, vy, pose.yaw, "body")
            if envelope is not None:
                evidence = self.odometry_evidence["uav"]
                if evidence is None or evidence.stamp_s != self.cloud["stamp"]:
                    return False  # No pose/cloud skew bound is confirmed here.
                return pointcloud_sensor_envelope_path_clear(self.cloud["points"], (bx, by, vz),
                    self.channels["uav_cloud"], stop_distance=stop_distance, corridor_radius=corridor_radius,
                    envelope=envelope, body_to_level=yaw_level_rotation(evidence.rotation_xyzw, pose.yaw))
            from .perception import pointcloud_sensor_path_clear
            return pointcloud_sensor_path_clear(self.cloud["points"], (bx, by, vz),
                self.channels["uav_cloud"], stop_distance=stop_distance, corridor_radius=corridor_radius)
        speed = math.hypot(vx, vy)
        if speed > 1e-8:
            directions = [math.atan2(vy, vx)]
        elif abs(omega) > 1e-8:
            directions = [pose.yaw + index * math.pi / 2 for index in range(4)]
        else:
            directions = []
        for direction in directions:
            if not scan_path_clear(scan["ranges"], angle_min=scan["angle_min"],
                                   angle_increment=scan["angle_increment"],
                                   world_direction=direction, robot_yaw=pose.yaw,
                                   stop_distance=stop_distance, range_min=scan["range_min"],
                                   range_max=scan["range_max"],
                                   corridor_radius=corridor_radius,
                                   scan_yaw_offset=self.channels[source + "_scan"]["yaw_offset_rad"]):
                return False
        if source == "uav" and "uav_cloud" in self.channels and math.sqrt(vx*vx+vy*vy+vz*vz) > 1e-8:
            if self.cloud is None or not self._fresh(self.cloud["stamp"], now, self.scan_timeout_s):
                return False
            bx, by = planar_velocity(vx, vy, pose.yaw, "body")
            if envelope is not None:
                evidence = self.odometry_evidence["uav"]
                if evidence is None or evidence.stamp_s != self.cloud["stamp"] or abs(omega) > 1e-8:
                    return False
                rotation = yaw_level_rotation(evidence.rotation_xyzw, pose.yaw)
                # This legacy optional cloud is already in body coordinates.
                return pointcloud_envelope_path_clear(
                    [(rotation @ point).tolist() for point in self.cloud["points"]], (bx, by, vz),
                    stop_distance=stop_distance, corridor_radius=corridor_radius,
                    below_origin_m=envelope.below_origin_m, above_origin_m=envelope.above_origin_m)
            if not pointcloud_path_clear(self.cloud["points"], (bx, by, vz),
                                         stop_distance=stop_distance, corridor_radius=corridor_radius):
                return False
        return True
