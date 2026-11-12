"""Complete inspection controller using only observable poses and detections.

This module has no ROS dependency and contains no default field coordinates.
UAV velocity is in the configured world frame; ground velocity is forward/yaw.
Recognition defaults to explicit event acknowledgements. A configured
transport-only contract advances after observed publication and a short wait;
neither publication nor controller completion proves official scoring.
"""

from __future__ import annotations

import copy
import math
import uuid
from dataclasses import dataclass

from .control import Pose2D, angle_error
from .envelope import uav_envelope_from_plan
from .protocol import recognition_confirmation_mode


@dataclass(frozen=True)
class MissionCommand:
    ground: tuple[float, float] = (0.0, 0.0)
    uav: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    light: bool = False
    recognition: dict | None = None
    state: str = "WAIT_ORDER"
    done: bool = False
    fault: str = ""
    declaration: dict | None = None


@dataclass(frozen=True)
class _Stage:
    name: str
    vehicle: str
    target: tuple[float, ...]
    kind: str = "navigate"
    target_id: str = ""
    window_index: int = -1
    point_id: str = ""
    declaration: tuple[str, str] | None = None
    camera_source: str | None = None
    camera_frame_id: str | None = None


_DEFAULTS = {
    "require_ground_order": True,
    # Recognition is part of every inspection stage.  Keep this explicit so a
    # caller cannot accidentally configure a run that reports success without
    # fresh camera evidence and the explicitly configured confirmation contract.
    "require_detection": True,
    "recognition_confirmation_mode": "event_ack",
    "recognition_transport_wait_s": 0.2,
    "pose_timeout_s": 0.5,
    "detection_timeout_s": 0.35,
    "future_tolerance_s": 0.05,
    "recognition_stable_s": 0.5,
    "illumination_s": 2.0,
    "phase_timeout_s": 60.0,
    "mission_timeout_s": 900.0,
    "no_progress_timeout_s": 5.0,
    "obstacle_timeout_s": 3.0,
    "position_tolerance_m": 0.08,
    "yaw_tolerance_rad": 0.15,
    "window_z_tolerance_m": 0.04,
    "landing_z_tolerance_m": 0.03,
    "max_ground_speed_mps": 0.28,
    "max_ground_yaw_rate_rps": 0.6,
    "max_uav_speed_mps": 0.35,
    "max_uav_yaw_rate_rps": 0.6,
    "stable_speed_mps": 0.08,
    "stable_yaw_rate_rps": 0.15,
    "stable_radius_m": 0.05,
    "center_tolerance": 0.12,
    "minimum_confidence": 0.65,
    "min_aerial_height_m": 0.8,
    "window_descent_epsilon_m": 0.005,
    "visual_servo_enabled": False,
    "visual_max_yaw_offset_rad": 0.45,
    "visual_max_yaw_rate_rps": 0.15,
    "visual_yaw_gain": 0.8,
    "visual_control_horizon_s": 0.2,
    "light_feedback_timeout_s": 0.5,
}


def _finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _vector(value, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (tuple, list)) or len(value) != length or not all(_finite_number(v) for v in value):
        raise ValueError(f"{label} must contain {length} finite coordinates")
    return tuple(float(v) for v in value)


class MissionController:
    """Run three ground inspections, two windows, two lit aerial inspections.

    Plan fields are ``ground_points`` (3..5 candidates, 3 selected), ``aerial_points`` (2), ``windows``
    (2), ``home_ground``, ``home_uav``, ``robot_radius_m``, ``clearance_m``.
    Point dictionaries carry id, target_id and pose; windows carry id,
    target_id, approach, exit, crossing_z, bottom and top.  ``exit.z`` is the
    low crossing altitude. An optional complete ``uav_body_envelope`` separates
    level below/above-origin extents and a maximum tilt from the horizontal
    ``robot_radius_m``; its absence retains the legacy spherical clearance.
    Supplied coordinates must come from a real map.
    ``target_id='*'`` requests the unique observed camera identity at a physical
    point; reports always carry the decoded identity and the map's ``point_id``.
    A ground point may explicitly bind ``camera_source`` and ``camera_frame_id``
    for recognition only. This selects an observation, never its control vehicle.
    """

    def __init__(self, plan: dict, settings: dict | None = None):
        supplied = copy.deepcopy(settings or {})
        if "uav_altitude_m" in supplied:
            raise ValueError("uav_altitude_m is unsupported; use measured per-stage plan altitudes and min_aerial_height_m")
        self.settings = {**_DEFAULTS, **supplied}
        recognition_confirmation_mode(self.settings)
        for key in _DEFAULTS:
            if key == "recognition_confirmation_mode":
                continue
            if key in ("require_ground_order", "require_detection", "visual_servo_enabled"):
                if not isinstance(self.settings[key], bool):
                    raise ValueError(f"{key} must be boolean")
            elif not _finite_number(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not 1.0 <= self.settings["illumination_s"] <= 3.0:
            raise ValueError("illumination_s must be between 1 and 3 seconds")
        if self.settings["recognition_stable_s"] > 3.0:
            raise ValueError("recognition_stable_s cannot exceed the three-second illumination limit")
        if self.settings["min_aerial_height_m"] < 0.8:
            raise ValueError("minimum aerial inspection height is 0.8 metres")
        if self.settings["minimum_confidence"] > 1 or self.settings["center_tolerance"] > 0.5:
            raise ValueError("invalid detection confidence or center tolerance")
        if self.settings["visual_max_yaw_offset_rad"] > math.pi / 2:
            raise ValueError("visual yaw correction cannot exceed pi/2")
        if self.settings["visual_servo_enabled"]:
            sign = supplied.get("camera_yaw_sign")
            if not _finite_number(sign) or sign not in (-1, 1):
                raise ValueError("visual servo requires an explicit camera_yaw_sign of -1 or +1")
        if self.settings["require_detection"] is not True:
            raise ValueError("require_detection must remain true for inspection scoring")
        self.session_id = supplied.get("session_id", uuid.uuid4().hex)
        if not isinstance(self.session_id, str) or not self.session_id.strip():
            raise ValueError("session_id must be a nonempty string")
        self.plan = self._validate_plan(copy.deepcopy(plan))
        self.state = "WAIT_ORDER" if self.settings["require_ground_order"] else "WAIT_READY"
        self.completed = False
        self.fault_reason = ""
        self._order: list[str] | None = None
        self._stages: list[_Stage] = []
        self._stage_index = -1
        self._started: float | None = None
        self._phase_started: float | None = None
        self._last_now: float | None = None
        self._last_pose: dict[str, Pose2D] = {}
        self._last_stability: dict[str, bool] = {}
        self._evidence_since: float | None = None
        self._evidence_image_since: float | None = None
        self._last_evidence_image: float | None = None
        self._evidence_anchor: Pose2D | None = None
        self._evidence_target_id: str | None = None
        self._pending: dict | None = None
        self._pending_published_at: float | None = None
        self._published_reports: set[str] = set()
        self._report_serial = 0
        self._blocked_since: float | None = None
        self._progress_since: float | None = None
        self._progress_anchor: Pose2D | None = None
        self._identified_window_z: dict[int, float] = {}
        self._illumination_complete = False
        self._light_feedback_wait_since: float | None = None
        self._declaration_emitted = False
        self._reported_aerial: tuple[str, str] | None = None
        self._light_command_started_at: float | None = None

    @property
    def active_vehicle(self) -> str:
        if self._stage_index >= 0 and not self.completed and not self.fault_reason:
            return self._stages[self._stage_index].vehicle
        return "none"

    def mark_recognition_published(self, event_id: str, now: float) -> None:
        """Record only local successful publish; this never inserts a referee ACK."""
        if not _finite_number(now):
            raise ValueError("recognition publication time must be finite")
        if self._pending is None or event_id != self._pending["event_id"]:
            return
        if now < self._pending["stamp"]:
            raise ValueError("recognition publication precedes its report")
        self._published_reports.add(event_id)
        if self._pending_published_at is None:
            self._pending_published_at = now

    def recognition_status(self) -> dict:
        return {"recognition_confirmation_mode": self.settings["recognition_confirmation_mode"],
                "recognition_reports_published": len(self._published_reports),
                "official_scoring": "unverified"}

    def _validate_plan(self, plan: dict) -> dict:
        if not isinstance(plan, dict):
            raise ValueError("a measured mission plan is required")
        if plan.get("protocol") not in ("legacy", "inspection_v1"):
            raise ValueError("protocol must explicitly be legacy or inspection_v1")
        self.protocol = plan["protocol"]
        if self.protocol == "inspection_v1" and self.settings["require_ground_order"] is not True:
            raise ValueError("official inspection requires the public ground order")
        if self.protocol == "inspection_v1" and any(self.settings[key] > 0.5 for key in (
                "max_ground_speed_mps", "max_uav_speed_mps")):
            raise ValueError("official inspection speed limit is 0.5 m/s")
        plan["home_ground"] = _vector(plan.get("home_ground"), 3, "home_ground")
        plan["home_uav"] = _vector(plan.get("home_uav"), 4, "home_uav")
        for key in ("robot_radius_m", "clearance_m"):
            if not _finite_number(plan.get(key)) or plan[key] < 0 or (key == "robot_radius_m" and plan[key] == 0):
                raise ValueError(f"{key} is required and must be finite")
        self.uav_envelope = uav_envelope_from_plan(plan)
        ids, targets = set(), set()
        for group, count, maximum, dimensions in (
            # The inspection field has five candidate ground points; the referee sends
            # the three selected IDs for the current run.  Keeping the
            # candidate map separate from the run order prevents us from
            # treating the hidden draw as a fixed three-point map.
            ("ground_points", 3, 5, 3),
            ("aerial_points", 2, 2, 4),
            ("windows", 2, 2, 4),
        ):
            points = plan.get(group)
            if not isinstance(points, list) or not count <= len(points) <= maximum:
                if group == "ground_points":
                    raise ValueError("ground_points must contain between 3 and 5 candidate points")
                raise ValueError(f"{group} must contain exactly {count} points")
            for point in points:
                if not isinstance(point, dict):
                    raise ValueError(f"invalid {group} point")
                if "camera_source" in point or "camera_frame_id" in point:
                    if group != "ground_points":
                        raise ValueError("camera overrides are currently supported only for ground_points")
                    if point.get("camera_source") not in ("ground", "uav"):
                        raise ValueError("ground camera_source must explicitly be ground or uav")
                    frame = point.get("camera_frame_id")
                    if not isinstance(frame, str) or not frame or any(character.isspace() for character in frame):
                        raise ValueError("ground camera binding requires an explicitly confirmed camera_frame_id")
                for key, seen in (("id", ids), ("target_id", targets)):
                    value = point.get(key)
                    wildcard = key == "target_id" and value == "*"
                    if not isinstance(value, str) or not value.strip() or (not wildcard and value in seen):
                        raise ValueError(f"{key} must be nonempty and unique")
                    if not wildcard:
                        seen.add(value)
                point["via"] = self._validate_via(point.get("via", []), dimensions, group)
                if group != "windows":
                    point["pose"] = _vector(point.get("pose"), dimensions, f"{group}.pose")
                    if group == "ground_points" and self.protocol == "inspection_v1":
                        point["approach"] = _vector(point.get("approach"), 3, "ground_points.approach")
                        if math.dist(point["approach"][:2], point["pose"][:2]) <= 2 * self.settings["position_tolerance_m"]:
                            raise ValueError("ground approach must remain distinct from the entry pose")
                    if group == "aerial_points" and point["pose"][2] < self.settings["min_aerial_height_m"]:
                        raise ValueError("aerial point is below minimum inspection height")
                    continue
                point["approach"] = _vector(point.get("approach"), 4, "window.approach")
                point["exit"] = _vector(point.get("exit"), 4, "window.exit")
                if not all(_finite_number(point.get(k)) for k in ("crossing_z", "bottom", "top")):
                    raise ValueError("window bottom, top and crossing_z must be finite")
                margin = plan["robot_radius_m"] + plan["clearance_m"]
                below = margin if self.uav_envelope is None else self.uav_envelope.below_origin_m
                above = margin if self.uav_envelope is None else self.uav_envelope.above_origin_m
                lo, hi = point["bottom"] + below, point["top"] - above
                z = point["crossing_z"]
                if not lo <= z - self.settings["window_z_tolerance_m"] <= z + self.settings["window_z_tolerance_m"] <= hi:
                    raise ValueError("window crossing does not preserve vertical clearance")
                if self.uav_envelope is not None:
                    width = point.get("width_m")
                    if not _finite_number(width) or width <= 0:
                        raise ValueError("window width_m is required with uav_body_envelope")
                    # The configured centerline follows the aperture center;
                    # even a valid on-tolerance pose must keep every rotor clear.
                    if 2*(self.uav_envelope.horizontal_radius_m + self.settings["position_tolerance_m"]) > width:
                        raise ValueError("window horizontal clearance cannot contain position tolerance")
                if abs(point["exit"][2] - z) > 1e-9:
                    raise ValueError("window exit must be at crossing_z")
                descent_margin = (self.settings["window_descent_epsilon_m"]
                                  + self.settings["position_tolerance_m"]
                                  + self.settings["window_z_tolerance_m"])
                if point["approach"][2] - z + 1e-9 < descent_margin:
                    raise ValueError("window approach/crossing geometry lacks descent margin after navigation tolerances")
                if point["approach"][2] < self.settings["min_aerial_height_m"]:
                    raise ValueError("window approach must restore aerial cruise height")
                horizontal = (plan["robot_radius_m"] if self.uav_envelope is None
                              else self.uav_envelope.horizontal_radius_m + self.settings["position_tolerance_m"])
                if math.dist(point["approach"][:2], point["exit"][:2]) <= 2 * horizontal:
                    raise ValueError("window approach and exit must lie on distinct sides")
        plan["home_ground_via"] = self._validate_via(plan.get("home_ground_via", []), 3, "home_ground")
        plan["home_uav_via"] = self._validate_via(plan.get("home_uav_via", []), 4, "home_uav")
        cruise = max(self.settings["min_aerial_height_m"], plan["windows"][1]["approach"][2], plan["home_uav"][2])
        if any(abs(point[2] - cruise) > 1e-9 for point in plan["home_uav_via"]):
            raise ValueError("home_uav_via must preserve return cruise altitude before landing")
        return plan

    def _validate_via(self, via, dimensions, label):
        if not isinstance(via, list):
            raise ValueError(f"{label}.via must be a list")
        parsed = [_vector(point, dimensions, f"{label}.via") for point in via]
        if dimensions == 4 and any(point[2] < self.settings["min_aerial_height_m"] for point in parsed):
            raise ValueError("aerial via point is below minimum cruise height")
        return parsed

    def _add_route(self, name, vehicle, target, via=(), kind="navigate"):
        for index, point in enumerate(via, 1):
            self._stages.append(_Stage(f"{name}_VIA_{index}", vehicle, point, kind))
        self._stages.append(_Stage(name, vehicle, target, kind))

    def _build_stages(self, order: list[str]) -> None:
        by_id = {point["id"]: point for point in self.plan["ground_points"]}
        for index, point_id in enumerate(order, 1):
            point = by_id[point_id]
            official = self.protocol == "inspection_v1"
            observation = point["approach"] if official else point["pose"]
            self._add_route(f"GROUND_APPROACH_{index}" if official else f"GROUND_NAV_{index}", "ground", observation, point["via"])
            self._stages.append(_Stage(f"GROUND_RECOGNIZE_{index}", "ground", observation, "recognize", point["target_id"], point_id=point["id"],
                                       camera_source=point.get("camera_source"), camera_frame_id=point.get("camera_frame_id")))
            if official:
                self._add_route(f"GROUND_ENTER_{index}", "ground", point["pose"])
                self._stages.append(_Stage(f"GROUND_DECLARE_{index}", "ground", point["pose"], "declare", point_id=point["id"],
                                           declaration=("current_target", point["id"])))
        self._add_route("RETURN_GROUND", "ground", self.plan["home_ground"], self.plan["home_ground_via"])
        home = self.plan["home_uav"]
        cruise = max(self.settings["min_aerial_height_m"] + self.settings["position_tolerance_m"], self.plan["windows"][0]["approach"][2])
        self._stages.append(_Stage("TAKEOFF", "uav", (home[0], home[1], cruise, home[3]), "vertical"))
        self._add_window(0)
        first = self.plan["aerial_points"][0]
        exit1 = self.plan["windows"][0]["exit"]
        recovery = max(first["pose"][2], self.settings["min_aerial_height_m"] + self.settings["position_tolerance_m"])
        self._stages.append(_Stage("RECOVER_AFTER_WINDOW_1", "uav", (exit1[0], exit1[1], recovery, exit1[3]), "vertical"))
        for index, point in enumerate(self.plan["aerial_points"], 1):
            self._add_route(f"AIR_NAV_{index}", "uav", point["pose"], point["via"])
            if self.protocol == "inspection_v1":
                self._stages.append(_Stage(f"AIR_RECOGNIZE_{index}", "uav", point["pose"], "recognize", point["target_id"], point_id=point["id"]))
            kind = "illuminate_reported" if self.protocol == "inspection_v1" else "illuminate"
            self._stages.append(_Stage(f"AIR_INSPECT_{index}", "uav", point["pose"], kind, point["target_id"], point_id=point["id"]))
        self._add_window(1)
        exit2 = self.plan["windows"][1]["exit"]
        cruise = max(self.settings["min_aerial_height_m"], self.plan["windows"][1]["approach"][2], home[2])
        self._stages.append(_Stage("RECOVER_AFTER_WINDOW_2", "uav", (exit2[0], exit2[1], cruise, exit2[3]), "vertical"))
        self._add_route("RETURN_UAV", "uav", (home[0], home[1], cruise, home[3]), self.plan["home_uav_via"], "horizontal")
        self._stages.append(_Stage("LAND_DECLARE", "uav", (home[0], home[1], cruise, home[3]), "declare",
                                   declaration=("cruise_event", "landing_marker")))
        self._stages.append(_Stage("LAND", "uav", home, "vertical"))

    def _add_window(self, index: int) -> None:
        window = self.plan["windows"][index]
        label = f"WINDOW_{index + 1}"
        approach = window["approach"]
        self._add_route(f"{label}_APPROACH", "uav", approach, window["via"])
        self._stages.extend([
            _Stage(f"{label}_DECLARE", "uav", approach, "declare", point_id=window["id"],
                   declaration=("cruise_event", f"window_{index + 1}")),
            _Stage(f"{label}_RECOGNIZE", "uav", approach, "recognize", window["target_id"], index, point_id=window["id"]),
            _Stage(f"{label}_DESCEND", "uav", (approach[0], approach[1], window["crossing_z"], approach[3]), "descend", window_index=index),
            _Stage(f"{label}_CROSS", "uav", window["exit"], "cross", window_index=index),
        ])

    def _command(self, **fields) -> MissionCommand:
        return MissionCommand(state=self.state, done=self.completed, fault=self.fault_reason, **fields)

    def _fail(self, reason: str) -> MissionCommand:
        self.fault_reason = reason
        self.state = "FAILSAFE"
        self._reset_evidence()
        return self._command()

    def _reset_evidence(self) -> None:
        self._evidence_since = None
        self._evidence_image_since = None
        self._last_evidence_image = None
        self._evidence_anchor = None
        self._evidence_target_id = None
        self._pending = None
        self._pending_published_at = None
        self._illumination_complete = False

    def _clear_motion_progress(self) -> None:
        self._progress_since = None
        self._progress_anchor = None

    def _motion_progress_timed_out(self, pose: Pose2D, now: float) -> bool:
        if self._progress_anchor is None:
            self._progress_anchor, self._progress_since = copy.copy(pose), now
            return False
        anchor = self._progress_anchor
        movement = math.dist((pose.x, pose.y, pose.z), (anchor.x, anchor.y, anchor.z)) + abs(angle_error(pose.yaw, anchor.yaw))
        if movement >= 0.005:
            self._progress_anchor, self._progress_since = copy.copy(pose), now
            return False
        return now - self._progress_since >= self.settings["no_progress_timeout_s"]

    def _advance(self, now: float) -> MissionCommand:
        self._stage_index += 1
        self._reset_evidence()
        self._blocked_since = None
        self._progress_since = None
        self._progress_anchor = None
        self._light_feedback_wait_since = None
        self._declaration_emitted = False
        self._light_command_started_at = None
        self._phase_started = now
        if self._stage_index == len(self._stages):
            self.completed = True
            self.state = "COMPLETE"
        else:
            self.state = self._stages[self._stage_index].name
        return self._command()

    def _fresh_pose(self, pose: Pose2D | None, now: float) -> bool:
        return (pose is not None and all(_finite_number(getattr(pose, key, None)) for key in ("x", "y", "z", "yaw", "stamp"))
                and -self.settings["future_tolerance_s"] <= now - pose.stamp <= self.settings["pose_timeout_s"])

    def _reached(self, pose: Pose2D, stage: _Stage) -> bool:
        tol = self.settings["position_tolerance_m"]
        xy = math.hypot(stage.target[0] - pose.x, stage.target[1] - pose.y)
        yaw = stage.target[-1]
        yawtol = self.settings["yaw_tolerance_rad"]
        if stage.kind in ("recognize", "illuminate", "illuminate_reported") and self.settings["visual_servo_enabled"]:
            yawtol = self.settings["visual_max_yaw_offset_rad"]
        if xy > tol or abs(angle_error(yaw, pose.yaw)) > yawtol + 1e-9:
            return False
        if stage.vehicle == "ground":
            return True
        ztol = self.settings["window_z_tolerance_m"] if stage.kind in ("descend", "cross") else tol
        if stage.name == "LAND":
            ztol = self.settings["landing_z_tolerance_m"]
        return abs(stage.target[2] - pose.z) <= ztol

    def _velocity(self, pose: Pose2D, stage: _Stage) -> tuple:
        xy = math.hypot(stage.target[0] - pose.x, stage.target[1] - pose.y)
        tol = self.settings["position_tolerance_m"]
        if stage.vehicle == "ground":
            heading = math.atan2(stage.target[1] - pose.y, stage.target[0] - pose.x) if xy > tol else stage.target[2]
            err = angle_error(heading, pose.yaw)
            omega = max(-self.settings["max_ground_yaw_rate_rps"], min(self.settings["max_ground_yaw_rate_rps"], 1.5 * err))
            speed = min(self.settings["max_ground_speed_mps"], 0.8 * xy) if xy > tol and abs(err) < 0.45 else 0.0
            return speed, omega
        delta = [stage.target[0] - pose.x, stage.target[1] - pose.y, stage.target[2] - pose.z]
        if stage.kind in ("vertical", "descend"):
            delta[0] = delta[1] = 0.0
        elif stage.kind == "horizontal":
            delta[2] = 0.0
        norm = math.sqrt(sum(value * value for value in delta))
        scale = min(0.8, self.settings["max_uav_speed_mps"] / max(norm, 1e-12))
        angular = max(-self.settings["max_uav_yaw_rate_rps"], min(self.settings["max_uav_yaw_rate_rps"], 1.5 * angle_error(stage.target[3], pose.yaw)))
        return *(value * scale for value in delta), angular

    def _stable(self, pose: Pose2D, previous: Pose2D | None, now: float) -> bool:
        if previous is None:
            return False
        dt = pose.stamp - previous.stamp
        distance = math.dist((pose.x, pose.y, pose.z), (previous.x, previous.y, previous.z))
        turn = abs(angle_error(pose.yaw, previous.yaw))
        if dt <= 0:
            return dt == 0 and distance < 1e-9 and turn < 1e-9 and self._last_stability.get(self.active_vehicle, False)
        stable = distance / dt <= self.settings["stable_speed_mps"] and turn / dt <= self.settings["stable_yaw_rate_rps"]
        self._last_stability[self.active_vehicle] = stable
        return stable

    def _target_detection(self, detections: list[dict], stage: _Stage, now: float) -> dict | None:
        candidates = []
        camera_source = stage.camera_source or stage.vehicle
        for item in detections:
            if not isinstance(item, dict) or item.get("source") != camera_source:
                continue
            if stage.camera_frame_id is not None and item.get("frame_id") != stage.camera_frame_id:
                continue
            observed_id = item.get("target_id")
            if not isinstance(observed_id, str) or not observed_id.strip() or observed_id == "*":
                continue
            if stage.target_id != "*" and observed_id != stage.target_id:
                continue
            stamp, confidence, center = item.get("stamp"), item.get("confidence"), item.get("center")
            if not _finite_number(stamp) or not -self.settings["future_tolerance_s"] <= now - stamp <= self.settings["detection_timeout_s"]:
                continue
            if not _finite_number(confidence) or not self.settings["minimum_confidence"] <= confidence <= 1.0:
                continue
            if not isinstance(center, (tuple, list)) or len(center) != 2 or not all(_finite_number(v) and 0 <= v <= 1 for v in center):
                continue
            candidates.append(item)
        if stage.target_id != "*":
            return max(candidates, key=lambda item: item["stamp"], default=None)
        centered = [item for item in candidates
                    if max(abs(value - 0.5) for value in item["center"]) <= self.settings["center_tolerance"]]
        if len(centered) == 1:
            return centered[0]
        if not centered and len(candidates) == 1 and self.settings["visual_servo_enabled"]:
            return candidates[0]
        # Multiple centered observations are ambiguous even if two decoders
        # happen to return the same label. Never resolve identity by list order.
        return None

    def _visual_yaw_velocity(self, pose, stage, image):
        horizontal = image["center"][0] - 0.5
        rate = self.settings["camera_yaw_sign"] * self.settings["visual_yaw_gain"] * horizontal
        offset = angle_error(pose.yaw, stage.target[-1])
        remaining = max(0.0, self.settings["visual_max_yaw_offset_rad"] - (offset if rate > 0 else -offset))
        vehicle_limit = self.settings["max_ground_yaw_rate_rps"] if stage.vehicle == "ground" else self.settings["max_uav_yaw_rate_rps"]
        limit = min(self.settings["visual_max_yaw_rate_rps"], vehicle_limit, remaining / self.settings["visual_control_horizon_s"])
        rate = max(-limit, min(limit, rate))
        return (0.0, rate) if stage.vehicle == "ground" else (0.0, 0.0, 0.0, rate)

    def step(self, now: float, ground: Pose2D | None, uav: Pose2D | None, detections: list[dict], *,
             ground_clear: bool = True, uav_clear: bool = True, light_ready: bool = False,
             controls_ready: bool = True, recognition_ack: set[str] | None = None,
             ground_order: list[str] | None = None) -> MissionCommand:
        if self.completed or self.fault_reason:
            return self._command()
        if not _finite_number(now) or (self._last_now is not None and now < self._last_now):
            return self._fail("clock_invalid_or_reversed")
        if self._started is None:
            self._started = self._phase_started = now
        self._last_now = now
        if now - self._started > self.settings["mission_timeout_s"]:
            return self._fail("mission_timeout")
        if now - self._phase_started > self.settings["phase_timeout_s"]:
            return self._fail(f"phase_timeout:{self.state}")
        if ground_order is not None:
            ids = [point["id"] for point in self.plan["ground_points"]]
            if (not isinstance(ground_order, list) or len(ground_order) != 3
                    or any(not isinstance(value, str) for value in ground_order)
                    or len(set(ground_order)) != 3
                    or any(value not in ids for value in ground_order)):
                return self._fail("ground_order_invalid")
            if self._order is not None and ground_order != self._order:
                return self._fail("ground_order_changed")
            self._order = list(ground_order)
        if self._stage_index < 0:
            if self._order is None and self.settings["require_ground_order"]:
                self.state = "WAIT_ORDER"
                return self._command()
            if not controls_ready or not self._fresh_pose(ground, now):
                self.state = "WAIT_READY"
                return self._command()
            if self._order is None:
                self._order = [point["id"] for point in self.plan["ground_points"]]
            self._build_stages(self._order)
            self._last_pose["ground"] = copy.copy(ground)
            return self._advance(now)
        stage = self._stages[self._stage_index]
        if not controls_ready:
            return self._fail("control_unavailable")
        pose = ground if stage.vehicle == "ground" else uav
        if not self._fresh_pose(pose, now):
            return self._fail(f"{stage.vehicle}_pose_unavailable_or_stale")
        previous = self._last_pose.get(stage.vehicle)
        if previous is not None:
            if pose.stamp < previous.stamp:
                return self._fail(f"{stage.vehicle}_pose_stamp_reversed")
            if pose.stamp == previous.stamp and any(abs(getattr(pose, key) - getattr(previous, key)) > 1e-9 for key in ("x", "y", "z", "yaw")):
                return self._fail(f"{stage.vehicle}_pose_stamp_inconsistent")
        self._last_pose[stage.vehicle] = copy.copy(pose)
        clear = ground_clear if stage.vehicle == "ground" else uav_clear
        if not clear:
            self._reset_evidence()
            if self._blocked_since is None:
                self._blocked_since = now
            if now - self._blocked_since >= self.settings["obstacle_timeout_s"]:
                return self._fail(f"obstacle_timeout:{self.state}")
            return self._command()
        self._blocked_since = None
        if stage.vehicle == "uav":
            if stage.name.startswith("AIR_") and pose.z < self.settings["min_aerial_height_m"]:
                return self._fail("aerial_below_minimum_height")
            if stage.kind in ("vertical", "descend") and math.hypot(pose.x - stage.target[0], pose.y - stage.target[1]) > self.settings["position_tolerance_m"]:
                return self._fail(f"vertical_motion_lateral_drift:{self.state}")
            if stage.kind == "cross":
                window = self.plan["windows"][stage.window_index]
                if abs(pose.z - window["crossing_z"]) > self.settings["window_z_tolerance_m"] + 1e-9:
                    return self._fail("window_crossing_altitude_invalid")
            if stage.kind == "horizontal" and abs(pose.z - stage.target[2]) > self.settings["position_tolerance_m"]:
                return self._fail("return_cruise_altitude_drift")
        if stage.kind in ("recognize", "illuminate"):
            return self._inspect(now, pose, previous, stage, detections, light_ready, recognition_ack or set())
        if stage.kind == "illuminate_reported":
            return self._illuminate_reported(now, pose, previous, stage, detections, light_ready)
        if stage.kind == "declare" and self._reached(pose, stage):
            # Declaration is a separate zero-motion command before the next
            # stage can recognize or descend.  The adapter binds this semantic
            # channel to the verified official ROS transport.
            if self._declaration_emitted:
                return self._advance(now)
            self._declaration_emitted = True
            channel, value = stage.declaration
            return self._command(declaration={"channel": channel, "value": value, "stamp": now,
                                               "state": stage.name, "point_id": stage.point_id})
        if self._reached(pose, stage):
            if stage.kind == "descend":
                start_z = self._identified_window_z.get(stage.window_index)
                if start_z is None or start_z - pose.z < self.settings["window_descent_epsilon_m"]:
                    return self._fail("window_descent_not_observed")
            return self._advance(now)
        if self._motion_progress_timed_out(pose, now):
            return self._fail(f"motion_no_progress:{self.state}")
        velocity = self._velocity(pose, stage)
        return self._command(**{stage.vehicle: velocity})

    def _inspect(self, now, pose, previous, stage, detections, light_ready, acknowledgements):
        image = self._target_detection(detections, stage, now)
        if image is not None and self._evidence_target_id is not None and image["target_id"] != self._evidence_target_id:
            self._reset_evidence()
        image_stamp = image["stamp"] if image is not None else None
        centered = image is not None and max(abs(value - 0.5) for value in image["center"]) <= self.settings["center_tolerance"]
        reached = self._reached(pose, stage)
        valid = reached and self._stable(pose, previous, now) and centered
        lit = stage.kind == "illuminate"
        light = lit and valid and not self._illumination_complete
        if light and not light_ready:
            if self._light_feedback_wait_since is None:
                self._light_feedback_wait_since = now
            elif now - self._light_feedback_wait_since >= self.settings["light_feedback_timeout_s"]:
                return self._fail("light_feedback_timeout")
        else:
            self._light_feedback_wait_since = None
        if not valid or (light and not light_ready):
            self._reset_evidence()
            # Reacquire the measured inspection pose before relying on camera visibility.
            if not reached:
                if self._motion_progress_timed_out(pose, now):
                    return self._fail(f"motion_no_progress:{self.state}")
                velocity = self._velocity(pose, stage)
                return self._command(**{stage.vehicle: velocity})
            if image is None:
                self._clear_motion_progress()
                return self._command()
            if (self.settings["visual_servo_enabled"] and reached
                    and abs(image["center"][0] - 0.5) > self.settings["center_tolerance"]):
                if self._motion_progress_timed_out(pose, now):
                    return self._fail(f"motion_no_progress:{self.state}")
                return self._command(**{stage.vehicle: self._visual_yaw_velocity(pose, stage, image)})
            self._clear_motion_progress()
            return self._command(light=light)
        self._clear_motion_progress()
        if self._evidence_anchor is not None:
            anchor = self._evidence_anchor
            if math.dist((pose.x, pose.y, pose.z), (anchor.x, anchor.y, anchor.z)) > self.settings["stable_radius_m"]:
                self._reset_evidence()
        if self._last_evidence_image is not None and image_stamp < self._last_evidence_image:
            self._reset_evidence()
        if self._evidence_since is None:
            self._evidence_since, self._evidence_anchor = now, copy.copy(pose)
            self._evidence_image_since = image_stamp
            self._evidence_target_id = image["target_id"]
        self._last_evidence_image = image_stamp
        dwell = max(self.settings["recognition_stable_s"], self.settings["illumination_s"] if lit else 0)
        if (now - self._evidence_since + 1e-9 < dwell
                or image_stamp - self._evidence_image_since + 1e-9 < self.settings["recognition_stable_s"]):
            return self._command(light=light)
        mode = self.settings["recognition_confirmation_mode"]
        confirmed = (self._pending is not None and self._pending["event_id"] in acknowledgements
                     if mode == "event_ack" else self._pending_published_at is not None
                     and now - self._pending_published_at + 1e-9 >= self.settings["recognition_transport_wait_s"])
        if confirmed:
            if stage.window_index >= 0:
                self._identified_window_z[stage.window_index] = pose.z
            elif self.protocol == "inspection_v1" and stage.vehicle == "uav":
                self._reported_aerial = (stage.point_id, image["target_id"])
            return self._advance(now)
        if self._pending is None:
            self._report_serial += 1
            event_id = f"{self.session_id}:{self._report_serial}"
            self._pending = {"event_id": event_id, "id": event_id, "target_id": image["target_id"], "point_id": stage.point_id,
                             "source": image["source"], "camera_source": image["source"], "vehicle": stage.vehicle,
                             "image_stamp": image["stamp"], "stamp": now, "state": stage.name,
                             "kind": "window" if stage.window_index >= 0 else ("ground" if stage.vehicle == "ground" else "aerial"),
                             "pose": [pose.x, pose.y, pose.z, pose.yaw]}
            if "frame_id" in image:
                self._pending["frame_id"] = image["frame_id"]
            if lit:
                self._illumination_complete = True
                light = False
        return self._command(light=light, recognition=copy.deepcopy(self._pending))

    def _restart_aerial_recognition(self, now: float) -> MissionCommand:
        """A stale/changed target may not reuse the preceding reported identity."""
        self._reported_aerial = None
        if self._stage_index < 1 or not self._stages[self._stage_index - 1].name.startswith("AIR_RECOGNIZE_"):
            return self._fail("aerial_recognition_stage_missing")
        self._stage_index -= 2
        return self._advance(now)

    def _illuminate_reported(self, now, pose, previous, stage, detections, light_ready):
        image = self._target_detection(detections, stage, now)
        identity = (stage.point_id, image["target_id"]) if image is not None else None
        centered = image is not None and max(abs(value - 0.5) for value in image["center"]) <= self.settings["center_tolerance"]
        if (identity != self._reported_aerial or not centered or not self._reached(pose, stage)
                or not self._stable(pose, previous, now)):
            return self._restart_aerial_recognition(now)
        if self._light_command_started_at is None:
            self._light_command_started_at = now
        command_elapsed = now - self._light_command_started_at
        if not light_ready:
            self._reset_evidence()
            if command_elapsed + 1e-9 >= 3.0:
                return self._fail("illumination_time_limit")
            if self._light_feedback_wait_since is None:
                self._light_feedback_wait_since = now
            elif now - self._light_feedback_wait_since >= self.settings["light_feedback_timeout_s"]:
                return self._fail("light_feedback_timeout")
            return self._command(light=True)
        self._light_feedback_wait_since = None
        if self._evidence_anchor is not None:
            anchor = self._evidence_anchor
            if math.dist((pose.x, pose.y, pose.z), (anchor.x, anchor.y, anchor.z)) > self.settings["stable_radius_m"]:
                return self._restart_aerial_recognition(now)
        if self._last_evidence_image is not None and image["stamp"] < self._last_evidence_image:
            return self._restart_aerial_recognition(now)
        if self._evidence_since is None:
            self._evidence_since, self._evidence_anchor = now, copy.copy(pose)
            self._evidence_image_since = image["stamp"]
        self._last_evidence_image = image["stamp"]
        if (now - self._evidence_since + 1e-9 >= self.settings["illumination_s"]
                and image["stamp"] - self._evidence_image_since + 1e-9 >= self.settings["recognition_stable_s"]
                and command_elapsed <= 3.0 + 1e-9):
            self._reported_aerial = None
            return self._advance(now)
        if command_elapsed + 1e-9 >= 3.0:
            return self._fail("illumination_time_limit")
        return self._command(light=True)
