"""Finite ground-only turn, measured approach and stationary public images.

The local scan swept-motion budget stays .06m. Independent permanent origins
and cumulative budgets bound this whole experiment; no scoring output is sent.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import importlib.util
import math
import os
import time
from dataclasses import asdict, dataclass

from .bounded_ground_probe import (GroundMotionGuard, MotionDecision, MotionLimits,
                                  _json_safe, create_bounded_node, finite, spin_control_loop)
from .ground_observation_survey import CANDIDATE_POINTS, angle_wrap, yaw_from_q
from .ground_short_drive import _ShortDriveObservationRecorder


@dataclass(frozen=True)
class ApproachLimits:
    target_distance_m: float = .20
    global_wall_s: float = 90.
    max_path_m: float = .30
    max_origin_displacement_m: float = .30
    local_swept_motion_budget_m: float = .06
    scan_anchor_motion_limit_m: float = .03
    local_stop_reserve_m: float = .03
    linear_command_mps: float = .03
    minimum_yaw_rate: float = .35
    maximum_yaw_rate: float = .5
    max_rotation_rad: float = 1.8
    max_heading_error_rad: float = .05
    max_reverse_m: float = .005
    max_cross_track_m: float = .02
    max_turn_drift_m: float = .06
    wait_wall_s: float = 5.
    turn_wall_s: float = 45.
    drive_wall_s: float = 55.
    capture_wall_s: float = 5.

    def __post_init__(self):
        caps = {"target_distance_m": .20, "global_wall_s": 90., "max_path_m": .30,
            "max_origin_displacement_m": .30, "local_swept_motion_budget_m": .06,
            "scan_anchor_motion_limit_m": .03, "local_stop_reserve_m": .03,
            "linear_command_mps": .03, "minimum_yaw_rate": .35, "maximum_yaw_rate": .5,
            "max_rotation_rad": 1.8, "max_heading_error_rad": .05, "max_reverse_m": .005,
            "max_cross_track_m": .02, "max_turn_drift_m": .06, "wait_wall_s": 5.,
            "turn_wall_s": 45., "drive_wall_s": 55., "capture_wall_s": 5.}
        for field, cap in caps.items():
            if not finite(getattr(self, field)) or not 0 < getattr(self, field) <= cap:
                raise ValueError(field + " exceeds bounded-approach limit")
        if (self.minimum_yaw_rate > self.maximum_yaw_rate
                or self.scan_anchor_motion_limit_m + self.local_stop_reserve_m > self.local_swept_motion_budget_m + 1e-9):
            raise ValueError("inconsistent local reserve or angular bounds")


class _ApproachObservations(_ShortDriveObservationRecorder):
    mode = "ground_bounded_approach"
    TRAJECTORY_CAP = 1200
    COMMAND_CAP = 5000

    def __init__(self, limits, *, started_wall_s):
        super().__init__(limits, started_wall_s=started_wall_s)
        self.approach_context = {}
        self.scan_records = []
        self.module_availability = {}
        for name in ("cv2", "numpy", "torch", "onnxruntime"):
            try:
                self.module_availability[name] = {"available": importlib.util.find_spec(name) is not None,
                                                  "method": "importlib.util.find_spec; not imported"}
            except (ImportError, ValueError, AttributeError) as exc:
                self.module_availability[name] = {"available": None, "error": str(exc)[:128]}

    def update_odom(self, payload, *, wall_s):
        # Reuse validation, not survey's 500-row recording or yaw callbacks.
        GroundMotionGuard.update_odom(self, payload, wall_s=wall_s)
        with self._lock:
            if self.odom is None:
                self.full_odom_history.clear()
                self.ground_odom_error = "ground_odom_invalid:" + self.reason
                if "odom_stamp_reversed" in self.reason:
                    self.epoch_valid = False
                return
            self.ground_odom_error = None
            sample = {"stamp_s": self.odom["stamp"], "wall_elapsed_s": wall_s-self.started_wall,
                "frame_id": "odom", "child_frame_id": "base_footprint", "source_topic": "/odom",
                "position": list(self.odom["xyz"]), "quaternion": list(self.odom["q"]),
                "yaw_rad": yaw_from_q(self.odom["q"]), "linear_speed": self.odom["speed"],
                "angular_speed": self.odom["yaw_speed"]}
            self.full_odom_history.append(sample)
            self._record_odom_period("/odom", sample)
            if not self.trajectory or sample["wall_elapsed_s"]-self.trajectory[-1]["wall_elapsed_s"] >= .1-1e-9:
                self.trajectory.append(copy.deepcopy(sample))
                if len(self.trajectory) > self.TRAJECTORY_CAP:
                    self.trajectory.pop(1)
                    self.trajectory_dropped += 1

    def record_command(self, *, wall_s, linear_x, angular_z):
        with self._lock:
            self.actual_commands.append({"wall_elapsed_s": wall_s-self.started_wall,
                "clock_stamp_s": None if self.clock is None else self.clock[0],
                "linear_x": linear_x, "angular_z": angular_z, "state": self.state})
            if len(self.actual_commands) > self.COMMAND_CAP:
                self.actual_commands.pop(1)
                self.actual_commands_dropped += 1

    def survey_evidence(self):
        with self._lock:
            evidence = super().survey_evidence()
            evidence.pop("drive", None)
            evidence.update(approach=copy.deepcopy(self.approach_context),
                scan_records=copy.deepcopy(self.scan_records),
                module_availability=copy.deepcopy(self.module_availability),
                series_capacities={"trajectory_10hz": self.TRAJECTORY_CAP, "actual_commands": self.COMMAND_CAP})
            return evidence


class GroundBoundedApproachGuard(GroundMotionGuard):
    mode = "ground_bounded_approach"
    wall_limit_reason = "approach_global_wall_time_limit"
    final_report_wall_s = 60.
    ACTIVE = frozenset(("TURN", "TURN_SETTLE", "DRIVE", "DRIVE_SETTLE", "CAPTURE", "PAUSE"))
    ZERO_PHASES = frozenset(("TURN_SETTLE", "DRIVE_SETTLE", "CAPTURE"))
    FRESHNESS = frozenset(("clock_stale", "odom_stale", "scan_stale", "odom_stale_or_invalid",
                          "scan_stale_or_unsafe", "scan_anchor_unavailable"))

    def __init__(self, *, started_wall_s, approach_limits=None):
        self.approach_limits = approach_limits or ApproachLimits()
        super().__init__(MotionLimits(.447, .114, .02), started_wall_s=started_wall_s)
        self.motion_wall_limit_s = self.approach_limits.global_wall_s
        # The adapter's watchdog deadline is global creation +90s, even if it
        # is armed later. first_motion_wall is recorded separately.
        self.pulse_started = started_wall_s
        self.phase = "WAIT"
        self.draw = self.target_id = self.target_xy = self.target_yaw = None
        self.global_origin = self.turn_origin = self.drive_origin = self.drive_yaw = None
        self.first_motion_wall = self.turn_started_wall = self.drive_started_wall = self.capture_started_wall = None
        self.total_path_m = self.accumulated_rotation_rad = 0.
        self.previous_path_xyz = self.previous_yaw = self.initial_turn_error = None
        self.resume_count = 0
        self.scan_anchor = self.latest_scan_payload = None
        self.capture_frames = []
        self._observations = _ApproachObservations(self.limits, started_wall_s=started_wall_s)

    def _budget_fault(self, wall):
        if not finite(wall) or wall < max(self.last_wall, self.last_observation_wall):
            return "wall_clock_reversed_or_invalid"
        if wall-self.started_wall >= self.approach_limits.global_wall_s-1e-9:
            return self.wall_limit_reason
        if (self.phase in ("TURN", "TURN_SETTLE") and self.turn_started_wall is not None
                and wall-self.turn_started_wall >= self.approach_limits.turn_wall_s-1e-9):
            return "approach_turn_wall_time_limit"
        if (self.phase in ("DRIVE", "DRIVE_SETTLE") and self.drive_started_wall is not None
                and wall-self.drive_started_wall >= self.approach_limits.drive_wall_s-1e-9):
            return "approach_drive_wall_time_limit"
        if (self.phase == "CAPTURE" and self.capture_started_wall is not None
                and wall-self.capture_started_wall >= self.approach_limits.capture_wall_s-1e-9):
            return "approach_capture_wall_time_limit"
        return ""

    def _observation_wall(self, topic, wall):
        if not finite(wall) or wall < max(self.last_wall, self.last_observation_wall):
            self.stop(topic+"_arrival_wall_reversed_or_invalid", wall)
            return False
        previous = self.last_sensor_arrival.get(topic)
        self.last_observation_wall = wall
        self.last_sensor_arrival[topic] = wall
        if self.state in self.ACTIVE:
            fault = self._budget_fault(wall)
            if fault:
                self.stop(fault, wall)
            elif previous is not None and wall-previous > self.limits.sensor_age_s+1e-9:
                self.temporal_fault(topic+"_stale", wall)
        return True

    def measured_forward(self):
        if self.drive_origin is None or self.odom is None:
            return None
        dx, dy = (self.odom["xyz"][i]-self.drive_origin[i] for i in (0, 1))
        return dx*math.cos(self.drive_yaw)+dy*math.sin(self.drive_yaw)

    def cross_track(self):
        if self.drive_origin is None or self.odom is None:
            return None
        dx, dy = (self.odom["xyz"][i]-self.drive_origin[i] for i in (0, 1))
        return -dx*math.sin(self.drive_yaw)+dy*math.cos(self.drive_yaw)

    def observed_motion_fault(self):
        if self.odom is None:
            return ""
        a = self.approach_limits
        if self.odom["tilt"] > .03:
            return "ground_tilt"
        if self.max_observed_displacement_m >= a.max_origin_displacement_m-1e-9:
            return "approach_origin_displacement_limit"
        if self.total_path_m >= a.max_path_m-1e-9:
            return "approach_path_limit"
        if self.accumulated_rotation_rad >= a.max_rotation_rad-1e-9:
            return "approach_rotation_limit"
        yaw_limit = a.maximum_yaw_rate+.05 if self.phase in ("TURN", "TURN_SETTLE") else .10
        if self.odom["speed"] > .10 or self.odom["yaw_speed"] > yaw_limit:
            return "unexpected_motion"
        if self.phase in ("TURN", "TURN_SETTLE") and self.turn_origin is not None:
            if math.dist(self.odom["xyz"], self.turn_origin) >= a.max_turn_drift_m-1e-9:
                return "approach_turn_displacement_limit"
        if (self.phase == "TURN_SETTLE" and self.target_yaw is not None
                and abs(angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))) > a.max_heading_error_rad):
            return "approach_turn_overshoot_limit"
        if self.drive_origin is not None:
            if abs(angle_wrap(yaw_from_q(self.odom["q"])-self.drive_yaw)) > a.max_heading_error_rad:
                return "approach_heading_limit"
            if self.measured_forward() < -a.max_reverse_m:
                return "approach_reverse_limit"
            if abs(self.cross_track()) > a.max_cross_track_m:
                return "approach_cross_track_limit"
        if self.scan_anchor is not None and self.state in self.ACTIVE:
            if math.dist(self.odom["xyz"], self.scan_anchor["odom"]["position"]) > a.scan_anchor_motion_limit_m:
                return "local_scan_motion_reserve_exhausted"
        return ""

    def _condition(self, wall):
        reason = super()._condition(wall)
        if reason:
            return reason
        if self.scan_anchor is None:
            return "scan_anchor_unavailable"
        return ""

    def _check_active_timing(self, wall):
        if self.state not in self.ACTIVE:
            return
        fault = self._budget_fault(wall) or self.observed_motion_fault()
        if fault:
            self.stop(fault, wall)
            return
        reason = self._condition(wall)
        if reason:
            self.temporal_fault(reason, wall)

    def update_clock(self, stamp_s, *, wall_s):
        with self._lock:
            super().update_clock(stamp_s, wall_s=wall_s)
            self._observations.update_clock(stamp_s, wall_s=wall_s)

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            active = self.state in self.ACTIVE
            super().update_odom(payload, wall_s=wall_s)
            self._observations.update_odom(payload, wall_s=wall_s)
            if active and self.odom is None:
                self.stop("odom_invalid:"+self.reason, wall_s)
                return
            if self.odom is None:
                return
            if self.global_origin is not None:
                xyz, yaw = self.odom["xyz"], yaw_from_q(self.odom["q"])
                self.total_path_m += math.dist(xyz, self.previous_path_xyz)
                self.accumulated_rotation_rad += abs(angle_wrap(yaw-self.previous_yaw))
                self.previous_path_xyz, self.previous_yaw = xyz, yaw
            if self.state not in self.ACTIVE:
                return
            fault = self._budget_fault(wall_s) or self.observed_motion_fault()
            if fault:
                self.stop(fault, wall_s)
                return
            if self.phase == "TURN":
                error = angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
                if abs(error) <= .025 or error*self.initial_turn_error < 0:
                    if abs(error) > self.approach_limits.max_heading_error_rad:
                        self.stop("approach_turn_overshoot_limit", wall_s)
                    else:
                        self._enter_phase("TURN_SETTLE")
            elif self.phase == "DRIVE":
                if self.measured_forward() >= .10:
                    self._record_scan("mid_drive", wall_s)
                if self.measured_forward() >= self.approach_limits.target_distance_m:
                    self._enter_phase("DRIVE_SETTLE")

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            active = self.state in self.ACTIVE
            super().update_scan(payload, wall_s=wall_s)
            if self.scan is None:
                self.scan_anchor = None
                if active:
                    self.stop("scan_invalid:"+self.reason, wall_s)
                return
            prior = next((row for row in reversed(self._observations.full_odom_history)
                          if row["stamp_s"] <= self.scan["stamp"]+1e-9), None)
            self.scan_anchor = None
            if prior is not None and 0 <= self.scan["stamp"]-prior["stamp_s"] <= .05+1e-9:
                self.scan_anchor = {"stamp_s": self.scan["stamp"], "wall_s": wall_s,
                    "odom": copy.deepcopy(prior), "scan_minus_odom_s": self.scan["stamp"]-prior["stamp_s"]}
            self.latest_scan_payload = _json_safe(copy.deepcopy(payload))
            if self.drive_origin is not None and self.measured_forward() >= .10:
                self._record_scan("mid_drive", wall_s)

    def temporal_fault(self, reason, wall):
        with self._lock:
            if self.state in self.ACTIVE and reason in self.FRESHNESS:
                self.stable_anchor = None
                self.state, self.reason = "PAUSE", reason
            else:
                self.stop(reason, wall)

    def handle_watchdog(self, reason, wall):
        with self._lock:
            hard = self._budget_fault(wall) or self.observed_motion_fault()
            if hard:
                self.stop(hard, wall)
            elif reason == "approach_zero_phase":
                return
            elif reason in self.FRESHNESS:
                self.temporal_fault(reason, wall)
            else:
                self.stop(reason, wall)

    def watchdog_reason(self, wall):
        with self._lock:
            if self.state not in self.ACTIVE:
                return ""
            fault = self._budget_fault(wall) or self.observed_motion_fault() or self._condition(wall)
            return fault or ("approach_zero_phase" if self.state in self.ZERO_PHASES else "")

    def _enter_phase(self, phase):
        self.phase = phase
        if self.state != "PAUSE":
            self.state = phase
        self.reason = "approach_"+phase.lower()
        if phase != "CAPTURE":
            self.stable_anchor = None

    def _stable_dwell(self, wall):
        pose = self.odom
        stable = pose["speed"] <= .01 and pose["yaw_speed"] <= .03
        if self.stable_anchor is not None:
            a = self.stable_anchor
            stable = (stable and math.dist(pose["xyz"], a[2]) <= .005
                      and abs(sum(x*y for x, y in zip(pose["q"], a[3]))) >= math.cos(.02/2))
        if not stable:
            self.stable_anchor = None
            return False
        if self.stable_anchor is None:
            self.stable_anchor = (wall, pose["stamp"], pose["xyz"], pose["q"])
        a = self.stable_anchor
        return min(wall-a[0], pose["stamp"]-a[1]) >= 1.-1e-9

    def _record_scan(self, stage, wall):
        if self.scan_anchor is None or any(row["stage"] == stage for row in self._observations.scan_records):
            return
        if stage == "mid_drive":
            p = self.scan_anchor["odom"]["position"]
            forward = ((p[0]-self.drive_origin[0])*math.cos(self.drive_yaw)
                       +(p[1]-self.drive_origin[1])*math.sin(self.drive_yaw))
            if forward < .10:
                return
        self._observations.scan_records.append({"stage": stage, "recorded_wall_elapsed_s": wall-self.started_wall,
            "source_topic": "/scan", "payload": copy.deepcopy(self.latest_scan_payload),
            "odom": copy.deepcopy(self.scan_anchor["odom"]),
            "scan_minus_odom_s": self.scan_anchor["scan_minus_odom_s"]})

    def step(self, *, wall_s):
        with self._lock:
            if not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall):
                return self.stop("wall_clock_reversed_or_invalid", wall_s)
            self.last_wall = wall_s
            if self.state in ("STOP", "FINISHED"):
                if wall_s-self.stopped_wall >= self.limits.stop_wall_s:
                    self.state = "FINISHED"
                return MotionDecision(0., self.state, self.reason)
            fault = self._budget_fault(wall_s)
            if fault:
                return self.stop(fault, wall_s)
            if self.state in self.ACTIVE:
                fault = self.observed_motion_fault() or self._condition(wall_s)
                if fault:
                    self.temporal_fault(fault, wall_s)
                    return MotionDecision(0., self.state, self.reason)
                if self.state == "PAUSE":
                    if not self._stable_dwell(wall_s):
                        return MotionDecision(0., self.state, "pause_stability_dwell")
                    if self.resume_count >= 3:
                        return self.stop("pause_resume_limit", wall_s)
                    self.resume_count += 1
                    self.state, self.reason = self.phase, "approach_"+self.phase.lower()
                    self.stable_anchor = None
            else:
                if wall_s-self.started_wall >= self.approach_limits.wait_wall_s:
                    return self.stop("wait_timeout:"+self.reason, wall_s)
                reason = self._condition(wall_s)
                if reason or self.draw is None:
                    self.stable_anchor = None
                    self.reason = reason or "waiting_for_public_draw"
                    return MotionDecision(0., self.state, self.reason)
                if not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, "start_stability_dwell")
                if math.hypot(*self.odom["xyz"][:2]) > .05 or abs(yaw_from_q(self.odom["q"])) > .05:
                    return self.stop("approach_initial_pose_outside_contract", wall_s)
                self.global_origin = self.turn_origin = self.odom["xyz"]
                self.pulse_origin = self.global_origin
                self.previous_path_xyz = self.global_origin
                self.previous_yaw = yaw_from_q(self.odom["q"])
                self.target_id, self.target_xy = self.draw[0], CANDIDATE_POINTS[self.draw[0]]
                self.target_yaw = math.atan2(self.target_xy[1]-self.global_origin[1], self.target_xy[0]-self.global_origin[0])
                self.initial_turn_error = angle_wrap(self.target_yaw-self.previous_yaw)
                if abs(self.initial_turn_error) > self.approach_limits.max_rotation_rad:
                    return self.stop("approach_requested_turn_exceeds_budget", wall_s)
                self.first_motion_wall = self.turn_started_wall = wall_s
                self._record_scan("start", wall_s)
                self._enter_phase("TURN")
            if self.phase == "TURN":
                error = angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
                if abs(error) <= .025 or error*self.initial_turn_error < 0:
                    if abs(error) > self.approach_limits.max_heading_error_rad:
                        return self.stop("approach_turn_overshoot_limit", wall_s)
                    self._enter_phase("TURN_SETTLE")
                else:
                    rate = math.copysign(min(self.approach_limits.maximum_yaw_rate,
                        max(self.approach_limits.minimum_yaw_rate, .6*abs(error))), error)
                    return MotionDecision(0., self.state, self.reason, rate)
            if self.phase in ("TURN_SETTLE", "DRIVE_SETTLE"):
                if not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, self.reason)
                if self.phase == "TURN_SETTLE":
                    self._record_scan("after_turn", wall_s)
                    self.drive_origin, self.drive_yaw = self.odom["xyz"], yaw_from_q(self.odom["q"])
                    self.drive_started_wall = wall_s
                    self._enter_phase("DRIVE")
                else:
                    self._record_scan("after_drive", wall_s)
                    self.capture_started_wall = wall_s
                    self._enter_phase("CAPTURE")
            if self.phase == "DRIVE":
                return MotionDecision(self.approach_limits.linear_command_mps, self.state, self.reason)
            return MotionDecision(0., self.state, self.reason)

    def observe_public(self, topic, payload, *, wall_s):
        if topic == "/uav/odom":
            self._observations.update_uav_odom(payload, wall_s=wall_s)
        elif topic == "/referee/ground_tasks":
            with self._lock:
                data = payload.get("data")
                parts = [p.strip() for p in data.split(",")] if isinstance(data, str) else []
                if len(parts) != 3 or len(set(parts)) != 3 or any(p not in CANDIDATE_POINTS for p in parts):
                    self.stop("invalid_public_draw", wall_s)
                elif self.draw is not None and parts != self.draw:
                    self.stop("public_draw_changed", wall_s)
                else:
                    self.draw = parts

    def observe_image(self, topic, msg, *, wall_s):
        with self._lock, self._observations._lock:
            self._observations.observe_image(topic, msg, wall_s=wall_s)
            if self.state in self.ACTIVE:
                hard = self._budget_fault(wall_s) or self.observed_motion_fault()
                if hard:
                    self.stop(hard, wall_s)
            if topic != "/uav/camera/image" or self.state != "CAPTURE" or len(self.capture_frames) >= 3:
                return
            try:
                sec, nano = msg.header.stamp.sec, msg.header.stamp.nanosec
                if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 1_000_000_000:
                    raise ValueError("invalid_capture_image_stamp")
                stamp = sec+nano/1e9
                channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
                if (channels is None or type(msg.width) is not int or type(msg.height) is not int
                        or type(msg.step) is not int or not 0 < msg.width <= 4096 or not 0 < msg.height <= 4096):
                    raise ValueError("invalid_capture_image_shape_or_encoding")
                raw = memoryview(msg.data).cast("B")
                if len(raw) != msg.step*msg.height or msg.step < msg.width*channels or len(raw) > 64*1024*1024:
                    raise ValueError("invalid_capture_image_stride")
            except (AttributeError, TypeError, ValueError) as exc:
                if len(self._observations.image_errors) < 16:
                    self._observations.image_errors.append("capture_source_invalid:"+str(exc)[:128])
                return
            record = self._observations.exposures.get(topic, {}).get("last")
            if (record is None or record["odom"] is None or self._condition(wall_s)
                    or record["stamp_s"] != stamp or record["frame_id"] != msg.header.frame_id
                    or record["wall_elapsed_s"] != wall_s-self.started_wall
                    or self.odom["speed"] > .01 or self.odom["yaw_speed"] > .03
                    or not self._stable_dwell(wall_s)
                    or record["odom"]["linear_speed"] > .01 or record["odom"]["angular_speed"] > .03
                    or record["frame_id"] != "uav/camera_link"
                    or not -self.limits.clock_sampling_lag_s-1e-9 <= self.clock[0]-record["stamp_s"] <= self.limits.sensor_age_s+1e-9
                    or (self.capture_frames and record["stamp_s"] <= self.capture_frames[-1]["record"]["stamp_s"])):
                return
            phase = "capture_"+str(len(self.capture_frames))
            self.capture_frames.append({"phase": phase, "msg": msg, "record": copy.deepcopy(record)})
            self._observations.exposures[topic][phase] = copy.deepcopy(record)
            if len(self.capture_frames) >= 3:
                self.stop("approach_observation_complete", wall_s)

    def record_command(self, *, wall_s, linear_x, angular_z):
        with self._lock, self._observations._lock:
            self._observations.record_command(wall_s=wall_s, linear_x=linear_x, angular_z=angular_z)
            self._observations.actual_commands[-1]["state"] = self.state

    def approach_evidence(self):
        with self._lock:
            return {"mode": self.mode, "limits": asdict(self.approach_limits), "draw": copy.deepcopy(self.draw),
                "target_id": self.target_id, "target_xy": self.target_xy, "target_yaw_rad": self.target_yaw,
                "candidate_mapping": "candidate_unverified", "phase": self.phase, "state": self.state, "reason": self.reason,
                "global_origin": self.global_origin, "turn_origin": self.turn_origin,
                "drive_origin": self.drive_origin, "drive_yaw_rad": self.drive_yaw,
                "first_motion_wall_s": self.first_motion_wall, "turn_started_wall_s": self.turn_started_wall,
                "drive_started_wall_s": self.drive_started_wall, "capture_started_wall_s": self.capture_started_wall,
                "measured_forward_m": self.measured_forward(), "cross_track_m": self.cross_track(),
                "total_path_m": self.total_path_m, "max_origin_displacement_m": self.max_observed_displacement_m,
                "accumulated_rotation_rad": self.accumulated_rotation_rad, "resume_count": self.resume_count,
                "max_resumes": 3, "scan_anchor": copy.deepcopy(self.scan_anchor),
                "capture_count": len(self.capture_frames), "capture_required": 3, "task_completion": "unverified"}

    def _sync_observation_context(self):
        with self._lock, self._observations._lock:
            self._observations.approach_context = self.approach_evidence()

    def survey_evidence(self):
        self._sync_observation_context()
        return self._observations.survey_evidence()

    def snapshot_survey_evidence(self):
        self._sync_observation_context()
        return self._observations.snapshot_survey_evidence()

    def _capture_image_events(self):
        events = []
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            self._observations.image_errors.append("capture_image_dependency:"+str(exc)[:128])
            return events
        for frame in self.capture_frames:
            msg, record = frame["msg"], frame["record"]
            try:
                channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
                if channels is None or not 0 < msg.width <= 4096 or not 0 < msg.height <= 4096:
                    raise ValueError("invalid_capture_image_shape_or_encoding")
                raw = memoryview(msg.data).cast("B")
                if len(raw) != msg.step*msg.height or msg.step < msg.width*channels or len(raw) > 64*1024*1024:
                    raise ValueError("invalid_capture_image_stride")
                pixels = np.frombuffer(raw, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*channels]
                pixels = pixels.reshape(msg.height, msg.width, channels) if channels == 3 else pixels.reshape(msg.height, msg.width)
                if msg.encoding == "rgb8":
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
                scale = min(1., 800/max(msg.width, msg.height))
                if scale < 1:
                    pixels = cv2.resize(pixels, (round(msg.width*scale), round(msg.height*scale)))
                ok, jpeg = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok or len(jpeg) > 384*1024:
                    raise ValueError("capture_jpeg_encode_or_size_limit")
                content = jpeg.tobytes()
                encoded = base64.b64encode(content).decode("ascii")
                chunks = [encoded[i:i+32000] for i in range(0, len(encoded), 32000)]
                for index, chunk in enumerate(chunks):
                    events.append(("probe_snapshot", {"mode": self.mode, "read_only": False,
                        "official_scoring": "unverified", "section": "survey_image_part", "partial": True,
                        "topic": "/uav/camera/image", "phase": frame["phase"], "stamp_s": record["stamp_s"],
                        "frame_id": record["frame_id"], "width": pixels.shape[1], "height": pixels.shape[0],
                        "encoding": "jpeg", "jpeg_sha256": hashlib.sha256(content).hexdigest(),
                        "part_index": index, "part_count": len(chunks), "jpeg_base64_part": chunk}))
            except Exception as exc:
                self._observations.image_errors.append("capture_image_error:"+str(exc)[:128])
        return events

    def additional_events(self):
        self._sync_observation_context()
        captures = self._capture_image_events() if self.capture_frames else []
        events = self._observations.additional_events()
        index = next((i for i, (_, payload) in enumerate(events) if payload.get("section") == "survey_image_part"), len(events))
        return events[:index]+captures+events[index:]


def create_approach_node(reporter=None, *, wall_clock=time.monotonic, approach_limits=None):
    def factory(limits, *, started_wall_s):
        return GroundBoundedApproachGuard(started_wall_s=started_wall_s, approach_limits=approach_limits)
    return create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock, guard_factory=factory)


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="Finite ground turn/approach/images; scoring unverified")
    parser.add_argument("--duration", type=float, default=90.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ApproachLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_approach_node(reporter, approach_limits=limits)
        node.report_started(parsed.duration)
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("approach_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("approach_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
