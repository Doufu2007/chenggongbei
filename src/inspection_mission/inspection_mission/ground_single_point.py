"""One measured first-ground-point experiment, not full mission completion.

Extends the frozen observation route without changing its default behavior.
Only current visual evidence can select a category. Local publish success is
transport evidence; unknown referee String feedback is never treated as ACK.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid

from .bounded_ground_probe import MotionDecision, MotionLimits, create_bounded_node, finite, spin_control_loop
from .ground_observation_route import GroundObservationRouteGuard, RouteCallbackTimingRecorder, RouteLimits
from .ground_observation_route_plan import GroundRouteSegment, plan_ground_observation_route, _POINTS
from .ground_observation_survey import angle_wrap, yaw_from_q
from .official_protocol import recognition_phrase


GROUND_LABELS = {"collapsed_ruins": "坍塌废墟", "landslide": "滑坡塌方", "trapped_people": "被困人员",
                 "damaged_vehicle": "受损车辆", "rescue_camp": "救援营地"}
ENTRY_WAYPOINTS = {
    "ground_0": ((.6, -1.35), (1.115, -1.35)),
    "ground_1": ((0., -.5), (0., -1.35)),
    "ground_2": ((0., -1.8), (0., -2.7)),
    "ground_3": ((.6, -1.925), (1.115, -1.925)),
    "ground_4": ((.35, -.775), (1.115, -.775)),
}
TEMPLATE_HASHES = {
    "collapsed_ruins_02.jpg": "200732c46709ed2d6fba5017ce7f56c8cc0908d60eed14b6063b84a9c37cba7f",
    "damaged_vehicle_01.png": "76feb761210089a1c3885ce09024ea3da6e991c52fc4bd7cc3f13d7e28b1c13b",
    "landslide_01.jpg": "2653c12e906773f388fde39fbee3633ede5531460bb1442096452c41fe10a30d",
    "landslide_02.jpg": "f79dc767cacfd16dc8d3f989ec3ae1743f46c85bafca95223120e19c516cc689",
    "rescue_camp_01.png": "781d7c9e505a5a46c9177f584d5aff220560d28a290d1046a6378bcbb38bd014",
    "rescue_camp_02.jpg": "b81c0269dc79776a10edcb54043b3e06a9188d5bbbf4b28ad951e4e998cb864a",
    "trapped_people_01.png": "e31429ccbf79d3898bc24fe5fb5bd8faefbfc6c29d9fc0ed573f78db1fde90b7",
    "trapped_people_02.jpg": "b8aefe526d7d3a3a9045107aae75f0f9cbdcc190c5a61c9d27a9c67646308777",
}
TEMPLATE_REPOSITORY = "wbxxmz/ros2-car-first"
TEMPLATE_COMMIT = "48b3dac26b6f562ff21dd321db03fd6ffc91b492"
# v24b delivered image headers every .331-.336s, wall gaps up to .384845s.
# Half a second covers that measured cadence plus one .05s control slot.
# This only bounds camera liveness during stationary recognition/reporting;
# control odom/scan age remains .20s and each exposure's prior remains .05s.
CURRENT_CAMERA_MAX_AGE_S = .50


@dataclass(frozen=True)
class FirstGroundPlan:
    observation: object
    entry_waypoints_xy_m: tuple
    total_path_m: float
    planned_rotation_rad: float
    budgets: object
    entry_allowed: bool
    entry_rejection_reason: str
    candidate_mapping_status: str = "candidate_unverified"


def plan_first_ground_point(first_id, initial_pose):
    observation = plan_ground_observation_route(first_id, initial_pose)
    waypoints = ENTRY_WAYPOINTS[first_id]
    position, heading = observation.waypoints_xy_m[-1], observation.final_yaw_rad
    length, rotation = observation.total_path_m, observation.planned_rotation_rad
    for target in waypoints:
        yaw = math.atan2(target[1]-position[1], target[0]-position[0])
        length += math.dist(position, target)
        rotation += abs(angle_wrap(yaw-heading))
        position, heading = target, yaw
    reason = ("path_exceeds_3m_and_south_wall_clearance" if first_id == "ground_2" else
              "path_exceeds_3m" if first_id == "ground_3" else "")
    allowed = not reason and length+.30 <= 3.+1e-9 and rotation+.40 <= 6.+1e-9
    if not allowed and not reason:
        reason = "composite_geometry_budget_exceeded"
    budgets = observation.budgets
    if allowed:
        peak = max(math.dist(initial_pose[:2], p) for p in (*observation.waypoints_xy_m, *waypoints))
        budgets = replace(budgets, max_path_m=length+.30, max_origin_displacement_m=peak+.15,
                          max_rotation_rad=rotation+.40)
    return FirstGroundPlan(observation, waypoints, length, rotation, budgets, bool(allowed), reason)


def _xy_list(value, count):
    return (isinstance(value, (tuple, list)) and len(value) == count
            and all(isinstance(p, (tuple, list)) and len(p) == 2
                    and all(type(v) in (int, float) and math.isfinite(v) for v in p) for p in value))


def bind_ground_candidate(result, first_id):
    """Pure current-pose/region association; caller owns original-job identity."""
    try:
        pose = result["job"]["pose"]
        position, quaternion = pose["position"], pose["quaternion"]
        if (len(position) != 3 or len(quaternion) != 4
                or not all(type(v) in (int, float) and math.isfinite(v) for v in [*position, *quaternion])
                or abs(sum(v*v for v in quaternion)-1.) > .01):
            return None
        yaw = yaw_from_q(quaternion)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        def world(point):
            return [position[0]+cosine*point[0]-sine*point[1],
                    position[1]+sine*point[0]+cosine*point[1]]
        associated = []
        for candidate in result["matcher"]["candidates"]:
            if candidate.get("accepted_candidate") is not True or candidate.get("class_id") not in GROUND_LABELS:
                continue
            source = candidate.get("source_quad_xy")
            center = candidate.get("body_plane_center_forward_left_m")
            quad = candidate.get("body_plane_quad_forward_left_m")
            if (not _xy_list(source, 4) or not all(0 <= x < 800 and 0 <= y < 800 for x, y in source)
                    or not _xy_list([center], 1) or not _xy_list(quad, 4)):
                continue
            center = world(center)
            nearby = [key for key, values in _POINTS.items() if math.dist(center, values[0]) <= .08+1e-9]
            if nearby != [first_id]:
                continue
            world_quad = [world(point) for point in quad]
            board, size, _ = _POINTS[first_id]
            if any(abs(p[0]-board[0]) > size[0]/2+.08 or abs(p[1]-board[1]) > size[1]/2+.08 for p in world_quad):
                continue
            associated.append({"class_id": candidate["class_id"], "label": GROUND_LABELS[candidate["class_id"]],
                "world_center_xy_m": center, "world_quad_xy_m": world_quad, "source_quad_xy": source,
                "template_sha256": candidate.get("template_sha256"), "provenance": candidate.get("provenance"),
                "candidate_calibration": True, "official_acceptance": False})
        if (not associated or len({item["class_id"] for item in associated}) != 1
                or any(math.dist(a["world_center_xy_m"], b["world_center_xy_m"]) > .03
                       for a in associated for b in associated)):
            return None
        return associated[0]
    except (KeyError, TypeError, ValueError, IndexError):
        return None


class GroundSinglePointGuard(GroundObservationRouteGuard):
    mode = "ground_single_point"
    EXTRA_ZERO = frozenset(("RECOGNIZE_WAIT", "REPORT_WAIT", "DECLARE_WAIT"))
    ACTIVE = GroundObservationRouteGuard.ACTIVE | EXTRA_ZERO
    ZERO_PHASES = GroundObservationRouteGuard.ZERO_PHASES | EXTRA_ZERO
    wall_limit_reason = "single_point_global_wall_time_limit"

    def __init__(self, *, started_wall_s, route_limits=None, session_id=None):
        super().__init__(started_wall_s=started_wall_s, route_limits=route_limits)
        self.session_id = session_id or uuid.uuid4().hex
        self.single_plan = None
        self.in_entry = False
        self.stationary_epoch = 0
        self.stationary_anchor = self.stationary_uav_anchor = None
        self.epoch_invalid = None
        self.recapture_count = 0
        self.recognition_started_wall = self.zero_started_wall = None
        self.camera_infos = []
        self.camera_signature = None
        self.last_uav_image = None
        self.last_camera_liveness_check = None
        self.expected_jobs = {}
        self.match_results = []
        self.matcher_failures = []
        self.recognition = None
        self.attempts = {}
        self.public_status = []

    def _start_turn(self, wall, *, final=False):
        if self.single_plan is None:
            self.single_plan = plan_first_ground_point(self.target_id, self.route_plan.initial_pose)
            b = self.single_plan.budgets
            a = self.approach_limits
            # This happens before the first command; no later budget growth.
            if b.max_path_m > 3. or b.max_rotation_rad > 6.:
                return self.stop("single_point_plan_budget_exceeded", wall)
            self.route_plan = replace(self.route_plan, budgets=replace(b,
                **{key: getattr(a, key) for key in ("global_wall_s", "drive_wall_s", "turn_wall_s",
                    "turn_total_wall_s", "capture_wall_s", "wait_wall_s")}))
            self.approach_limits = replace(a, max_path_m=b.max_path_m,
                max_origin_displacement_m=b.max_origin_displacement_m, max_rotation_rad=b.max_rotation_rad)
        if self.in_entry and final:
            self.drive_origin = self.drive_yaw = None
            self._enter_phase("DECLARE_WAIT")
            return
        super()._start_turn(wall, final=final)

    def _enter_phase(self, phase):
        settled_anchor = self.stable_anchor if phase == "DECLARE_WAIT" and self.phase == "DRIVE_SETTLE" else None
        super()._enter_phase(phase)
        if settled_anchor is not None:
            # No new motion/turn: retain the just-proven final-leg dual-clock dwell.
            self.stable_anchor = settled_anchor
        wall = max(self.last_wall, self.last_observation_wall)
        if phase in self.EXTRA_ZERO:
            self.zero_started_wall = wall
        if phase == "CAPTURE":
            self.stationary_epoch += 1
            self.stationary_anchor = copy.deepcopy(self.odom)
            self.stationary_uav_anchor = None
            self.epoch_invalid = None
            self.expected_jobs.clear()
            self.match_results.clear()
            self.capture_frames.clear()
            self.recognition = None
        if phase == "RECOGNIZE_WAIT":
            self.recognition_started_wall = wall

    def _budget_fault(self, wall):
        fault = super()._budget_fault(wall)
        if not fault and self.phase == "RECOGNIZE_WAIT" and self.recognition_started_wall is not None:
            if wall-self.recognition_started_wall >= 20.-1e-9:
                fault = "single_point_match_wall_time_limit"
        return fault

    def _stationary_epoch_fault(self):
        if self.phase not in ("CAPTURE", "RECOGNIZE_WAIT", "REPORT_WAIT") or self.stationary_anchor is None:
            return ""
        if self.epoch_invalid:
            return "single_point_stationary_epoch_invalid:"+self.epoch_invalid
        current, anchor = self.odom, self.stationary_anchor
        if current is not None and (math.dist(current["xyz"], anchor["xyz"]) > .01+1e-9
                or abs(angle_wrap(yaw_from_q(current["q"])-yaw_from_q(anchor["q"]))) > .02+1e-9
                or current["speed"] > .01 or current["yaw_speed"] > .03):
            return "single_point_stationary_epoch_motion"
        return ""

    def observed_motion_fault(self):
        fault = super().observed_motion_fault()
        if self.state == "PAUSE" and self.epoch_invalid == "sensor_freshness_lost":
            return fault
        if self.phase == "DECLARE_WAIT" and self.leg_stop_path_m is not None:
            if self.total_path_m-self.leg_stop_path_m > .01+1e-9:
                return "route_drive_settle_path_limit"
        return fault or self._stationary_epoch_fault()

    def _current_recognition_condition(self, wall):
        if self.phase not in ("RECOGNIZE_WAIT", "REPORT_WAIT"):
            return ""
        age = None if self.last_uav_image is None else wall-self.last_uav_image[0]
        delta = None if self.last_uav_image is None or self.clock is None else self.clock[0]-self.last_uav_image[1]
        self.last_camera_liveness_check = {"wall_s": wall, "phase": self.phase,
            "camera_max_wall_age_s": CURRENT_CAMERA_MAX_AGE_S,
            "camera_max_clock_age_s": CURRENT_CAMERA_MAX_AGE_S,
            "camera_min_clock_age_s": -self.limits.clock_sampling_lag_s,
            "last_image_arrival_wall_s": None if self.last_uav_image is None else self.last_uav_image[0],
            "last_image_stamp_s": None if self.last_uav_image is None else self.last_uav_image[1],
            "image_wall_age_s": age, "clock_minus_image_stamp_s": delta,
            "interpretation": "current camera liveness only; original capture stamps and priors unchanged"}
        if age is None or age > CURRENT_CAMERA_MAX_AGE_S+1e-9:
            return "single_point_uav_image_stale"
        if delta is None or not -.12-1e-9 <= delta <= CURRENT_CAMERA_MAX_AGE_S+1e-9:
            return "single_point_current_image_clock_inconsistent"
        history = self._observations.uav_odom_history
        if (not history or self._observations.uav_odom_error or not self._observations.uav_odom_epoch_valid
                or wall-self.started_wall-history[-1]["wall_elapsed_s"] > .20+1e-9):
            return "single_point_uav_pose_stale"
        return ""

    def temporal_fault(self, reason, wall):
        if self.phase == "REPORT_WAIT" and reason in self.FRESHNESS:
            self.epoch_invalid = "sensor_freshness_lost_after_report"
            return self.stop("single_point_report_wait_sensor_freshness_lost:"+reason, wall)
        if self.phase in ("CAPTURE", "RECOGNIZE_WAIT") and reason in self.FRESHNESS:
            self.epoch_invalid = "sensor_freshness_lost"
        super().temporal_fault(reason, wall)

    def stop(self, reason, wall_s):
        if (reason == "approach_observation_complete" and self.phase == "CAPTURE"
                and self.state == "CAPTURE" and len(self.capture_frames) == 3 and not self._stationary_epoch_fault()):
            self._enter_phase("RECOGNIZE_WAIT")
            return MotionDecision(0., self.state, self.reason)
        return super().stop(reason, wall_s)

    def observe_public(self, topic, payload, *, wall_s):
        with self._lock:
            super().observe_public(topic, payload, wall_s=wall_s)
            if topic == "/referee/recognition_status":
                data = payload.get("data")
                if isinstance(data, str):
                    encoded = data.encode("utf-8")
                    self.public_status.append({"data": encoded[:4096].decode("utf-8", errors="ignore"), "wall_s": wall_s,
                        "original_utf8_bytes": len(encoded), "data_truncated": len(encoded) > 4096,
                        "original_sha256": hashlib.sha256(encoded).hexdigest(),
                        "clock_stamp_s": None if self.clock is None else self.clock[0],
                        "phase": self.phase, "acceptance_semantics": "unknown_string_not_parsed_as_ack"})
                    self.public_status = self.public_status[-8:]
            elif topic == "/uav/camera/camera_info":
                signature = (payload.get("width"), payload.get("height"), payload.get("header", {}).get("frame_id"),
                             tuple(payload.get("k", [])), tuple(payload.get("d", [])))
                if self.camera_signature is not None and signature != self.camera_signature:
                    self.stop("single_point_camera_calibration_changed", wall_s)
                self.camera_signature = signature
                self.camera_infos.append(copy.deepcopy(payload))
                self.camera_infos = self.camera_infos[-8:]
            elif topic == "/uav/odom" and self.phase in ("CAPTURE", "RECOGNIZE_WAIT", "REPORT_WAIT"):
                observations = self._observations
                if observations.uav_odom_error or not observations.uav_odom_epoch_valid or not observations.uav_odom_history:
                    self.stop("single_point_uav_pose_invalid", wall_s)
                    return
                pose = observations.uav_odom_history[-1]
                if self.stationary_uav_anchor is None:
                    self.stationary_uav_anchor = copy.deepcopy(pose)
                anchor = self.stationary_uav_anchor
                if (math.dist(pose["position"], anchor["position"]) > .01+1e-9
                        or abs(angle_wrap(pose["yaw_rad"]-anchor["yaw_rad"])) > .02+1e-9
                        or pose["linear_speed"] > .01 or pose["angular_speed"] > .03):
                    self.stop("single_point_uav_stationary_epoch_motion", wall_s)

    def observe_image(self, topic, msg, *, wall_s):
        with self._lock:
            if topic == "/uav/camera/image":
                try:
                    sec, nano = msg.header.stamp.sec, msg.header.stamp.nanosec
                    if (type(sec) is int and type(nano) is int and sec >= 0 and 0 <= nano < 10**9
                            and msg.header.frame_id == "uav/camera_link"):
                        stamp = sec+nano/1e9
                        if self.last_uav_image is None or stamp > self.last_uav_image[1]:
                            self.last_uav_image = (wall_s, stamp)
                except (AttributeError, TypeError):
                    pass
            before = len(self.capture_frames)
            super().observe_image(topic, msg, wall_s=wall_s)
            if len(self.capture_frames) > before:
                stamp = self.capture_frames[-1]["record"]["stamp_s"]
                prior = None
                for info in reversed(self.camera_infos):
                    header = info.get("header", {}).get("stamp", {})
                    if type(header.get("sec")) is int and type(header.get("nanosec")) is int:
                        if header["sec"]+header["nanosec"]/1e9 <= stamp+1e-9:
                            prior = info
                            break
                self.capture_frames[-1]["camera_info"] = copy.deepcopy(prior)
                self.capture_frames[-1]["stationary_epoch"] = self.stationary_epoch

    def record_command(self, *, wall_s, linear_x, angular_z):
        if self.phase in ("CAPTURE", "RECOGNIZE_WAIT", "REPORT_WAIT") and (linear_x or angular_z):
            self.epoch_invalid = "nonzero_command_since_exposure"
        super().record_command(wall_s=wall_s, linear_x=linear_x, angular_z=angular_z)

    def register_prepared_jobs(self, jobs, *, epoch):
        with self._lock:
            if (self.state != "RECOGNIZE_WAIT" or epoch != self.stationary_epoch
                    or self.epoch_invalid or len(jobs) != 3 or len(self.capture_frames) != 3):
                return False
            for job, frame in zip(jobs, self.capture_frames):
                record = frame["record"]
                if (job.get("session_id") != self.session_id or job.get("frozen_first_ID") != self.target_id
                        or job.get("stationary_epoch") != epoch or job.get("image_stamp_s") != record["stamp_s"]
                        or job.get("pose") != record["odom"] or job.get("camera_info") != frame["camera_info"]
                        or job.get("frame_id") != record["frame_id"] or job.get("topic") != record["topic"]):
                    return False
            self.expected_jobs = {job["job_seq"]: {k: copy.deepcopy(v) for k, v in job.items() if k != "source_jpeg"}
                                  for job in jobs}
            return len(self.expected_jobs) == 3

    def accept_match_result(self, result, *, wall_s):
        with self._lock:
            if (self.state != "RECOGNIZE_WAIT" or self._condition(wall_s) or self._stationary_epoch_fault()
                    or self._current_recognition_condition(wall_s)):
                return False
            job = result.get("job", {})
            original = {key: value for key, value in job.items() if key != "_context_generation"}
            expected = self.expected_jobs.get(job.get("job_seq"))
            if (result.get("matches_current_context") is not True or expected is None or original != expected
                    or result.get("error") or any(row["job_seq"] == job.get("job_seq") for row in self.match_results)):
                return False
            bound = bind_ground_candidate(result, self.target_id)
            if bound is None:
                self.stop("single_point_visual_binding_rejected", wall_s)
                return False
            if self.match_results and (bound["class_id"] != self.match_results[0]["class_id"]
                    or any(math.dist(bound["world_center_xy_m"], item["world_center_xy_m"]) > .03 for item in self.match_results)):
                self.stop("single_point_multiframe_disagreement", wall_s)
                return False
            self.match_results.append({**bound, "job_seq": job["job_seq"], "image_stamp_s": job["image_stamp_s"],
                "source_sha256": job["source_sha256"], "pose": copy.deepcopy(job["pose"]),
                "job": copy.deepcopy(original),
                "worker": {key: result.get(key) for key in ("worker_pid", "worker_started_utc", "worker_finished_utc",
                    "worker_started_monotonic_s", "worker_finished_monotonic_s", "template_load_ms", "decode_ms", "match_ms")}})
            if len(self.match_results) == 3:
                ordered = sorted(self.match_results, key=lambda value: value["image_stamp_s"])
                if len({item["image_stamp_s"] for item in ordered}) != 3:
                    self.stop("single_point_duplicate_exposure", wall_s)
                    return False
                self.recognition = {"class_id": bound["class_id"], "label": bound["label"],
                    "data": recognition_phrase("ground", bound["label"]), "first_id": self.target_id,
                    "stationary_epoch": self.stationary_epoch, "official_acceptance": False}
            return True

    def record_match_failure(self, result, *, wall_s):
        """Preserve observed failure/source identity without claiming execution."""
        with self._lock:
            job = {key: copy.deepcopy(value) for key, value in result.get("job", {}).items()
                   if key not in ("source_jpeg", "_context_generation")}
            event = {"job": job, "error": str(result.get("error", ""))[:2048], "wall_s": wall_s,
                "matches_current_context": result.get("matches_current_context"),
                "job_was_dispatched": result.get("job_was_dispatched"),
                "job_was_executed": result.get("job_was_executed"),
                "official_acceptance": False}
            if not any(item["job"] == job and item["error"] == event["error"] for item in self.matcher_failures):
                self.matcher_failures.append(event)
                self.matcher_failures = self.matcher_failures[-3:]

    def _inside_candidate_region(self):
        if self.odom is None or self.target_id not in _POINTS:
            return False
        center, size, _ = _POINTS[self.target_id]
        return all(abs(self.odom["xyz"][i]-center[i]) <= size[i]/2-.02+1e-9 for i in (0, 1))

    def dispatch_one(self, publish, *, wall_s):
        """Caller holds command_lock. attempted latches before an uncertain send."""
        with self._lock:
            if (self.state != self.phase or self._budget_fault(wall_s) or self._condition(wall_s)
                    or self.observed_motion_fault() or self._current_recognition_condition(wall_s)):
                return None
            if self.phase == "RECOGNIZE_WAIT" and self.recognition is not None:
                kind, topic, data = "recognition", "/inspection/detections", self.recognition["data"]
            elif (self.phase == "DECLARE_WAIT" and self.in_entry and self._inside_candidate_region()
                    and self.attempts.get("recognition", {}).get("sent") and self._stable_dwell(wall_s)):
                kind, topic, data = "declaration", "/referee/current_target", self.target_id
            else:
                return None
            if kind in self.attempts:
                return None
            event = {"topic": topic, "data": data, "attempted": True, "sent": False, "wall_s": wall_s,
                     "clock_stamp_s": self.clock[0], "odom_stamp_s": self.odom["stamp"],
                     "position": list(self.odom["xyz"]), "first_id": self.target_id,
                     "stationary_epoch": self.stationary_epoch, "official_acceptance": False,
                     "interpretation": "local_transport_attempt_not_referee_ack"}
            self.attempts[kind] = event
            try:
                publish(topic, data)
            except Exception as exc:
                event["error"] = type(exc).__name__+":"+str(exc)[:256]
                self.stop("single_point_"+kind+"_publish_uncertain", wall_s)
                return event
            event["sent"] = True
            if kind == "recognition":
                self._enter_phase("REPORT_WAIT")
            return event

    def _begin_entry(self, wall):
        if not self.single_plan.entry_allowed:
            return self.stop("single_point_entry_not_permitted:"+self.single_plan.entry_rejection_reason, wall)
        position = self.single_plan.observation.waypoints_xy_m[-1]
        segments = []
        for target in self.single_plan.entry_waypoints_xy_m:
            segments.append(GroundRouteSegment(position, target, math.atan2(target[1]-position[1], target[0]-position[0]), math.dist(position, target)))
            position = target
        self.route_plan = replace(self.route_plan, waypoints_xy_m=self.single_plan.entry_waypoints_xy_m,
                                  segments=tuple(segments))
        self.in_entry, self.leg_index = True, 0
        self._start_turn(wall)
        return MotionDecision(0., self.state, self.reason)

    def step(self, *, wall_s):
        with self._lock:
            recovering_capture = self.state == "PAUSE" and self.epoch_invalid == "sensor_freshness_lost"
            if self.state in ("STOP", "FINISHED") or (self.phase not in self.EXTRA_ZERO and not recovering_capture):
                return super().step(wall_s=wall_s)
            if not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall):
                return self.stop("wall_clock_reversed_or_invalid", wall_s)
            self.last_wall = wall_s
            fault = self._budget_fault(wall_s)
            if fault:
                return self.stop(fault, wall_s)
            if self.state == "PAUSE" and self.epoch_invalid == "sensor_freshness_lost":
                fault = super().observed_motion_fault() or self._condition(wall_s)
                if fault or not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, fault or "pause_stability_dwell")
                if self.recapture_count >= 1 or self.resume_count >= 3:
                    return self.stop("single_point_recapture_limit", wall_s)
                self.recapture_count += 1
                self.resume_count += 1
                self.state = "CAPTURE"
                self._enter_phase("CAPTURE")
                return MotionDecision(0., self.state, self.reason)
            fault = self.observed_motion_fault() or self._condition(wall_s) or self._current_recognition_condition(wall_s)
            if fault:
                self.temporal_fault(fault, wall_s)
                return MotionDecision(0., self.state, self.reason)
            if self.state == "PAUSE":
                if not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, "pause_stability_dwell")
                if self.resume_count >= 3:
                    return self.stop("pause_resume_limit", wall_s)
                self.resume_count += 1
                self.state = self.phase
            if self.phase == "REPORT_WAIT" and wall_s-self.zero_started_wall >= 1.-1e-9:
                return self._begin_entry(wall_s)
            if self.phase == "DECLARE_WAIT":
                if not self._inside_candidate_region():
                    return self.stop("single_point_entry_region_not_reached", wall_s)
                event = self.attempts.get("declaration")
                if event and event.get("sent") and wall_s-event["wall_s"] >= 2.-1e-9:
                    return self.stop("single_point_transport_complete_official_score_unverified", wall_s)
                if wall_s-self.zero_started_wall >= 5.-1e-9:
                    return self.stop("single_point_declaration_wait_timeout", wall_s)
            return MotionDecision(0., self.state, self.reason)

    def approach_evidence(self):
        with self._lock:
            evidence = super().approach_evidence()
            evidence["revision"] = "ground_single_point_v1"
            evidence["route"]["observation_only"] = False
            evidence["single_point"] = {"session_id": self.session_id,
                "plan": None if self.single_plan is None else asdict(self.single_plan), "in_entry": self.in_entry,
                "stationary_epoch": self.stationary_epoch, "epoch_invalid": self.epoch_invalid,
                "recapture_count": self.recapture_count, "recognition": copy.deepcopy(self.recognition),
                "match_results": copy.deepcopy(self.match_results), "attempts": copy.deepcopy(self.attempts),
                "matcher_failures": copy.deepcopy(self.matcher_failures),
                "camera_liveness": copy.deepcopy(self.last_camera_liveness_check),
                "recognition_status": copy.deepcopy(self.public_status), "official_acceptance": False,
                "body_fully_inside": "not_claimed_center_entry_only", "full_mission_complete": False}
            return evidence

    def additional_events(self):
        events = super().additional_events()
        # The complete single-point evidence has its own bounded envelope.
        # Do not duplicate it inside legacy metadata and overflow that event.
        for _, payload in events:
            if payload.get("section") == "survey_metadata":
                approach = payload.get("survey", {}).get("approach", {})
                if isinstance(approach, dict):
                    approach.pop("single_point", None)
                    approach["single_point_transport"] = {"section": "single_point_core",
                        "session_id": self.session_id, "interpretation": "dedicated_event_no_duplicate_payload"}
        evidence = self.approach_evidence()["single_point"]
        return [("probe_snapshot", {"mode": self.mode, "revision": "ground_single_point_v1",
                 "read_only": False, "official_scoring": "unverified", "section": "single_point_core",
                 "partial": True, "state": self.state, "reason": self.reason, **evidence})]+events


def load_ground_templates(asset_dir=None):
    """Resolve only package-relative explicit assets; verify the frozen public eight."""
    from .ground_plane_matcher import TemplateSpec
    root = Path(asset_dir) if asset_dir is not None else Path(__file__).resolve().parent/"assets"/"ground_templates"
    root = root.resolve()
    manifest = json.loads((root/"manifest.json").read_text(encoding="utf-8"))
    entries = manifest.get("templates", [])
    if (manifest.get("repository") != TEMPLATE_REPOSITORY or manifest.get("commit") != TEMPLATE_COMMIT
            or {row.get("file") for row in entries} != set(TEMPLATE_HASHES) or len(entries) != 8):
        raise ValueError("explicit frozen eight-template manifest required")
    specs = []
    for row in entries:
        filename = row["file"]
        path = (root/filename).resolve()
        category = filename.rsplit("_", 1)[0]
        if (path.parent != root or category not in GROUND_LABELS or row.get("class_id") != category
                or row.get("sha256") != TEMPLATE_HASHES[filename]
                or hashlib.sha256(path.read_bytes()).hexdigest() != TEMPLATE_HASHES[filename]):
            raise ValueError("template source/class/hash mismatch: "+filename)
        specs.append(TemplateSpec(path, category, GROUND_LABELS[category],
            {"repository": TEMPLATE_REPOSITORY, "commit": TEMPLATE_COMMIT,
             "repository_path": "src/ican_inspection_examples/ican_inspection_examples/templates/"+filename,
             "source_url": row.get("source_url"), "license_status": "public_repository_provenance_preserved"},
            TEMPLATE_HASHES[filename]))
    return specs


def prepare_capture_jobs(frames, *, session_id, first_id, epoch, first_seq):
    """Encode off the ROS/control thread, retaining original exposure/calibration."""
    import cv2
    import numpy as np
    jobs = []
    for index, frame in enumerate(frames):
        msg, record, info = frame["msg"], frame["record"], frame.get("camera_info")
        if info is None or (msg.width, msg.height) != (800, 800) or frame["stationary_epoch"] != epoch:
            raise ValueError("original 800x800 capture, epoch and prior CameraInfo required")
        channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(msg.encoding)
        if channels is None:
            raise ValueError("unsupported original encoding")
        raw = memoryview(msg.data).cast("B")
        if len(raw) != msg.step*msg.height or msg.step < msg.width*channels:
            raise ValueError("invalid original image stride")
        pixels = np.frombuffer(raw, np.uint8).reshape(msg.height, msg.step)[:, :msg.width*channels]
        pixels = pixels.reshape(msg.height, msg.width, channels) if channels == 3 else pixels.reshape(msg.height, msg.width)
        if msg.encoding == "rgb8":
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "mono8":
            pixels = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
        ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            raise ValueError("capture JPEG encoding failed")
        jpeg = encoded.tobytes()
        flat = list(info["k"])
        jobs.append({"job_seq": first_seq+index, "session_id": session_id, "frozen_first_ID": first_id,
            "stationary_epoch": epoch, "source_jpeg": jpeg, "source_sha256": hashlib.sha256(jpeg).hexdigest(),
            "image_stamp_s": record["stamp_s"], "frame_id": record["frame_id"], "topic": record["topic"],
            "pose": copy.deepcopy(record["odom"]), "K": [flat[i:i+3] for i in (0, 3, 6)],
            "camera_info": copy.deepcopy(info), "source_ros_encoding": msg.encoding,
            "source_ros_stride": msg.step, "source_ros_sha256": hashlib.sha256(raw).hexdigest(),
            "source_jpeg_derivation": "original ROS pixels, JPEG quality 75, no resize"})
    return jobs


class GroundSinglePointCoordinator:
    """Main-loop owner of a worker; image preparation has its own daemon thread."""
    def __init__(self, guard, worker, *, prepare=prepare_capture_jobs):
        self.guard, self.worker, self.prepare = guard, worker, prepare
        self.context = None
        self.prepared = queue.Queue(maxsize=1)
        self.next_seq = 0
        self.jobs = []
        self.next_job = 0
        self.awaiting_result = False
        self.preparer = None

    def poll_once(self, wall):
        guard = self.guard
        with guard._lock:
            context = ((guard.session_id, guard.target_id, guard.stationary_epoch)
                       if guard.state == "RECOGNIZE_WAIT" and not guard.epoch_invalid else None)
            if context != self.context:
                self.worker.cancel_context()
                self.context, self.jobs, self.next_job, self.awaiting_result = context, [], 0, False
                if context is not None:
                    self.worker.set_context(*context)
                    # Retain immutable ROS image references; copy only small metadata.
                    frames = [{**frame, "record": copy.deepcopy(frame["record"]),
                               "camera_info": copy.deepcopy(frame.get("camera_info"))} for frame in guard.capture_frames]
                    first_seq = self.next_seq
                    self.next_seq += 3
                    def prepare():
                        try:
                            value = self.prepare(frames, session_id=context[0], first_id=context[1], epoch=context[2], first_seq=first_seq)
                            item = (context, value, None)
                        except Exception as exc:
                            item = (context, None, type(exc).__name__+":"+str(exc)[:256])
                        try:
                            self.prepared.put_nowait(item)
                        except queue.Full:
                            pass
                    self.preparer = threading.Thread(target=prepare, name="ground-capture-jpeg-preparer", daemon=True)
                    self.preparer.start()
        try:
            prepared_context, jobs, error = self.prepared.get_nowait()
        except queue.Empty:
            jobs = error = None
            prepared_context = None
        if prepared_context is not None and prepared_context == self.context:
            if error:
                guard.stop("single_point_capture_preparation_failed:"+error, wall)
            elif guard.register_prepared_jobs(jobs, epoch=prepared_context[2]):
                self.jobs = jobs
        for result in self.worker.poll():
            if result.get("matches_current_context") is True:
                self.awaiting_result = False
                if result.get("error"):
                    guard.record_match_failure(result, wall_s=wall)
                    guard.stop("single_point_matcher_failed:"+str(result["error"])[:256], wall)
                else:
                    guard.accept_match_result(result, wall_s=wall)
        if (self.context is not None and self.jobs and self.next_job < len(self.jobs) and not self.awaiting_result
                and guard.state == "RECOGNIZE_WAIT" and guard.stationary_epoch == self.context[2]):
            self.worker.submit(self.jobs[self.next_job])
            self.next_job += 1
            self.awaiting_result = True

    def close(self):
        self.context = None
        return self.worker.close(timeout_s=.5)


def create_single_point_node(reporter=None, *, wall_clock=time.monotonic, route_limits=None, worker=None):
    def factory(limits, *, started_wall_s):
        return GroundSinglePointGuard(started_wall_s=started_wall_s, route_limits=route_limits)
    node = create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5, callback_observer=RouteCallbackTimingRecorder(),
        input_reliability_overrides={"/odom": "BEST_EFFORT", "/uav/odom": "BEST_EFFORT"})
    from rosidl_runtime_py.utilities import get_message
    String = get_message("std_msgs/msg/String")
    original_refresh, original_tick = node.refresh_endpoints, node.tick
    publisher_counts = {topic: {"attempted": 0, "returned": 0}
                        for topic in ("/inspection/detections", "/referee/current_target")}
    def refresh():
        original_refresh()
        for topic in ("/inspection/detections", "/referee/current_target"):
            item = node.command_endpoints.setdefault(topic, {})
            counts = publisher_counts[topic]
            item.update(silent=False, publication_policy="one_attempt_after_current_visual_and_physical_gate",
                        actual_publish_attempt_count=counts["attempted"],
                        actual_publish_return_count=counts["returned"],
                        interpretation="enabled_policy_not_evidence_of_referee_acceptance")
        node.record_endpoint_history()
    def send(topic, data):
        publisher = node.publisher_by_topic.get(topic)
        if publisher is None or publisher.get_subscription_count() <= 0:
            raise RuntimeError("no matching official String subscriber: "+topic)
        message = String()
        message.data = data
        publisher_counts[topic]["attempted"] += 1
        publisher.publish(message)
        publisher_counts[topic]["returned"] += 1
    coordinator = None if worker is None else GroundSinglePointCoordinator(node.guard, worker)
    def tick():
        decision = original_tick()
        try:
            if coordinator is not None:
                coordinator.poll_once(wall_clock())
            with node.command_lock:
                node.guard.dispatch_one(send, wall_s=wall_clock())
                for topic, counts in publisher_counts.items():
                    node.command_endpoints[topic].update(actual_publish_attempt_count=counts["attempted"],
                        actual_publish_return_count=counts["returned"])
        except Exception as exc:
            node.stop("single_point_coordinator_failed:"+type(exc).__name__+":"+str(exc)[:256])
        return decision
    node.refresh_endpoints, node.tick = refresh, tick
    node.single_point_coordinator = coordinator
    refresh()
    return node


def main(args=None):
    import rclpy
    from .ground_matcher_worker import GroundMatcherWorker
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="One current-image ground point; no full mission completion")
    parser.add_argument("--duration", type=float, default=300.)
    parser.add_argument("--template-dir", default=None)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = RouteLimits(global_wall_s=parsed.duration)
    templates = load_ground_templates(parsed.template_dir)
    worker = GroundMatcherWorker(templates)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_single_point_node(reporter, route_limits=limits, worker=worker)
        node.report_started(parsed.duration)
        node.arm_watchdog()
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("single_point_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("single_point_cleanup")
            node.close_watchdog()
            node.destroy_node()
        worker.close(timeout_s=.5)
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
