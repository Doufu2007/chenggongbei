"""Bounded three-ground-point practice, with one permanent measured ledger.

Reuses v25 sensor, image/source and send validation. Candidate geometry only
permits a plan; fresh live scans retain final motion authority. Transport is
never interpreted as an official score or full aerial mission completion.
"""
from __future__ import annotations

import argparse
import base64
import copy
from dataclasses import asdict, dataclass, replace
import gzip
import hashlib
import json
import math
import os
import time
from types import SimpleNamespace

from .bounded_ground_probe import MotionDecision, MotionLimits, create_bounded_node, finite, spin_control_loop
from .ground_bounded_approach import GroundBoundedApproachGuard
from .ground_observation_route import RouteLimits, RouteCallbackTimingRecorder
from .ground_observation_survey import angle_wrap, yaw_from_q
from .ground_single_point import GroundSinglePointGuard, GroundSinglePointCoordinator, load_ground_templates


@dataclass(frozen=True)
class ThreePointLimits(RouteLimits):
    """New pre-run contract; not a mutation of the frozen single-point cap.

    Time extrapolates v25's 36.0058 wall s/m and 22.9369 wall s/rad, with
    reserve. The platform's unknown external timeout remains independent.
    """
    global_wall_s: float = 1200.
    max_path_m: float = 12.5
    max_origin_displacement_m: float = 3.
    max_rotation_rad: float = 25.
    turn_total_wall_s: float = 700.
    drive_wall_s: float = 500.

    def __post_init__(self):
        caps = {"target_distance_m": 3., "global_wall_s": 1200., "max_path_m": 12.5,
            "max_origin_displacement_m": 3., "max_rotation_rad": 25.,
            "local_swept_motion_budget_m": .06, "scan_anchor_motion_limit_m": .03,
            "local_stop_reserve_m": .03, "linear_command_mps": .10,
            "minimum_yaw_rate": .35, "maximum_yaw_rate": .5, "max_heading_error_rad": .05,
            "max_reverse_m": .005, "max_cross_track_m": .02, "max_turn_drift_m": .06,
            "wait_wall_s": 5., "turn_wall_s": 45., "turn_total_wall_s": 700.,
            "drive_wall_s": 500., "capture_wall_s": 10.}
        for key, cap in caps.items():
            if not finite(getattr(self, key)) or not 0 < getattr(self, key) <= cap:
                raise ValueError(key+" exceeds three-ground-point limit")
        if (self.minimum_yaw_rate > self.maximum_yaw_rate
                or self.scan_anchor_motion_limit_m+self.local_stop_reserve_m > self.local_swept_motion_budget_m+1e-9):
            raise ValueError("inconsistent local reserve or angular bounds")


@dataclass(frozen=True)
class _RouteView:
    first_id: str
    initial_pose: tuple
    target_xy_m: tuple
    waypoints_xy_m: tuple
    final_yaw_rad: float
    budgets: ThreePointLimits
    segments: tuple = ()


class GroundThreePointGuard(GroundSinglePointGuard):
    mode = "ground_three_point"
    wall_limit_reason = "three_ground_global_wall_time_limit"
    final_report_wall_s = 180.
    EXTRA_ZERO = GroundSinglePointGuard.EXTRA_ZERO | {"POINT_HANDOFF"}
    ACTIVE = GroundSinglePointGuard.ACTIVE | {"POINT_HANDOFF"}
    ZERO_PHASES = GroundSinglePointGuard.ZERO_PHASES | {"POINT_HANDOFF"}

    def __init__(self, *, started_wall_s, route_limits=None, session_id=None):
        limits = route_limits or ThreePointLimits()
        if not isinstance(limits, ThreePointLimits):
            raise ValueError("ThreePointLimits required for the new mission contract")
        super().__init__(started_wall_s=started_wall_s, route_limits=limits, session_id=session_id)
        self.frozen_limits = limits
        self.mission_draw = None
        self.point_index = 0
        self.point_plan = None
        self.operations = ()
        self.operation_index = 0
        self.operation_records = []
        self.completed_points = []
        self.archived_capture_frames = []
        self.point_visual = None
        self.current_operations_role = None
        self._observations.TRAJECTORY_CAP = 12500
        self._observations.COMMAND_CAP = 30000

    def _turn_tolerance(self):
        if self.operation_index+1 < len(self.operations):
            following = self.operations[self.operation_index+1]
            if following.kind == "DRIVE_TO_XY" and self.odom is not None:
                distance = math.dist(self.odom["xyz"][:2], following.target_xy_m)
                # The g2 final diagonal has only 9mm nominal ring reserve.
                # A 10mm heading-only lateral allowance can consume it even
                # with zero external disturbance. Tighten this aim to 3mm;
                # retain every original physical, freshness and stop limit.
                lateral_aim = (.003 if self.target_id == "ground_2" and self.current_operations_role == "entry"
                    and not any(op.kind == "DRIVE_TO_XY" for op in self.operations[self.operation_index+2:]) else .01)
                return min(.025, math.atan2(lateral_aim, max(distance, .01)))
        return .025

    def observe_public(self, topic, payload, *, wall_s):
        with self._lock:
            super().observe_public(topic, payload, wall_s=wall_s)
            if topic == "/referee/ground_tasks" and self.draw is not None and self.mission_draw is None:
                self.mission_draw = tuple(self.draw)

    def _current_pose(self):
        return (*self.odom["xyz"][:2], yaw_from_q(self.odom["q"]))

    def _remaining_plan_fault(self, path, rotation):
        # Reserve remains part of the same permanent frozen ledger.
        if self.total_path_m+path+.10 > self.frozen_limits.max_path_m+1e-9:
            return "three_ground_remaining_path_budget_insufficient"
        if self.accumulated_rotation_rad+rotation+.10 > self.frozen_limits.max_rotation_rad+1e-9:
            return "three_ground_remaining_rotation_budget_insufficient"
        return ""

    def _plan_point(self, previous, target, wall):
        from .ground_three_point_plan import plan_ground_leg
        try:
            plan = plan_ground_leg(previous, target, self._current_pose())
        except ValueError as exc:
            return self.stop("three_ground_candidate_plan_rejected:"+str(exc)[:256], wall)
        fault = self._remaining_plan_fault(plan.path_m, plan.rotation_rad)
        if fault:
            return self.stop(fault, wall)
        self.point_plan = plan
        self.target_id, self.target_xy = target, plan.endpoint_xy_m
        self.route_plan = _RouteView(target, plan.initial_pose, plan.endpoint_xy_m,
            (plan.observation_xy_m,), plan.observation_yaw_rad, self.frozen_limits)
        self._load_operations(plan.to_observation_operations, "observation", wall)
        return MotionDecision(0., self.state, self.reason)

    def _invalidate_visual_context(self):
        # The coordinator sees context=None before it can submit more work.
        self.stationary_epoch += 1
        self.stationary_anchor = self.stationary_uav_anchor = None
        self.epoch_invalid = "motion_after_stationary_epoch"
        self.capture_frames.clear()
        self.expected_jobs.clear()
        self.match_results.clear()
        self.recognition = None

    def _load_operations(self, operations, role, wall):
        self.operations, self.operation_index = tuple(operations), 0
        self.current_operations_role = role
        self._invalidate_visual_context()
        self._start_operation(wall)

    def _start_operation(self, wall):
        if self.state in ("STOP", "FINISHED"):
            return
        self.drive_origin = self.drive_yaw = None
        if self.operation_index == len(self.operations):
            if self.current_operations_role == "observation":
                if math.dist(self.odom["xyz"][:2], self.point_plan.observation_xy_m) > .02+1e-9:
                    return self.stop("route_waypoint_error_limit", wall)
                self.target_yaw = self.point_plan.observation_yaw_rad
                self._record_scan("final_heading", wall)
                self._enter_phase("CAPTURE")
            else:
                if (math.dist(self.odom["xyz"][:2], self.point_plan.endpoint_xy_m) > .02+1e-9
                        or abs(angle_wrap(self._current_pose()[2]-self.point_plan.endpoint_yaw_rad)) > .025+1e-9):
                    return self.stop("three_ground_final_pose_outside_contract", wall)
                self.target_yaw = self.point_plan.endpoint_yaw_rad
                self._enter_phase("DECLARE_WAIT")
            return
        op = self.operations[self.operation_index]
        self.operation_records.append({"point_index": self.point_index, "target_id": self.target_id,
            "role": self.current_operations_role, "operation_index": self.operation_index,
            "operation": asdict(op), "actual_start_pose": self._current_pose(), "started_wall_s": wall,
            "path_origin_m": self.total_path_m, "rotation_origin_rad": self.accumulated_rotation_rad})
        if op.kind == "TURN_ONLY":
            if (math.dist(self.odom["xyz"][:2], op.anchor_xy_m) > .02+1e-9
                    or abs(angle_wrap(self._current_pose()[2]-op.expected_start_yaw_rad)) > .05+1e-9
                    or not 0 < abs(op.signed_subturn_rad) <= math.pi/2+1e-9):
                return self.stop("three_ground_turn_anchor_outside_contract", wall)
            target_yaw = op.target_yaw_rad
            if self.operation_index+1 < len(self.operations) and self.operations[self.operation_index+1].kind == "DRIVE_TO_XY":
                destination = self.operations[self.operation_index+1].target_xy_m
                target_yaw = math.atan2(destination[1]-self.odom["xyz"][1], destination[0]-self.odom["xyz"][0])
            error = angle_wrap(target_yaw-self._current_pose()[2])
            if abs(error) > self._turn_tolerance() and error*op.signed_subturn_rad <= 0:
                return self.stop("three_ground_turn_direction_mismatch", wall)
            from .ground_three_point_plan import _turn_clearance, RING_THRESHOLD_M
            if _turn_clearance(self.odom["xyz"][:2], self._current_pose()[2], error) < RING_THRESHOLD_M-1e-12:
                return self.stop("three_ground_actual_turn_candidate_unsafe", wall)
            self.target_yaw = target_yaw
            self.operation_records[-1]["runtime_target_yaw_rad"] = target_yaw
            self.initial_turn_error = error
            self.turn_origin = self.odom["xyz"]
            self.turn_started_wall = wall
            self._enter_phase("TURN")
        elif op.kind == "DRIVE_TO_XY":
            self.leg_target_xy = tuple(op.target_xy_m)
            self.target_yaw = math.atan2(self.leg_target_xy[1]-self.odom["xyz"][1],
                                         self.leg_target_xy[0]-self.odom["xyz"][0])
            distance = math.dist(self.odom["xyz"][:2], self.leg_target_xy)
            if distance <= .002:
                self.operation_records[-1]["within_2mm_no_motion"] = True
                self.operation_index += 1
                return self._start_operation(wall)
            if (distance <= .02 or abs(angle_wrap(self.target_yaw-self._current_pose()[2]))
                    > min(.025, math.atan2(.01, distance))+1e-9):
                return self.stop("three_ground_drive_heading_outside_contract", wall)
            self.drive_origin, self.drive_yaw = self.odom["xyz"], self.target_yaw
            self.leg_target_distance_m = distance
            self.approach_limits = replace(self.approach_limits, target_distance_m=distance-.002)
            self.leg_stop_path_m = None
            self.drive_started_wall = wall
            self.leg_records.append({"point_index": self.point_index, "leg_index": len(self.leg_records),
                "origin": self.drive_origin, "yaw_rad": self.drive_yaw,
                "waypoint_xy_m": self.leg_target_xy, "target_projection_m": distance, "started_wall_s": wall})
            self._enter_phase("DRIVE")
        else:
            self.stop("three_ground_unknown_operation", wall)

    def _finish_operation(self, wall):
        self.operation_records[-1].update(ended_wall_s=wall, actual_end_pose=self._current_pose(),
            path_end_m=self.total_path_m, rotation_end_rad=self.accumulated_rotation_rad)
        self.operation_index += 1
        self._start_operation(wall)

    def _begin_entry(self, wall):
        from .ground_three_point_plan import plan_ground_entry
        if (self.state != "REPORT_WAIT" or self.phase != "REPORT_WAIT"
                or not self.attempts.get("recognition", {}).get("sent") or self.recognition is None
                or self.recognition.get("first_id") != self.target_id
                or self.recognition.get("stationary_epoch") != self.stationary_epoch
                or len(self.match_results) != 3 or self._stationary_epoch_fault()
                or self._current_recognition_condition(wall) or self._condition(wall)):
            return self.stop("three_ground_entry_visual_gate_missing", wall)
        try:
            operations = plan_ground_entry(self.target_id, self._current_pose())
        except ValueError as exc:
            return self.stop("three_ground_candidate_entry_rejected:"+str(exc)[:256], wall)
        fault = self._remaining_plan_fault(sum(op.length_m for op in operations),
                                           sum(abs(op.signed_subturn_rad) for op in operations))
        if fault:
            return self.stop(fault, wall)
        self.point_visual = {"stationary_epoch": self.stationary_epoch,
            "recognition": copy.deepcopy(self.recognition), "match_results": copy.deepcopy(self.match_results),
            "capture_frames": list(self.capture_frames)}
        self.in_entry = True
        self._load_operations(operations, "entry", wall)
        return MotionDecision(0., self.state, self.reason)

    def _archive_point(self):
        if any(point["point_index"] == self.point_index for point in self.completed_points):
            return
        visual = self.point_visual or {}
        self.completed_points.append({"point_index": self.point_index, "target_id": self.target_id,
            "stationary_epoch": visual.get("stationary_epoch"), "plan": asdict(self.point_plan),
            "recognition": copy.deepcopy(visual.get("recognition")), "match_results": copy.deepcopy(visual.get("match_results", [])),
            "attempts": copy.deepcopy(self.attempts), "recognition_status": copy.deepcopy(self.public_status),
            "pose": self._current_pose(), "path_m": self.total_path_m,
            "rotation_rad": self.accumulated_rotation_rad, "official_acceptance": False})
        for frame in visual.get("capture_frames", []):
            self.archived_capture_frames.append({**frame, "phase": f"point_{self.point_index}_"+frame["phase"],
                "record": copy.deepcopy(frame["record"]), "point_index": self.point_index, "target_id": self.target_id})

    def _handoff(self, wall):
        # This point's transport fact survives refusal of the next plan.
        self._archive_point()
        if self.point_index == 2:
            return self.stop("three_ground_transport_complete_official_score_unverified", wall)
        from .ground_three_point_plan import plan_ground_leg
        target = self.mission_draw[self.point_index+1]
        try:
            next_plan = plan_ground_leg(self.target_id, target, self._current_pose())
        except ValueError as exc:
            return self.stop("three_ground_candidate_plan_rejected:"+str(exc)[:256], wall)
        fault = self._remaining_plan_fault(next_plan.path_m, next_plan.rotation_rad)
        if fault:
            return self.stop(fault, wall)
        previous = self.target_id
        self._enter_phase("POINT_HANDOFF")
        self.point_index += 1
        self.attempts = {}
        self.matcher_failures = []
        self.public_status = []
        self.recapture_count = 0
        self.in_entry = False
        self.point_visual = None
        return self._plan_point(previous, target, wall)

    def observed_motion_fault(self):
        fault = super().observed_motion_fault()
        if fault:
            return fault
        if self.phase == "DECLARE_WAIT" and self.odom is not None and self.point_plan is not None:
            if (math.dist(self.odom["xyz"][:2], self.point_plan.endpoint_xy_m) > .02+1e-9
                    or abs(angle_wrap(self._current_pose()[2]-self.point_plan.endpoint_yaw_rad)) > .025+1e-9):
                return "three_ground_final_pose_outside_contract"
        return ""

    def _linear_request(self, wall):
        # The first speed/stop calibration remains exactly the inherited one.
        if not self.initial_slow_validated:
            if self.initial_slow_anchor is None:
                self.initial_slow_anchor = (wall, self.odom["stamp"])
            if min(wall-self.initial_slow_anchor[0], self.odom["stamp"]-self.initial_slow_anchor[1]) < 1.-1e-9:
                return min(.03, self.approach_limits.linear_command_mps)
            self.initial_slow_validated = True
            self.speed_validation.update(slow_validated_wall_s=wall,
                slow_validated_odom_stamp_s=self.odom["stamp"],
                slow_continuous_wall_s=wall-self.initial_slow_anchor[0],
                slow_continuous_sim_s=self.odom["stamp"]-self.initial_slow_anchor[1])
        final_drive = not any(op.kind == "DRIVE_TO_XY" for op in self.operations[self.operation_index+1:])
        return min(.03 if final_drive and self.leg_target_distance_m-self.measured_forward() <= .10+1e-9 else .10,
                   self.approach_limits.linear_command_mps)

    def step(self, *, wall_s):
        with self._lock:
            # Keep all v25 recognition/recapture rules. Intercept only the
            # normal declaration transport wait, before it can enter STOP.
            if self.phase == "DECLARE_WAIT" and self.state not in ("STOP", "FINISHED"):
                if not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall):
                    return self.stop("wall_clock_reversed_or_invalid", wall_s)
                self.last_wall = wall_s
                fault = self._budget_fault(wall_s) or self.observed_motion_fault() or self._condition(wall_s)
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
                event = self.attempts.get("declaration")
                if event and event.get("sent") and wall_s-event["wall_s"] >= 2.-1e-9:
                    return self._handoff(wall_s)
                return super().step(wall_s=wall_s)
            if self.phase in ("CAPTURE", "RECOGNIZE_WAIT", "REPORT_WAIT"):
                return super().step(wall_s=wall_s)
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
            if self.state == "WAIT":
                if wall_s-self.started_wall >= self.approach_limits.wait_wall_s:
                    return self.stop("wait_timeout:"+self.reason, wall_s)
                reason = self._condition(wall_s)
                if reason or self.mission_draw is None:
                    self.stable_anchor = None
                    self.reason = reason or "waiting_for_public_draw"
                    return MotionDecision(0., self.state, self.reason)
                if not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, "start_stability_dwell")
                pose = self._current_pose()
                if math.hypot(*pose[:2]) > .05 or abs(pose[2]) > .05:
                    return self.stop("three_ground_initial_pose_outside_contract", wall_s)
                self.global_origin = self.pulse_origin = self.previous_path_xyz = self.odom["xyz"]
                self.previous_yaw, self.first_motion_wall = pose[2], wall_s
                self._record_scan("start", wall_s)
                return self._plan_point(None, self.mission_draw[0], wall_s)
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
                self.state, self.reason = self.phase, "three_ground_"+self.phase.lower()
                self.stable_anchor = None
            if self.phase == "TURN":
                error = angle_wrap(self.target_yaw-self._current_pose()[2])
                if abs(error) <= self._turn_tolerance():
                    self._enter_phase("TURN_SETTLE")
                else:
                    rate = math.copysign(min(self.approach_limits.maximum_yaw_rate,
                        max(self.approach_limits.minimum_yaw_rate, 1.5*abs(error))), error)
                    return MotionDecision(0., self.state, self.reason, rate)
            if self.phase in ("TURN_SETTLE", "DRIVE_SETTLE", "CALIBRATION_SETTLE"):
                if not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, self.reason)
                if self.phase == "CALIBRATION_SETTLE":
                    anchor = self.stable_anchor
                    self.speed_validation.update(status="verified",
                        stop_path_increment_m=self.total_path_m-self.calibration_stop_path_m,
                        stationary_wall_s=wall_s-anchor[0], stationary_sim_s=self.odom["stamp"]-anchor[1],
                        verified_wall_s=wall_s, verified_odom_stamp_s=self.odom["stamp"])
                    self.calibration_verified = True
                    self._enter_phase("DRIVE")
                else:
                    if self.phase == "TURN_SETTLE" and self.operation_index+1 < len(self.operations):
                        following = self.operations[self.operation_index+1]
                        if following.kind == "DRIVE_TO_XY":
                            destination = following.target_xy_m
                            desired = math.atan2(destination[1]-self.odom["xyz"][1], destination[0]-self.odom["xyz"][0])
                            from .ground_three_point_plan import _turn_clearance, RING_THRESHOLD_M
                            delta = angle_wrap(desired-self._current_pose()[2])
                            if _turn_clearance(self.odom["xyz"][:2], self._current_pose()[2], delta) < RING_THRESHOLD_M-1e-12:
                                return self.stop("three_ground_actual_turn_candidate_unsafe", wall_s)
                            self.target_yaw = desired
                    if self.phase == "TURN_SETTLE" and abs(angle_wrap(self.target_yaw-self._current_pose()[2])) > self._turn_tolerance():
                        self.initial_turn_error = angle_wrap(self.target_yaw-self._current_pose()[2])
                        self.settle_reaim_count += 1
                        self._enter_phase("TURN")
                        return MotionDecision(0., self.state, self.reason)
                    if self.phase == "DRIVE_SETTLE":
                        gap = math.dist(self.odom["xyz"][:2], self.leg_target_xy)
                        self.leg_records[-1].update(ended_wall_s=wall_s, final_position=self.odom["xyz"], waypoint_error_m=gap)
                        if gap > .02+1e-9:
                            return self.stop("route_waypoint_error_limit", wall_s)
                    self._finish_operation(wall_s)
                    return MotionDecision(0., self.state, self.reason)
            if self.phase == "DRIVE":
                return MotionDecision(self._linear_request(wall_s), self.state, self.reason)
            return MotionDecision(0., self.state, self.reason)

    def approach_evidence(self):
        with self._lock:
            return self._approach_evidence_locked()

    def _approach_evidence_locked(self):
        evidence = super().approach_evidence()
        evidence["revision"] = "ground_three_point_v1"
        evidence["three_point"] = {"session_id": self.session_id, "mission_draw": self.mission_draw,
            "point_index": self.point_index, "target_id": self.target_id,
            "operations_role": self.current_operations_role, "operation_index": self.operation_index,
            "plan": None if self.point_plan is None else asdict(self.point_plan),
            "operation_records": copy.deepcopy(self.operation_records),
            "completed_points": copy.deepcopy(self.completed_points),
            "frozen_limits": asdict(self.frozen_limits), "official_acceptance": False,
            "full_mission_complete": False, "interpretation": "ground_transport_only_no_aerial_completion"}
        return evidence

    def additional_events(self):
        self._sync_observation_context()
        current = self.approach_evidence()["single_point"]
        if self.point_visual:
            current["visual_epoch"] = {key: copy.deepcopy(value) for key, value in self.point_visual.items()
                                        if key != "capture_frames"}
        common = {"mode": self.mode, "revision": "ground_three_point_v1", "read_only": False,
            "official_scoring": "unverified", "partial": True, "session_id": self.session_id}
        core = {**common, "section": "three_point_core", "state": self.state, "reason": self.reason,
            "mission_draw": self.mission_draw, "point_index": self.point_index, "target_id": self.target_id,
            "global_origin": self.global_origin, "total_path_m": self.total_path_m,
            "accumulated_rotation_rad": self.accumulated_rotation_rad, "completed_count": len(self.completed_points),
            "full_mission_complete": False, "frozen_limits": asdict(self.frozen_limits)}
        events = [("probe_snapshot", core)]
        for point in self.completed_points:
            # Plans and verbose matcher geometry live in the compressed bundle.
            compact = {key: copy.deepcopy(point[key]) for key in ("point_index", "target_id", "stationary_epoch",
                "recognition", "attempts", "recognition_status", "pose", "path_m", "rotation_rad", "official_acceptance")}
            events.append(("probe_snapshot", {**common, "section": "three_point_point", **compact}))
        frames = list(self.archived_capture_frames)
        # On the final normal handoff current captures are already archived.
        archived_stamps = {frame["record"]["stamp_s"] for frame in frames}
        active_frames = list(self.capture_frames)
        if self.point_visual:
            active_frames += self.point_visual.get("capture_frames", [])
        frames += [{**frame, "phase": f"point_{self.point_index}_"+frame["phase"]}
                   for frame in active_frames if frame["record"]["stamp_s"] not in archived_stamps]
        view = SimpleNamespace(capture_frames=frames, mode=self.mode, _observations=self._observations)
        capture_events = GroundBoundedApproachGuard._capture_image_events(view) if frames else []
        bundle = {"schema": "ground_three_point_evidence_v1", "session_id": self.session_id,
            "survey": self._observations.survey_evidence(), "completed_points": self.completed_points,
            "current_point": current, "capture_image_events": capture_events}
        raw = json.dumps(bundle, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        packed = gzip.compress(raw, compresslevel=6, mtime=0)
        encoded = base64.b64encode(packed).decode("ascii")
        chunks = [encoded[i:i+48000] for i in range(0, len(encoded), 48000)]
        metadata = {**common, "section": "three_ground_evidence_gzip_part", "encoding": "gzip+base64",
            "bundle": "ground_three_point_v1", "uncompressed_sha256": hashlib.sha256(raw).hexdigest(),
            "compressed_sha256": hashlib.sha256(packed).hexdigest(), "uncompressed_bytes": len(raw),
            "compressed_bytes": len(packed), "part_count": len(chunks)}
        return events+[("probe_snapshot", {**metadata, "part_index": i, "base64_part": part})
                       for i, part in enumerate(chunks)]


def create_three_point_node(reporter=None, *, wall_clock=time.monotonic, route_limits=None, worker=None):
    def factory(limits, *, started_wall_s):
        return GroundThreePointGuard(started_wall_s=started_wall_s, route_limits=route_limits)
    node = create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5, callback_observer=RouteCallbackTimingRecorder(),
        input_reliability_overrides={"/odom": "BEST_EFFORT", "/uav/odom": "BEST_EFFORT"})
    from rosidl_runtime_py.utilities import get_message
    String = get_message("std_msgs/msg/String")
    original_refresh, original_tick = node.refresh_endpoints, node.tick
    counts = {topic: {"attempted": 0, "returned": 0}
              for topic in ("/inspection/detections", "/referee/current_target")}
    def refresh():
        original_refresh()
        for topic, values in counts.items():
            node.command_endpoints.setdefault(topic, {}).update(silent=False,
                publication_policy="one_attempt_per_current_visual_and_physical_ground_point",
                actual_publish_attempt_count=values["attempted"], actual_publish_return_count=values["returned"],
                interpretation="enabled_policy_not_evidence_of_referee_acceptance")
        node.record_endpoint_history()
    def send(topic, data):
        publisher = node.publisher_by_topic.get(topic)
        if publisher is None or publisher.get_subscription_count() <= 0:
            raise RuntimeError("no matching official String subscriber: "+topic)
        message = String()
        message.data = data
        counts[topic]["attempted"] += 1
        publisher.publish(message)
        counts[topic]["returned"] += 1
    coordinator = None if worker is None else GroundSinglePointCoordinator(node.guard, worker)
    def tick():
        decision = original_tick()
        try:
            if coordinator is not None:
                coordinator.poll_once(wall_clock())
            with node.command_lock:
                node.guard.dispatch_one(send, wall_s=wall_clock())
                for topic, values in counts.items():
                    node.command_endpoints[topic].update(actual_publish_attempt_count=values["attempted"],
                        actual_publish_return_count=values["returned"])
        except Exception as exc:
            node.stop("three_ground_coordinator_failed:"+type(exc).__name__+":"+str(exc)[:256])
        return decision
    node.refresh_endpoints, node.tick = refresh, tick
    node.single_point_coordinator = coordinator
    refresh()
    return node


def main(args=None):
    import rclpy
    from .ground_matcher_worker import GroundMatcherWorker
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="Continuous three current-image ground points; no aerial completion")
    parser.add_argument("--duration", type=float, default=1200.)
    parser.add_argument("--template-dir", default=None)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ThreePointLimits(global_wall_s=parsed.duration)
    worker = GroundMatcherWorker(load_ground_templates(parsed.template_dir))
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_three_point_node(reporter, route_limits=limits, worker=worker)
        node.report_started(parsed.duration)
        node.arm_watchdog()
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("three_ground_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("three_ground_cleanup")
            node.close_watchdog()
            node.destroy_node()
        worker.close(timeout_s=.5)
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
