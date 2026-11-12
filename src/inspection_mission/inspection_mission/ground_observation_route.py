"""Independent, measured route to one public first ground observation pose.

Reuses the synchronized approach guard's sensor validation, PAUSE, source
images and ROS adapter. Candidate geometry is never recognition or scoring.
Only /cmd_vel may publish; all inherited task/flight outputs remain silent.
"""
from __future__ import annotations

import argparse
import copy
import math
import os
import time
from dataclasses import asdict, dataclass, replace

from .bounded_ground_probe import MotionDecision, MotionLimits, create_bounded_node, finite, spin_control_loop
from .ground_bounded_approach_sync import GroundSynchronizedApproachGuard
from .ground_callback_timing import CallbackTimingRecorder
from .ground_observation_route_plan import plan_ground_observation_route
from .ground_observation_survey import angle_wrap, yaw_from_q


class RouteCallbackTimingRecorder(CallbackTimingRecorder):
    revision = "ground_observation_route_v2"


@dataclass(frozen=True)
class RouteLimits:
    target_distance_m: float = .20
    global_wall_s: float = 300.
    max_path_m: float = 3.
    max_origin_displacement_m: float = 3.
    max_rotation_rad: float = 6.
    local_swept_motion_budget_m: float = .06
    scan_anchor_motion_limit_m: float = .03
    local_stop_reserve_m: float = .03
    linear_command_mps: float = .10
    minimum_yaw_rate: float = .35
    maximum_yaw_rate: float = .5
    max_heading_error_rad: float = .05
    max_reverse_m: float = .005
    max_cross_track_m: float = .02
    max_turn_drift_m: float = .06
    wait_wall_s: float = 5.
    turn_wall_s: float = 45.
    turn_total_wall_s: float = 100.
    drive_wall_s: float = 190.
    capture_wall_s: float = 10.

    def __post_init__(self):
        caps = {"target_distance_m": 3., "global_wall_s": 300., "max_path_m": 3.,
            "max_origin_displacement_m": 3., "max_rotation_rad": 6.,
            "local_swept_motion_budget_m": .06, "scan_anchor_motion_limit_m": .03,
            "local_stop_reserve_m": .03, "linear_command_mps": .10,
            "minimum_yaw_rate": .35, "maximum_yaw_rate": .5, "max_heading_error_rad": .05,
            "max_reverse_m": .005, "max_cross_track_m": .02, "max_turn_drift_m": .06,
            "wait_wall_s": 5., "turn_wall_s": 45., "turn_total_wall_s": 100.,
            "drive_wall_s": 190., "capture_wall_s": 10.}
        for field, cap in caps.items():
            if not finite(getattr(self, field)) or not 0 < getattr(self, field) <= cap:
                raise ValueError(field+" exceeds observation-route limit")
        if (self.minimum_yaw_rate > self.maximum_yaw_rate
                or self.scan_anchor_motion_limit_m+self.local_stop_reserve_m > self.local_swept_motion_budget_m+1e-9):
            raise ValueError("inconsistent route local reserve or angular bounds")


class GroundObservationRouteGuard(GroundSynchronizedApproachGuard):
    mode = "ground_observation_route"
    wall_limit_reason = "route_global_wall_time_limit"
    ACTIVE = GroundSynchronizedApproachGuard.ACTIVE | {"CALIBRATION_SETTLE"}
    ZERO_PHASES = GroundSynchronizedApproachGuard.ZERO_PHASES | {"CALIBRATION_SETTLE"}
    SCAN_STAGES = frozenset(("start", "first_leg_end", "observation_arrival", "final_heading"))

    def __init__(self, *, started_wall_s, route_limits=None):
        limits = route_limits or RouteLimits()
        if not isinstance(limits, RouteLimits):
            raise ValueError("route_limits must be RouteLimits")
        super().__init__(started_wall_s=started_wall_s, approach_limits=limits)
        self._observations.mode = self.mode
        self._observations.TRAJECTORY_CAP = 3200
        self._observations.COMMAND_CAP = 14000
        self.route_plan = None
        self.leg_index = 0
        self.phase_started_wall = None
        self.turn_elapsed_wall_s = self.drive_elapsed_wall_s = 0.
        self.leg_target_xy = None
        self.leg_target_distance_m = None
        self.leg_records = []
        self.settle_reaim_count = 0
        self.initial_slow_anchor = None
        self.initial_slow_validated = False
        self.calibration_verified = False
        self.calibration_stop_path_m = None
        self.leg_stop_path_m = None
        self.speed_validation = {"status": "not_yet_measured",
            "requested_tiers_mps": [min(.03, limits.linear_command_mps), limits.linear_command_mps]}

    @staticmethod
    def _phase_group(phase):
        if phase in ("TURN", "TURN_SETTLE"):
            return "turn"
        if phase in ("DRIVE", "DRIVE_SETTLE", "CALIBRATION_SETTLE"):
            return "drive"
        return None

    def _elapsed_group(self, group, wall):
        complete = self.turn_elapsed_wall_s if group == "turn" else self.drive_elapsed_wall_s
        if self._phase_group(self.phase) == group and self.phase_started_wall is not None:
            complete += max(0., wall-self.phase_started_wall)
        return complete

    def _budget_fault(self, wall):
        if not finite(wall) or wall < max(self.last_wall, self.last_observation_wall):
            return "wall_clock_reversed_or_invalid"
        a = self.approach_limits
        if wall-self.started_wall >= a.global_wall_s-1e-9:
            return self.wall_limit_reason
        if self._phase_group(self.phase) == "turn":
            if self.turn_started_wall is not None and wall-self.turn_started_wall >= a.turn_wall_s-1e-9:
                return "route_single_turn_wall_time_limit"
            if self._elapsed_group("turn", wall) >= a.turn_total_wall_s-1e-9:
                return "route_turn_total_wall_time_limit"
        if self._elapsed_group("drive", wall) >= a.drive_wall_s-1e-9:
            return "route_drive_total_wall_time_limit"
        if (self.phase == "CAPTURE" and self.capture_started_wall is not None
                and wall-self.capture_started_wall >= a.capture_wall_s-1e-9):
            return "route_capture_wall_time_limit"
        return ""

    def _turn_tolerance(self):
        if self.route_plan is None or self.leg_index == len(self.route_plan.segments):
            return .025
        distance = math.dist(self.odom["xyz"][:2], self.route_plan.waypoints_xy_m[self.leg_index])
        return min(.025, math.atan2(.01, max(distance, .01)))

    def _enter_phase(self, phase):
        # Parent's short-drive .025rad window would spend >.02m lateral
        # budget on a 1.8m leg. Keep the guard, tighten this route's aim.
        if (phase == "TURN_SETTLE" and self.odom is not None and self.target_yaw is not None
                and abs(angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))) > self._turn_tolerance()):
            return
        if phase != "DRIVE" and not self.initial_slow_validated:
            self.initial_slow_anchor = None
        wall = max(self.last_wall, self.last_observation_wall)
        if phase == "DRIVE_SETTLE" and self.phase != "DRIVE_SETTLE":
            self.leg_stop_path_m = self.total_path_m
            self.leg_records[-1]["stop_request"] = {"path_origin_m": self.total_path_m,
                "path_increment_m": 0., "wall_s": wall, "odom_stamp_s": self.odom["stamp"],
                "interpretation": "phase requested zero; local publish and actuator receipt not implied"}
        old, new = self._phase_group(self.phase), self._phase_group(phase)
        if old != new:
            if self.phase_started_wall is not None:
                if old == "turn":
                    self.turn_elapsed_wall_s += wall-self.phase_started_wall
                elif old == "drive":
                    self.drive_elapsed_wall_s += wall-self.phase_started_wall
            self.phase_started_wall = wall if new else None
        super()._enter_phase(phase)
        self.reason = "route_"+phase.lower()
        if phase == "CAPTURE":
            self.capture_started_wall = wall

    def temporal_fault(self, reason, wall):
        if not self.initial_slow_validated:
            self.initial_slow_anchor = None
        super().temporal_fault(reason, wall)

    def observed_motion_fault(self):
        if self.phase == "DRIVE_SETTLE" and self.leg_stop_path_m is not None:
            increment = self.total_path_m-self.leg_stop_path_m
            self.leg_records[-1]["stop_request"]["path_increment_m"] = increment
            if increment > .01+1e-9:
                return "route_drive_settle_path_limit"
        fault = super().observed_motion_fault()
        if fault:
            return fault
        if (self.phase == "CAPTURE" and self.odom is not None and self.target_yaw is not None
                and abs(angle_wrap(yaw_from_q(self.odom["q"])-self.target_yaw)) > self.approach_limits.max_heading_error_rad):
            return "approach_heading_limit"
        if (self.phase == "CAPTURE" and self.odom is not None and self.route_plan is not None
                and math.dist(self.odom["xyz"][:2], self.route_plan.waypoints_xy_m[-1]) > .02+1e-9):
            return "route_waypoint_error_limit"
        if (self.calibration_stop_path_m is not None and not self.calibration_verified
                and self.total_path_m-self.calibration_stop_path_m > .01+1e-9):
            return "route_calibration_stop_distance_limit"
        return ""

    def _record_scan(self, stage, wall):
        # Parent callbacks request mid_drive repeatedly. Retain only the four
        # route boundary scans so metadata cannot grow once per leg.
        if stage in self.SCAN_STAGES:
            super()._record_scan(stage, wall)

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            super().update_odom(payload, wall_s=wall_s)
            if (self.state in self.ACTIVE and self.phase == "DRIVE"
                    and not self.calibration_verified and self.calibration_stop_path_m is None
                    and self.measured_forward() >= .05-1e-9):
                self.calibration_stop_path_m = self.total_path_m
                self.speed_validation.update(status="measuring_stop", stop_forward_m=self.measured_forward(),
                    zero_requested_wall_s=wall_s, zero_requested_odom_stamp_s=self.odom["stamp"],
                    stop_path_m=self.total_path_m)
                self._enter_phase("CALIBRATION_SETTLE")

    def _start_turn(self, wall, *, final=False):
        # Only local leg state changes. Permanent origin/path/angle never reset.
        self.drive_origin = self.drive_yaw = None
        self.turn_origin = self.odom["xyz"]
        x, y = self.odom["xyz"][:2]
        destination = self.target_xy if final else self.route_plan.waypoints_xy_m[self.leg_index]
        self.target_yaw = math.atan2(destination[1]-y, destination[0]-x)
        self.initial_turn_error = angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
        self.turn_started_wall = wall
        self._enter_phase("TURN")

    def _start_drive(self, wall):
        self.leg_target_xy = self.route_plan.waypoints_xy_m[self.leg_index]
        self.drive_origin, self.drive_yaw = self.odom["xyz"], self.target_yaw
        dx, dy = (self.leg_target_xy[i]-self.drive_origin[i] for i in (0, 1))
        self.leg_target_distance_m = dx*math.cos(self.drive_yaw)+dy*math.sin(self.drive_yaw)
        if self.leg_target_distance_m <= .02:
            self.stop("route_leg_too_short_or_behind", wall)
            return
        # Request zero 2mm early; independently bound all post-trigger path to 1cm.
        self.approach_limits = replace(self.approach_limits, target_distance_m=self.leg_target_distance_m-.002)
        self.leg_stop_path_m = None
        self.drive_started_wall = wall
        self.leg_records.append({"leg_index": self.leg_index, "origin": self.drive_origin,
            "yaw_rad": self.drive_yaw, "waypoint_xy_m": self.leg_target_xy,
            "target_projection_m": self.leg_target_distance_m, "started_wall_s": wall})
        self._enter_phase("DRIVE")

    def _linear_request(self, wall):
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
        if (self.leg_index == len(self.route_plan.segments)-1
                and self.leg_target_distance_m-self.measured_forward() <= .10+1e-9):
            return min(.03, self.approach_limits.linear_command_mps)
        return min(.10, self.approach_limits.linear_command_mps)

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
                    self.state, self.reason = self.phase, "route_"+self.phase.lower()
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
                pose = (*self.odom["xyz"][:2], yaw_from_q(self.odom["q"]))
                try:
                    self.route_plan = plan_ground_observation_route(self.draw[0], pose)
                except ValueError:
                    return self.stop("route_initial_pose_outside_contract", wall_s)
                self.route_plan = replace(self.route_plan, budgets=replace(self.route_plan.budgets,
                    **{name: getattr(self.approach_limits, name) for name in (
                        "global_wall_s", "drive_wall_s", "turn_wall_s", "turn_total_wall_s",
                        "capture_wall_s", "wait_wall_s")}))
                b = self.route_plan.budgets
                self.approach_limits = replace(self.approach_limits, max_path_m=b.max_path_m,
                    max_origin_displacement_m=b.max_origin_displacement_m, max_rotation_rad=b.max_rotation_rad)
                self.global_origin = self.pulse_origin = self.previous_path_xyz = self.odom["xyz"]
                self.previous_yaw = pose[2]
                self.target_id, self.target_xy = self.route_plan.first_id, self.route_plan.target_xy_m
                self.first_motion_wall = wall_s
                self._record_scan("start", wall_s)
                self._start_turn(wall_s)
            if self.phase == "TURN":
                error = angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
                if abs(error) <= self._turn_tolerance():
                    if abs(error) > self.approach_limits.max_heading_error_rad:
                        return self.stop("approach_turn_overshoot_limit", wall_s)
                    self._enter_phase("TURN_SETTLE")
                else:
                    minimum = self.approach_limits.minimum_yaw_rate
                    rate = math.copysign(min(self.approach_limits.maximum_yaw_rate,
                        max(minimum, .6*abs(error))), error)
                    return MotionDecision(0., self.state, self.reason, rate)
            if self.phase in ("TURN_SETTLE", "DRIVE_SETTLE", "CALIBRATION_SETTLE"):
                if not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, self.reason)
                if self.phase == "TURN_SETTLE":
                    error = angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
                    if abs(error) > self._turn_tolerance():
                        # Static tolerance permits .02rad of drift, which is
                        # wider than a long leg's heading contract. Re-aim
                        # inside this same TURN allocation; keep all origins
                        # and single/cumulative deadlines unchanged.
                        self.initial_turn_error = error
                        self.settle_reaim_count += 1
                        self._enter_phase("TURN")
                        return MotionDecision(0., self.state, self.reason)
                    if self.leg_index == len(self.route_plan.segments):
                        if math.dist(self.odom["xyz"][:2], self.route_plan.waypoints_xy_m[-1]) > .02+1e-9:
                            return self.stop("route_waypoint_error_limit", wall_s)
                        self._record_scan("final_heading", wall_s)
                        self._enter_phase("CAPTURE")
                    else:
                        self._start_drive(wall_s)
                elif self.phase == "CALIBRATION_SETTLE":
                    anchor = self.stable_anchor
                    self.speed_validation.update(status="verified", stop_path_increment_m=self.total_path_m-self.calibration_stop_path_m,
                        stationary_wall_s=wall_s-anchor[0], stationary_sim_s=self.odom["stamp"]-anchor[1],
                        verified_wall_s=wall_s, verified_odom_stamp_s=self.odom["stamp"])
                    self.calibration_verified = True
                    self._enter_phase("DRIVE")
                else:
                    gap = math.dist(self.odom["xyz"][:2], self.leg_target_xy)
                    self.leg_records[-1].update(ended_wall_s=wall_s, final_position=self.odom["xyz"], waypoint_error_m=gap)
                    if gap > .02+1e-9:
                        return self.stop("route_waypoint_error_limit", wall_s)
                    if self.leg_index == 0:
                        self._record_scan("first_leg_end", wall_s)
                    self.leg_index += 1
                    final = self.leg_index == len(self.route_plan.segments)
                    if final:
                        self._record_scan("observation_arrival", wall_s)
                    self._start_turn(wall_s, final=final)
                    # New turn is evaluated on the next tick; never emit an
                    # unintended linear command while changing legs.
                    return MotionDecision(0., self.state, self.reason)
            if self.phase == "DRIVE" and self.state not in ("STOP", "FINISHED"):
                return MotionDecision(self._linear_request(wall_s), self.state, self.reason)
            return MotionDecision(0., self.state, self.reason)

    def stop(self, reason, wall_s):
        if reason == "approach_observation_complete":
            reason = "route_observation_complete"
        return super().stop(reason, wall_s)

    def approach_evidence(self):
        with self._lock:
            evidence = super().approach_evidence()
            wall = max(self.last_wall, self.last_observation_wall)
            evidence["revision"] = "ground_observation_route_v2"
            evidence["route"] = {"mode": self.mode, "plan": None if self.route_plan is None else asdict(self.route_plan),
                "leg_index": self.leg_index, "legs": copy.deepcopy(self.leg_records),
                "settle_reaim_count": self.settle_reaim_count,
                "leg_target_xy_m": self.leg_target_xy, "leg_target_projection_m": self.leg_target_distance_m,
                "turn_elapsed_wall_s": self._elapsed_group("turn", wall),
                "drive_elapsed_wall_s": self._elapsed_group("drive", wall),
                "speed_validation": copy.deepcopy(self.speed_validation),
                "runtime_budget_status": "engineering_allocation_pending_platform_measurement",
                "candidate_geometry_status": "candidate_unverified", "observation_only": True}
            return evidence


def create_route_node(reporter=None, *, wall_clock=time.monotonic, route_limits=None):
    def factory(limits, *, started_wall_s):
        return GroundObservationRouteGuard(started_wall_s=started_wall_s, route_limits=route_limits)
    return create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5, callback_observer=RouteCallbackTimingRecorder(),
        input_reliability_overrides={"/odom": "BEST_EFFORT", "/uav/odom": "BEST_EFFORT"})


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="Independent first ground outside observation route; no scoring output")
    parser.add_argument("--duration", type=float, default=300.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = RouteLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_route_node(reporter, route_limits=limits)
        node.report_started(parsed.duration)
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("route_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("route_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
