"""A separate measured 2 cm forward experiment, without scoring messages.

The motion guard is independent of the yaw survey. Its observation recorder
reuses only the survey's public odometry/image transport and never its control.
"""
from __future__ import annotations

import argparse
import copy
import math
import os
import time
from dataclasses import asdict, dataclass

from .bounded_ground_probe import (GroundMotionGuard, MotionDecision, MotionLimits,
                                  create_bounded_node, finite, spin_control_loop)
from .ground_observation_survey import GroundSurveyGuard, angle_wrap, yaw_from_q


@dataclass(frozen=True)
class ShortDriveLimits:
    target_distance_m: float = .02
    max_motion_wall_s: float = 10.
    max_heading_error_rad: float = .05
    max_reverse_m: float = .005
    global_wall_s: float = 25.

    def __post_init__(self):
        for field, maximum in (("target_distance_m", .02), ("max_motion_wall_s", 10.),
                               ("max_heading_error_rad", .05), ("max_reverse_m", .005),
                               ("global_wall_s", 25.)):
            value = getattr(self, field)
            if not finite(value) or not 0 < value <= maximum:
                raise ValueError(field + " exceeds short-drive bound")
        if self.global_wall_s < 5.:
            raise ValueError("global wall limit too short")


class _ShortDriveObservationRecorder(GroundSurveyGuard):
    """Observation reuse with a permanently inactive control state.

    No step, watchdog, public draw validation, or survey target calculation is
    invoked. PAUSE/DRIVE state belongs exclusively to GroundShortDriveGuard.
    """
    mode = "ground_short_drive"

    def __init__(self, limits, *, started_wall_s):
        super().__init__(limits, started_wall_s=started_wall_s)
        self.drive_context = {}

    def survey_evidence(self):
        with self._lock:
            return {"mode": self.mode, "drive": copy.deepcopy(self.drive_context),
                "trajectory_10hz": copy.deepcopy(self.trajectory),
                "trajectory_dropped": self.trajectory_dropped,
                "actual_commands": copy.deepcopy(self.actual_commands),
                "actual_commands_dropped": self.actual_commands_dropped,
                "exposure_records": [{**copy.deepcopy(record), "phase": phase}
                    for records in self.exposures.values() for phase, record in records.items()],
                "image_errors": list(self.image_errors),
                "observed_odom_periods": self.observed_periods(),
                "task_completion": "unverified"}


class GroundShortDriveGuard(GroundMotionGuard):
    mode = "ground_short_drive"
    wall_limit_reason = "short_drive_wall_time_limit"
    final_report_wall_s = 60.
    ACTIVE_STATES = frozenset(("DRIVE", "PAUSE"))
    FRESHNESS_REASONS = frozenset(("clock_stale", "odom_stale", "scan_stale",
                                 "odom_stale_or_invalid", "scan_stale_or_unsafe"))

    def __init__(self, limits, *, started_wall_s, drive_limits=None):
        super().__init__(limits, started_wall_s=started_wall_s)
        self.drive_limits = drive_limits or ShortDriveLimits()
        self.motion_wall_limit_s = self.drive_limits.max_motion_wall_s
        self.initial_yaw = None
        self.resume_count = 0
        self._observations = _ShortDriveObservationRecorder(limits, started_wall_s=started_wall_s)

    def _observation_wall(self, topic, wall):
        # The original guard deliberately knows only PULSE/TURN. Keep it frozen
        # and explicitly enforce every arrival boundary for this new DRIVE mode.
        if not finite(wall) or wall < max(self.last_wall, self.last_observation_wall):
            self.stop(topic + "_arrival_wall_reversed_or_invalid", wall)
            return False
        previous = self.last_sensor_arrival.get(topic)
        self.last_observation_wall = wall
        self.last_sensor_arrival[topic] = wall
        if self.state in self.ACTIVE_STATES:
            budget = self._budget_fault(wall)
            if budget:
                self.stop(budget, wall)
            elif previous is not None and wall - previous > self.limits.sensor_age_s + 1e-9:
                self.temporal_fault(topic + "_stale", wall)
        return True

    def _budget_fault(self, wall):
        if not finite(wall) or wall < max(self.last_wall, self.last_observation_wall):
            return "wall_clock_reversed_or_invalid"
        if wall - self.started_wall >= self.drive_limits.global_wall_s:
            return "short_drive_global_wall_time_limit"
        if self.pulse_started is not None and wall - self.pulse_started >= self.motion_wall_limit_s - 1e-9:
            return self.wall_limit_reason
        return ""

    def measured_forward(self):
        if self.pulse_origin is None or self.odom is None or self.initial_yaw is None:
            return None
        dx, dy = (self.odom["xyz"][i] - self.pulse_origin[i] for i in (0, 1))
        return dx * math.cos(self.initial_yaw) + dy * math.sin(self.initial_yaw)

    def observed_motion_fault(self):
        fault = super().observed_motion_fault()
        if fault or self.odom is None or self.initial_yaw is None:
            return fault
        if abs(angle_wrap(yaw_from_q(self.odom["q"]) - self.initial_yaw)) > self.drive_limits.max_heading_error_rad:
            return "short_drive_heading_limit"
        if self.measured_forward() < -self.drive_limits.max_reverse_m:
            return "short_drive_reverse_limit"
        return ""

    def _check_active_timing(self, wall):
        if self.state not in self.ACTIVE_STATES:
            return
        fault = self._budget_fault(wall) or self.observed_motion_fault()
        if fault:
            self.stop(fault, wall)
            return
        if (self.clock is None or wall - self.clock[1] > self.limits.sensor_age_s + 1e-9
                or self.last_clock_progress_wall is None
                or wall - self.last_clock_progress_wall > self.limits.sensor_age_s + 1e-9):
            self.temporal_fault("clock_stale", wall)
            return
        for name, sample in (("odom", self.odom), ("scan", self.scan)):
            if not self._fresh(sample, wall):
                self.temporal_fault("odom_stale_or_invalid" if name == "odom" else "scan_stale_or_unsafe", wall)
                return

    def update_clock(self, stamp_s, *, wall_s):
        with self._lock:
            super().update_clock(stamp_s, wall_s=wall_s)
            self._observations.update_clock(stamp_s, wall_s=wall_s)

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            active = self.state in self.ACTIVE_STATES
            super().update_odom(payload, wall_s=wall_s)
            if active and self.odom is None:
                self.stop("odom_invalid:" + self.reason, wall_s)
            if self.state in self.ACTIVE_STATES:
                fault = self._budget_fault(wall_s) or self.observed_motion_fault()
                if fault:
                    self.stop(fault, wall_s)
                elif self.measured_forward() >= self.drive_limits.target_distance_m:
                    self.stop("short_drive_distance_reached", wall_s)
            # The recorder always remains WAIT/STOP, so its turn-to-target
            # callbacks can never execute, including while this guard is PAUSE.
            self._observations.update_odom(payload, wall_s=wall_s)

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            active = self.state in self.ACTIVE_STATES
            super().update_scan(payload, wall_s=wall_s)
            if active and self.scan is None:
                self.stop("scan_invalid:" + self.reason, wall_s)

    def temporal_fault(self, reason, wall):
        with self._lock:
            if self.state in self.ACTIVE_STATES and reason in self.FRESHNESS_REASONS:
                self.stable_anchor = None
                self.state, self.reason = "PAUSE", reason
            else:
                self.stop(reason, wall)

    def handle_watchdog(self, reason, wall):
        with self._lock:
            hard_fault = self._budget_fault(wall)
            if hard_fault:
                self.stop(hard_fault, wall)
            elif reason in self.FRESHNESS_REASONS:
                self.temporal_fault(reason, wall)
            else:
                self.stop(reason, wall)

    def _drive_fault(self, wall):
        return self._budget_fault(wall) or self.observed_motion_fault() or self._condition(wall)

    def watchdog_reason(self, wall):
        with self._lock:
            return self._drive_fault(wall) if self.state in self.ACTIVE_STATES else ""

    def _stable_dwell(self, wall):
        pose = self.odom
        stable = pose["speed"] <= .01 and pose["yaw_speed"] <= .03
        if self.stable_anchor is not None:
            a = self.stable_anchor
            stable = (stable and math.dist(pose["xyz"], a[2]) <= .005
                      and abs(sum(x * y for x, y in zip(pose["q"], a[3]))) >= math.cos(.02 / 2))
        if not stable:
            self.stable_anchor = None
            return False
        if self.stable_anchor is None:
            self.stable_anchor = (wall, pose["stamp"], pose["xyz"], pose["q"])
        anchor = self.stable_anchor
        return min(wall - anchor[0], pose["stamp"] - anchor[1]) >= self.limits.stable_s - 1e-9

    def step(self, *, wall_s):
        with self._lock:
            if not finite(wall_s) or wall_s < self.last_wall:
                return self.stop("wall_clock_reversed_or_invalid", self.last_wall)
            self.last_wall = wall_s
            if self.state in ("STOP", "FINISHED"):
                if wall_s - self.stopped_wall >= self.limits.stop_wall_s:
                    self.state = "FINISHED"
                return MotionDecision(0., self.state, self.reason)
            budget = self._budget_fault(wall_s)
            if budget:
                return self.stop(budget, wall_s)
            if self.state in self.ACTIVE_STATES:
                fault = self._drive_fault(wall_s)
                if fault:
                    if fault in self.FRESHNESS_REASONS:
                        self.temporal_fault(fault, wall_s)
                        return MotionDecision(0., self.state, self.reason)
                    return self.stop(fault, wall_s)
                if self.measured_forward() >= self.drive_limits.target_distance_m:
                    return self.stop("short_drive_distance_reached", wall_s)
                if self.state == "PAUSE":
                    if not self._stable_dwell(wall_s):
                        return MotionDecision(0., self.state, "pause_stability_dwell")
                    if self.resume_count >= 3:
                        return self.stop("pause_resume_limit", wall_s)
                    self.resume_count += 1
                    self.state, self.reason = "DRIVE", "bounded_short_drive"
                    self.stable_anchor = None
                return MotionDecision(self.limits.speed_mps, self.state, self.reason)
            if wall_s - self.started_wall >= self.limits.wait_wall_s:
                return self.stop("wait_timeout:" + self.reason, wall_s)
            reason = self._condition(wall_s)
            if reason:
                self.stable_anchor = None
                self.reason = reason
                return MotionDecision(0., self.state, reason)
            if not self._stable_dwell(wall_s):
                return MotionDecision(0., self.state, "start_stability_dwell")
            self.initial_yaw = yaw_from_q(self.odom["q"])
            self.pulse_started, self.pulse_origin = wall_s, self.odom["xyz"]
            self.state, self.reason = "DRIVE", "bounded_short_drive"
            self.stable_anchor = None
            return MotionDecision(self.limits.speed_mps, self.state, self.reason)

    def observe_public(self, topic, payload, *, wall_s):
        if topic == "/uav/odom":
            self._observations.update_uav_odom(payload, wall_s=wall_s)

    def observe_image(self, topic, msg, *, wall_s):
        self._observations.observe_image(topic, msg, wall_s=wall_s)

    def record_command(self, *, wall_s, linear_x, angular_z):
        with self._lock, self._observations._lock:
            self._observations.record_command(wall_s=wall_s, linear_x=linear_x, angular_z=angular_z)
            self._observations.actual_commands[-1]["state"] = self.state

    def drive_evidence(self):
        with self._lock:
            return {"mode": self.mode, "limits": asdict(self.drive_limits),
                "target_distance_m": self.drive_limits.target_distance_m,
                "initial_yaw_rad": self.initial_yaw, "origin_position": self.pulse_origin,
                "first_motion_wall_s": self.pulse_started, "measured_forward_m": self.measured_forward(),
                "max_observed_displacement_m": self.max_observed_displacement_m,
                "resume_count": self.resume_count, "max_resumes": 3,
                "state": self.state, "reason": self.reason, "task_completion": "unverified"}

    def _sync_observation_context(self):
        with self._lock, self._observations._lock:
            self._observations.drive_context = self.drive_evidence()

    def survey_evidence(self):
        self._sync_observation_context()
        return self._observations.survey_evidence()

    def snapshot_survey_evidence(self):
        self._sync_observation_context()
        return self._observations.snapshot_survey_evidence()

    def additional_events(self):
        self._sync_observation_context()
        return self._observations.additional_events()


def create_short_drive_node(reporter=None, *, wall_clock=time.monotonic, drive_limits=None):
    drive_limits = drive_limits or ShortDriveLimits()
    def factory(limits, *, started_wall_s):
        return GroundShortDriveGuard(limits, started_wall_s=started_wall_s, drive_limits=drive_limits)
    return create_bounded_node(MotionLimits(.447, .114, .02), reporter,
                               wall_clock=wall_clock, guard_factory=factory)


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="Measured ground-only 2cm drive; no task or scoring messages")
    parser.add_argument("--duration", type=float, default=25.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ShortDriveLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_short_drive_node(reporter, drive_limits=limits)
        node.report_started(parsed.duration)
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("short_drive_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("short_drive_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
