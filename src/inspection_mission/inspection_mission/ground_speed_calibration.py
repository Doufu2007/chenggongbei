"""v19: isolated two-tier/stop calibration inside the original 20 cm budget."""
from __future__ import annotations

import argparse
import copy
import os
import time

from .bounded_ground_probe import MotionDecision, MotionLimits, create_bounded_node, spin_control_loop
from .ground_bounded_approach import ApproachLimits
from .ground_bounded_approach_sync import GroundSynchronizedApproachGuard
from .ground_callback_timing import CallbackTimingRecorder


class SpeedCalibrationTimingRecorder(CallbackTimingRecorder):
    revision = "v19_speed_calibration"


class GroundSpeedCalibrationGuard(GroundSynchronizedApproachGuard):
    ACTIVE = GroundSynchronizedApproachGuard.ACTIVE | {"LOW_STOP", "HIGH_PULSE", "HIGH_STOP"}
    ZERO_PHASES = GroundSynchronizedApproachGuard.ZERO_PHASES | {"LOW_STOP", "HIGH_STOP"}

    def __init__(self, *, started_wall_s, approach_limits=None):
        limits = ApproachLimits() if approach_limits is None else approach_limits
        if (not isinstance(limits, ApproachLimits) or limits.linear_command_mps != .03
                or limits.target_distance_m != .20):
            raise ValueError("fixed calibration contract requires .03 slow tier and .20 total distance")
        super().__init__(started_wall_s=started_wall_s, approach_limits=limits)
        self.calibration_stage = "low"
        self.low_request_start = None
        self.low_continuous_wall_s = self.low_continuous_sim_s = 0.
        self.low_duration_verified = False
        self.high_request_start = None
        self.high_origin_path = self.high_origin_forward = self.high_end_path = None
        self.stop_records = []
        self.tiers = {name: {"requested_mps": request, "sample_count": 0,
            "max_speed_mps": 0., "first": None, "last": None,
            "first_successful_nonzero_publish": None}
            for name, request in (("low", .03), ("high", .10), ("final_slow", .03))}

    def _sample(self, wall):
        return {"wall_elapsed_s": wall-self.started_wall, "stamp_s": self.odom["stamp"],
            "position": list(self.odom["xyz"]), "linear_speed_mps": self.odom["speed"],
            "angular_speed_radps": self.odom["yaw_speed"], "forward_m": self.measured_forward(),
            "total_path_m": self.total_path_m}

    def _high_path(self):
        if self.high_origin_path is None:
            return 0.
        end = self.total_path_m if self.high_end_path is None else self.high_end_path
        return end-self.high_origin_path

    def _budget_fault(self, wall):
        reason = super()._budget_fault(wall)
        if reason:
            return reason
        if (self.phase in ("LOW_STOP", "HIGH_PULSE", "HIGH_STOP")
                and self.drive_started_wall is not None
                and wall-self.drive_started_wall >= self.approach_limits.drive_wall_s-1e-9):
            return "approach_drive_wall_time_limit"
        if self.phase == "HIGH_PULSE" and self.high_request_start is not None:
            if wall-self.started_wall-self.high_request_start["wall_elapsed_s"] >= 5.-1e-9:
                return "calibration_high_wall_time_limit"
            if self.odom is not None and self.odom["stamp"]-self.high_request_start["stamp_s"] >= 5.-1e-9:
                return "calibration_high_sim_time_limit"
        return ""

    def observed_motion_fault(self):
        reason = super().observed_motion_fault()
        if reason:
            return reason
        if self.stop_records and not self.stop_records[-1]["verified"]:
            stop = self.stop_records[-1]
            if self.total_path_m-stop["path_origin_m"] > .01+1e-9:
                return "calibration_"+stop["tier"]+"_stop_path_limit"
        if self.high_origin_path is not None and self.high_end_path is None and self._high_path() > .05+1e-9:
            return "calibration_high_total_path_limit"
        return ""

    def temporal_fault(self, reason, wall):
        with self._lock:
            if (self.phase == "DRIVE" and self.calibration_stage == "low"
                    and not self.low_duration_verified):
                self.low_request_start = None
                self.low_continuous_wall_s = self.low_continuous_sim_s = 0.
            super().temporal_fault(reason, wall)

    def _begin_stop(self, tier, wall):
        self.stop_records.append({"tier": tier, "phase_switch": self._sample(wall),
            "path_origin_m": self.total_path_m, "path_increment_m": 0.,
            "first_successful_zero_publish": None, "verified": False,
            "publish_interpretation": "local publish returned; actuator receipt/ACK unverified"})
        self._enter_phase("LOW_STOP" if tier == "low" else "HIGH_STOP")

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            previous_phase, previous_stage = self.phase, self.calibration_stage
            super().update_odom(payload, wall_s=wall_s)
            if self.odom is None:
                return
            if previous_phase in ("DRIVE", "HIGH_PULSE") and previous_stage in self.tiers:
                tier = self.tiers[previous_stage]
                if tier["first_successful_nonzero_publish"] is not None:
                    sample = self._sample(wall_s)
                    tier["sample_count"] += 1
                    tier["max_speed_mps"] = max(tier["max_speed_mps"], self.odom["speed"])
                    if tier["first"] is None:
                        tier["first"] = copy.deepcopy(sample)
                    tier["last"] = sample
            if self.stop_records and not self.stop_records[-1]["verified"]:
                self.stop_records[-1]["path_increment_m"] = self.total_path_m-self.stop_records[-1]["path_origin_m"]
            if self.state in ("STOP", "FINISHED"):
                return
            if self.phase == "DRIVE" and self.calibration_stage == "low" and self.measured_forward() >= .05-1e-9:
                if not self.low_duration_verified:
                    self.stop("calibration_low_continuous_time_insufficient", wall_s)
                else:
                    self._begin_stop("low", wall_s)
            elif (self.phase == "HIGH_PULSE"
                    and self.measured_forward()-self.high_origin_forward >= .04-1e-9):
                self._begin_stop("high", wall_s)

    def _note_request(self, tier, wall):
        if tier == "low":
            if self.low_duration_verified:
                return
            if self.low_request_start is None:
                self.low_request_start = self._sample(wall)
            self.low_continuous_wall_s = wall-self.started_wall-self.low_request_start["wall_elapsed_s"]
            self.low_continuous_sim_s = self.odom["stamp"]-self.low_request_start["stamp_s"]
            if min(self.low_continuous_wall_s, self.low_continuous_sim_s) >= 1.-1e-9:
                self.low_duration_verified = True
        elif self.high_request_start is None:
            self.high_request_start = self._sample(wall)

    def step(self, *, wall_s):
        with self._lock:
            decision = super().step(wall_s=wall_s)
            if self.state in ("STOP", "FINISHED", "PAUSE"):
                return decision
            if self.phase in ("LOW_STOP", "HIGH_STOP"):
                stop = self.stop_records[-1]
                if stop["first_successful_zero_publish"] is None or not self._stable_dwell(wall_s):
                    return MotionDecision(0., self.state, self.reason)
                anchor = self.stable_anchor
                stop.update(verified=True, path_increment_m=self.total_path_m-stop["path_origin_m"],
                    stationary_wall_s=wall_s-anchor[0], stationary_sim_s=self.odom["stamp"]-anchor[1],
                    verified_sample=self._sample(wall_s))
                if self.phase == "LOW_STOP":
                    self.calibration_stage = "high"
                    self.high_origin_path, self.high_origin_forward = self.total_path_m, self.measured_forward()
                    self._enter_phase("HIGH_PULSE")
                else:
                    self.high_end_path = self.total_path_m
                    self.calibration_stage = "final_slow"
                    self._enter_phase("DRIVE")
            if self.phase == "HIGH_PULSE":
                self._note_request("high", wall_s)
                return MotionDecision(.10, self.state, self.reason)
            if self.phase == "DRIVE" and self.calibration_stage == "low":
                self._note_request("low", wall_s)
            if self.phase == "DRIVE":
                return MotionDecision(.03, self.state, self.reason)
            return decision

    def record_command(self, *, wall_s, linear_x, angular_z):
        with self._lock:
            super().record_command(wall_s=wall_s, linear_x=linear_x, angular_z=angular_z)
            if self.odom is None:
                return
            if linear_x > 0 and self.phase in ("DRIVE", "HIGH_PULSE"):
                tier = self.tiers[self.calibration_stage]
                if tier["first_successful_nonzero_publish"] is None:
                    tier["first_successful_nonzero_publish"] = self._sample(wall_s)
            if (linear_x == angular_z == 0. and self.phase in ("LOW_STOP", "HIGH_STOP")
                    and self.stop_records[-1]["first_successful_zero_publish"] is None):
                self.stop_records[-1]["first_successful_zero_publish"] = self._sample(wall_s) | {
                    "path_increment_m": self.total_path_m-self.stop_records[-1]["path_origin_m"]}

    def approach_evidence(self):
        with self._lock:
            evidence = super().approach_evidence()
            evidence["revision"] = "v19_speed_calibration"
            evidence["speed_calibration"] = {"revision": "v19_speed_calibration",
                "stage": self.calibration_stage, "low_request_start": copy.deepcopy(self.low_request_start),
                "low_continuous_wall_s": self.low_continuous_wall_s,
                "low_continuous_sim_s": self.low_continuous_sim_s,
                "low_duration_verified": self.low_duration_verified,
                "high_request_start": copy.deepcopy(self.high_request_start),
                "high_total_path_m": self._high_path(), "high_trigger_distance_m": .04,
                "high_total_path_limit_m": .05, "high_wall_limit_s": 5., "high_sim_limit_s": 5.,
                "stops": copy.deepcopy(self.stop_records), "tiers": copy.deepcopy(self.tiers),
                "interpretation": "bounded tier/stop observation; sustained route response and official score unverified"}
            return evidence


def create_speed_calibration_node(reporter=None, *, wall_clock=time.monotonic, approach_limits=None):
    def factory(limits, *, started_wall_s):
        return GroundSpeedCalibrationGuard(started_wall_s=started_wall_s, approach_limits=approach_limits)
    return create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5, callback_observer=SpeedCalibrationTimingRecorder(),
        input_reliability_overrides={"/odom": "BEST_EFFORT"})


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="v19 bounded two-tier speed/stop calibration")
    parser.add_argument("--duration", type=float, default=90.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ApproachLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_speed_calibration_node(reporter, approach_limits=limits)
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
