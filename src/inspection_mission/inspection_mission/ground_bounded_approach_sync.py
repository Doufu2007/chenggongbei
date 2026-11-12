"""v16 timing variant: retain only fresh scans with real preceding odometry.

The v15 motion state machine and limits are inherited unchanged. A newer valid
scan waits briefly for its actual prior pose while the last fresh pair remains
usable. Original message stamps and arrival times are never rewritten.
"""
from __future__ import annotations

import argparse
import copy
import os
import time

from .bounded_ground_probe import GroundMotionGuard, MotionLimits, _json_safe, create_bounded_node, spin_control_loop
from .ground_bounded_approach import ApproachLimits, GroundBoundedApproachGuard


class GroundSynchronizedApproachGuard(GroundBoundedApproachGuard):
    PENDING_CAP = 4
    TRACE_CAP = 12

    def __init__(self, *, started_wall_s, approach_limits=None):
        super().__init__(started_wall_s=started_wall_s, approach_limits=approach_limits)
        # A WAIT-only parser applies the complete original ring/return/clearance
        # validation before an incoming scan can enter the pairing queue.
        self._scan_validator = GroundMotionGuard(self.limits, started_wall_s=started_wall_s)
        self._pending_scans = []
        self._defer_odom_pairing = False
        self._sync_counts = {"incoming_scans": 0, "immediate_promotions": 0,
            "late_prior_promotions": 0, "deferred_existing_prior_promotions": 0,
            "expired_pending": 0, "capacity_dropped": 0,
            "superseded_pending": 0, "promotion_trace_dropped": 0}
        self._promotion_trace = []

    def _check_active_timing(self, wall):
        # The parent odom callback holds the guard lock until validation and
        # observation recording finish. Check the complete condition atomically
        # after a newly validated pose has had a chance to pair a pending scan.
        if not self._defer_odom_pairing:
            super()._check_active_timing(wall)

    def _expire_pending(self, wall):
        kept = []
        for candidate in self._pending_scans:
            sample = candidate["scan"]
            expired = (wall-sample["wall"] > self.limits.sensor_age_s+1e-9
                       or (self.clock is not None
                           and self.clock[0]-sample["stamp"] > self.limits.sensor_age_s+1e-9))
            if expired:
                self._sync_counts["expired_pending"] += 1
            else:
                kept.append(candidate)
        self._pending_scans = kept

    def _promote_pending(self, wall, *, late):
        if self.state in ("STOP", "FINISHED") or not self._observations.epoch_valid:
            self._pending_scans.clear()
            return
        self._expire_pending(wall)
        for candidate in reversed(self._pending_scans):
            sample = candidate["scan"]
            if not self._fresh(sample, wall):
                continue
            # Strictly preceding means a future pose is never borrowed, even
            # by a sub-nanosecond tolerance. Only the skew limit has tolerance.
            prior = next((row for row in reversed(self._observations.full_odom_history)
                          if row["stamp_s"] <= sample["stamp"]), None)
            if prior is None or not 0 <= sample["stamp"]-prior["stamp_s"] <= .05+1e-9:
                continue
            self.scan = copy.deepcopy(sample)
            self.scan_anchor = {"stamp_s": sample["stamp"], "wall_s": sample["wall"],
                "odom": copy.deepcopy(prior), "scan_minus_odom_s": sample["stamp"]-prior["stamp_s"]}
            self.latest_scan_payload = candidate["payload"]
            late_prior = prior["wall_elapsed_s"] > sample["wall"]-self.started_wall
            key = ("late_prior_promotions" if late_prior else
                   "deferred_existing_prior_promotions" if late else "immediate_promotions")
            self._sync_counts[key] += 1
            self._promotion_trace.append({"scan_stamp_s": sample["stamp"],
                "scan_arrival_wall_elapsed_s": sample["wall"]-self.started_wall,
                "latest_odom_stamp_at_scan_arrival_s": candidate["odom_stamp_at_arrival"],
                "prior_odom_stamp_s": prior["stamp_s"],
                "prior_odom_arrival_wall_elapsed_s": prior["wall_elapsed_s"],
                "scan_minus_odom_s": sample["stamp"]-prior["stamp_s"],
                "pairing_wall_elapsed_s": wall-self.started_wall, "late_prior": late_prior,
                "scan_original_wall_age_s": wall-sample["wall"],
                "clock_minus_scan_s": self.clock[0]-sample["stamp"]})
            if len(self._promotion_trace) > self.TRACE_CAP:
                self._promotion_trace.pop(0)
                self._sync_counts["promotion_trace_dropped"] += 1
            retained = [row for row in self._pending_scans if row["scan"]["stamp"] > sample["stamp"]]
            self._sync_counts["superseded_pending"] += len(self._pending_scans)-len(retained)-1
            self._pending_scans = retained
            if self.drive_origin is not None and self.measured_forward() >= .10:
                self._record_scan("mid_drive", wall)
            return

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            if self.state in ("STOP", "FINISHED"):
                self._pending_scans.clear()
                return
            if not self._observation_wall("scan", wall_s) or self.state in ("STOP", "FINISHED"):
                self._pending_scans.clear()
                return
            self._sync_counts["incoming_scans"] += 1
            self._scan_validator.update_scan(payload, wall_s=wall_s)
            sample = self._scan_validator.scan
            if sample is None or self._scan_validator.state in ("STOP", "FINISHED"):
                self._pending_scans.clear()
                self.stop("scan_invalid:"+self._scan_validator.reason, wall_s)
                return
            self.last_sensor_stamp["scan"] = sample["stamp"]
            self._expire_pending(wall_s)
            self._pending_scans.append({"scan": copy.deepcopy(sample),
                "payload": _json_safe(copy.deepcopy(payload)),
                "odom_stamp_at_arrival": None if self.odom is None else self.odom["stamp"]})
            if len(self._pending_scans) > self.PENDING_CAP:
                self._pending_scans.pop(0)
                self._sync_counts["capacity_dropped"] += 1
            self._promote_pending(wall_s, late=False)
            self._check_active_timing(wall_s)

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            self._defer_odom_pairing = True
            try:
                super().update_odom(payload, wall_s=wall_s)
            finally:
                self._defer_odom_pairing = False
            self._promote_pending(wall_s, late=True)
            self._check_active_timing(wall_s)

    def update_clock(self, stamp_s, *, wall_s):
        with self._lock:
            super().update_clock(stamp_s, wall_s=wall_s)
            if self.state in ("STOP", "FINISHED"):
                self._pending_scans.clear()
            else:
                self._expire_pending(wall_s)

    def sync_diagnostics(self):
        with self._lock:
            return {**self._sync_counts, "pending_count": len(self._pending_scans),
                "pending_capacity": self.PENDING_CAP, "promotion_trace_capacity": self.TRACE_CAP,
                "promotion_trace": copy.deepcopy(self._promotion_trace),
                "policy": "original scan stamp/arrival; actual odom stamp <= scan; skew <=0.05s; freshness unchanged"}

    def approach_evidence(self):
        with self._lock:
            return {**super().approach_evidence(), "revision": "v16_sensor_sync_graph_throttle",
                "sensor_sync": self.sync_diagnostics()}


def create_sync_approach_node(reporter=None, *, wall_clock=time.monotonic, approach_limits=None):
    def factory(limits, *, started_wall_s):
        return GroundSynchronizedApproachGuard(started_wall_s=started_wall_s, approach_limits=approach_limits)
    return create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5)


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="v16 bounded ground approach with original-time sensor pairing")
    parser.add_argument("--duration", type=float, default=90.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ApproachLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_sync_approach_node(reporter, approach_limits=limits)
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
