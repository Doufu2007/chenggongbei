"""v18: change only /odom requested reliability; retain v17 timing and motion."""
from __future__ import annotations

import argparse
import os
import time

from .bounded_ground_probe import MotionLimits, create_bounded_node, spin_control_loop
from .ground_bounded_approach import ApproachLimits
from .ground_bounded_approach_sync import GroundSynchronizedApproachGuard
from .ground_callback_timing import CallbackTimingRecorder


class OdomQosTimingRecorder(CallbackTimingRecorder):
    revision = "v18_odom_best_effort"


def create_odom_qos_approach_node(reporter=None, *, wall_clock=time.monotonic, approach_limits=None):
    def factory(limits, *, started_wall_s):
        return GroundSynchronizedApproachGuard(started_wall_s=started_wall_s, approach_limits=approach_limits)
    return create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5, callback_observer=OdomQosTimingRecorder(),
        input_reliability_overrides={"/odom": "BEST_EFFORT"})


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="v18 /odom BEST_EFFORT; unchanged v17 bounded approach/timing")
    parser.add_argument("--duration", type=float, default=90.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ApproachLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_odom_qos_approach_node(reporter, approach_limits=limits)
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
