"""Finite negative-X request response at north-facing yaw; no scoring outputs.

The v28 runtime remains frozen. Only the accumulated rotation allowance grows
to 2rad to reach +pi/2; every other bound and actual transport path is reused.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import math
import os
import time

from .bounded_ground_probe import finite, spin_control_loop
from .ground_direction_response import (DirectionLimits, GroundDirectionResponseGuard,
                                       create_direction_response_node, yaw_from_q)
from .ground_observation_survey import angle_wrap


@dataclass(frozen=True)
class ReverseLimits(DirectionLimits):
    max_rotation_rad: float = 2.

    def __post_init__(self):
        if not finite(self.max_rotation_rad) or not 0 < self.max_rotation_rad <= 2.:
            raise ValueError("reverse rotation allowance exceeds 2rad")
        original = asdict(self)
        original["max_rotation_rad"] = min(self.max_rotation_rad, 1.)
        # Validate every other field and coupled reserve using frozen v28.
        DirectionLimits(**original)


class GroundReverseResponseGuard(GroundDirectionResponseGuard):
    mode = "ground_reverse_response"
    PULSES = ((-.03, 0.), (-.10, 0.), (-.10, 0.))

    def __init__(self, *, started_wall_s, reverse_limits=None, session_id=None):
        limits = ReverseLimits() if reverse_limits is None else reverse_limits
        if not isinstance(limits, ReverseLimits):
            raise ValueError("reverse_limits must use the explicit reverse contract")
        super().__init__(started_wall_s=started_wall_s, direction_limits=limits,
                         session_id=session_id)

    def _enter_phase(self, phase):
        if phase == "TURN":
            self.target_yaw = math.pi / 2
            self.initial_turn_error = angle_wrap(self.target_yaw - yaw_from_q(self.odom["q"]))
        super()._enter_phase(phase)

    def core_evidence(self):
        with self._lock:
            evidence = super().core_evidence()
            evidence["reverse_contract"] = {
                "revision": "ground_reverse_response_v1",
                "target_yaw_rad": math.pi / 2,
                "requested_body_xy_mps": self.PULSES,
                "rotation_allowance_rad": self.direction_limits.max_rotation_rad,
                "actual_reverse_motion_confirmed": False,
                "interpretation": "negative request only; actual direction must be measured from source poses"}
            return evidence


def create_reverse_response_node(reporter=None, *, wall_clock=time.monotonic, reverse_limits=None):
    """Replace the guard before spinning; callbacks/tick resolve node.guard live."""
    limits = ReverseLimits() if reverse_limits is None else reverse_limits
    if not isinstance(limits, ReverseLimits):
        raise ValueError("reverse_limits must use the explicit reverse contract")
    node = create_direction_response_node(reporter, wall_clock=wall_clock)
    original = node.guard
    node.guard = GroundReverseResponseGuard(started_wall_s=original.started_wall,
                                           reverse_limits=limits)
    node.guard.update_endpoints(original.endpoints,
                                matched_subscriptions=original.matched_subscriptions)
    node.experiment_mode = node.guard.mode

    def tick():
        # Keep the frozen v28 code unchanged. Its final DDS query can consume
        # the remaining pulse budget, so this adapter checks again afterwards.
        try:
            matched = node.check_command_match()
            if not matched and node.guard.state not in ("WAIT", "STOP", "FINISHED"):
                node.stop("no_external_matching_cmd_subscriber")
            elif node.last_graph_refresh_wall is None or node.wall_clock()-node.last_graph_refresh_wall >= .5-1e-9:
                node.refresh_endpoints()
        except Exception as exc:
            node.stop("endpoint_refresh_error:"+str(exc)[:128])
        with node.command_lock:
            guard = node.guard
            decision = guard.step(wall_s=node.wall_clock())
            if decision.linear_x or decision.linear_y or decision.angular_z:
                try:
                    if not node.check_command_match():
                        guard.stop("no_external_matching_cmd_subscriber", node.wall_clock())
                except Exception as exc:
                    guard.stop("endpoint_publish_check_error:"+str(exc)[:128], node.wall_clock())
                wall = node.wall_clock()
                hard = guard._budget_fault(wall) or guard.observed_motion_fault()
                if hard:
                    guard.stop(hard, wall)
                condition = guard._condition(wall)
                if condition:
                    guard.temporal_fault(condition, wall)
                translation_ok = not (decision.linear_x or decision.linear_y) or guard.state == guard.phase == "PULSE"
                rotation_ok = not decision.angular_z or guard.state == guard.phase == "TURN"
                if not translation_ok or not rotation_ok or guard.state in ("STOP", "FINISHED", "PAUSE"):
                    decision = guard._zero()
            node.publish_direction(decision)
            if not node.motion_log or (node.motion_log[-1]["state"], node.motion_log[-1]["reason"]) != (decision.state, decision.reason):
                node.motion_log.append({"wall_elapsed_s":node.wall_clock()-guard.started_wall,
                    **asdict(decision), "timing":guard.timing_snapshot(wall_s=node.wall_clock())})
                if len(node.motion_log)>64:
                    node.motion_log.pop(1)
                    node.motion_log_dropped += 1
            return guard._zero() if guard.state in ("STOP", "FINISHED", "PAUSE") else decision

    node.tick = tick
    return node


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="Finite reverse requests; no scoring outputs")
    parser.add_argument("--duration", type=float, default=120.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ReverseLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_reverse_response_node(reporter, reverse_limits=limits)
        node.report_started(parsed.duration)
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("direction_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("reverse_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
