"""Fixed low-height signed-Z identification, separate from all mission control.

The frozen vertical response guard provides all sensor and geometric checks.
This experiment measures trim response; it does not implement height holding.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import os
import time
import types

from .bounded_ground_probe import finite, spin_control_loop
from .bounded_vertical_response import (VerticalDecision, VerticalLimits,
    VerticalResponseGuard, WORLD_SHA256, create_vertical_node)


TRIM_LIMITS = VerticalLimits(target_rise_m=.10, hard_rise_m=.15)
PROTOCOL = (("TRIM_POS_025", .025, 1.), ("TRIM_POS_030", .030, 1.),
            ("TRIM_NEG_010", -.010, .30))
PHASES = {name: (speed, duration) for name, speed, duration in PROTOCOL}


class VerticalTrimGuard(VerticalResponseGuard):
    mode = "bounded_vertical_trim"

    def __init__(self, *, started_wall_s):
        super().__init__(started_wall_s=started_wall_s, limits=TRIM_LIMITS)
        self.phase, self.phase_started_wall = "WAIT", None
        self.phase_transitions, self.phase_samples = [], {}
        self.protocol_complete = self.support_return_verified = False
        self.protocol_abort_reason = ""

    def _budget(self, wall):
        # The one motion clock covers ascent, phase gates, PAUSE and zero return.
        if wall-self.started_wall >= self.limits.global_wall_s-1e-9:
            self.stop("global_wall_time_limit", wall)
        elif (self.motion_started_wall is not None and self.state not in ("STOP", "FINISHED")
              and wall-self.motion_started_wall >= self.limits.motion_wall_s-1e-9):
            self.stop("motion_wall_time_limit", wall)

    def _enter_phase(self, phase, wall, reason):
        previous = self.phase
        self.phase = phase
        self.phase_started_wall = wall
        self.phase_transitions.append({"wall_elapsed_s": wall-self.started_wall,
            "from": previous, "to": phase, "reason": reason,
            "odom_stamp": self.odom["stamp"] if self.odom else None})
        self.phase_samples[phase] = {"entry_odom_stamp": self.odom["stamp"] if self.odom else None,
            "advancing_odom_count": 0, "first_odom_stamp": None, "last_odom_stamp": None,
            "requested_command_count": 0}

    def _zero_return(self, reason, wall, *, completed=False):
        if self.state in ("STOP", "FINISHED"): return
        if self.phase != "ZERO_RETURN":
            self._enter_phase("ZERO_RETURN", wall, reason)
            self.observe_started_wall = wall
            self.stable_anchor = None
            self.protocol_complete = completed
            if not completed: self.protocol_abort_reason = reason
        self.state, self.reason = "OBSERVE", reason

    def pause(self, reason, wall_s):
        with self._lock:
            if not finite(wall_s): return self.stop("pause_wall_invalid", wall_s)
            self._budget(max(wall_s, self.last_observation_wall))
            if self.phase in PHASES and self.state not in ("STOP", "FINISHED"):
                self._zero_return("trim_interrupted:"+str(reason), wall_s)
                return VerticalDecision(0., self.state, self.reason)
            return super().pause(reason, wall_s)

    def _wall(self, wall):
        if not super()._wall(wall): return False
        if (self.state == "ASCEND" and self.phase in PHASES
                and wall-self.phase_started_wall >= PHASES[self.phase][1]-1e-9):
            # The inherited input adapter immediately publishes zero when an
            # active state leaves ASCEND. A subsequent tick may start the next
            # phase only after checking current and advancing odometry.
            self.state, self.reason = "PHASE_GATE", "phase_duration_elapsed_zero"
        return True

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            old_stamp = None if self.odom is None else self.odom["stamp"]
            super().update_odom(payload, wall_s=wall_s)
            if self.odom is None:
                if self.state == "PHASE_GATE": self.stop("trim_gate_odom_invalid", wall_s)
                return
            sample = self.odom
            if sample["stamp"] != old_stamp and self.phase in self.phase_samples:
                record = self.phase_samples[self.phase]
                if record["entry_odom_stamp"] is not None and sample["stamp"] > record["entry_odom_stamp"]:
                    record["advancing_odom_count"] += 1
                    if record["first_odom_stamp"] is None: record["first_odom_stamp"] = sample["stamp"]
                    record["last_odom_stamp"] = sample["stamp"]
            if (self.origin is not None and self.phase in PHASES
                    and sample["xyz"][2]-self.origin[2] <= .035+1e-9):
                self._zero_return("trim_low_height_nonzero_aborted", wall_s)

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            super().update_scan(payload, wall_s=wall_s)
            if self.scan is None and self.state == "PHASE_GATE":
                self.stop("trim_gate_scan_invalid", wall_s)

    def _phase_has_feedback(self):
        record = self.phase_samples[self.phase]
        return record["advancing_odom_count"] > 0 and record["requested_command_count"] > 0

    def step(self, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return VerticalDecision(0., self.state, self.reason)
            self.last_wall = wall_s
            if self.state == "STOP":
                if wall_s-self.stopped_wall >= 2.-1e-9: self.state = "FINISHED"
                return VerticalDecision(0., self.state, self.reason)
            if self.state == "FINISHED": return VerticalDecision(0., self.state, self.reason)
            condition = self._condition(wall_s)
            if condition:
                if self.state in ("ASCEND", "PAUSE", "PHASE_GATE"):
                    self.pause(condition, wall_s)
                elif self.state == "OBSERVE": self.stop("return_sensor_lost:"+condition, wall_s)
                else: self.reason = condition
                self.stable_anchor = None
                return VerticalDecision(0., self.state, self.reason)
            if self.state in ("WAIT", "PAUSE"):
                if not self._stable(wall_s): return VerticalDecision(0., self.state, "waiting_for_stationary_dwell")
                if self.origin is None:
                    self.origin = self.odom["xyz"]
                    self.motion_started_wall = wall_s
                    self._enter_phase("ASCEND_TO_010", wall_s, "initial_pose_stable")
                self.state, self.reason = "ASCEND", "bounded_trim_ascent"
            if self.state == "ASCEND" and self.phase == "ASCEND_TO_010":
                if self.odom["xyz"][2]-self.origin[2] >= self.limits.target_rise_m-1e-9:
                    self.target_reached = True
                    self._enter_phase(PROTOCOL[0][0], wall_s, "measured_target_rise_reached")
                    self.reason = "fixed_trim_response"
                else: return VerticalDecision(.05, self.state, self.reason)
            if self.state == "PHASE_GATE":
                if not self._phase_has_feedback():
                    self._zero_return("trim_insufficient_advancing_feedback_or_requests", wall_s)
                elif self.odom["xyz"][2]-self.origin[2] <= .035+1e-9:
                    self._zero_return("trim_low_height_nonzero_aborted", wall_s)
                else:
                    index = [name for name, _, _ in PROTOCOL].index(self.phase)
                    if index+1 == len(PROTOCOL):
                        self._zero_return("fixed_trim_sequence_completed_zero_return", wall_s, completed=True)
                    else:
                        self._enter_phase(PROTOCOL[index+1][0], wall_s, "phase_duration_and_feedback_verified")
                        self.state, self.reason = "ASCEND", "fixed_trim_response"
            if self.state == "ASCEND" and self.phase in PHASES:
                return VerticalDecision(PHASES[self.phase][0], self.state, self.reason)
            if self.state == "OBSERVE":
                if abs(self.odom["xyz"][2]-self.origin[2]) <= .005+1e-9 and self._stable(wall_s):
                    self.support_return_verified = True
                    self.state = "FINISHED"
                    self.reason = "support_return_verified_trim_complete" if self.protocol_complete else "support_return_verified_trim_incomplete"
                elif abs(self.odom["xyz"][2]-self.origin[2]) > .005+1e-9:
                    self.stable_anchor = None
            return VerticalDecision(0., self.state, self.reason)

    def command_allowed(self, speed):
        if not finite(speed): return False
        if speed == 0.: return True
        expected = .05 if self.phase == "ASCEND_TO_010" else PHASES.get(self.phase, (None,))[0]
        return self.state == "ASCEND" and speed == expected

    def record_command(self, *, wall_s, linear_z):
        with self._lock:
            if not self.command_allowed(linear_z): raise ValueError("signed Z command outside fixed phase")
            self._append("commands", {"wall_elapsed_s": wall_s-self.started_wall, "linear_z": linear_z,
                "ground_linear_x": 0., "ground_angular_z": 0., "state": self.state, "phase": self.phase}, 1200)
            if linear_z and self.phase in self.phase_samples:
                self.phase_samples[self.phase]["requested_command_count"] += 1

    def evidence(self):
        with self._lock:
            data = super().evidence()
            data.update(schema="bounded_vertical_trim_v1", phase=self.phase,
                phase_started_wall=self.phase_started_wall, protocol=[list(x) for x in PROTOCOL],
                protocol_complete=self.protocol_complete, protocol_abort_reason=self.protocol_abort_reason,
                support_return_verified=self.support_return_verified,
                return_policy={"height_tolerance_m": .005, "stationary_s": self.limits.stable_s,
                    "max_speed_mps": .005, "max_angular_speed_rps": .01,
                    "position_dwell_radius_m": .002, "inherited_observe_s_used": False},
                phase_transitions=[dict(x) for x in self.phase_transitions],
                phase_samples={k: dict(v) for k, v in self.phase_samples.items()})
            data["assumptions"]["initial_support_floor"] = "known initial support; signed Z trim only above .035m relative height; zero return is observed, not controlled landing"
            data["assumptions"]["body_world_signed_z_contract"] = "unverified; fixed low-height identification, not hover PID"
            return data

    def diagnostic_events(self):
        # Preserve the frozen, tested compression/receiver contract. Only the
        # diagnostic section names differ, while the full dynamic evidence
        # includes signed commands and all phase metadata.
        events = super().diagnostic_events()
        for _, payload in events:
            payload["section"] = payload["section"].replace("bounded_vertical_", "bounded_vertical_trim_")
            if payload["section"] == "bounded_vertical_trim_core":
                payload.update(phase=self.phase, protocol_complete=self.protocol_complete,
                    support_return_verified=self.support_return_verified,
                    protocol_abort_reason=self.protocol_abort_reason)
        return events


def create_trim_node(reporter=None, *, wall_clock=time.monotonic):
    """Reuse the frozen input/graph/watchdog adapter; explicitly replace two seams."""
    node = create_vertical_node(reporter, wall_clock=wall_clock, limits=TRIM_LIMITS)
    node.guard = VerticalTrimGuard(started_wall_s=wall_clock())
    node.publish_counts["uav_negative_z"] = 0
    from rosidl_runtime_py.utilities import get_message
    Twist = get_message("geometry_msgs/msg/Twist")

    def publish_signed(self, speed):
        with self.guard._lock:
            wall = wall_clock()
            self.guard._wall(wall)
            if speed:
                condition = self.guard._condition(wall)
                if condition: self.guard.pause("publish_boundary:"+condition, wall)
                if not self.guard.command_allowed(speed):
                    speed = 0.
            if not self._attempt_publish("/cmd_vel", Twist(), "ground_zero"):
                speed = 0.
            uav = Twist()
            uav.linear.z = float(speed)
            count = "uav_negative_z" if speed < 0. else "uav_positive_z" if speed > 0. else "uav_zero"
            sent = self._attempt_publish("/uav/cmd_vel", uav, count)
            if not sent and speed:
                speed = 0.
                sent = self._attempt_publish("/uav/cmd_vel", Twist(), "uav_zero")
            if sent: self.guard.record_command(wall_s=wall_clock(), linear_z=speed)

    node._publish = types.MethodType(publish_signed, node)
    return node


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_trim_node(reporter)
        reporter.emit("probe_started", {"mode": node.guard.mode, "read_only": False,
            "limits": asdict(TRIM_LIMITS), "protocol": PROTOCOL, "world_sha256": WORLD_SHA256,
            "official_scoring": "unverified", "full_mission_complete": False})
        node.arm_watchdog()
        spin_control_loop(node, rclpy, duration_s=TRIM_LIMITS.global_wall_s)
        node.stop("trim_experiment_cleanup")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.close_watchdog()
        node.report_final()
    finally:
        if node is not None:
            node.stop("trim_experiment_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
