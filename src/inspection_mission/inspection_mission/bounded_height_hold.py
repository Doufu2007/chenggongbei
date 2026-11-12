"""Finite local height feedback inside the declared initial static column.

The trim is a single-run local interpolation, not a verified global model.
No mission, recognition, laser, score or completion messages are published.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import os
import time
import types

from .bounded_ground_probe import finite, spin_control_loop
from .bounded_vertical_response import (VerticalDecision, VerticalLimits,
    VerticalResponseGuard, WORLD_SHA256, create_vertical_node)


@dataclass(frozen=True)
class HeightHoldLimits(VerticalLimits):
    target_rise_m: float = .10
    hard_rise_m: float = .15
    motion_wall_s: float = 25.

    def __post_init__(self):
        # The old experiment's 10s cap stays frozen. Validate inherited bounds
        # there, then independently validate this preapproved fixed budget.
        values = asdict(self)
        values["motion_wall_s"] = 10.
        VerticalLimits(**values)
        if self.motion_wall_s != 25.:
            raise ValueError("height experiment requires fixed 25s motion budget")


HEIGHT_LIMITS = HeightHoldLimits()
ACTIVE_PHASES = frozenset(("ASCEND_TO_010", "HOLD_010", "DESCEND_002"))


def feedback_command(target_z, z):
    if not finite(target_z) or not finite(z): raise ValueError("finite height required")
    wanted_vz = max(-.03, min(.03, .5*(target_z-z)))
    return max(.01, min(.05, .02695+.5*wanted_vz))


class HeightHoldGuard(VerticalResponseGuard):
    mode = "bounded_height_hold"

    def __init__(self, *, started_wall_s):
        super().__init__(started_wall_s=started_wall_s, limits=HEIGHT_LIMITS)
        self.phase, self.phase_started_wall, self.pending_phase = "WAIT", None, None
        self.phase_transitions, self.phase_samples = [], {}
        self.hold_anchor = None
        self.hold_dwell_resets, self.hold_dwell_verified = [], None
        self.hold_verified = self.descent_verified = False
        self.protocol_complete = self.support_return_verified = False
        self.protocol_abort_reason = ""

    def _budget(self, wall):
        if wall-self.started_wall >= self.limits.global_wall_s-1e-9:
            self.stop("global_wall_time_limit", wall)
        elif (self.motion_started_wall is not None and self.state not in ("STOP", "FINISHED")
              and wall-self.motion_started_wall >= self.limits.motion_wall_s-1e-9):
            self.stop("motion_wall_time_limit", wall)

    def _enter_phase(self, phase, wall, reason):
        previous = self.phase
        self.phase, self.phase_started_wall = phase, wall
        self.phase_transitions.append({"wall_elapsed_s":wall-self.started_wall,
            "from":previous, "to":phase, "reason":reason,
            "odom_stamp":self.odom["stamp"] if self.odom else None})
        self.phase_samples[phase] = {"entry_odom_stamp":self.odom["stamp"] if self.odom else None,
            "advancing_odom_count":0, "requested_command_count":0,
            "first_odom_stamp":None, "last_odom_stamp":None}

    def _zero_return(self, reason, wall, *, completed=False):
        if self.state in ("STOP", "FINISHED"): return
        if self.phase != "ZERO_RETURN":
            self._enter_phase("ZERO_RETURN", wall, reason)
            self.observe_started_wall, self.stable_anchor = wall, None
            self.protocol_complete = completed
            if not completed: self.protocol_abort_reason = reason
        self.pending_phase = None
        self.state, self.reason = "OBSERVE", reason

    def pause(self, reason, wall_s):
        with self._lock:
            if not finite(wall_s): return self.stop("pause_wall_invalid", wall_s)
            self._budget(max(wall_s,self.last_observation_wall))
            if self.phase in ACTIVE_PHASES and self.state not in ("STOP", "FINISHED"):
                self._zero_return("height_feedback_interrupted:"+str(reason), wall_s)
            return VerticalDecision(0.,self.state,self.reason)

    def _wall(self, wall):
        if not super()._wall(wall): return False
        if (self.state == "ASCEND" and self.phase == "HOLD_010"
                and wall-self.phase_started_wall >= 8.-1e-9):
            self._zero_return("hold_wall_time_limit", wall)
        return True

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            old_stamp = self.odom["stamp"] if self.odom else None
            super().update_odom(payload, wall_s=wall_s)
            if self.odom is None:
                if self.state == "PHASE_GATE": self.stop("height_gate_odom_invalid", wall_s)
                return
            sample = self.odom
            if sample["stamp"] != old_stamp and self.phase in self.phase_samples:
                record = self.phase_samples[self.phase]
                if record["entry_odom_stamp"] is not None and sample["stamp"] > record["entry_odom_stamp"]:
                    record["advancing_odom_count"] += 1
                    if record["first_odom_stamp"] is None: record["first_odom_stamp"] = sample["stamp"]
                    record["last_odom_stamp"] = sample["stamp"]
            if self.state == "ASCEND" and self.phase == "HOLD_010" and (
                    abs(sample["xyz"][2]-self._target_z()) > .005+1e-9 or abs(sample["linear"][2]) > .01+1e-9):
                # A bad observation cannot disappear behind a good frame before
                # the next control tick and falsely extend continuous dwell.
                self._hold_stable(wall_s)
            self._check_phase_height(wall_s)

    def _check_phase_height(self, wall):
        if self.origin is None or self.odom is None or self.state != "ASCEND": return
        rise = self.odom["xyz"][2]-self.origin[2]
        if self.phase == "HOLD_010" and rise <= .035+1e-9:
            self._zero_return("hold_low_height_nonzero_aborted",wall)
        elif self.phase == "DESCEND_002" and rise <= .025+1e-9:
            self.descent_verified = abs(self.odom["linear"][2]) <= .02+1e-9
            self._zero_return("descent_height_and_speed_verified" if self.descent_verified
                else "descent_low_height_speed_not_verified", wall,
                completed=self.hold_verified and self.descent_verified)

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            super().update_scan(payload, wall_s=wall_s)
            if self.scan is None and self.state == "PHASE_GATE": self.stop("height_gate_scan_invalid",wall_s)

    def _target_z(self):
        if self.origin is None: return None
        return self.origin[2]+(.02 if self.phase == "DESCEND_002" else .10)

    def _requested_z(self):
        if self.state != "ASCEND": return 0.
        if self.phase == "ASCEND_TO_010": return .05
        if self.phase in ("HOLD_010", "DESCEND_002"):
            return feedback_command(self._target_z(), self.odom["xyz"][2])
        return 0.

    def _phase_has_feedback(self):
        row = self.phase_samples[self.phase]
        return row["advancing_odom_count"] > 0 and row["requested_command_count"] > 0

    def _gate(self, pending, reason):
        self.state, self.reason, self.pending_phase = "PHASE_GATE", reason, pending

    def _hold_stable(self, wall):
        good = abs(self.odom["xyz"][2]-self._target_z()) <= .005+1e-9 and abs(self.odom["linear"][2]) <= .01+1e-9
        if not good:
            if self.hold_anchor is not None:
                self.hold_dwell_resets.append({"wall_elapsed_s":wall-self.started_wall,
                    "odom_stamp":self.odom["stamp"], "reason":"height_or_vz_outside_hold_band"})
            self.hold_anchor = None
            self.hold_dwell_verified = None
            return False
        if self.hold_anchor is None:
            self.hold_anchor = (wall,self.odom["stamp"])
        if wall-self.hold_anchor[0] >= 3.-1e-9 and self.odom["stamp"]-self.hold_anchor[1] >= 3.-1e-9:
            self.hold_dwell_verified = {"started_wall_elapsed_s":self.hold_anchor[0]-self.started_wall,
                "ended_wall_elapsed_s":wall-self.started_wall,
                "started_odom_stamp":self.hold_anchor[1], "ended_odom_stamp":self.odom["stamp"]}
            return True
        return False

    def step(self, *, wall_s):
        with self._lock:
            if not self._wall(wall_s): return VerticalDecision(0.,self.state,self.reason)
            self.last_wall = wall_s
            if self.state == "STOP":
                if wall_s-self.stopped_wall >= 2.-1e-9: self.state = "FINISHED"
                return VerticalDecision(0.,self.state,self.reason)
            if self.state == "FINISHED": return VerticalDecision(0.,self.state,self.reason)
            condition = self._condition(wall_s)
            if condition:
                if self.state in ("ASCEND", "PHASE_GATE"): self.pause(condition,wall_s)
                elif self.state == "OBSERVE": self.stop("return_sensor_lost:"+condition,wall_s)
                else: self.reason = condition
                self.stable_anchor = None
                return VerticalDecision(0.,self.state,self.reason)
            if self.state == "WAIT":
                if not self._stable(wall_s): return VerticalDecision(0.,self.state,"waiting_for_stationary_dwell")
                self.origin, self.motion_started_wall = self.odom["xyz"],wall_s
                self._enter_phase("ASCEND_TO_010",wall_s,"initial_pose_stable")
                self.state,self.reason = "ASCEND","bounded_height_ascent"
            elif self.state == "PHASE_GATE":
                if not self._phase_has_feedback():
                    self._zero_return("height_insufficient_advancing_feedback_or_requests",wall_s)
                else:
                    pending = self.pending_phase
                    self.pending_phase = None
                    self._enter_phase(pending,wall_s,"phase_feedback_verified_after_zero")
                    self.state,self.reason = "ASCEND","bounded_local_height_feedback"
            self._check_phase_height(wall_s)
            if self.state == "ASCEND":
                if self.phase == "ASCEND_TO_010" and self.odom["xyz"][2]-self.origin[2] >= .10-1e-9:
                    self.target_reached = True
                    self._gate("HOLD_010","measured_ascent_target_zero_gate")
                elif self.phase == "HOLD_010" and self._hold_stable(wall_s):
                    self.hold_verified = True
                    self._gate("DESCEND_002","actual_hold_dwell_verified_zero_gate")
            if self.state == "OBSERVE":
                if abs(self.odom["xyz"][2]-self.origin[2]) <= .005+1e-9 and self._stable(wall_s):
                    self.support_return_verified = True
                    self.state = "FINISHED"
                    self.reason = "support_return_verified_height_protocol_complete" if self.protocol_complete else "support_return_verified_height_protocol_incomplete"
                elif abs(self.odom["xyz"][2]-self.origin[2]) > .005+1e-9:
                    self.stable_anchor = None
            return VerticalDecision(self._requested_z(),self.state,self.reason)

    def command_allowed(self, speed):
        return finite(speed) and (speed == 0. or (self.state == "ASCEND" and self.phase in ACTIVE_PHASES
            and .01 <= speed <= .05 and abs(speed-self._requested_z()) <= 1e-12))

    def record_command(self, *, wall_s, linear_z):
        with self._lock:
            if not self.command_allowed(linear_z): raise ValueError("height command outside current feedback phase")
            target = self._target_z() if self.phase in ACTIVE_PHASES else None
            self._append("commands",{"wall_elapsed_s":wall_s-self.started_wall,"linear_z":linear_z,
                "ground_linear_x":0.,"ground_angular_z":0.,"state":self.state,"phase":self.phase,
                "target_z":target,"measured_z":self.odom["xyz"][2] if self.odom else None,
                "measured_vz":self.odom["linear"][2] if self.odom else None,
                "height_error_m":target-self.odom["xyz"][2] if target is not None and self.odom else None,
                "odom_stamp":self.odom["stamp"] if self.odom else None},1200)
            if linear_z: self.phase_samples[self.phase]["requested_command_count"] += 1

    def evidence(self):
        with self._lock:
            data = super().evidence()
            data.update(schema="bounded_height_hold_v1",phase=self.phase,pending_phase=self.pending_phase,
                phase_started_wall=self.phase_started_wall,phase_transitions=[dict(x) for x in self.phase_transitions],
                phase_samples={k:dict(v) for k,v in self.phase_samples.items()},
                hold_verified=self.hold_verified,descent_verified=self.descent_verified,
                hold_dwell_verified=self.hold_dwell_verified,hold_dwell_resets=list(self.hold_dwell_resets),
                protocol_complete=self.protocol_complete,protocol_abort_reason=self.protocol_abort_reason,
                support_return_verified=self.support_return_verified,
                feedback={"trim":.02695,"height_to_vz_gain":.5,"inverse_local_gain":.5,
                    "wanted_vz_limit":.03,"request_range":[.01,.05],"hold_rise_m":.10,"descent_rise_m":.02,
                    "model_status":"single_run_local_interpolation_not_global_or_verified_hover"},
                forward_cloud_control_used=False,forward_cloud_collection="not_attached")
            data["assumptions"]["initial_support_floor"] = "known initial support; positive-trim descent; final zero return observed, not independent touchdown"
            data["assumptions"]["body_world_positive_z_contract"] = "local near-level initial-column experiment only"
            return data

    def diagnostic_events(self):
        events = super().diagnostic_events()
        for _,data in events:
            data["section"] = data["section"].replace("bounded_vertical_","bounded_height_hold_")
            if data["section"] == "bounded_height_hold_core":
                data.update(phase=self.phase,hold_verified=self.hold_verified,descent_verified=self.descent_verified,
                    protocol_complete=self.protocol_complete,support_return_verified=self.support_return_verified,
                    protocol_abort_reason=self.protocol_abort_reason)
        return events


def create_height_node(reporter=None, *, wall_clock=time.monotonic):
    node = create_vertical_node(reporter,wall_clock=wall_clock,limits=HEIGHT_LIMITS)
    node.guard = HeightHoldGuard(started_wall_s=wall_clock())
    from rosidl_runtime_py.utilities import get_message
    Twist = get_message("geometry_msgs/msg/Twist")

    def publish_feedback(self,speed):
        with self.guard._lock:
            wall = wall_clock()
            self.guard._wall(wall)
            if speed:
                condition = self.guard._condition(wall)
                if condition: self.guard.pause("publish_boundary:"+condition,wall)
                if not self.guard.command_allowed(speed):
                    if self.guard.state == "ASCEND": self.guard.pause("feedback_request_changed_at_publish",wall)
                    speed = 0.
            if not self._attempt_publish("/cmd_vel",Twist(),"ground_zero"): speed = 0.
            if speed:
                try:
                    matches = self.publisher_by_topic["/uav/cmd_vel"].get_subscription_count()
                    self.guard.update_endpoints(self.endpoint_descriptions,matched_subscriptions=matches)
                except Exception as exc:
                    self.errors.append("uav_publish_match_query_failed:"+type(exc).__name__+":"+str(exc)[:128])
                    self.guard.stop("uav_publish_match_query_failed:"+type(exc).__name__,wall_clock())
                    speed = 0.
                wall = wall_clock()
                self.guard._wall(wall)
                condition = self.guard._condition(wall)
                if condition: self.guard.pause("publish_boundary_after_ground:"+condition,wall)
                if not self.guard.command_allowed(speed):
                    if self.guard.state == "ASCEND": self.guard.pause("feedback_request_changed_after_ground",wall)
                    speed = 0.
            uav = Twist(); uav.linear.z = float(speed)
            sent = self._attempt_publish("/uav/cmd_vel",uav,"uav_positive_z" if speed else "uav_zero")
            if not sent and speed:
                speed = 0.
                sent = self._attempt_publish("/uav/cmd_vel",Twist(),"uav_zero")
            if sent: self.guard.record_command(wall_s=wall_clock(),linear_z=speed)

    node._publish = types.MethodType(publish_feedback,node)
    return node


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report-url",default=os.environ.get("INSPECTION_DIAGNOSTIC_URL",""))
    parsed,ros_args = parser.parse_known_args(args)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_height_node(reporter)
        reporter.emit("probe_started",{"mode":node.guard.mode,"read_only":False,"limits":asdict(HEIGHT_LIMITS),
            "world_sha256":WORLD_SHA256,"official_scoring":"unverified","full_mission_complete":False})
        node.arm_watchdog()
        spin_control_loop(node,rclpy,duration_s=HEIGHT_LIMITS.global_wall_s)
        node.stop("height_experiment_cleanup")
        for _ in range(5):
            node.publish_zero(); rclpy.spin_once(node,timeout_sec=.02)
        node.close_watchdog(); node.report_final()
    finally:
        if node is not None:
            node.stop("height_experiment_cleanup"); node.close_watchdog(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        reporter.close()


if __name__ == "__main__": main()
