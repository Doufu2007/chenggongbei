"""Finite south preposition and negative-X response; no scoring authority.

The v28/v30 modules remain frozen. Current scan, timing, DDS and measured
motion gates decide each output; the public candidate map is not live truth.
"""
from __future__ import annotations

import argparse
import base64
import copy
from dataclasses import asdict, dataclass, fields
import gzip
import hashlib
import json
import math
import os
import time

from .bounded_ground_probe import MotionLimits, _json_safe, create_bounded_node, finite, spin_control_loop
from .ground_direction_response import (DirectionDecision, DirectionLimits,
                                       GroundDirectionResponseGuard, response_from_odom, yaw_from_q)
from .ground_observation_survey import angle_wrap


@dataclass(frozen=True)
class RelocatedReverseLimits(DirectionLimits):
    global_wall_s: float = 180.
    max_path_m: float = .35
    max_origin_displacement_m: float = .25
    max_rotation_rad: float = 5.
    total_turn_wall_s: float = 120.
    preposition_wall_s: float = 40.
    preposition_distance_m: float = .15
    preposition_request_early_m: float = .002
    preposition_endpoint_error_m: float = .02
    preposition_command_mps: float = .03
    preposition_cross_track_m: float = .02
    preposition_reverse_m: float = .005

    def __post_init__(self):
        caps = {"global_wall_s":180., "max_path_m":.35, "max_origin_displacement_m":.25,
                "max_rotation_rad":5., "total_turn_wall_s":120., "preposition_wall_s":40.,
                "preposition_distance_m":.15, "preposition_request_early_m":.002,
                "preposition_endpoint_error_m":.02, "preposition_command_mps":.03,
                "preposition_cross_track_m":.02, "preposition_reverse_m":.005}
        for name, cap in caps.items():
            if not finite(getattr(self, name)) or not 0 < getattr(self, name) <= cap:
                raise ValueError("relocated reverse contract exceeded: " + name)
        if (self.preposition_request_early_m >= self.preposition_distance_m
                or self.preposition_request_early_m+self.stop_drift_m > self.preposition_endpoint_error_m
                or self.preposition_wall_s > self.global_wall_s):
            raise ValueError("inconsistent relocated preposition bounds")
        frozen = {f.name:getattr(self, f.name) for f in fields(DirectionLimits)}
        for name, cap in {"global_wall_s":120., "max_path_m":.18,
                          "max_origin_displacement_m":.12, "max_rotation_rad":1.}.items():
            frozen[name] = min(frozen[name], cap)
        DirectionLimits(**frozen)  # All original local/pulse/sensor bounds are unchanged.


class GroundRelocatedReverseGuard(GroundDirectionResponseGuard):
    mode = "ground_relocated_reverse"
    wall_limit_reason = "relocated_global_wall_time_limit"
    ACTIVE = frozenset(("TURN", "TURN_SETTLE", "PREPOSITION", "PREPOSITION_SETTLE",
                        "PULSE", "PULSE_SETTLE", "PAUSE"))
    ZERO_PHASES = frozenset(("TURN_SETTLE", "PREPOSITION_SETTLE", "PULSE_SETTLE"))
    PULSES = ((-.03,0.),(-.10,0.),(-.10,0.))

    def __init__(self, *, started_wall_s, relocated_limits=None, session_id=None):
        limits = RelocatedReverseLimits() if relocated_limits is None else relocated_limits
        if not isinstance(limits, RelocatedReverseLimits):
            raise ValueError("explicit relocated reverse contract required")
        # Initialize fields used by dynamically dispatched inherited validators.
        self.stage_index = 0
        self.turn_records = []
        self.completed_turn_wall_s = 0.
        self.preposition_started_wall = self.preposition_origin = self.preposition_target = None
        self.preposition_record = None
        self.rejected_scan = None
        super().__init__(started_wall_s=started_wall_s, direction_limits=limits, session_id=session_id)

    def total_turn_elapsed_s(self, wall):
        if self.state in ("STOP", "FINISHED") and self.stopped_wall is not None:
            wall = min(wall, self.stopped_wall)
        active = (self.turn_started_wall is not None and self.turn_records
                  and "settled" not in self.turn_records[-1])
        return self.completed_turn_wall_s + (max(0., wall-self.turn_started_wall) if active else 0.)

    def _budget_fault(self, wall):
        fault = super()._budget_fault(wall)
        if fault:
            return fault
        if self.total_turn_elapsed_s(wall) >= self.direction_limits.total_turn_wall_s-1e-9:
            return "relocated_total_turn_wall_time_limit"
        if (self.phase in ("PREPOSITION", "PREPOSITION_SETTLE")
                and self.preposition_started_wall is not None
                and wall-self.preposition_started_wall >= self.direction_limits.preposition_wall_s-1e-9):
            return "relocated_preposition_wall_time_limit"
        return ""

    def preposition_progress_m(self):
        if self.odom is None or self.preposition_origin is None:
            return None
        return self.preposition_origin[1]-self.odom["xyz"][1]

    def observed_motion_fault(self):
        fault = super().observed_motion_fault()
        if fault or self.odom is None:
            return fault
        if self.phase in ("PREPOSITION", "PREPOSITION_SETTLE") and self.preposition_origin is not None:
            a = self.direction_limits
            if abs(angle_wrap(yaw_from_q(self.odom["q"])+math.pi/2)) > a.max_heading_error_rad:
                return "relocated_preposition_heading_limit"
            if abs(self.odom["xyz"][0]-self.preposition_origin[0]) > a.preposition_cross_track_m:
                return "relocated_preposition_cross_track_limit"
            if self.preposition_progress_m() < -a.preposition_reverse_m:
                return "relocated_preposition_reverse_limit"
            if self.preposition_progress_m() > a.preposition_distance_m+a.preposition_endpoint_error_m:
                return "relocated_preposition_overshoot_limit"
        return ""

    def update_scan(self, payload, *, wall_s):
        with self._lock:
            terminal_before = self.state in ("STOP", "FINISHED")
            super().update_scan(payload, wall_s=wall_s)
            if (not terminal_before and self.state == "STOP" and "scan" in self.reason
                    and self.rejected_scan is None):
                self.rejected_scan = {"wall_elapsed_s":wall_s-self.started_wall,
                    "reason":self.reason, "phase":self.phase, "stage_index":self.stage_index,
                    "payload":copy.deepcopy(payload), "raw_prior_received_odom":copy.deepcopy(self.last_raw_odom),
                    "interpretation":"first rejected frame only; latest received odom is not automatically a valid source-prior pair"}

    def update_odom(self, payload, *, wall_s):
        with self._lock:
            super().update_odom(payload, wall_s=wall_s)
            if (self.state in self.ACTIVE and self.phase == "PREPOSITION" and self.odom is not None
                    and self.preposition_progress_m() >= self.direction_limits.preposition_distance_m
                        -self.direction_limits.preposition_request_early_m-1e-9):
                self._enter_phase("PREPOSITION_SETTLE")

    def _begin_turn(self, target, wall, stage):
        self.stage_index = stage
        self.turn_origin = self.odom["xyz"]
        self.target_yaw = target
        self.initial_turn_error = angle_wrap(target-yaw_from_q(self.odom["q"]))
        self.turn_started_wall = wall
        self.zero_request_path = None
        self.turn_records.append({"index":len(self.turn_records), "stage_index":stage,
            "target_yaw_rad":target, "start_wall_elapsed_s":wall-self.started_wall,
            "start_raw_odom":copy.deepcopy(self.last_raw_odom), "start_path_m":self.total_path_m,
            "start_rotation_rad":self.accumulated_rotation_rad})
        self._enter_phase("TURN")

    def _settled(self, wall):
        anchor = self.stable_anchor
        return {"wall_elapsed_s":wall-self.started_wall, "raw_odom":copy.deepcopy(self.last_raw_odom),
                "stable_wall_s":wall-anchor[0], "stable_source_s":self.odom["stamp"]-anchor[1],
                "stop_path_increment_m":self.total_path_m-self.zero_request_path}

    def _finish_turn(self, wall):
        elapsed = wall-self.turn_started_wall
        self.turn_records[-1].update(settled=self._settled(wall), elapsed_s=elapsed)
        self.completed_turn_wall_s += elapsed
        self.turn_started_wall = None

    def _begin_preposition(self, wall):
        self.stage_index = 1
        self.preposition_origin = self.odom["xyz"]
        self.preposition_target = (self.preposition_origin[0],
                                   self.preposition_origin[1]-self.direction_limits.preposition_distance_m,
                                   self.preposition_origin[2])
        self.preposition_started_wall = wall
        self.zero_request_path = None
        self.preposition_record = {"start_wall_elapsed_s":wall-self.started_wall,
            "start_raw_odom":copy.deepcopy(self.last_raw_odom), "target_xyz_m":self.preposition_target,
            "start_path_m":self.total_path_m, "requested_body_x_mps":self.direction_limits.preposition_command_mps,
            "measurement":"world south progress from actual segment-start pose; no assumed command gain"}
        self._enter_phase("PREPOSITION")

    def step(self, *, wall_s):
        with self._lock:
            if not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall):
                return self._stop_decision("wall_clock_reversed_or_invalid", wall_s)
            self.last_wall = wall_s
            if self.state in ("STOP", "FINISHED"):
                if wall_s-self.stopped_wall >= self.limits.stop_wall_s:
                    self.state = "FINISHED"
                return self._zero()
            fault = self._budget_fault(wall_s) or self.observed_motion_fault()
            if fault:
                return self._stop_decision(fault, wall_s)
            fault = self._condition(wall_s)
            if fault:
                self.stable_anchor = None
                if self.state in self.ACTIVE:
                    self.temporal_fault(fault, wall_s)
                else:
                    self.reason = fault
                return self._zero()
            if self.state == "PAUSE":
                if not self._stable_dwell(wall_s):
                    return self._zero()
                if self.resume_count >= 3:
                    return self._stop_decision("pause_resume_limit", wall_s)
                self.resume_count += 1
                self.state = self.phase
                self.stable_anchor = None
            elif self.state == "WAIT":
                if not self._stable_dwell(wall_s):
                    return self._zero()
                if math.hypot(*self.odom["xyz"][:2]) > .05 or abs(yaw_from_q(self.odom["q"])) > .05:
                    return self._stop_decision("direction_initial_pose_outside_contract", wall_s)
                self.global_origin = self.pulse_origin = self.odom["xyz"]
                self.previous_path_xyz = self.global_origin
                self.previous_yaw = self._derived_previous_yaw = yaw_from_q(self.odom["q"])
                self.first_motion_wall = wall_s
                self._record_scan("relocated_start", wall_s)
                self._begin_turn(-math.pi/2, wall_s, 0)

            if self.phase == "TURN":
                error = angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
                if abs(error) <= .025 or error*self.initial_turn_error < 0:
                    if abs(error) > self.direction_limits.max_heading_error_rad:
                        return self._stop_decision("direction_turn_overshoot_limit", wall_s)
                    self._enter_phase("TURN_SETTLE")
                else:
                    self.zero_request_path = None
                    rate = math.copysign(min(self.direction_limits.maximum_yaw_rate,
                        max(self.direction_limits.minimum_yaw_rate, .6*abs(error))), error)
                    return DirectionDecision(0.,0.,rate,self.state,self.reason)
            if self.phase == "TURN_SETTLE":
                if not self._stable_dwell(wall_s):
                    return self._zero()
                self._finish_turn(wall_s)
                if self.stage_index == 0:
                    self._begin_preposition(wall_s)
                elif self.stage_index == 1:
                    self._begin_turn(math.pi/2, wall_s, 2)
                else:
                    self.stage_index = 3
                    self._begin_pulse(wall_s)
                return self._zero()  # Every handoff has a real zero command.
            if self.phase == "PREPOSITION":
                self.zero_request_path = None
                return DirectionDecision(self.direction_limits.preposition_command_mps,0.,0.,self.state,self.reason)
            if self.phase == "PREPOSITION_SETTLE":
                if not self._stable_dwell(wall_s):
                    return self._zero()
                error = math.dist(self.odom["xyz"], self.preposition_target)
                settled = self._settled(wall_s)
                settled.update(endpoint_error_m=error, south_progress_m=self.preposition_progress_m())
                self.preposition_record["settled"] = settled
                if error > self.direction_limits.preposition_endpoint_error_m:
                    return self._stop_decision("relocated_preposition_endpoint_error", wall_s)
                self._begin_turn(0., wall_s, 1)
                return self._zero()
            if self.phase == "PULSE_SETTLE":
                if not self._stable_dwell(wall_s):
                    return self._zero()
                record = self.pulse_records[self.pulse_index]
                record["settled"] = self._settled(wall_s)
                record["response"] = response_from_odom(record["start"]["raw_odom"], self.last_raw_odom)
                self._record_scan("relocated_pulse_"+str(self.pulse_index), wall_s)
                if self.pulse_index == 2:
                    return self._stop_decision("relocated_reverse_response_observed_official_score_unverified", wall_s)
                self._begin_pulse(wall_s)
                return self._zero()
            if self.phase == "PULSE":
                self.zero_request_path = None
                x,y = self.PULSES[self.pulse_index]
                return DirectionDecision(x,y,0.,self.state,self.reason)
            return self._zero()

    def core_evidence(self):
        with self._lock:
            core = super().core_evidence()
            core.update(stage_index=self.stage_index, turn_records=copy.deepcopy(self.turn_records),
                completed_turn_wall_s=self.completed_turn_wall_s,
                total_turn_elapsed_s=self.total_turn_elapsed_s(max(self.last_wall,self.last_observation_wall)),
                preposition_record=copy.deepcopy(self.preposition_record), rejected_scan=copy.deepcopy(self.rejected_scan),
                relocated_contract={"revision":"ground_relocated_reverse_v1", "candidate_map_status":"candidate_unverified",
                    "target_yaws_rad":[-math.pi/2,0.,math.pi/2], "requested_negative_body_xy_mps":self.PULSES,
                    "actual_reverse_motion_confirmed":False,
                    "time_budget_interpretation":"failure deadlines; completion duration has not been verified"})
            return core

    def additional_events(self):
        with self._lock:
            core = self.core_evidence()
            body = {**core,"schema":"ground_relocated_reverse_evidence_v1",
                "raw_odometry":copy.deepcopy(self.raw_odometry), "raw_scans":copy.deepcopy(self.raw_scans),
                "commands":copy.deepcopy(self.direction_commands), "state_records":copy.deepcopy(self.state_records),
                "zero_requests":copy.deepcopy(self.zero_requests)}
        raw = json.dumps(_json_safe(body),ensure_ascii=False,separators=(",",":"),allow_nan=False).encode("utf-8")
        packed = gzip.compress(raw,mtime=0)
        encoded = base64.b64encode(packed).decode("ascii")
        chunks = [encoded[i:i+48000] for i in range(0,len(encoded),48000)]
        meta = {"mode":self.mode,"session_id":self.session_id,"read_only":False,"official_scoring":"unverified",
            "section":"ground_relocated_reverse_evidence_gzip_part","encoding":"gzip+base64",
            "bundle":"ground_relocated_reverse_v1","uncompressed_bytes":len(raw),"compressed_bytes":len(packed),
            "uncompressed_sha256":hashlib.sha256(raw).hexdigest(),"compressed_sha256":hashlib.sha256(packed).hexdigest(),
            "part_count":len(chunks)}
        return [("probe_snapshot",{"mode":self.mode,"section":"ground_relocated_reverse_core","data":core})]+[
            ("probe_snapshot",{**meta,"part_index":i,"base64_part":chunk}) for i,chunk in enumerate(chunks)]


def create_relocated_reverse_node(reporter=None, *, wall_clock=time.monotonic, relocated_limits=None):
    from geometry_msgs.msg import Twist
    limits = RelocatedReverseLimits() if relocated_limits is None else relocated_limits
    if not isinstance(limits, RelocatedReverseLimits):
        raise ValueError("explicit relocated reverse contract required")
    def factory(unused, *, started_wall_s):
        return GroundRelocatedReverseGuard(started_wall_s=started_wall_s, relocated_limits=limits)
    node = create_bounded_node(MotionLimits(.447,.114,.02),reporter,wall_clock=wall_clock,
        guard_factory=factory,graph_refresh_interval_s=.5,
        input_reliability_overrides={"/odom":"BEST_EFFORT","/uav/odom":"BEST_EFFORT"})

    def authorized(decision):
        g = node.guard
        x,y,w = decision.linear_x,decision.linear_y,decision.angular_z
        if not all(finite(v) for v in (x,y,w)) or y != 0. or (x and w):
            return False
        if not (x or w):
            return True
        if g.state != g.phase:
            return False
        if w:
            return g.phase == "TURN" and abs(w) <= .5+1e-9 and w*angle_wrap(g.target_yaw-yaw_from_q(g.odom["q"])) > 0.
        if g.phase == "PREPOSITION":
            return x == g.direction_limits.preposition_command_mps
        return g.phase == "PULSE" and 0 <= g.pulse_index < 3 and (x,y) == g.PULSES[g.pulse_index]

    def emit(decision):
        g = node.guard
        x,y,w = decision.linear_x,decision.linear_y,decision.angular_z
        if not authorized(decision):
            g.stop("relocated_output_bound_or_phase_violation",node.wall_clock())
            x=y=w=0.
        if g.state in ("STOP","FINISHED","PAUSE"):
            x=y=w=0.
        publisher = node.publisher_by_topic.get("/cmd_vel")
        if publisher is None:
            g.stop("relocated_cmd_publisher_missing",node.wall_clock())
            return False
        def attempt(ax,ay,aw):
            msg=Twist();msg.linear.x=ax;msg.linear.y=ay;msg.angular.z=aw
            try:
                publisher.publish(msg)
                g.record_command(wall_s=node.wall_clock(),linear_x=ax,linear_y=ay,angular_z=aw,sent=True)
                if ax or ay or aw:
                    g.published_nonzero_count+=1;node.arm_watchdog()
                else:
                    g.published_zero_count+=1
                return True
            except Exception as exc:
                reason="relocated_publish_error:"+str(exc)[:128]
                g.record_command(wall_s=node.wall_clock(),linear_x=ax,linear_y=ay,angular_z=aw,sent=False,error=reason)
                g.stop(reason,node.wall_clock())
                if reason not in node.errors and len(node.errors)<64:
                    node.errors.append(reason)
                return False
        if attempt(x,y,w):
            return True
        attempt(0.,0.,0.)  # Independent zero attempt after any failed publish.
        node.arm_watchdog()
        return False

    def publish_zero():
        with node.command_lock:
            return emit(node.guard._zero())

    def tick():
        try:
            matched=node.check_command_match()
            if not matched and node.guard.state not in ("WAIT","STOP","FINISHED"):
                node.stop("no_external_matching_cmd_subscriber")
            elif node.last_graph_refresh_wall is None or node.wall_clock()-node.last_graph_refresh_wall>=.5-1e-9:
                node.refresh_endpoints()
        except Exception as exc:
            node.stop("endpoint_refresh_error:"+str(exc)[:128])
        with node.command_lock:
            g=node.guard
            decision=g.step(wall_s=node.wall_clock())
            if decision.linear_x or decision.linear_y or decision.angular_z:
                try:
                    if not node.check_command_match():
                        g.stop("no_external_matching_cmd_subscriber",node.wall_clock())
                except Exception as exc:
                    g.stop("endpoint_publish_check_error:"+str(exc)[:128],node.wall_clock())
                wall=node.wall_clock()  # All graph/DDS calls have returned.
                fault=g._budget_fault(wall) or g.observed_motion_fault()
                if fault:
                    g.stop(fault,wall)
                condition=g._condition(wall)
                if condition:
                    g.temporal_fault(condition,wall)
                if g.state in ("STOP","FINISHED","PAUSE") or not authorized(decision):
                    decision=g._zero()
            emit(decision)
            if not node.motion_log or (node.motion_log[-1]["state"],node.motion_log[-1]["reason"]) != (decision.state,decision.reason):
                node.motion_log.append({"wall_elapsed_s":node.wall_clock()-g.started_wall,
                    **asdict(decision),"timing":g.timing_snapshot(wall_s=node.wall_clock())})
                if len(node.motion_log)>64:
                    node.motion_log.pop(1);node.motion_log_dropped+=1
            return g._zero() if g.state in ("STOP","FINISHED","PAUSE") else decision
    node.publish_direction=emit
    node.publish_zero=publish_zero
    node.tick=tick
    return node


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser=argparse.ArgumentParser(description="Finite south preposition and reverse response; no scoring outputs")
    parser.add_argument("--duration",type=float,default=180.)
    parser.add_argument("--report-url",default=os.environ.get("INSPECTION_DIAGNOSTIC_URL",""))
    parsed,ros_args=parser.parse_known_args(args)
    limits=RelocatedReverseLimits(global_wall_s=parsed.duration)
    reporter=DiagnosticReporter(parsed.report_url);rclpy.init(args=ros_args);node=None
    try:
        node=create_relocated_reverse_node(reporter,relocated_limits=limits)
        node.report_started(parsed.duration)
        spin_control_loop(node,rclpy,duration_s=parsed.duration)
        node.stop("relocated_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero();rclpy.spin_once(node,timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("relocated_cleanup");node.close_watchdog();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
