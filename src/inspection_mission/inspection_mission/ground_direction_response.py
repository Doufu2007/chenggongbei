"""One finite ground XY response experiment, separate from all scoring logic.

Frozen sensor pairing and ROS transport are reused; only /cmd_vel can emit.
The initial static region is a candidate assumption, not full map validation.
"""
from __future__ import annotations

import argparse
import base64
import copy
from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
import math
import os
import time
import uuid

from .bounded_ground_probe import (GroundMotionGuard, MotionLimits, _json_safe,
    create_bounded_node, finite, spin_control_loop)
from .ground_bounded_approach_sync import GroundSynchronizedApproachGuard
from .ground_observation_survey import angle_wrap, yaw_from_q as _unit_yaw


def yaw_from_q(q):
    """Derived geometry uses unit q; original payloads remain in raw evidence."""
    norm=math.hypot(*q)
    return _unit_yaw(tuple(v/norm for v in q))


@dataclass(frozen=True)
class DirectionLimits:
    global_wall_s: float = 120.
    max_path_m: float = .18
    max_origin_displacement_m: float = .12
    max_rotation_rad: float = 1.
    turn_wall_s: float = 45.
    maximum_yaw_rate: float = .5
    minimum_yaw_rate: float = .35
    max_turn_drift_m: float = .06
    max_heading_error_rad: float = .05
    local_swept_motion_budget_m: float = .06
    scan_anchor_motion_limit_m: float = .03
    local_stop_reserve_m: float = .03
    pulse_wall_s: float = 2.
    pulse_displacement_m: float = .04
    pulse_request_stop_m: float = .010
    max_translation_speed_mps: float = .15
    max_translation_yaw_speed_rps: float = .15
    max_pulse_yaw_drift_rad: float = .03
    stable_s: float = 1.
    stable_drift_m: float = .002
    stop_drift_m: float = .01

    def __post_init__(self):
        caps = {"global_wall_s":120., "max_path_m":.18, "max_origin_displacement_m":.12,
            "max_rotation_rad":1., "turn_wall_s":45., "maximum_yaw_rate":.5,
            "minimum_yaw_rate":.35, "max_turn_drift_m":.06, "max_heading_error_rad":.05,
            "local_swept_motion_budget_m":.06, "scan_anchor_motion_limit_m":.03,
            "local_stop_reserve_m":.03, "pulse_wall_s":2., "pulse_displacement_m":.04,
            "pulse_request_stop_m":.010, "max_translation_speed_mps":.15,
            "max_translation_yaw_speed_rps":.15, "max_pulse_yaw_drift_rad":.03,
            "stable_s":2., "stable_drift_m":.002, "stop_drift_m":.01}
        for key, cap in caps.items():
            if not finite(getattr(self,key)) or not 0 < getattr(self,key) <= cap:
                raise ValueError("finite direction bound exceeded: "+key)
        if (self.stable_s < 1. or self.minimum_yaw_rate > self.maximum_yaw_rate
                or self.scan_anchor_motion_limit_m+self.local_stop_reserve_m > self.local_swept_motion_budget_m+1e-9
                or self.pulse_request_stop_m+self.local_stop_reserve_m > self.pulse_displacement_m+1e-9):
            raise ValueError("inconsistent direction response reserves")


@dataclass(frozen=True)
class DirectionDecision:
    linear_x: float
    linear_y: float
    angular_z: float
    state: str
    reason: str


def response_from_odom(start, end):
    """Source-stamp displacement; no actuator model or semantic identification."""
    first = GroundMotionGuard._stamp(start,"odom")
    last = GroundMotionGuard._stamp(end,"odom")
    a,b = start["pose"]["pose"],end["pose"]["pose"]
    dx,dy,dz = (b["position"][k]-a["position"][k] for k in ("x","y","z"))
    q = tuple(a["orientation"][k] for k in ("x","y","z","w"))
    q2 = tuple(b["orientation"][k] for k in ("x","y","z","w"))
    if not all(finite(v) for v in (dx,dy,dz,*q,*q2)) or last <= first:
        raise ValueError("response requires finite distinct source poses")
    q = tuple(v/math.hypot(*q) for v in q)
    q2 = tuple(v/math.hypot(*q2) for v in q2)
    yaw = yaw_from_q(q)
    distance = math.hypot(dx,dy)
    bearing = math.atan2(dy,dx) if distance > 1e-12 else None
    return {"source_dt_s":last-first, "delta_world_m":[dx,dy,dz],
        "mean_world_velocity_mps":[dx/(last-first),dy/(last-first),dz/(last-first)],
        "delta_start_body_xy_m":[dx*math.cos(yaw)+dy*math.sin(yaw),-dx*math.sin(yaw)+dy*math.cos(yaw)],
        "horizontal_displacement_m":distance, "start_yaw_rad":yaw,
        "yaw_delta_rad":angle_wrap(yaw_from_q(q2)-yaw), "displacement_bearing_rad":bearing,
        "bearing_minus_start_yaw_rad":None if bearing is None else angle_wrap(bearing-yaw),
        "interface_semantics_confirmed":False}


class GroundDirectionResponseGuard(GroundSynchronizedApproachGuard):
    mode = "ground_direction_response"
    wall_limit_reason = "direction_global_wall_time_limit"
    final_report_wall_s = 180.
    ACTIVE = frozenset(("TURN","TURN_SETTLE","PULSE","PULSE_SETTLE","PAUSE"))
    ZERO_PHASES = frozenset(("TURN_SETTLE","PULSE_SETTLE"))
    PULSES = ((.03,0.),(0.,.03),(.10,0.))

    def __init__(self, *, started_wall_s, direction_limits=None, session_id=None):
        a=direction_limits or DirectionLimits()
        if not isinstance(a,DirectionLimits):raise ValueError("direction_limits required")
        super().__init__(started_wall_s=started_wall_s,approach_limits=a)
        self.session_id=session_id or uuid.uuid4().hex
        self.direction_limits=a
        self.pulse_index=-1
        self.pulse_started_wall=self.direction_pulse_origin=self.direction_pulse_yaw=None
        self.zero_request_path=None
        self.zero_requests=[]
        self.pulse_records=[]
        self.last_raw_scan=self.last_raw_odom=None
        self._derived_previous_yaw=None
        self.raw_odometry,self.raw_scans,self.direction_commands,self.state_records=[],[],[],[]
        self.direction_dropped={"raw_odometry":0,"raw_scans":0,"direction_commands":0,"state_records":0}
        self._odom_validator=GroundMotionGuard(self.limits,started_wall_s=started_wall_s)
        self._observations.mode=self.mode
        self._observations.TRAJECTORY_CAP=1500
        self._observations.COMMAND_CAP=10000

    def _append(self,key,item,cap):
        rows=getattr(self,key);rows.append(item)
        if len(rows)>cap:
            rows.pop(1);self.direction_dropped[key]+=1

    def _zero(self):
        return DirectionDecision(0.,0.,0.,self.state,self.reason)

    def _stop_decision(self,reason,wall):
        self.stop(reason,wall)
        return self._zero()

    def _mark_zero(self,wall,reason):
        if self.zero_request_path is None:
            self.zero_request_path=self.total_path_m
            record={"wall_elapsed_s":wall-self.started_wall,"reason":reason,
                "path_origin_m":self.total_path_m,"raw_odom":copy.deepcopy(self.last_raw_odom),
                "interpretation":"zero requested; transport and physical stop are separate observations"}
            self.zero_requests.append(record)
            if 0 <= self.pulse_index < len(self.pulse_records):
                self.pulse_records[self.pulse_index]["zero_requests"].append(copy.deepcopy(record))

    def _enter_phase(self,phase):
        if (phase=="TURN_SETTLE" and self.odom is not None and self.target_yaw is not None
                and abs(angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"])))>.025):
            return
        super()._enter_phase(phase)
        self.reason="direction_"+phase.lower()
        wall=max(self.last_wall,self.last_observation_wall)
        if phase in self.ZERO_PHASES:self._mark_zero(wall,self.reason)
        self._append("state_records",{"wall_elapsed_s":wall-self.started_wall,"phase":phase,
            "state":self.state,"pulse_index":self.pulse_index,"reason":self.reason},128)

    def _begin_settle(self,wall,reason):
        if self.phase=="PULSE":
            self._mark_zero(wall,reason)
            self._enter_phase("PULSE_SETTLE")
            self.pulse_records[self.pulse_index]["motion_end_reason"]=reason

    def _budget_fault(self,wall):
        if not finite(wall) or wall<max(self.last_wall,self.last_observation_wall):
            return "wall_clock_reversed_or_invalid"
        if wall-self.started_wall>=self.direction_limits.global_wall_s-1e-9:
            return self.wall_limit_reason
        if (self.phase in ("TURN","TURN_SETTLE") and self.turn_started_wall is not None
                and wall-self.turn_started_wall>=self.direction_limits.turn_wall_s-1e-9):
            return "direction_turn_wall_time_limit"
        if (self.phase=="PULSE" and self.pulse_started_wall is not None
                and wall-self.pulse_started_wall>=self.direction_limits.pulse_wall_s-1e-9):
            self._begin_settle(wall,"pulse_wall_time_reached")
        return ""

    def observed_motion_fault(self):
        if self.odom is None:return ""
        a=self.direction_limits;p=self.odom;q=p["q"];norm=math.hypot(*q)
        tilt=math.acos(max(-1.,min(1.,1-2*(q[0]**2+q[1]**2)/(norm*norm))))
        if tilt>.03:return "ground_tilt"
        if self.global_origin is not None and math.dist(p["xyz"],self.global_origin)>=a.max_origin_displacement_m-1e-9:
            return "direction_origin_displacement_limit"
        if self.total_path_m>=a.max_path_m-1e-9:return "direction_path_limit"
        if self.accumulated_rotation_rad>=a.max_rotation_rad-1e-9:return "direction_rotation_limit"
        turning=self.phase in ("TURN","TURN_SETTLE")
        if p["speed"]>(.10 if turning else a.max_translation_speed_mps):return "direction_speed_limit"
        if p["yaw_speed"]>(a.maximum_yaw_rate+.05 if turning else a.max_translation_yaw_speed_rps):
            return "direction_angular_speed_limit"
        if turning and self.turn_origin is not None and math.dist(p["xyz"],self.turn_origin)>=a.max_turn_drift_m-1e-9:
            return "direction_turn_displacement_limit"
        if self.phase=="TURN_SETTLE" and abs(angle_wrap(yaw_from_q(q)-self.target_yaw))>a.max_heading_error_rad:
            return "direction_turn_overshoot_limit"
        if self.direction_pulse_origin is not None:
            if math.dist(p["xyz"],self.direction_pulse_origin)>=a.pulse_displacement_m-1e-9:
                return "direction_pulse_displacement_limit"
            if abs(angle_wrap(yaw_from_q(q)-self.direction_pulse_yaw))>a.max_pulse_yaw_drift_rad:
                return "direction_pulse_yaw_drift_limit"
        if self.zero_request_path is not None and self.total_path_m-self.zero_request_path>a.stop_drift_m+1e-9:
            return "direction_zero_stop_path_limit"
        if self.scan_anchor is not None and self.state in self.ACTIVE:
            if math.dist(p["xyz"],self.scan_anchor["odom"]["position"])>a.scan_anchor_motion_limit_m:
                return "local_scan_motion_reserve_exhausted"
        return ""

    def temporal_fault(self,reason,wall):
        super().temporal_fault(reason,wall)
        if self.state=="PAUSE":self._mark_zero(wall,reason)

    def _stable_dwell(self,wall):
        if self.odom is None:return False
        p=self.odom;stable=p["speed"]<=.005 and p["yaw_speed"]<=.01
        q=p["q"];norm=math.hypot(*q)
        if self.stable_anchor is not None:
            anchor=self.stable_anchor
            stable=(stable and math.dist(p["xyz"],anchor[2])<=self.direction_limits.stable_drift_m
                and self.total_path_m-anchor[4]<=self.direction_limits.stable_drift_m
                and abs(sum(x*y for x,y in zip(q,anchor[3])))/(norm*math.hypot(*anchor[3]))>=math.cos(.02/2))
        if not stable:self.stable_anchor=None;return False
        if self.stable_anchor is None:
            self.stable_anchor=(wall,p["stamp"],p["xyz"],p["q"],self.total_path_m)
        anchor=self.stable_anchor
        return min(wall-anchor[0],p["stamp"]-anchor[1])>=self.direction_limits.stable_s-1e-9

    def update_scan(self,payload,*,wall_s):
        with self._lock:
            if self.state in ("STOP","FINISHED"):return
            # Every callback is validated before duplicate classification. An
            # identical duplicate never enters the authoritative pairing path.
            values=payload.get("ranges") if isinstance(payload,dict) else None
            if not isinstance(values,(tuple,list)) or not values or not all(finite(v) for v in values):
                self.stop("direction_scan_nonfinite_or_invalid",wall_s);return
            self._scan_validator.update_scan(payload,wall_s=wall_s)
            if self._scan_validator.scan is None:
                self.stop("direction_scan_invalid:"+self._scan_validator.reason,wall_s);return
            stamp=self._scan_validator.scan["stamp"]
            if self.last_raw_scan is not None:
                previous=self._stamp(self.last_raw_scan,"laser_link")
                if stamp==previous:
                    if payload!=self.last_raw_scan:self.stop("direction_scan_duplicate_changed",wall_s)
                    return
                if stamp<previous:self.stop("direction_scan_stamp_reversed",wall_s);return
            super().update_scan(payload,wall_s=wall_s)
            if self.state not in ("STOP","FINISHED"):
                self.last_raw_scan=copy.deepcopy(payload)
                self._append("raw_scans",{"wall_elapsed_s":wall_s-self.started_wall,
                    "payload":copy.deepcopy(payload)},2000)

    def update_odom(self,payload,*,wall_s):
        with self._lock:
            self._odom_validator.update_odom(payload,wall_s=wall_s)
            if self._odom_validator.odom is None:
                self.stop("direction_odom_invalid:"+self._odom_validator.reason,wall_s);return
            stamp=self._odom_validator.odom["stamp"]
            if self.last_raw_odom is not None:
                previous=self._stamp(self.last_raw_odom,"odom")
                if stamp==previous:
                    if payload!=self.last_raw_odom:self.stop("direction_odom_duplicate_changed",wall_s)
                    return
                if stamp<previous:self.stop("direction_odom_stamp_reversed",wall_s);return
            self.last_raw_odom=copy.deepcopy(payload)
            self._append("raw_odometry",{"wall_elapsed_s":wall_s-self.started_wall,
                "phase":self.phase,"state":self.state,"pulse_index":self.pulse_index,
                "payload":copy.deepcopy(payload)},10000)
            old_rotation=self.accumulated_rotation_rad
            previous_yaw=self._derived_previous_yaw
            super().update_odom(payload,wall_s=wall_s)
            if self.odom is not None and self.global_origin is not None:
                current_yaw=yaw_from_q(self.odom["q"])
                if previous_yaw is not None:
                    self.accumulated_rotation_rad=old_rotation+abs(angle_wrap(current_yaw-previous_yaw))
                self.previous_yaw=self._derived_previous_yaw=current_yaw
                fault=self._budget_fault(wall_s) or self.observed_motion_fault()
                if self.state in self.ACTIVE and fault:self.stop(fault,wall_s)
            if self.state in self.ACTIVE and self.phase=="PULSE" and self.odom is not None:
                if math.dist(self.odom["xyz"],self.direction_pulse_origin)>=self.direction_limits.pulse_request_stop_m-1e-9:
                    self._begin_settle(wall_s,"pulse_measured_request_stop")

    def observe_public(self,topic,payload,*,wall_s):
        # Public draw/task state has no control role in an identification run.
        return

    def observe_image(self,topic,msg,*,wall_s):
        return

    def _begin_pulse(self,wall):
        self.pulse_index+=1
        self.direction_pulse_origin=self.odom["xyz"]
        self.direction_pulse_yaw=yaw_from_q(self.odom["q"])
        self.pulse_started_wall=wall
        self.zero_request_path=None
        self.pulse_records.append({"index":self.pulse_index,"requested_body_xy_mps":self.PULSES[self.pulse_index],
            "start":{"wall_elapsed_s":wall-self.started_wall,"raw_odom":copy.deepcopy(self.last_raw_odom),
                "yaw_rad":self.direction_pulse_yaw,"path_origin_m":self.total_path_m,
                "rotation_origin_rad":self.accumulated_rotation_rad},"zero_requests":[]})
        self._enter_phase("PULSE")

    def step(self,*,wall_s):
        with self._lock:
            if not finite(wall_s) or wall_s<max(self.last_wall,self.last_observation_wall):
                return self._stop_decision("wall_clock_reversed_or_invalid",wall_s)
            self.last_wall=wall_s
            if self.state in ("STOP","FINISHED"):
                if wall_s-self.stopped_wall>=self.limits.stop_wall_s:self.state="FINISHED"
                return self._zero()
            fault=self._budget_fault(wall_s)
            if fault:return self._stop_decision(fault,wall_s)
            fault=self.observed_motion_fault() or self._condition(wall_s)
            if fault:
                self.stable_anchor=None
                if self.state in self.ACTIVE:self.temporal_fault(fault,wall_s)
                else:self.reason=fault
                return self._zero()
            if self.state=="PAUSE":
                if not self._stable_dwell(wall_s):return self._zero()
                if self.resume_count>=3:return self._stop_decision("pause_resume_limit",wall_s)
                self.resume_count+=1;self.state=self.phase;self.stable_anchor=None
            elif self.state=="WAIT":
                if not self._stable_dwell(wall_s):return self._zero()
                if math.hypot(*self.odom["xyz"][:2])>.05 or abs(yaw_from_q(self.odom["q"]))>.05:
                    return self._stop_decision("direction_initial_pose_outside_contract",wall_s)
                self.global_origin=self.turn_origin=self.odom["xyz"]
                self.pulse_origin=self.global_origin
                self.previous_path_xyz=self.global_origin;self.previous_yaw=yaw_from_q(self.odom["q"])
                self._derived_previous_yaw=self.previous_yaw
                self.target_yaw=-.60;self.initial_turn_error=angle_wrap(self.target_yaw-self.previous_yaw)
                self.first_motion_wall=self.turn_started_wall=wall_s
                self._record_scan("direction_start",wall_s);self._enter_phase("TURN")
            if self.phase=="TURN":
                error=angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
                if abs(error)<=.025 or error*self.initial_turn_error<0:
                    if abs(error)>self.direction_limits.max_heading_error_rad:
                        return self._stop_decision("direction_turn_overshoot_limit",wall_s)
                    self._enter_phase("TURN_SETTLE")
                else:
                    self.zero_request_path=None
                    rate=math.copysign(min(self.direction_limits.maximum_yaw_rate,
                        max(self.direction_limits.minimum_yaw_rate,.6*abs(error))),error)
                    return DirectionDecision(0.,0.,rate,self.state,self.reason)
            if self.phase=="TURN_SETTLE":
                if not self._stable_dwell(wall_s):return self._zero()
                self._record_scan("direction_after_turn",wall_s);self._begin_pulse(wall_s)
            elif self.phase=="PULSE_SETTLE":
                if not self._stable_dwell(wall_s):return self._zero()
                anchor=self.stable_anchor;record=self.pulse_records[self.pulse_index]
                record["settled"]={"wall_elapsed_s":wall_s-self.started_wall,
                    "raw_odom":copy.deepcopy(self.last_raw_odom),"stable_wall_s":wall_s-anchor[0],
                    "stable_source_s":self.odom["stamp"]-anchor[1],
                    "stop_path_increment_m":self.total_path_m-self.zero_request_path}
                record["response"]=response_from_odom(record["start"]["raw_odom"],self.last_raw_odom)
                self._record_scan("direction_pulse_"+str(self.pulse_index),wall_s)
                if self.pulse_index==2:
                    return self._stop_decision("direction_response_observed_official_score_unverified",wall_s)
                self._begin_pulse(wall_s)
            if self.phase=="PULSE":
                self.zero_request_path=None
                x,y=self.PULSES[self.pulse_index]
                return DirectionDecision(x,y,0.,self.state,self.reason)
            return self._zero()

    def record_command(self,*,wall_s,linear_x,linear_y=0.,angular_z,sent=True,error=None):
        with self._lock:
            self._append("direction_commands",{"wall_elapsed_s":wall_s-self.started_wall,
                "clock_stamp_s":None if self.clock is None else self.clock[0],
                "linear_x":linear_x,"linear_y":linear_y,"angular_z":angular_z,
                "sent":bool(sent),"error":error,"phase":self.phase,"state":self.state,
                "pulse_index":self.pulse_index,"timing":self.timing_snapshot(wall_s=wall_s)},12000)

    def core_evidence(self):
        with self._lock:
            return {"mode":self.mode,"session_id":self.session_id,"state":self.state,"phase":self.phase,
                "reason":self.reason,"limits":asdict(self.direction_limits),"pulse_index":self.pulse_index,
                "pulse_records":copy.deepcopy(self.pulse_records),"global_origin":self.global_origin,
                "total_path_m":self.total_path_m,"accumulated_rotation_rad":self.accumulated_rotation_rad,
                "max_origin_displacement_m":self.max_observed_displacement_m,"resume_count":self.resume_count,
                "dropped":dict(self.direction_dropped),"sensor_sync":self.sync_diagnostics(),
                "official_score":None,"full_mission_complete":False,"assumptions":{
                    "static_initial_region":"public candidate geometry; current platform map not fully verified",
                    "direction_semantics":"to be measured; body XY is request interpretation only",
                    "stopping_distance":"not yet physically established; .04m is observed hard-stop threshold"}}

    def snapshot_survey_evidence(self):
        return self.core_evidence()

    def survey_evidence(self):
        return self.core_evidence()

    def additional_events(self):
        with self._lock:
            core=self.core_evidence()
            body={**core,"schema":"ground_direction_response_evidence_v1",
                "raw_odometry":copy.deepcopy(self.raw_odometry),"raw_scans":copy.deepcopy(self.raw_scans),
                "commands":copy.deepcopy(self.direction_commands),"state_records":copy.deepcopy(self.state_records),
                "zero_requests":copy.deepcopy(self.zero_requests)}
        raw=json.dumps(_json_safe(body),ensure_ascii=False,separators=(",",":"),allow_nan=False).encode("utf-8")
        packed=gzip.compress(raw,mtime=0);encoded=base64.b64encode(packed).decode("ascii")
        chunks=[encoded[i:i+48000] for i in range(0,len(encoded),48000)]
        metadata={"mode":self.mode,"session_id":self.session_id,"read_only":False,
            "official_scoring":"unverified","section":"ground_direction_evidence_gzip_part",
            "encoding":"gzip+base64","bundle":"ground_direction_response_v1",
            "uncompressed_bytes":len(raw),"compressed_bytes":len(packed),
            "uncompressed_sha256":hashlib.sha256(raw).hexdigest(),"compressed_sha256":hashlib.sha256(packed).hexdigest(),
            "part_count":len(chunks)}
        return [("probe_snapshot",{"mode":self.mode,"section":"ground_direction_core","data":core})]+[
            ("probe_snapshot",{**metadata,"part_index":i,"base64_part":chunk}) for i,chunk in enumerate(chunks)]


def create_direction_response_node(reporter=None,*,wall_clock=time.monotonic,direction_limits=None):
    """Reuse proven subscription/graph/watchdog transport; replace actual XY loop."""
    from geometry_msgs.msg import Twist
    def factory(limits,*,started_wall_s):
        return GroundDirectionResponseGuard(started_wall_s=started_wall_s,direction_limits=direction_limits)
    node=create_bounded_node(MotionLimits(.447,.114,.02),reporter,wall_clock=wall_clock,
        guard_factory=factory,graph_refresh_interval_s=.5,
        input_reliability_overrides={"/odom":"BEST_EFFORT","/uav/odom":"BEST_EFFORT"})

    def emit(decision):
        now=node.wall_clock();g=node.guard
        x,y,w=decision.linear_x,decision.linear_y,decision.angular_z
        if (not all(finite(v) for v in (x,y,w)) or math.hypot(x,y)>.10+1e-9 or abs(w)>.5+1e-9
                or ((x or y) and w)):
            g.stop("direction_output_bound_violation",now);x=y=w=0.
        if g.state in ("STOP","FINISHED","PAUSE"):x=y=w=0.
        publisher=node.publisher_by_topic.get("/cmd_vel")
        if publisher is None:
            g.stop("direction_cmd_publisher_missing",now);return False
        def attempt(ax,ay,aw):
            msg=Twist();msg.linear.x=ax;msg.linear.y=ay;msg.angular.z=aw
            try:
                publisher.publish(msg)
                g.record_command(wall_s=node.wall_clock(),linear_x=ax,linear_y=ay,angular_z=aw,sent=True)
                if ax or ay or aw:g.published_nonzero_count+=1;node.arm_watchdog()
                else:g.published_zero_count+=1
                return True
            except Exception as exc:
                error="direction_publish_error:"+str(exc)[:128]
                g.record_command(wall_s=node.wall_clock(),linear_x=ax,linear_y=ay,angular_z=aw,sent=False,error=error)
                g.stop(error,node.wall_clock())
                if error not in node.errors and len(node.errors)<64:node.errors.append(error)
                return False
        if attempt(x,y,w):return True
        # A failed nonzero publish cannot suppress the independent zero attempt.
        attempt(0.,0.,0.)
        node.arm_watchdog()
        return False

    def publish_zero():
        with node.command_lock:
            return emit(DirectionDecision(0.,0.,0.,node.guard.state,node.guard.reason))

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
            decision=node.guard.step(wall_s=node.wall_clock())
            if decision.linear_x or decision.linear_y or decision.angular_z:
                # Graph refresh/step may consume time after the first match
                # sample. Recheck actual DDS immediately before nonzero output.
                if not node.check_command_match():
                    node.guard.stop("no_external_matching_cmd_subscriber",node.wall_clock())
                    decision=node.guard._zero()
                reason=node.guard._condition(node.wall_clock()) or node.guard.observed_motion_fault()
                if reason:
                    node.guard.temporal_fault(reason,node.wall_clock());decision=node.guard._zero()
            emit(decision)
            if not node.motion_log or (node.motion_log[-1]["state"],node.motion_log[-1]["reason"])!=(decision.state,decision.reason):
                node.motion_log.append({"wall_elapsed_s":node.wall_clock()-node.guard.started_wall,
                    **asdict(decision),"timing":node.guard.timing_snapshot(wall_s=node.wall_clock())})
                if len(node.motion_log)>64:node.motion_log.pop(1);node.motion_log_dropped+=1
            return node.guard._zero() if node.guard.state in ("STOP","FINISHED","PAUSE") else decision

    node.publish_direction=emit
    node.publish_zero=publish_zero
    node.tick=tick
    return node


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser=argparse.ArgumentParser(description="Finite ground direction response; no scoring outputs")
    parser.add_argument("--duration",type=float,default=120.)
    parser.add_argument("--report-url",default=os.environ.get("INSPECTION_DIAGNOSTIC_URL",""))
    parsed,ros_args=parser.parse_known_args(args)
    limits=DirectionLimits(global_wall_s=parsed.duration)
    reporter=DiagnosticReporter(parsed.report_url);rclpy.init(args=ros_args);node=None
    try:
        node=create_direction_response_node(reporter,direction_limits=limits)
        node.report_started(parsed.duration)
        spin_control_loop(node,rclpy,duration_s=parsed.duration)
        node.stop("direction_global_wall_time_limit")
        for _ in range(5):node.publish_zero();rclpy.spin_once(node,timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("direction_cleanup");node.close_watchdog();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        reporter.close()


if __name__=="__main__":main()
