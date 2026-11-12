"""One ground yaw survey, using public draw IDs and explicitly candidate geometry.

This collects real images and motion evidence. It never sends recognition,
arrival, flight, laser or completion messages and never claims a task score.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import math
import os
import statistics
import time
from collections import deque
from dataclasses import asdict, dataclass

from .bounded_ground_probe import (GroundMotionGuard, MotionDecision, MotionLimits,
                                  create_bounded_node, finite, spin_control_loop)


# Public static candidate world, not referee-private data or image class labels.
CANDIDATE_POINTS = {
    "ground_0": (1.125, -1.35), "ground_1": (0., -1.35),
    "ground_2": (0., -2.70), "ground_3": (1.125, -1.925),
    "ground_4": (1.125, -.775),
}


def angle_wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_q(q):
    x,y,z,w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


@dataclass(frozen=True)
class SurveyLimits:
    yaw_rate: float = .5
    minimum_yaw_rate: float = .04
    max_turn_wall_s: float = 25.
    max_rotation_rad: float = 1.8
    yaw_tolerance_rad: float = .025
    target_yaw: float | None = None
    global_wall_s: float = 45.

    def __post_init__(self):
        caps = {"yaw_rate": .5, "minimum_yaw_rate":.35, "max_turn_wall_s":45., "max_rotation_rad":1.8,
                "yaw_tolerance_rad":.025, "global_wall_s":60.}
        for field,maximum in caps.items():
            value=getattr(self,field)
            if not finite(value) or not 0<value<=maximum:
                raise ValueError(field+" exceeds survey bound")
        if self.target_yaw is not None and not finite(self.target_yaw):
            raise ValueError("target yaw must be finite")
        if self.minimum_yaw_rate>self.yaw_rate:
            raise ValueError("minimum yaw rate exceeds maximum")
        if self.global_wall_s<5:
            raise ValueError("global wall limit too short")


class GroundSurveyGuard(GroundMotionGuard):
    mode="ground_observation_survey"
    wall_limit_reason="turn_wall_time_limit"
    final_report_wall_s=60.

    def __init__(self,limits,*,started_wall_s,survey_limits=None):
        super().__init__(limits,started_wall_s=started_wall_s)
        self.survey_limits=survey_limits or SurveyLimits()
        self.motion_wall_limit_s=self.survey_limits.max_turn_wall_s
        self.observed_yaw_speed_limit=self.survey_limits.yaw_rate+.05
        self.resume_count=0
        self.draw=None
        self.target_id=self.target_yaw=None
        self.target_xy=None
        self.initial_yaw=self.previous_yaw=None
        self.accumulated_rotation_rad=0.
        self.full_odom_history=deque(maxlen=2048)
        self.uav_odom_history=deque(maxlen=2048)
        self.uav_odom_epoch_valid=True
        self.ground_odom_error=self.uav_odom_error=None
        self.odom_periods={}
        self.trajectory=[]
        self.actual_commands=[]
        self.actual_commands_dropped=0
        self.trajectory_dropped=0
        self.exposures={}
        self.image_frames={}
        self.image_errors=[]
        self.epoch_valid=True

    def update_clock(self,stamp_s,*,wall_s):
        with self._lock:
            if self.clock is not None and finite(stamp_s) and stamp_s<self.clock[0]:
                self.epoch_valid=False
                self.full_odom_history.clear()
                self.uav_odom_history.clear()
            super().update_clock(stamp_s,wall_s=wall_s)

    def observe_public(self,topic,payload,*,wall_s):
        if topic=="/uav/odom":
            self.update_uav_odom(payload,wall_s=wall_s)
            return
        if topic!="/referee/ground_tasks":
            return
        with self._lock:
            data=payload.get("data")
            parts=[part.strip() for part in data.split(",")] if isinstance(data,str) else []
            if len(parts)!=3 or len(set(parts))!=3 or any(part not in CANDIDATE_POINTS for part in parts):
                self.stop("invalid_public_draw",wall_s)
                return
            if self.draw is not None and parts!=self.draw:
                self.stop("public_draw_changed",wall_s)
                return
            self.draw=parts

    def update_odom(self,payload,*,wall_s):
        super().update_odom(payload,wall_s=wall_s)
        with self._lock:
            if self.odom is None:
                self.full_odom_history.clear()
                self.ground_odom_error="ground_odom_invalid:"+self.reason
                if "odom_stamp_reversed" in self.reason:
                    self.epoch_valid=False
                return
            self.ground_odom_error=None
            sample={"stamp_s":self.odom["stamp"],"wall_elapsed_s":wall_s-self.started_wall,
                "frame_id":"odom","child_frame_id":"base_footprint","source_topic":"/odom",
                "position":list(self.odom["xyz"]),"quaternion":list(self.odom["q"]),
                "yaw_rad":yaw_from_q(self.odom["q"]),"linear_speed":self.odom["speed"],
                "angular_speed":self.odom["yaw_speed"]}
            self.full_odom_history.append(sample)
            self._record_odom_period("/odom",sample)
            if not self.trajectory or wall_s-self.started_wall-self.trajectory[-1]["wall_elapsed_s"]>=.1-1e-9:
                self.trajectory.append(copy.deepcopy(sample))
                if len(self.trajectory)>500:
                    self.trajectory.pop(1)
                    self.trajectory_dropped+=1
            if self.state in ("TURN","PAUSE"):
                if self.previous_yaw is not None:
                    self.accumulated_rotation_rad+=abs(angle_wrap(sample["yaw_rad"]-self.previous_yaw))
                self.previous_yaw=sample["yaw_rad"]
                if self.accumulated_rotation_rad>=self.survey_limits.max_rotation_rad-1e-9:
                    self.stop("rotation_budget",wall_s)
                elif self.target_yaw is not None:
                    error=angle_wrap(self.target_yaw-sample["yaw_rad"])
                    if abs(error)<=self.survey_limits.yaw_tolerance_rad:
                        self.stop("target_yaw_reached",wall_s)
                    elif error*self.initial_error<0:
                        self.stop("target_yaw_crossed",wall_s)

    FRESHNESS_REASONS=frozenset(("clock_stale","odom_stale","scan_stale",
        "odom_stale_or_invalid","scan_stale_or_unsafe"))

    def temporal_fault(self,reason,wall):
        with self._lock:
            if self.state in ("TURN","PAUSE") and reason in self.FRESHNESS_REASONS:
                self.stable_anchor=None
                self.state,self.reason="PAUSE",reason
            else:
                self.stop(reason,wall)

    def handle_watchdog(self,reason,wall):
        with self._lock:
            if reason in self.FRESHNESS_REASONS:
                self.temporal_fault(reason,wall)
            else:
                self.stop(reason,wall)

    def update_uav_odom(self,payload,*,wall_s):
        with self._lock:
            try:
                stamp=self._stamp(payload,"odom")
                if payload["child_frame_id"]!="uav/base_link":
                    raise ValueError("uav_odom_child_frame")
                pose,twist=payload["pose"]["pose"],payload["twist"]["twist"]
                xyz=[pose["position"][key] for key in ("x","y","z")]
                q=[pose["orientation"][key] for key in ("x","y","z","w")]
                linear=[twist["linear"][key] for key in ("x","y","z")]
                angular=[twist["angular"][key] for key in ("x","y","z")]
                if not all(finite(value) for value in xyz+q+linear+angular) or abs(math.hypot(*q)-1)>.001:
                    raise ValueError("uav_odom_numeric_or_quaternion")
                if self.uav_odom_history and stamp<self.uav_odom_history[-1]["stamp_s"]:
                    self.uav_odom_epoch_valid=False
                    self.uav_odom_history.clear()
                    raise ValueError("uav_odom_stamp_reversed")
                self.uav_odom_history.append({"stamp_s":stamp,"wall_elapsed_s":wall_s-self.started_wall,
                    "source_topic":"/uav/odom","frame_id":"odom","child_frame_id":"uav/base_link",
                    "position":xyz,"quaternion":q,"yaw_rad":yaw_from_q(q),
                    "linear_speed":math.hypot(*linear),"angular_speed":math.hypot(*angular)})
                self._record_odom_period("/uav/odom",self.uav_odom_history[-1])
                self.uav_odom_error=None
            except (KeyError,ValueError,TypeError,OverflowError) as exc:
                self.uav_odom_history.clear()
                error=str(exc)[:128]
                self.uav_odom_error="uav_odom_invalid:"+error
                if error not in self.image_errors and len(self.image_errors)<16:
                    self.image_errors.append(error)

    def _record_odom_period(self,topic,sample):
        periods=self.odom_periods.setdefault(topic,{"last":None,"count":0,
            "header":deque(maxlen=2048),"arrival":deque(maxlen=2048)})
        previous=periods["last"]
        if previous is not None:
            header=sample["stamp_s"]-previous[0]
            arrival=sample["wall_elapsed_s"]-previous[1]
            if header>0 and arrival>=0:
                periods["header"].append(header)
                periods["arrival"].append(arrival)
                periods["count"]+=1
        periods["last"]=(sample["stamp_s"],sample["wall_elapsed_s"])

    def observed_periods(self):
        result={}
        for topic,periods in self.odom_periods.items():
            item={"interval_count":periods["count"],"retained_interval_count":len(periods["header"]),
                  "interpretation":"observed_intervals_only; source cause unverified"}
            for key,field in (("header_delta_s","header"),("arrival_delta_s","arrival")):
                values=list(periods[field])
                item[key]=(None if not values else {"min":min(values),"median":statistics.median(values),
                    "max":max(values),"last":values[-1],"recent_intervals":values[-12:]})
            result[topic]=item
        return result

    def observe_image(self,topic,msg,*,wall_s):
        with self._lock:
            try:
                sec,nano=msg.header.stamp.sec,msg.header.stamp.nanosec
                if type(sec) is not int or type(nano) is not int or sec<0 or not 0<=nano<1_000_000_000:
                    raise ValueError("invalid_image_stamp")
                stamp=sec+nano/1e9
                record={"topic":topic,"stamp_s":stamp,"frame_id":msg.header.frame_id,
                    "wall_elapsed_s":wall_s-self.started_wall,"odom":None,"association_error":None}
                if not self.epoch_valid:
                    record["association_error"]="clock_epoch_invalid"
                elif topic not in ("/camera/image","/uav/camera/image"):
                    record["association_error"]="unsupported_camera_topic"
                else:
                    body="ground" if topic=="/camera/image" else "uav"
                    history=self.full_odom_history if body=="ground" else self.uav_odom_history
                    source_error=self.ground_odom_error if body=="ground" else self.uav_odom_error
                    prior=next((sample for sample in reversed(history)
                                if sample["stamp_s"]<=stamp+1e-9),None)
                    if source_error is not None:
                        record["association_error"]=source_error
                    elif body=="uav" and not self.uav_odom_epoch_valid:
                        record["association_error"]="uav_odom_epoch_invalid"
                    elif prior is None:
                        record["association_error"]="no_prior_"+body+"_odom"
                    elif stamp-prior["stamp_s"]>.05+1e-9:
                        record["association_error"]="prior_odom_too_old"
                    else:
                        record["odom"]={**copy.deepcopy(prior),"association":"prior_"+body+"_odom",
                                        "image_minus_odom_s":stamp-prior["stamp_s"]}
                records=self.exposures.setdefault(topic,{"first":record,"last":record})
                records["last"]=record
                frames=self.image_frames.setdefault(topic,{"first":msg,"last":msg})
                frames["last"]=msg
                if record["odom"] is not None:
                    # Retain its actual sensor header and associated prior
                    # source pose even if newer frames arrive without a pose.
                    records["last_associated"]=copy.deepcopy(record)
                    frames["last_associated"]=msg
            except (AttributeError,TypeError,ValueError) as exc:
                error=str(exc)[:128]
                if error not in self.image_errors and len(self.image_errors)<8:
                    self.image_errors.append(error)

    def record_command(self,*,wall_s,linear_x,angular_z):
        with self._lock:
            self.actual_commands.append({"wall_elapsed_s":wall_s-self.started_wall,
                "clock_stamp_s":None if self.clock is None else self.clock[0],
                "linear_x":linear_x,"angular_z":angular_z,"state":self.state})
            if len(self.actual_commands)>1200:
                self.actual_commands.pop(1)
                self.actual_commands_dropped+=1

    def _turn_fault(self,wall):
        if wall-self.started_wall>=self.survey_limits.global_wall_s:
            return "survey_global_wall_time_limit"
        if self.pulse_started is not None and wall-self.pulse_started>=self.motion_wall_limit_s-1e-9:
            return self.wall_limit_reason
        reason=self._condition(wall)
        if reason:
            return reason
        if self.pulse_origin is not None and math.dist(self.odom["xyz"],self.pulse_origin)>=self.limits.max_displacement_m-1e-9:
            return "displacement_limit"
        if self.accumulated_rotation_rad>=self.survey_limits.max_rotation_rad-1e-9:
            return "rotation_budget"
        if self.odom["speed"]>.10 or self.odom["yaw_speed"]>self.survey_limits.yaw_rate+.05:
            return "unexpected_motion"
        return ""

    def watchdog_reason(self,wall):
        with self._lock:
            if self.state not in ("TURN","PAUSE"):
                return ""
            return self._turn_fault(wall)

    def step(self,*,wall_s):
        with self._lock:
            if not finite(wall_s) or wall_s<self.last_wall:
                return self.stop("wall_clock_reversed_or_invalid",self.last_wall)
            self.last_wall=wall_s
            if self.state in ("STOP","FINISHED"):
                if wall_s-self.stopped_wall>=self.limits.stop_wall_s:
                    self.state="FINISHED"
                return MotionDecision(0.,self.state,self.reason)
            if self.state in ("TURN","PAUSE"):
                fault=self._turn_fault(wall_s)
                if fault:
                    if fault in self.FRESHNESS_REASONS:
                        self.temporal_fault(fault,wall_s)
                        self.stable_anchor=None
                        return MotionDecision(0.,self.state,self.reason)
                    return self.stop(fault,wall_s)
                if self.state=="PAUSE":
                    pose=self.odom
                    stable=pose["speed"]<=.01 and pose["yaw_speed"]<=.03
                    if self.stable_anchor is not None:
                        a=self.stable_anchor
                        stable=stable and math.dist(pose["xyz"],a[2])<=.005
                        stable=stable and abs(sum(x*y for x,y in zip(pose["q"],a[3])))>=math.cos(.02/2)
                    if not stable:
                        self.stable_anchor=None
                        return MotionDecision(0.,self.state,"pause_waiting_for_stability")
                    if self.stable_anchor is None:
                        self.stable_anchor=(wall_s,pose["stamp"],pose["xyz"],pose["q"])
                    a=self.stable_anchor
                    if min(wall_s-a[0],pose["stamp"]-a[1])<self.limits.stable_s-1e-9:
                        return MotionDecision(0.,self.state,"pause_stability_dwell")
                    if self.resume_count>=3:
                        return self.stop("pause_resume_limit",wall_s)
                    self.resume_count+=1
                    self.state,self.reason="TURN","bounded_yaw_turn"
                    self.stable_anchor=None
                error=angle_wrap(self.target_yaw-yaw_from_q(self.odom["q"]))
                if abs(error)<=self.survey_limits.yaw_tolerance_rad:
                    return self.stop("target_yaw_reached",wall_s)
                # A crossed target stops this one-shot action; never oscillate
                # back and forth to chase a pose after overshoot.
                if error*self.initial_error<0:
                    return self.stop("target_yaw_crossed",wall_s)
                rate=math.copysign(min(self.survey_limits.yaw_rate,
                    max(self.survey_limits.minimum_yaw_rate,.6*abs(error))),error)
                return MotionDecision(0.,self.state,"bounded_yaw_turn",rate)
            if wall_s-self.started_wall>=min(self.limits.wait_wall_s,self.survey_limits.global_wall_s):
                return self.stop("wait_timeout:"+self.reason,wall_s)
            reason=self._condition(wall_s)
            if reason or self.draw is None:
                self.stable_anchor=None
                self.reason=reason or "waiting_for_public_draw"
                return MotionDecision(0.,self.state,self.reason)
            pose=self.odom
            stable=pose["speed"]<=.01 and pose["yaw_speed"]<=.03
            if self.stable_anchor is not None:
                anchor=self.stable_anchor
                stable=stable and math.dist(pose["xyz"],anchor[2])<=.005
                stable=stable and abs(sum(x*y for x,y in zip(pose["q"],anchor[3])))>=math.cos(.02/2)
            if not stable:
                self.stable_anchor=None
                return MotionDecision(0.,self.state,"start_not_stable")
            if self.stable_anchor is None:
                self.stable_anchor=(wall_s,pose["stamp"],pose["xyz"],pose["q"])
            anchor=self.stable_anchor
            if min(wall_s-anchor[0],pose["stamp"]-anchor[1])<self.limits.stable_s-1e-9:
                return MotionDecision(0.,self.state,"start_stability_dwell")
            self.target_id=self.draw[0]
            self.target_xy=CANDIDATE_POINTS[self.target_id]
            self.target_yaw=(math.atan2(self.target_xy[1]-pose["xyz"][1],self.target_xy[0]-pose["xyz"][0])
                if self.survey_limits.target_yaw is None else self.survey_limits.target_yaw)
            self.initial_yaw=self.previous_yaw=yaw_from_q(pose["q"])
            self.initial_error=angle_wrap(self.target_yaw-self.initial_yaw)
            if abs(self.initial_error)>self.survey_limits.max_rotation_rad:
                return self.stop("requested_yaw_exceeds_budget",wall_s)
            self.state,self.reason="TURN","bounded_yaw_turn"
            self.pulse_started,self.pulse_origin=wall_s,pose["xyz"]
            return self.step(wall_s=wall_s)

    def survey_evidence(self):
        with self._lock:
            return {"limits":asdict(self.survey_limits),"draw":copy.deepcopy(self.draw),
                "target_id":self.target_id,"target_xy":self.target_xy,"target_yaw":self.target_yaw,
                "initial_yaw":self.initial_yaw,"accumulated_rotation_rad":self.accumulated_rotation_rad,
                "resume_count":self.resume_count,"max_resumes":3,
                "candidate_mapping":{"status":"candidate_unverified",
                    "hypothesis":"ground_N -> static ground_task_point_N",
                    "source":"public candidate inspection_task1.world; live 720-beam static-map agreement",
                    "official_id_mapping_confirmed":False,"candidate_points":CANDIDATE_POINTS},
                "trajectory_10hz":copy.deepcopy(self.trajectory),"trajectory_dropped":self.trajectory_dropped,
                "actual_commands":copy.deepcopy(self.actual_commands),"actual_commands_dropped":self.actual_commands_dropped,
                "exposure_records":[{**copy.deepcopy(record),"phase":phase}
                    for records in self.exposures.values() for phase,record in records.items()],
                "image_errors":list(self.image_errors),"observed_odom_periods":self.observed_periods(),
                "task_completion":"unverified"}

    def additional_events(self):
        """Native-sized JPEG evidence split into ordinary bounded diagnostic events.

        image_sample retains its existing thumbnail contract. These separately
        named public-image fragments preserve up to 800px for later recognition.
        """
        for topic in ("/camera/image","/uav/camera/image"):
            missing="native_image_missing:"+topic
            if topic not in self.image_frames and missing not in self.image_errors and len(self.image_errors)<16:
                self.image_errors.append(missing)
        survey=self.survey_evidence()
        metadata={key:value for key,value in survey.items() if key not in ("trajectory_10hz","actual_commands")}
        common={"mode":self.mode,"read_only":False,"official_scoring":"unverified","partial":True}
        events=[("probe_snapshot",{**common,"section":"survey_metadata","survey":metadata})]
        # Compact rows preserve all recorded values while keeping each series
        # independent of image uploads and full-snapshot envelope size.
        trajectory_columns=["wall_elapsed_s","stamp_s","x","y","z","qx","qy","qz","qw",
                            "yaw_rad","linear_speed","angular_speed"]
        trajectory_rows=[[round(row["wall_elapsed_s"],6),round(row["stamp_s"],6),
                          *[round(value,7) for value in row["position"]+row["quaternion"]],
                          round(row["yaw_rad"],7),round(row["linear_speed"],7),round(row["angular_speed"],7)]
                         for row in survey["trajectory_10hz"]]
        command_columns=["wall_elapsed_s","clock_stamp_s","linear_x","angular_z","state"]
        command_rows=[[round(row["wall_elapsed_s"],6),None if row["clock_stamp_s"] is None else round(row["clock_stamp_s"],6),
                       row["linear_x"],row["angular_z"],row["state"]] for row in survey["actual_commands"]]
        for name,columns,rows in (("trajectory_10hz",trajectory_columns,trajectory_rows),
                                  ("actual_commands",command_columns,command_rows)):
            # Explicit offsets allow independent reassembly in long or noisy
            # runs; 300 compact rows remain below a 64KB envelope.
            for offset in range(0,max(1,len(rows)),300):
                events.append(("probe_snapshot",{**common,"section":"survey_motion_series","series":name,
                    "columns":columns,"offset":offset,"total_rows":len(rows),"rows":rows[offset:offset+300]}))
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            self.image_errors.append("native_image_dependency:"+str(exc)[:128])
            events.append(("probe_snapshot",{**common,"section":"survey_image_errors","errors":self.image_errors[:16]}))
            return events
        jobs=[(topic,phase) for phase in ("last_associated","last","first")
              for topic in ("/uav/camera/image","/camera/image")
              if phase in self.image_frames.get(topic,{})]
        seen=set()
        for topic,phase in jobs:
                msg=self.image_frames[topic][phase]
                try:
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                    if (topic,stamp) in seen:
                        continue
                    seen.add((topic,stamp))
                    channels={"rgb8":3,"bgr8":3,"mono8":1}.get(msg.encoding)
                    if channels is None or not 0<msg.width<=4096 or not 0<msg.height<=4096:
                        raise ValueError("invalid_native_image_shape_or_encoding")
                    raw=memoryview(msg.data).cast("B")
                    if len(raw)!=msg.step*msg.height or msg.step<msg.width*channels or len(raw)>64*1024*1024:
                        raise ValueError("invalid_native_image_stride")
                    pixels=np.frombuffer(raw,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels]
                    pixels=pixels.reshape(msg.height,msg.width,channels) if channels==3 else pixels.reshape(msg.height,msg.width)
                    if msg.encoding=="rgb8":pixels=cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
                    scale=min(1.,800/max(msg.width,msg.height))
                    if scale<1:pixels=cv2.resize(pixels,(round(msg.width*scale),round(msg.height*scale)))
                    ok,jpeg=cv2.imencode(".jpg",pixels,[cv2.IMWRITE_JPEG_QUALITY,75])
                    if not ok:raise ValueError("native_jpeg_encode_failed")
                    jpeg_bytes=jpeg.tobytes()
                    if len(jpeg_bytes)>384*1024:raise ValueError("native_jpeg_evidence_size_limit")
                    encoded=base64.b64encode(jpeg_bytes).decode("ascii")
                    chunks=[encoded[i:i+32000] for i in range(0,len(encoded),32000)]
                    for index,chunk in enumerate(chunks):
                        events.append(("probe_snapshot",{"mode":self.mode,"read_only":False,"official_scoring":"unverified",
                            "section":"survey_image_part","partial":True,"topic":topic,"phase":phase,"stamp_s":stamp,
                            "frame_id":msg.header.frame_id,"width":pixels.shape[1],"height":pixels.shape[0],
                            "encoding":"jpeg","jpeg_sha256":hashlib.sha256(jpeg_bytes).hexdigest(),
                            "part_index":index,"part_count":len(chunks),"jpeg_base64_part":chunk}))
                except Exception as exc:
                    if len(self.image_errors)<16:
                        self.image_errors.append("native_image_error:"+str(exc)[:128])
        if self.image_errors:
            events.append(("probe_snapshot",{**common,"section":"survey_image_errors","errors":list(self.image_errors)}))
        return events

    def snapshot_survey_evidence(self):
        survey=self.survey_evidence()
        trajectory=survey.pop("trajectory_10hz")
        commands=survey.pop("actual_commands")
        survey["trajectory_count"]=len(trajectory)
        survey["actual_command_count"]=len(commands)
        survey["motion_series_transport"]="independent compact survey_motion_series events"
        return survey


def create_survey_node(reporter=None,*,wall_clock=time.monotonic,survey_limits=None):
    survey_limits=survey_limits or SurveyLimits()
    def factory(limits,*,started_wall_s):
        return GroundSurveyGuard(limits,started_wall_s=started_wall_s,survey_limits=survey_limits)
    return create_bounded_node(MotionLimits(.447,.114,.02),reporter,wall_clock=wall_clock,guard_factory=factory)


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser=argparse.ArgumentParser(description="Ground-only bounded candidate observation yaw survey")
    parser.add_argument("--duration",type=float,default=45.)
    parser.add_argument("--target-yaw",type=float,default=None)
    parser.add_argument("--minimum-yaw-rate",type=float,default=.04)
    parser.add_argument("--max-turn-wall",type=float,default=25.)
    parser.add_argument("--report-url",default=os.environ.get("INSPECTION_DIAGNOSTIC_URL",""))
    parsed,ros_args=parser.parse_known_args(args)
    limits=SurveyLimits(target_yaw=parsed.target_yaw,global_wall_s=parsed.duration,
        minimum_yaw_rate=parsed.minimum_yaw_rate,max_turn_wall_s=parsed.max_turn_wall)
    reporter=DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node=None
    try:
        node=create_survey_node(reporter,survey_limits=limits)
        node.report_started(parsed.duration)
        spin_control_loop(node,rclpy,duration_s=parsed.duration)
        node.stop("survey_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node,timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("survey_cleanup")
            node.close_watchdog()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        reporter.close()


if __name__=="__main__":main()
