"""v37 fixed initial static-column experiment; no mission or scoring messages.

The finite scan +inf policy applies only under the declared public static world
assumption. It is not a claim of free unknown space or all-direction sensing.
"""
from __future__ import annotations
import argparse
from collections import deque
import copy
from dataclasses import asdict, dataclass
import math
import os
import time
import types

from .bounded_ground_probe import finite, spin_control_loop
from .bounded_vertical_response import (VerticalResponseGuard, VerticalDecision,
    STATIC_BOXES, WORLD_SHA256, _stamp, _pose)
from .bounded_height_hold import feedback_command
from .height_runtime_timing import HeightTimingRecorder, INPUTS, source_stamp, create_timing_height_node


@dataclass(frozen=True)
class CruiseHeightLimits:
    command_mps: float = .05
    target_absolute_z_m: float = .85
    hard_absolute_z_m: float = 1.
    target_rise_m: float = .815
    hard_rise_m: float = .965
    max_horizontal_drift_m: float = .04
    max_tilt_rad: float = .05
    max_speed_mps: float = .15
    max_angular_speed_rps: float = .10
    max_total_path_m: float = 2.
    motion_wall_s: float = 90.
    global_wall_s: float = 110.
    sensor_age_s: float = .20
    clock_sampling_lag_s: float = .12
    stable_s: float = 1.
    observe_s: float = 8.
    ascent_wall_s: float = 25.
    hold_wall_s: float = 8.
    descent_wall_s: float = 45.

    def __post_init__(self):
        fixed=(.05,.85,1.,.815,.965,.04,.05,.15,.10,2.,90.,110.,.20,.12,1.,8.,25.,8.,45.)
        if tuple(asdict(self).values()) != fixed or not all(finite(v) for v in asdict(self).values()):
            raise ValueError('v37 requires its independently declared fixed limits')


CRUISE_LIMITS=CruiseHeightLimits()
ACTIVE_PHASES=frozenset(('ASCEND_CRUISE','HOLD_CRUISE','DESCEND_SUPPORT'))


class CruiseCaptureTiming(HeightTimingRecorder):
    """Separate v37 callback totals; they overlap the retained v34 core spans."""
    revision='v37_capture'
    allowed_labels=frozenset('receive_total:'+topic for topic in INPUTS)|{'cloud_capture','cloud_encode'}

    def begin_span(self,label,**metadata):
        token=super().begin_span('control_tick',**metadata)
        token['label']=label if label in self.allowed_labels else 'other'
        return token

    def snapshot(self):
        data=super().snapshot()
        data['measurement']='v37 outer receive including raw metadata; cloud capture and terminal encoding; overlaps v34 core spans'
        data['core_timing_revision']='v34_timing'
        return data


def cruise_column_geometry(origin):
    valid=(len(origin)==3 and all(finite(v) for v in origin)
        and math.hypot(*origin[:2])<=.05+1e-9 and .035<=origin[2]<=.075)
    radius=.447+.166*math.sin(.05)+.02+.04
    bottom=origin[2]-(.068+.447*math.sin(.05)+.02) if valid else None
    top=1.+.166+.447*math.sin(.05)+.02
    collisions=[]
    if valid:
        for name,low,high in STATIC_BOXES:
            if high[2]<bottom or low[2]>top:continue
            dx=max(low[0]-origin[0],0.,origin[0]-high[0]);dy=max(low[1]-origin[1],0.,origin[1]-high[1])
            if math.hypot(dx,dy)<=radius:collisions.append(name)
    return {'clear':bool(valid and not collisions and top<1.8),'origin':list(origin),
        'radius_m':radius,'bottom_m':bottom,'top_m':top,'collision_boxes':collisions,
        'world_sha256':WORLD_SHA256,'public_plugin_max_z_m':1.8,
        'scope':'fixed public static column; known support floor exception only',
        'static_map_verified_on_platform':False}


class CruiseHeightGuard(VerticalResponseGuard):
    mode='bounded_cruise_height'

    def __init__(self,*,started_wall_s):
        super().__init__(started_wall_s=started_wall_s,limits=CRUISE_LIMITS)
        self.phase='WAIT';self.phase_started_wall=None;self.pending_phase=None
        self.phase_transitions=[];self.phase_samples={}
        self.hold_anchor=None;self.hold_dwell_verified=None;self.hold_dwell_resets=[]
        self.hold_verified=self.descent_verified=self.protocol_complete=self.support_return_verified=False
        self.protocol_abort_reason='';self.scan_semantics=[]
        self.initial_column_validation={'clear':False,'checked':False}
        self.runtime_snapshot=None;self.capture_runtime_snapshot=None;self.cloud_snapshot=None

    def _append(self,name,value,cap):
        caps={'trajectory':8000,'commands':4000,'scans':1500,'downscans':3500,'tf_records':9000}
        return super()._append(name,value,caps.get(name,cap))

    def _budget(self,wall):
        if wall-self.started_wall>=self.limits.global_wall_s-1e-9:self.stop('global_wall_time_limit',wall)
        elif self.motion_started_wall is not None and self.state not in ('STOP','FINISHED') and wall-self.motion_started_wall>=self.limits.motion_wall_s-1e-9:
            self.stop('motion_wall_time_limit',wall)

    def _condition(self,wall):
        condition=super()._condition(wall)
        if condition:return condition
        if self.origin is None:
            check=cruise_column_geometry(self.odom['xyz'])
            self.initial_column_validation={**check,'checked':True,'wall_elapsed_s':wall-self.started_wall,
                'odom_stamp':self.odom['stamp'],'scan_source_stamp':self.scan['stamp'],
                'scan_finite_min_m':self.scan['minimum_m']}
            if not check['clear']:return 'outside_cruise_static_column'
        return ''

    def _wall(self,wall):
        if not super()._wall(wall):return False
        if self.phase_started_wall is not None and self.state not in ('STOP','FINISHED'):
            elapsed=wall-self.phase_started_wall
            limits={'ASCEND_CRUISE':(25.,'ascent_wall_time_limit'),'HOLD_CRUISE':(8.,'hold_wall_time_limit'),
                'DESCEND_SUPPORT':(45.,'descent_wall_time_limit')}
            if self.phase in limits and elapsed>=limits[self.phase][0]-1e-9:
                self._zero_return(limits[self.phase][1],wall)
            elif self.phase=='ZERO_RETURN' and elapsed>=8.-1e-9:self.stop('zero_return_wall_time_limit',wall)
        return True

    def _enter_phase(self,phase,wall,reason):
        previous=self.phase;self.phase=phase;self.phase_started_wall=wall
        self.phase_transitions.append({'wall_elapsed_s':wall-self.started_wall,'from':previous,'to':phase,
            'reason':reason,'odom_stamp':self.odom['stamp'] if self.odom else None})
        self.phase_samples[phase]={'entry_odom_stamp':self.odom['stamp'] if self.odom else None,
            'advancing_odom_count':0,'requested_command_count':0,'first_odom_stamp':None,'last_odom_stamp':None}

    def _zero_return(self,reason,wall,*,completed=False):
        if self.state in ('STOP','FINISHED'):return
        if self.phase!='ZERO_RETURN':
            self._enter_phase('ZERO_RETURN',wall,reason);self.observe_started_wall=wall;self.stable_anchor=None
            self.protocol_complete=completed
            if not completed:self.protocol_abort_reason=reason
        self.pending_phase=None;self.state='OBSERVE';self.reason=reason

    def pause(self,reason,wall_s):
        with self._lock:
            if not finite(wall_s):return self.stop('pause_wall_invalid',wall_s)
            self._budget(max(wall_s,self.last_observation_wall))
            if self.phase in ACTIVE_PHASES and self.state not in ('STOP','FINISHED'):
                self._zero_return('cruise_feedback_interrupted:'+str(reason),wall_s)
            return VerticalDecision(0.,self.state,self.reason)

    def update_odom(self,payload,*,wall_s):
        with self._lock:
            old_stamp=self.odom['stamp'] if self.odom else None
            super().update_odom(payload,wall_s=wall_s)
            if self.odom is None:
                if self.state=='PHASE_GATE':self.stop('cruise_gate_odom_invalid',wall_s)
                return
            sample=self.odom
            if sample['xyz'][2]>=1.-1e-9:
                if self.state in ('STOP','FINISHED'):
                    if 'absolute_height_limit' not in self.post_stop_faults:self.post_stop_faults.append('absolute_height_limit')
                else:self.stop('absolute_height_limit',wall_s)
            if sample['stamp']!=old_stamp and self.phase in self.phase_samples:
                row=self.phase_samples[self.phase]
                if row['entry_odom_stamp'] is not None and sample['stamp']>row['entry_odom_stamp']:
                    row['advancing_odom_count']+=1;row['last_odom_stamp']=sample['stamp']
                    if row['first_odom_stamp'] is None:row['first_odom_stamp']=sample['stamp']
            if self.state=='ASCEND' and self.phase=='HOLD_CRUISE' and (abs(sample['xyz'][2]-.85)>.005+1e-9 or abs(sample['linear'][2])>.01+1e-9):
                self._hold_stable(wall_s)
            self._check_phase_height(wall_s)

    def update_scan(self,payload,*,wall_s):
        with self._lock:
            if not self._wall(wall_s):return
            try:
                source=_stamp(payload,'laser_link');previous=self.sensor_stamps.get('scan')
                if previous is not None and source<previous:return self.stop('scan_stamp_reversed',wall_s)
                amin,amax,delta,rmin,rmax=(payload[k] for k in ('angle_min','angle_max','angle_increment','range_min','range_max'))
                values=payload['ranges']
                if (not all(finite(v) for v in (amin,amax,delta,rmin,rmax)) or not 0<abs(delta)<=.02
                    or not isinstance(values,(tuple,list)) or not 12<=len(values)<=8192 or not 0<=rmin<rmax
                    or abs(amax-(amin+(len(values)-1)*delta))>abs(delta)*.05 or abs(delta)*len(values)<2*math.pi-1e-5):
                    raise ValueError('ring_coverage')
                finite_values=[];positive_inf=0
                for value in values:
                    if finite(value):
                        if not rmin<=value<=rmax:raise ValueError('range_invalid')
                        finite_values.append(value)
                    elif type(value) in (int,float) and value==math.inf:positive_inf+=1
                    else:raise ValueError('invalid_nonfinite_range')
                minimum=min(finite_values,default=None)
                if minimum is not None and minimum<=.641+1e-9:raise ValueError('horizontal_ring_too_close')
                if previous is not None and source==previous:
                    if payload!=self.last_scan_payload:return self.stop('scan_duplicate_changed',wall_s)
                    return
                self.sensor_stamps['scan']=source;self.last_scan_payload=copy.deepcopy(payload)
                self.scan={'stamp':source,'wall':wall_s,'minimum_m':minimum}
                self._append('scans',{'wall_elapsed_s':wall_s-self.started_wall,'message':copy.deepcopy(payload)},1500)
                self.scan_semantics.append({'wall_elapsed_s':wall_s-self.started_wall,'stamp':source,'phase':self.phase,
                    'positive_inf_fraction':positive_inf/len(values),'finite_count':len(finite_values),'finite_min_m':minimum,
                    'policy':'static_initial_column_no_return_only'})
                if len(self.scan_semantics)>1500:self.scan_semantics.pop(1)
                self._check_current_sources(wall_s)
            except (KeyError,TypeError,ValueError,OverflowError) as exc:
                self.scan=None;self.stable_anchor=None
                if self.state in ('ASCEND','PHASE_GATE','OBSERVE'):self.stop('scan_invalid:'+str(exc),wall_s)
                elif self.state=='WAIT':self.reason='scan_invalid:'+str(exc)

    def _target_z(self):
        return None if self.origin is None else self.origin[2]+.02 if self.phase=='DESCEND_SUPPORT' else .85

    def _requested_z(self):
        if self.state!='ASCEND':return 0.
        if self.phase=='ASCEND_CRUISE':return .05
        if self.phase in ('HOLD_CRUISE','DESCEND_SUPPORT'):return feedback_command(self._target_z(),self.odom['xyz'][2])
        return 0.

    def _check_phase_height(self,wall):
        if self.origin is None or self.odom is None or self.state!='ASCEND':return
        rise=self.odom['xyz'][2]-self.origin[2]
        if self.phase=='HOLD_CRUISE' and rise<=.035+1e-9:self._zero_return('hold_low_height_nonzero_aborted',wall)
        elif self.phase=='DESCEND_SUPPORT' and rise<=.025+1e-9:
            self.descent_verified=abs(self.odom['linear'][2])<=.02+1e-9
            self._zero_return('descent_height_and_speed_verified' if self.descent_verified else 'descent_low_height_speed_not_verified',wall,
                completed=self.hold_verified and self.descent_verified)

    def _hold_stable(self,wall):
        good=abs(self.odom['xyz'][2]-.85)<=.005+1e-9 and abs(self.odom['linear'][2])<=.01+1e-9
        if not good:
            if self.hold_anchor is not None:self.hold_dwell_resets.append({'wall_elapsed_s':wall-self.started_wall,'odom_stamp':self.odom['stamp'],'reason':'height_or_vz_outside_hold_band'})
            self.hold_anchor=None;self.hold_dwell_verified=None;return False
        if self.hold_anchor is None:self.hold_anchor=(wall,self.odom['stamp'])
        if wall-self.hold_anchor[0]>=3.-1e-9 and self.odom['stamp']-self.hold_anchor[1]>=3.-1e-9:
            self.hold_dwell_verified={'started_wall_elapsed_s':self.hold_anchor[0]-self.started_wall,'ended_wall_elapsed_s':wall-self.started_wall,
                'started_odom_stamp':self.hold_anchor[1],'ended_odom_stamp':self.odom['stamp']}
            return True
        return False

    def step(self,*,wall_s):
        with self._lock:
            if not self._wall(wall_s):return VerticalDecision(0.,self.state,self.reason)
            self.last_wall=wall_s
            if self.state=='STOP':
                if wall_s-self.stopped_wall>=2.-1e-9:self.state='FINISHED'
                return VerticalDecision(0.,self.state,self.reason)
            if self.state=='FINISHED':return VerticalDecision(0.,self.state,self.reason)
            condition=self._condition(wall_s)
            if condition:
                if self.state in ('ASCEND','PHASE_GATE'):self.pause(condition,wall_s)
                elif self.state=='OBSERVE':self.stop('return_sensor_lost:'+condition,wall_s)
                else:self.reason=condition
                self.stable_anchor=None;return VerticalDecision(0.,self.state,self.reason)
            if self.state=='WAIT':
                if not self._stable(wall_s):return VerticalDecision(0.,self.state,'waiting_for_stationary_dwell')
                self.origin=self.odom['xyz'];self.motion_started_wall=wall_s
                self._enter_phase('ASCEND_CRUISE',wall_s,'initial_pose_stable');self.state='ASCEND';self.reason='bounded_cruise_ascent'
            elif self.state=='PHASE_GATE':
                row=self.phase_samples[self.phase]
                if not row['advancing_odom_count'] or not row['requested_command_count']:
                    self._zero_return('cruise_insufficient_advancing_feedback_or_requests',wall_s)
                else:
                    pending=self.pending_phase;self.pending_phase=None
                    self._enter_phase(pending,wall_s,'phase_feedback_verified_after_zero');self.state='ASCEND';self.reason='bounded_cruise_feedback'
            self._check_phase_height(wall_s)
            if self.state=='ASCEND':
                if self.phase=='ASCEND_CRUISE' and self.odom['xyz'][2]>=.85-1e-9:
                    self.target_reached=True;self.state='PHASE_GATE';self.reason='measured_cruise_target_zero_gate';self.pending_phase='HOLD_CRUISE'
                elif self.phase=='HOLD_CRUISE' and self._hold_stable(wall_s):
                    self.hold_verified=True;self.state='PHASE_GATE';self.reason='actual_hold_dwell_verified_zero_gate';self.pending_phase='DESCEND_SUPPORT'
            if self.state=='OBSERVE':
                if abs(self.odom['xyz'][2]-self.origin[2])<=.005+1e-9 and self._stable(wall_s):
                    self.support_return_verified=True;self.state='FINISHED'
                    self.reason='support_return_verified_cruise_protocol_complete' if self.protocol_complete else 'support_return_verified_cruise_protocol_incomplete'
                elif abs(self.odom['xyz'][2]-self.origin[2])>.005+1e-9:self.stable_anchor=None
            return VerticalDecision(self._requested_z(),self.state,self.reason)

    def command_allowed(self,speed):
        return finite(speed) and (speed==0. or (self.state=='ASCEND' and self.phase in ACTIVE_PHASES and .01<=speed<=.05 and abs(speed-self._requested_z())<=1e-12))

    def record_command(self,*,wall_s,linear_z):
        with self._lock:
            if not self.command_allowed(linear_z):raise ValueError('cruise command not allowed')
            if linear_z and self.phase in self.phase_samples:self.phase_samples[self.phase]['requested_command_count']+=1
            target=self._target_z() if self.phase in ACTIVE_PHASES else None
            self._append('commands',{'wall_elapsed_s':wall_s-self.started_wall,'linear_z':linear_z,'ground_linear_x':0.,'ground_angular_z':0.,
                'state':self.state,'phase':self.phase,'target_z':target,'measured_z':self.odom['xyz'][2] if self.odom else None,
                'measured_vz':self.odom['linear'][2] if self.odom else None,
                'height_error_m':target-self.odom['xyz'][2] if target is not None and self.odom else None,
                'odom_stamp':self.odom['stamp'] if self.odom else None},4000)

    def evidence(self):
        data=super().evidence()
        with self._lock:
            data.update(schema='bounded_cruise_height_v1',phase=self.phase,pending_phase=self.pending_phase,
                phase_started_wall=self.phase_started_wall,phase_transitions=copy.deepcopy(self.phase_transitions),phase_samples=copy.deepcopy(self.phase_samples),
                hold_verified=self.hold_verified,descent_verified=self.descent_verified,hold_dwell_verified=copy.deepcopy(self.hold_dwell_verified),
                hold_dwell_resets=copy.deepcopy(self.hold_dwell_resets),protocol_complete=self.protocol_complete,
                protocol_abort_reason=self.protocol_abort_reason,support_return_verified=self.support_return_verified,
                initial_column_validation=copy.deepcopy(self.initial_column_validation),scan_semantics=copy.deepcopy(self.scan_semantics),
                absolute_target_z_m=.85,absolute_hard_z_m=1.,runtime_timing=self.runtime_snapshot() if self.runtime_snapshot else None,
                capture_runtime_timing=self.capture_runtime_snapshot() if self.capture_runtime_snapshot else None,
                feedback={'trim':.02695,'height_to_vz_gain':.5,'inverse_local_gain':.5,'wanted_vz_limit':.03,
                    'request_range':[.01,.05],'target_absolute_z_m':.85,'descent_rise_m':.02,
                    'model_status':'v34_local_interpolation_unverified_at_cruise_height'},
                forward_cloud_control_used=False,forward_cloud_collection=copy.deepcopy(self.cloud_snapshot) if self.cloud_snapshot is not None else {'finalized':False})
            data['assumptions'].update(initial_support_floor='known initial support; positive-trim descent; final zero return, not independent touchdown',
                positive_inf_scan_policy='static_initial_column_no_return_only',public_plugin_max_z_m=1.8,
                body_world_positive_z_contract='fixed near-level initial-column experiment only')
        return data

    def diagnostic_events(self):
        events=super().diagnostic_events()
        for _,data in events:
            data['section']=data['section'].replace('bounded_vertical_','bounded_cruise_height_')
            if data['section']=='bounded_cruise_height_core':
                data.update(phase=self.phase,hold_verified=self.hold_verified,descent_verified=self.descent_verified,
                    protocol_complete=self.protocol_complete,support_return_verified=self.support_return_verified,protocol_abort_reason=self.protocol_abort_reason)
        return events


def create_cruise_node(reporter=None,*,wall_clock=time.monotonic,cpu_clock=time.thread_time):
    # Reuse reviewed transport/publish/watchdog implementations, without changing
    # their frozen source or letting the old guard/limits govern the new trial.
    node=create_timing_height_node(reporter,wall_clock=wall_clock,cpu_clock=cpu_clock)
    node.guard=CruiseHeightGuard(started_wall_s=wall_clock())
    node.guard.runtime_snapshot=node.height_timing.snapshot
    capture_timing=CruiseCaptureTiming(wall_clock=wall_clock,cpu_clock=cpu_clock)
    node.cruise_capture_timing=capture_timing;node.guard.capture_runtime_snapshot=capture_timing.snapshot
    def begin(label,**metadata):
        try:return capture_timing.begin_span(label,**metadata)
        except Exception:capture_timing.instrumentation_errors+=1;return None
    def end(token):
        try:capture_timing.end(token)
        except Exception:capture_timing.instrumentation_errors+=1
    from .cruise_cloud_capture import CruiseCloudCapture
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    node.cloud_capture=CruiseCloudCapture();node.cloud_prior=deque(maxlen=256)
    node.cloud_tf=deque(maxlen=128);node.cloud_static_tf=deque(maxlen=16);node.cloud_metadata_errors=0;node.cloud_capture_errors=0
    node.cloud_stage_context={}
    node.cloud_stage_rejections=dict.fromkeys(('terminal_state','ineligible_phase_state','invalid_source_stamp','source_before_phase_entry'),0)
    node.cloud_stage_rejection_records=deque(maxlen=32);node.cloud_stage_rejections_dropped=0
    original_receive=node.receive
    def receive(self,topic,msg):
        token=begin('receive_total:'+topic,topic=topic)
        try:
            result=original_receive(topic,msg)
            if topic in ('/uav/odom','/tf','/tf_static'):
                try:
                    raw=message_to_ordereddict(msg);wall=wall_clock()
                    if topic=='/uav/odom' and self.guard.odom is not None:
                        sample=_pose(raw,'uav/base_link')
                        if sample['stamp']==self.guard.odom['stamp']:
                            self.cloud_prior.append({**sample,'message':copy.deepcopy(raw),'arrival_wall_s':wall,'source':'/uav/odom'})
                    elif topic.startswith('/tf'):
                        records=self.cloud_static_tf if topic=='/tf_static' else self.cloud_tf
                        records.append({'message':copy.deepcopy(raw),'wall_s':wall,'source':topic})
                except Exception:
                    self.cloud_metadata_errors+=1
            return result
        finally:end(token)
    node.receive=types.MethodType(receive,node)
    def cloud(msg):
        # No conversion of PointCloud2 data to a Python list and no encoding here.
        token=begin('cloud_capture',topic='/uav/lidar/points')
        try:
            # Read only scalar eligibility under the guard lock. Large copies
            # below remain outside both control locks, so watchdog zero can run.
            with node.guard._lock:
                context={'phase':node.guard.phase,'state_at_callback_entry':node.guard.state,
                    'callback_entry_wall_s':wall_clock(),
                    'phase_entry_source_stamp_s':node.guard.phase_samples.get(node.guard.phase,{}).get('entry_odom_stamp')}
            stamp=source_stamp('/uav/lidar/points',msg)
            context['cloud_source_stamp_s']=stamp
            phase,state=context['phase'],context['state_at_callback_entry']
            entry=context['phase_entry_source_stamp_s']
            reason=('terminal_state' if state in ('STOP','FINISHED') else
                'ineligible_phase_state' if not ((phase=='WAIT' and state=='WAIT') or (phase in ACTIVE_PHASES and state=='ASCEND')) else
                'invalid_source_stamp' if stamp is None else
                'source_before_phase_entry' if entry is not None and stamp<entry else '')
            if token is not None:token.update(context)
            if reason:
                node.cloud_stage_rejections[reason]+=1
                if len(node.cloud_stage_rejection_records)==node.cloud_stage_rejection_records.maxlen:node.cloud_stage_rejections_dropped+=1
                node.cloud_stage_rejection_records.append({'reason':reason,**context})
                return
            captured=node.cloud_capture.capture(msg,phase=phase,wall_s=context['callback_entry_wall_s'],trajectory=list(node.cloud_prior),
                tf_records=list(node.cloud_static_tf)+list(node.cloud_tf))
            if captured:node.cloud_stage_context[phase]=context
        except Exception:node.cloud_capture_errors+=1
        finally:end(token)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    node.subscriptions_keep.append(node.create_subscription(get_message('sensor_msgs/msg/PointCloud2'),'/uav/lidar/points',cloud,qos))
    def finalize(self):
        if self.guard.state not in ('STOP','FINISHED'):raise RuntimeError('cloud encoding requires terminal zero-output state')
        token=begin('cloud_encode')
        try:encoded=self.cloud_capture.snapshot(encode=True)  # outside guard/command locks
        finally:end(token)
        encoded['raw_metadata_errors']=self.cloud_metadata_errors
        encoded['callback_errors']=self.cloud_capture_errors
        encoded['stage_rejections']={'counters':dict(self.cloud_stage_rejections),
            'records':copy.deepcopy(list(self.cloud_stage_rejection_records)),'records_dropped':self.cloud_stage_rejections_dropped}
        for frame in encoded.get('clouds',[]):frame['stage_context']=copy.deepcopy(self.cloud_stage_context[frame['phase']])
        self.guard.cloud_snapshot=encoded
    node.finalize_cloud_capture=types.MethodType(finalize,node)
    return node


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-url',default=os.environ.get('INSPECTION_DIAGNOSTIC_URL',''))
    parsed,ros_args=parser.parse_known_args(args);reporter=DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args);node=None
    try:
        node=create_cruise_node(reporter)
        reporter.emit('probe_started',{'mode':node.guard.mode,'revision':'v37_cruise','read_only':False,'limits':asdict(CRUISE_LIMITS),
            'world_sha256':WORLD_SHA256,'official_scoring':'unverified','full_mission_complete':False})
        node.arm_watchdog();spin_control_loop(node,rclpy,duration_s=110.)
        node.stop('cruise_experiment_cleanup')
        for _ in range(5):node.publish_zero();rclpy.spin_once(node,timeout_sec=.02)
        node.close_watchdog();node.finalize_cloud_capture();node.report_final()
    finally:
        if node is not None:node.stop('cruise_experiment_cleanup');node.close_watchdog();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        reporter.close()

if __name__=='__main__':main()
