"""v34 bounded timing observation around the unchanged v31 height factory.

Timing adds overhead. It neither changes permission gates nor authenticates
source publication times. Nested spans must not be added together.
"""
from __future__ import annotations
import argparse
from collections import deque
import copy
from dataclasses import asdict
import math
import os
import threading
import time
import types

from .bounded_ground_probe import spin_control_loop
from .bounded_height_hold import HEIGHT_LIMITS,create_height_node
from .bounded_vertical_response import WORLD_SHA256

INPUTS=frozenset(('/clock','/uav/odom','/odom','/scan','/uav/laser/scan','/tf','/tf_static'))
LABELS=frozenset(('spin_once','tick','control_tick','dds:control_tick','dds:publish_boundary'))|frozenset('receive:'+t for t in INPUTS)


def source_stamp(topic,msg):
    """Only scalar metadata; no array conversion or payload copying."""
    def get(value,key):return value[key] if isinstance(value,dict) else getattr(value,key)
    try:
        if topic=='/clock':stamp=get(msg,'clock')
        else:
            if topic in ('/tf','/tf_static'):
                transforms=get(msg,'transforms')
                if not transforms:return None
                msg=transforms[0]
            stamp=get(get(msg,'header'),'stamp')
        sec,nano=get(stamp,'sec'),get(stamp,'nanosec')
        if type(sec) is int and type(nano) is int and sec>=0 and 0<=nano<1_000_000_000:
            return sec+nano/1e9
    except (AttributeError,KeyError,IndexError,TypeError):pass
    return None


class HeightTimingRecorder:
    revision='v34_timing'
    TRACE_CAPACITY=2048
    SLOW_CAPACITY=32

    def __init__(self,*,wall_clock=time.monotonic,cpu_clock=time.thread_time):
        self.wall_clock,self.cpu_clock=wall_clock,cpu_clock
        self.started_wall=wall_clock()
        self.labels={}
        self.trace=deque(maxlen=self.TRACE_CAPACITY)
        self.slow=[]
        self.trace_dropped=self.slow_dropped=self.instrumentation_errors=0

    def begin_span(self,label,**metadata):
        return {'label':label if label in LABELS else 'other',
            'entry_wall_s':self.wall_clock()-self.started_wall,'cpu_start_s':self.cpu_clock(),**metadata}

    def end(self,token):
        if token is None:return
        exit_wall=self.wall_clock()-self.started_wall
        cpu=self.cpu_clock()-token['cpu_start_s']
        duration=exit_wall-token['entry_wall_s']
        if not all(math.isfinite(x) and x>=0. for x in (duration,cpu)):
            self.instrumentation_errors+=1;return
        row={k:v for k,v in token.items() if k!='cpu_start_s'}
        row.update(exit_wall_s=exit_wall,wall_s=duration,cpu_s=cpu)
        stat=self.labels.setdefault(row['label'],{'count':0,'wall_total_s':0.,'wall_max_s':0.,
            'cpu_total_s':0.,'cpu_max_s':0.,'first_entry_wall_s':row['entry_wall_s'],'last_entry_wall_s':None})
        stat['count']+=1
        stat['wall_total_s']+=duration;stat['wall_max_s']=max(stat['wall_max_s'],duration)
        stat['cpu_total_s']+=cpu;stat['cpu_max_s']=max(stat['cpu_max_s'],cpu)
        stat['last_entry_wall_s']=row['entry_wall_s']
        if len(self.trace)==self.TRACE_CAPACITY:self.trace_dropped+=1
        self.trace.append(row)
        if duration>=.02:
            self.slow.append(row);self.slow.sort(key=lambda x:x['wall_s'],reverse=True)
            if len(self.slow)>self.SLOW_CAPACITY:self.slow.pop();self.slow_dropped+=1

    def snapshot(self):
        return {'revision':self.revision,'wall_clock':'time.monotonic','cpu_clock':'time.thread_time',
            'measurement':'entry/exit around unchanged calls; observer adds overhead; nested spans overlap',
            'trace_capacity':self.TRACE_CAPACITY,'trace_dropped':self.trace_dropped,
            'slow_capacity':self.SLOW_CAPACITY,'slow_dropped':self.slow_dropped,
            'instrumentation_errors':self.instrumentation_errors,'labels':copy.deepcopy(self.labels),
            'trace':copy.deepcopy(list(self.trace)),'slow_spans':copy.deepcopy(self.slow),
            'unmeasured':['source publication wall time','DDS readiness and take/deserialize split',
                'host CPU scheduling or quota','guard lock wait separately'],
            'thread_contract':'original single-thread callback/tick executor; watchdog zero has no DDS query',
            'source_stamp':'first header for TF only; original clock/header scalar, not wall publication time'}


def create_timing_height_node(reporter=None,*,wall_clock=time.monotonic,cpu_clock=time.thread_time):
    node=create_height_node(reporter,wall_clock=wall_clock)
    rec=HeightTimingRecorder(wall_clock=wall_clock,cpu_clock=cpu_clock)
    node.callback_observer=rec
    node.height_timing=rec
    context=threading.local()

    def begin(label,**metadata):
        try:return rec.begin_span(label,**metadata)
        except Exception:rec.instrumentation_errors+=1;return None
    def end(token):
        try:rec.end(token)
        except Exception:rec.instrumentation_errors+=1

    original_receive=node.receive
    def receive(self,topic,msg):
        token=begin('receive:'+topic,topic=topic)
        try:
            if token is not None:
                token.update(source_stamp_s=None,state_at_entry=self.guard.state,phase_at_entry=self.guard.phase)
                try:token['source_stamp_s']=source_stamp(topic,msg)
                except Exception:rec.instrumentation_errors+=1
            return original_receive(topic,msg)
        finally:end(token)
    node.receive=types.MethodType(receive,node)

    original_tick=node.tick
    def tick(self):
        token=begin('control_tick',state_at_entry=self.guard.state,phase_at_entry=self.guard.phase)
        try:return original_tick()
        finally:end(token)
    node.tick=types.MethodType(tick,node)

    original_publish=node._publish
    def publish(self,speed):
        previous=getattr(context,'in_publish',False)
        context.in_publish=True
        try:return original_publish(speed)
        finally:context.in_publish=previous
    node._publish=types.MethodType(publish,node)

    class QueryObservedPublisher:
        def __init__(self,publisher):self.publisher=publisher
        def __getattr__(self,name):return getattr(self.publisher,name)
        def publish(self,message):return self.publisher.publish(message)
        def get_subscription_count(self):
            label='dds:publish_boundary' if getattr(context,'in_publish',False) else 'dds:control_tick'
            token=begin(label,topic='/uav/cmd_vel')
            try:return self.publisher.get_subscription_count()
            finally:end(token)
    node.publisher_by_topic['/uav/cmd_vel']=QueryObservedPublisher(node.publisher_by_topic['/uav/cmd_vel'])
    original_evidence=node.guard.evidence
    def evidence(self):
        data=original_evidence();data['runtime_timing']=rec.snapshot()
        return data
    node.guard.evidence=types.MethodType(evidence,node.guard)
    return node


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-url',default=os.environ.get('INSPECTION_DIAGNOSTIC_URL',''))
    parsed,ros_args=parser.parse_known_args(args)
    reporter=DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args);node=None
    try:
        node=create_timing_height_node(reporter)
        reporter.emit('probe_started',{'mode':node.guard.mode,'revision':'v34_timing','read_only':False,
            'limits':asdict(HEIGHT_LIMITS),'world_sha256':WORLD_SHA256,
            'official_scoring':'unverified','full_mission_complete':False})
        node.arm_watchdog()
        spin_control_loop(node,rclpy,duration_s=HEIGHT_LIMITS.global_wall_s)
        node.stop('height_experiment_cleanup')
        for _ in range(5):node.publish_zero();rclpy.spin_once(node,timeout_sec=.02)
        node.close_watchdog();node.report_final()
    finally:
        if node is not None:
            node.stop('height_experiment_cleanup');node.close_watchdog();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        reporter.close()


if __name__=='__main__':main()
