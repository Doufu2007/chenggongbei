"""Timing observes real callbacks; it never replaces motion/header clocks."""
import importlib
import importlib.util
import json
from types import SimpleNamespace

import pytest

from test_bounded_ground_probe import install_ros_transport,odom


def api():
    name="inspection_mission.ground_callback_timing"
    assert importlib.util.find_spec(name) is not None,"independent v17 timing entry missing"
    return importlib.import_module(name)


def test_recorder_uses_injected_perf_and_thread_clocks_and_original_header_age():
    wall=[10.];cpu=[2.]
    rec=api().CallbackTimingRecorder(perf_counter=lambda:wall[0],thread_time=lambda:cpu[0])
    payload=odom(8.)
    token=rec.begin_receive("/odom",payload,(8.18,111.),"PAUSE")
    wall[0]+=.003;cpu[0]+=.001
    rec.end(token)
    snapshot=rec.snapshot();span=snapshot["spans"]["receive:/odom"]
    assert span["wall_s"]["count"]==1 and span["wall_s"]["total"]==pytest.approx(.003)
    assert span["cpu_s"]["total"]==pytest.approx(.001)
    assert span["clock_minus_header_s"]["min"]==pytest.approx(.18)
    assert payload["header"]["stamp"]=={"sec":8,"nanosec":0}
    assert snapshot["guard_lock_wait"]["status"]=="not_measured"


def test_native_image_measurement_does_not_read_or_copy_pixels():
    rec=api().CallbackTimingRecorder()
    class HeaderOnlyImage:
        header=SimpleNamespace(stamp=SimpleNamespace(sec=7,nanosec=50_000_000),frame_id="uav/camera_link")
        @property
        def data(self):raise AssertionError("timing must not touch image bytes")
    token=rec.begin_receive("/uav/camera/image",HeaderOnlyImage(),(7.1,0.),"DRIVE")
    rec.end(token)
    assert rec.snapshot()["spans"]["receive:/uav/camera/image"]["clock_minus_header_s"]["max"]==pytest.approx(.05)


def test_recorder_storage_and_encoded_snapshot_remain_bounded_under_flood():
    rec=api().CallbackTimingRecorder()
    for i in range(2000):
        token=rec.begin_receive("/untrusted/topic/"+str(i),odom(float(i)),(float(i),0.),"DRIVE")
        rec.end(token)
    result=rec.snapshot()
    assert len(result["spans"])<=17 and len(result["slow_events"])<=12
    assert len(json.dumps(result).encode())<65536


def test_optional_adapter_observer_records_qos_and_durations_without_rewriting_guard_wall(monkeypatch):
    from inspection_mission.bounded_ground_probe import MotionLimits,create_bounded_node
    callbacks,_,_=install_ros_transport(monkeypatch)
    rec=api().CallbackTimingRecorder();wall=[1.]
    node=create_bounded_node(MotionLimits(.447,.114,.02),wall_clock=lambda:wall[0],callback_observer=rec)
    callbacks["/clock"]({"clock":{"sec":8,"nanosec":100_000_000}})
    callbacks["/odom"](odom(8.))
    result=node.evidence_snapshot()["callback_timing"]
    assert node.guard.odom["wall"]==1. and node.guard.odom["stamp"]==8.
    assert result["spans"]["receive:/odom"]["wall_s"]["count"]==1
    assert result["input_qos"]["/odom"]["requested"]["depth"]==1
    assert "reliability" in result["input_qos"]["/odom"]["requested"]
    node.close_watchdog()


def test_control_loop_observes_spin_once_and_tick_without_changing_command_cadence():
    from inspection_mission.bounded_ground_probe import spin_control_loop
    now=[0.];ticks=[];rec=api().CallbackTimingRecorder()
    class Node:
        callback_observer=rec
        def tick(self):ticks.append(now[0]);return SimpleNamespace(state="WAIT")
    class Ros:
        @staticmethod
        def ok():return True
        @staticmethod
        def spin_once(node,timeout_sec):now[0]+=.001
    spin_control_loop(Node(),Ros,duration_s=1.,wall_clock=lambda:now[0])
    snapshot=rec.snapshot()
    assert snapshot["spans"]["spin_once"]["wall_s"]["count"]>=900
    assert snapshot["spans"]["tick"]["wall_s"]["count"]==len(ticks)
    assert 19<=len(ticks)<=21 and min(b-a for a,b in zip(ticks,ticks[1:]))>=.05-1e-9


def test_timing_variant_uses_same_sync_guard_and_original_input_qos(monkeypatch):
    from inspection_mission.ground_bounded_approach_sync import GroundSynchronizedApproachGuard
    from inspection_mission.ground_bounded_approach import ApproachLimits
    install_ros_transport(monkeypatch)
    node=api().create_timing_approach_node(wall_clock=lambda:0.)
    assert isinstance(node.guard,GroundSynchronizedApproachGuard)
    assert node.guard.approach_limits==ApproachLimits()
    assert node.experiment_mode=="ground_bounded_approach"
    assert node.callback_observer.snapshot()["revision"]=="v17_timing"
    node.close_watchdog()


def test_split_final_report_preserves_optional_timing_metrics(monkeypatch):
    from inspection_mission.telemetry import DiagnosticReporter
    install_ros_transport(monkeypatch);events=[]
    class Reporter:
        def send_now(self,event,payload,timeout_s):events.append(json.loads(DiagnosticReporter._encode(event,payload)))
        def emit(self,event,payload):return True
    node=api().create_timing_approach_node(Reporter(),wall_clock=lambda:0.)
    node.report_final()
    core=next(row["data"] for row in events if row["data"].get("section")=="ground_motion_core")
    assert core["callback_timing"]["revision"]=="v17_timing"
    assert len(DiagnosticReporter._encode("probe_snapshot",core))<65536
    node.close_watchdog()


def test_all_topic_histograms_qos_and_max_slow_trace_fit_real_core_envelope(monkeypatch):
    from inspection_mission.bounded_ground_probe import INPUT_TYPES
    from inspection_mission.telemetry import DiagnosticReporter
    install_ros_transport(monkeypatch)
    node=api().create_timing_approach_node(wall_clock=lambda:0.)
    perf=[0.];cpu=[0.]
    rec=api().CallbackTimingRecorder(perf_counter=lambda:perf[0],thread_time=lambda:cpu[0])
    node.callback_observer=rec
    for topic in INPUT_TYPES:
        rec.record_input_qos(topic,{"depth":10,"reliability":"ReliabilityPolicy.RELIABLE",
            "durability":"DurabilityPolicy.TRANSIENT_LOCAL"},[{"reliability":"ReliabilityPolicy.RELIABLE",
            "durability":"DurabilityPolicy.TRANSIENT_LOCAL"}]*4)
        for i in range(25):
            token=rec.begin_receive(topic,odom(10.+i), (10.2+i,0.),"PAUSE")
            perf[0]+=.03;cpu[0]+=.01;rec.end(token)
    for key in ("spin_once","tick"):
        token=rec.begin_span(key);perf[0]+=.1;cpu[0]+=.02;rec.end(token)
    snap=node.evidence_snapshot()
    assert len(rec.snapshot()["slow_events"])==12
    assert len(DiagnosticReporter._encode("probe_snapshot",snap))<65536
    node.close_watchdog()
