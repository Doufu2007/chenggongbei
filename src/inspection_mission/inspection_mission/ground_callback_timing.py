"""v17: observe callback/executor costs without changing v16 motion or QoS."""
from __future__ import annotations

import argparse
import bisect
import copy
import math
import os
import time

from .bounded_ground_probe import INPUT_TYPES, MotionLimits, create_bounded_node, spin_control_loop
from .ground_bounded_approach import ApproachLimits
from .ground_bounded_approach_sync import GroundSynchronizedApproachGuard


class CallbackTimingRecorder:
    revision = "v17_timing"
    DURATION_BOUNDS = (.0001, .001, .005, .02, .05, .1, .2, 1.)
    AGE_BOUNDS = (-.2, -.12, 0., .05, .1, .2, .3, 1.)

    def __init__(self, *, perf_counter=time.perf_counter, thread_time=time.thread_time):
        self.perf_counter, self.thread_time = perf_counter, thread_time
        self.started_perf = perf_counter()
        self.spans, self.input_qos = {}, {}
        self._last_entry, self._last_header = {}, {}
        self.slow_events = []
        self.slow_events_dropped = 0

    @staticmethod
    def _stat():
        return {"count": 0, "total": 0., "min": None, "max": None, "buckets": [0]*9}

    @classmethod
    def _add(cls, stat, value, bounds):
        if not isinstance(value, (float, int)) or not math.isfinite(value):
            return
        stat["count"] += 1
        stat["total"] += value
        stat["min"] = value if stat["min"] is None else min(stat["min"], value)
        stat["max"] = value if stat["max"] is None else max(stat["max"], value)
        stat["buckets"][bisect.bisect_left(bounds, value)] += 1

    def _span(self, key):
        return self.spans.setdefault(key, {name: self._stat() for name in (
            "wall_s", "cpu_s", "entry_gap_wall_s", "header_delta_s", "clock_minus_header_s")})

    @staticmethod
    def _header_stamp(msg):
        try:
            header = msg["header"] if isinstance(msg, dict) else msg.header
            stamp = header["stamp"] if isinstance(header, dict) else header.stamp
            sec = stamp["sec"] if isinstance(stamp, dict) else stamp.sec
            nano = stamp["nanosec"] if isinstance(stamp, dict) else stamp.nanosec
            if type(sec) is int and type(nano) is int and sec >= 0 and 0 <= nano < 1_000_000_000:
                return sec+nano/1e9
        except (KeyError, TypeError, AttributeError):
            pass
        return None

    def begin_span(self, key):
        if key not in ("spin_once", "tick"):
            key = "other_span"
        wall = self.perf_counter()
        return {"key": key, "wall": wall, "cpu": self.thread_time(),
            "entry_elapsed_perf_s": wall-self.started_perf}

    def begin_receive(self, topic, msg, clock_sample, state):
        topic = topic if topic in INPUT_TYPES else "__other__"
        wall, cpu = self.perf_counter(), self.thread_time()
        key = "receive:"+topic
        span = self._span(key)
        previous = self._last_entry.get(topic)
        if previous is not None:
            self._add(span["entry_gap_wall_s"], wall-previous, self.DURATION_BOUNDS)
        self._last_entry[topic] = wall
        stamp = self._header_stamp(msg)
        clock = None if clock_sample is None else clock_sample[0]
        if stamp is not None:
            previous = self._last_header.get(topic)
            if previous is not None:
                self._add(span["header_delta_s"], stamp-previous, self.AGE_BOUNDS)
            self._last_header[topic] = stamp
        if clock is not None and stamp is not None:
            self._add(span["clock_minus_header_s"], clock-stamp, self.AGE_BOUNDS)
        return {"key": key, "wall": wall, "cpu": cpu, "header_stamp_s": stamp,
            "entry_elapsed_perf_s": wall-self.started_perf,
            "clock_stamp_at_entry_s": clock, "state_at_entry": state}

    def end(self, token):
        wall, cpu = self.perf_counter()-token["wall"], self.thread_time()-token["cpu"]
        span = self._span(token["key"])
        self._add(span["wall_s"], wall, self.DURATION_BOUNDS)
        self._add(span["cpu_s"], cpu, self.DURATION_BOUNDS)
        if wall >= .02:
            self.slow_events.append({key: value for key, value in token.items() if key not in ("wall", "cpu")}
                                    | {"wall_s": wall, "cpu_s": cpu})
            # Keep the largest delays; fixed capacity, never an unbounded trace.
            self.slow_events.sort(key=lambda row: row["wall_s"], reverse=True)
            if len(self.slow_events) > 12:
                self.slow_events.pop()
                self.slow_events_dropped += 1

    def record_input_qos(self, topic, requested, publisher_profiles):
        if topic in INPUT_TYPES:
            self.input_qos[topic] = {"requested": copy.deepcopy(requested),
                "observed_publisher_profiles": copy.deepcopy(publisher_profiles[:4]),
                "interpretation": "actual subscription request and creation-time public publisher QoS; not latency evidence"}

    def snapshot(self):
        return {"revision": self.revision,
            "measurement": "perf_counter wall durations; thread_time current-thread CPU; original sensor/guard clocks untouched",
            "duration_histogram_upper_bounds_s": list(self.DURATION_BOUNDS),
            "age_histogram_upper_bounds_s": list(self.AGE_BOUNDS),
            "histogram_interpretation": "upper-inclusive bounds plus overflow; spin wall includes wait/take/callback",
            "guard_lock_wait": {"status": "not_measured", "reason": "receive includes lock waits; no extra guard lock acquisition or wrapper introduced"},
            "pre_receive_transport": "not_measured; entry age includes source/bridge/DDS/executor delay; cannot separate them from this metric alone",
            "clock_info": {name: {"implementation": time.get_clock_info(name).implementation,
                "resolution_s": time.get_clock_info(name).resolution} for name in ("perf_counter", "thread_time")},
            "spans": copy.deepcopy(self.spans), "input_qos": copy.deepcopy(self.input_qos),
            "slow_events": copy.deepcopy(self.slow_events), "slow_events_dropped": self.slow_events_dropped}


def create_timing_approach_node(reporter=None, *, wall_clock=time.monotonic, approach_limits=None):
    def factory(limits, *, started_wall_s):
        return GroundSynchronizedApproachGuard(started_wall_s=started_wall_s, approach_limits=approach_limits)
    return create_bounded_node(MotionLimits(.447, .114, .02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5, callback_observer=CallbackTimingRecorder())


def main(args=None):
    import rclpy
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="v17 receive/spin timing; unchanged v16 bounded approach")
    parser.add_argument("--duration", type=float, default=90.)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ApproachLimits(global_wall_s=parsed.duration)
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_timing_approach_node(reporter, approach_limits=limits)
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
