"""Independent bounded camera-gap recovery for the frozen axis ground route.

Only a valid camera's positive age gap, before a recognition transport attempt,
may pause and re-capture. Camera/transport limits and cumulative budgets stay
unchanged. Local software checks are not official platform scoring evidence.
"""
from __future__ import annotations

import argparse
import copy
import os
import time

from .bounded_ground_probe import MotionDecision, MotionLimits, create_bounded_node, finite, spin_control_loop
from .ground_observation_route import RouteCallbackTimingRecorder
from .ground_single_point import CURRENT_CAMERA_MAX_AGE_S, GroundSinglePointCoordinator
from .ground_three_point_axis import (
    AxisThreePointLimits, GroundThreePointAxisGuard, create_axis_three_point_node,
)


CAMERA_AGE_FAULTS = frozenset((
    "single_point_uav_image_stale", "single_point_current_image_clock_inconsistent",
))


class GroundThreePointCameraRecoveryGuard(GroundThreePointAxisGuard):
    mode = "ground_three_point_camera_recovery"

    def __init__(self, *, started_wall_s, route_limits=None, session_id=None):
        self.camera_recovery_events = []
        self.camera_recovery_events_dropped = 0
        self.camera_recovery_pending = False
        super().__init__(started_wall_s=started_wall_s, route_limits=route_limits, session_id=session_id)

    def _camera_event(self, event, reason, wall):
        image = self.last_uav_image
        clock = None if self.clock is None else self.clock[0]
        self.camera_recovery_events.append({
            "event": event, "reason": reason, "wall_s": wall,
            "wall_elapsed_s": wall-self.started_wall if finite(wall) else None,
            "clock_stamp_s": clock,
            "image_arrival_wall_s": None if image is None else image[0],
            "image_stamp_s": None if image is None else image[1],
            "image_wall_age_s": None if image is None or not finite(wall) else wall-image[0],
            "clock_minus_image_stamp_s": None if image is None or clock is None else clock-image[1],
            "camera_max_age_s": CURRENT_CAMERA_MAX_AGE_S,
            "camera_min_clock_age_s": -self.limits.clock_sampling_lag_s,
            "phase": self.phase, "state": self.state,
            "target_id": self.target_id, "point_index": self.point_index,
            "epoch": self.stationary_epoch, "resume_count": self.resume_count,
            "recapture_count": self.recapture_count, "max_resumes": 3,
            "max_recaptures_per_point": 1,
        })
        if len(self.camera_recovery_events) > 16:
            self.camera_recovery_events.pop(1)
            self.camera_recovery_events_dropped += 1

    def _camera_age_values_valid(self, wall):
        if self.last_uav_image is None or self.clock is None or not finite(wall):
            return False
        arrival, stamp = self.last_uav_image
        return (all(finite(v) for v in (arrival, stamp, self.clock[0]))
                and wall-arrival >= -1e-9
                and self.clock[0]-stamp >= -self.limits.clock_sampling_lag_s-1e-9)

    def _current_recognition_condition(self, wall):
        # CAPTURE needs the same current-camera check as recognition. Frozen
        # capture exposure/prior gates still apply independently in the parent.
        if self.phase not in ("CAPTURE", "RECOGNIZE_WAIT", "REPORT_WAIT"):
            return ""
        age = None if self.last_uav_image is None else wall-self.last_uav_image[0]
        delta = None if self.last_uav_image is None or self.clock is None else self.clock[0]-self.last_uav_image[1]
        self.last_camera_liveness_check = {
            "wall_s": wall, "phase": self.phase,
            "camera_max_wall_age_s": CURRENT_CAMERA_MAX_AGE_S,
            "camera_max_clock_age_s": CURRENT_CAMERA_MAX_AGE_S,
            "camera_min_clock_age_s": -self.limits.clock_sampling_lag_s,
            "last_image_arrival_wall_s": None if self.last_uav_image is None else self.last_uav_image[0],
            "last_image_stamp_s": None if self.last_uav_image is None else self.last_uav_image[1],
            "image_wall_age_s": age, "clock_minus_image_stamp_s": delta,
            "interpretation": "current camera liveness only; original capture stamps and priors unchanged",
        }
        # Invalid temporal direction takes priority over a simultaneous stale
        # wall age. It can never be reclassified as a recoverable source gap.
        if age is not None and (not finite(age) or age < -1e-9):
            return "single_point_current_image_clock_inconsistent"
        if delta is not None and (not finite(delta) or delta < -self.limits.clock_sampling_lag_s-1e-9):
            return "single_point_current_image_clock_inconsistent"
        if age is None or age > CURRENT_CAMERA_MAX_AGE_S+1e-9:
            return "single_point_uav_image_stale"
        if delta is None or delta > CURRENT_CAMERA_MAX_AGE_S+1e-9:
            return "single_point_current_image_clock_inconsistent"
        observations = self._observations
        history = observations.uav_odom_history
        if (not history or observations.uav_odom_error or not observations.uav_odom_epoch_valid
                or wall-self.started_wall-history[-1]["wall_elapsed_s"] > .20+1e-9):
            return "single_point_uav_pose_stale"
        return ""

    def temporal_fault(self, reason, wall):
        with self._lock:
            if reason not in CAMERA_AGE_FAULTS:
                return super().temporal_fault(reason, wall)
            recoverable = (
                self.state not in ("STOP", "FINISHED")
                and self.phase in ("CAPTURE", "RECOGNIZE_WAIT")
                and "recognition" not in self.attempts
                and self._camera_age_values_valid(wall)
                and (wall-self.last_uav_image[0] > CURRENT_CAMERA_MAX_AGE_S+1e-9
                     or self.clock[0]-self.last_uav_image[1] > CURRENT_CAMERA_MAX_AGE_S+1e-9)
            )
            if not recoverable:
                return self.stop(reason, wall)
            self.epoch_invalid = "sensor_freshness_lost"
            self.stable_anchor = None
            self.state, self.reason = "PAUSE", reason
            if not self.camera_recovery_pending:
                self.camera_recovery_pending = True
                self._camera_event("pause", reason, wall)
            return MotionDecision(0., self.state, self.reason)

    def observe_image(self, topic, msg, *, wall_s):
        with self._lock:
            if topic != "/uav/camera/image" or self.state in ("STOP", "FINISHED"):
                return super().observe_image(topic, msg, wall_s=wall_s)
            try:
                if not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall):
                    raise ValueError("wall_clock_reversed_or_invalid")
                sec, nano = msg.header.stamp.sec, msg.header.stamp.nanosec
                if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 10**9:
                    raise ValueError("invalid_capture_image_stamp")
                if msg.header.frame_id != "uav/camera_link":
                    raise ValueError("invalid_capture_image_frame")
                stamp = sec+nano/1e9
                if self.last_uav_image is not None and stamp < self.last_uav_image[1]:
                    raise ValueError("capture_image_stamp_reversed")
                if (self.phase in ("CAPTURE", "RECOGNIZE_WAIT", "REPORT_WAIT")
                        and self.clock is not None
                        and self.clock[0]-stamp < -self.limits.clock_sampling_lag_s-1e-9):
                    raise ValueError("capture_image_clock_direction_invalid")
                channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
                if (channels is None or type(msg.width) is not int or type(msg.height) is not int
                        or type(msg.step) is not int or not 0 < msg.width <= 4096 or not 0 < msg.height <= 4096):
                    raise ValueError("invalid_capture_image_shape_or_encoding")
                raw = memoryview(msg.data).cast("B")
                if len(raw) != msg.step*msg.height or msg.step < msg.width*channels or len(raw) > 64*1024*1024:
                    raise ValueError("invalid_capture_image_stride")
            except (AttributeError, TypeError, ValueError, BufferError) as exc:
                reason = "camera_source_invalid:"+str(exc)[:128]
                self._camera_event("invalid_source_stop", reason, wall_s)
                return self.stop(reason, wall_s)
            # Keep the original header, arrival and immutable ROS pixel buffer.
            # The parent does not refresh camera arrival on duplicate stamps.
            return super().observe_image(topic, msg, wall_s=wall_s)

    def step(self, *, wall_s):
        with self._lock:
            if self.state in ("STOP", "FINISHED"):
                return super().step(wall_s=wall_s)
            if self.phase not in ("CAPTURE", "RECOGNIZE_WAIT"):
                return super().step(wall_s=wall_s)
            hard = self._budget_fault(wall_s)
            if hard:
                return self.stop(hard, wall_s)
            # Even an early zero-only pause consumes its wall observation.
            # Otherwise a later tick could move backwards without a callback.
            self.last_wall = wall_s
            hard = self.observed_motion_fault()
            if hard:
                return self.stop(hard, wall_s)
            control = self._condition(wall_s)
            if control:
                self.temporal_fault(control, wall_s)
                return MotionDecision(0., self.state, self.reason)
            camera = self._current_recognition_condition(wall_s)
            if camera:
                self.temporal_fault(camera, wall_s)
                if self.camera_recovery_pending and self.state == "PAUSE":
                    self.stable_anchor = None
                return MotionDecision(0., self.state, self.reason)
            old_epoch = self.stationary_epoch
            pending = self.camera_recovery_pending
            # The original recovery branch owns the unchanged 1s dual-clock
            # stationary dwell, per-point/global quotas and fresh epoch reset.
            decision = super().step(wall_s=wall_s)
            if pending and self.stationary_epoch != old_epoch and self.state == "CAPTURE":
                self.camera_recovery_pending = False
                self._camera_event("recapture", self.reason, wall_s)
            elif pending and self.state in ("STOP", "FINISHED"):
                self._camera_event("recovery_stopped", self.reason, wall_s)
                self.camera_recovery_pending = False
            return decision

    def approach_evidence(self):
        with self._lock:
            data = super().approach_evidence()
            data["revision"] = "ground_three_point_camera_recovery_v1"
            data["camera_recovery"] = {
                "pending": self.camera_recovery_pending,
                "events": copy.deepcopy(self.camera_recovery_events),
                "events_dropped": self.camera_recovery_events_dropped,
                "camera_max_age_s": CURRENT_CAMERA_MAX_AGE_S,
                "max_resumes": 3, "max_recaptures_per_point": 1,
                "official_acceptance": False,
            }
            return data


def create_camera_recovery_three_point_node(reporter=None, *, wall_clock=time.monotonic, route_limits=None,
                                          worker=None, guard_factory=None):
    limits = route_limits or AxisThreePointLimits()
    def factory(_limits, *, started_wall_s):
        constructor = GroundThreePointCameraRecoveryGuard if guard_factory is None else guard_factory
        guard = constructor(started_wall_s=started_wall_s, route_limits=limits)
        if not isinstance(guard, GroundThreePointCameraRecoveryGuard):
            raise ValueError("GroundThreePointCameraRecoveryGuard or its subclass required")
        if guard.frozen_limits is not limits or guard.started_wall != started_wall_s:
            raise ValueError("guard factory must retain the same frozen contract and start wall")
        return guard
    node = create_bounded_node(MotionLimits(.447,.114,.02), reporter, wall_clock=wall_clock,
        guard_factory=factory, graph_refresh_interval_s=.5, callback_observer=RouteCallbackTimingRecorder(),
        input_reliability_overrides={"/odom":"BEST_EFFORT","/uav/odom":"BEST_EFFORT"})
    from geometry_msgs.msg import Twist
    from rosidl_runtime_py.utilities import get_message
    String = get_message("std_msgs/msg/String")
    original_refresh, original_snapshot = node.refresh_endpoints, node.evidence_snapshot
    counts = {topic:{"attempted":0,"returned":0}
        for topic in ("/inspection/detections","/referee/current_target")}
    node.axis_publish_attempts = []
    node.axis_publish_attempts_dropped = 0
    coordinator = None if worker is None else GroundSinglePointCoordinator(node.guard, worker)

    def error_record(reason):
        if reason not in node.errors and len(node.errors)<64:
            node.errors.append(reason)

    def refresh():
        original_refresh()
        for topic,values in counts.items():
            node.command_endpoints.setdefault(topic,{}).update(silent=False,
                publication_policy="one_attempt_per_current_visual_and_physical_ground_point",
                actual_publish_attempt_count=values["attempted"],actual_publish_return_count=values["returned"],
                interpretation="enabled_policy_not_evidence_of_referee_acceptance")
        node.record_endpoint_history()

    def attempt(x,w):
        """One transport attempt; failures never count as returned commands."""
        started=wall_clock()
        record={"wall_elapsed_s":started-node.guard.started_wall,"linear_x":x,"angular_z":w,
            "phase":node.guard.phase,"state":node.guard.state,"returned":False,"error":None}
        try:
            publisher=node.publisher_by_topic.get("/cmd_vel")
            if publisher is None:
                raise RuntimeError("cmd_vel publisher missing")
            message=Twist();message.linear.x=x;message.angular.z=w
            publisher.publish(message)
            record["returned"]=True
            node.record_command(x,w)
            if x or w:
                node.guard.published_nonzero_count+=1
                node.arm_watchdog()
            else:
                node.guard.published_zero_count+=1
            return True
        except Exception as exc:
            reason="axis_command_publish_failed:"+type(exc).__name__+":"+str(exc)[:128]
            record["error"]=reason
            error_record(reason)
            node.guard.stop(reason,wall_clock())
            return False
        finally:
            record["returned_wall_elapsed_s"]=wall_clock()-node.guard.started_wall
            node.axis_publish_attempts.append(record)
            if len(node.axis_publish_attempts)>1200:
                node.axis_publish_attempts.pop(1)
                node.axis_publish_attempts_dropped+=1

    def publish_zero():
        with node.command_lock,node.guard._lock:
            # Each call is independent. The watchdog survives a failed zero.
            return attempt(0.,0.)

    def send(topic,data):
        publisher=node.publisher_by_topic.get(topic)
        if publisher is None or publisher.get_subscription_count()<=0:
            raise RuntimeError("no matching official String subscriber: "+topic)
        message=String();message.data=data
        counts[topic]["attempted"]+=1
        publisher.publish(message)
        counts[topic]["returned"]+=1

    def zero_decision():
        return MotionDecision(0.,node.guard.state,node.guard.reason)

    def tick():
        # Descriptive graph work stays outside the command lock so the
        # independent watchdog can still request zero during a blocked query.
        try:
            matched=node.check_command_match()
            if not matched and node.guard.state not in ("WAIT","STOP","FINISHED"):
                node.stop("no_external_matching_cmd_subscriber")
            elif node.last_graph_refresh_wall is None or wall_clock()-node.last_graph_refresh_wall>=.5-1e-9:
                refresh()
            # A positive cache from before the other topic queries is not
            # motion permission. Check actual DDS after all graph queries.
            matched=node.check_command_match()
        except Exception as exc:
            reason="axis_endpoint_refresh_failed:"+type(exc).__name__+":"+str(exc)[:128]
            error_record(reason)
            node.stop(reason)
            matched=False
        with node.command_lock,node.guard._lock:
            g=node.guard
            if not matched and g.state not in ("WAIT","STOP","FINISHED"):
                g.stop("no_external_matching_cmd_subscriber",wall_clock())
            # The final DDS query itself may consume time or trigger a source
            # callback. Derive a new decision only after it has returned.
            decision=g.step(wall_s=wall_clock())
            if decision.linear_x or decision.angular_z:
                fault=g._budget_fault(wall_clock()) or g.observed_motion_fault() or g._condition(wall_clock())
                if fault:
                    g.temporal_fault(fault,wall_clock())
                    decision=zero_decision()
                elif (g.state!=decision.state or g.state not in ("DRIVE","TURN")
                        or abs(decision.linear_x)>.10+1e-9 or abs(decision.angular_z)>.5+1e-9
                        or (decision.linear_x and decision.angular_z)
                        or not all(finite(v) for v in (decision.linear_x,decision.angular_z))):
                    g.stop("axis_output_phase_or_bound_mismatch",wall_clock())
                    decision=zero_decision()
            if not attempt(decision.linear_x,decision.angular_z):
                # Even a negative publish that raised may have partially
                # reached transport. Lock STOP and independently attempt zero.
                attempt(0.,0.)
                decision=zero_decision()
            if not node.motion_log or (node.motion_log[-1]["state"],node.motion_log[-1]["reason"])!=(decision.state,decision.reason):
                node.motion_log.append({"wall_elapsed_s":wall_clock()-g.started_wall,
                    "state":decision.state,"reason":decision.reason,"linear_x":decision.linear_x,
                    "angular_z":decision.angular_z,"timing":g.timing_snapshot(wall_s=wall_clock())})
                if len(node.motion_log)>64:
                    node.motion_log.pop(1);node.motion_log_dropped+=1
        try:
            if coordinator is not None:
                coordinator.poll_once(wall_clock())
            with node.command_lock:
                node.guard.dispatch_one(send,wall_s=wall_clock())
                for topic,values in counts.items():
                    node.command_endpoints.setdefault(topic,{}).update(
                        actual_publish_attempt_count=values["attempted"],actual_publish_return_count=values["returned"])
        except Exception as exc:
            node.stop("three_ground_coordinator_failed:"+type(exc).__name__+":"+str(exc)[:256])
        return zero_decision() if node.guard.state in ("STOP","FINISHED","PAUSE") else decision

    def snapshot():
        data=original_snapshot()
        data.update(axis_publish_attempts=copy.deepcopy(node.axis_publish_attempts),
            axis_publish_attempts_dropped=node.axis_publish_attempts_dropped)
        return data

    node.refresh_endpoints,node.publish_zero,node.tick,node.evidence_snapshot=refresh,publish_zero,tick,snapshot
    node.single_point_coordinator=coordinator
    refresh()
    return node


def main(args=None):
    import rclpy
    from .ground_matcher_worker import GroundMatcherWorker
    from .ground_single_point import load_ground_templates
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description="Axis three-point ground practice with bounded pre-report camera recovery")
    parser.add_argument("--duration", type=float, default=1200.)
    parser.add_argument("--template-dir", default=None)
    parser.add_argument("--report-url", default=os.environ.get("INSPECTION_DIAGNOSTIC_URL", ""))
    parsed, ros_args = parser.parse_known_args(args)
    limits = AxisThreePointLimits(global_wall_s=parsed.duration)
    worker = GroundMatcherWorker(load_ground_templates(parsed.template_dir))
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_camera_recovery_three_point_node(reporter, route_limits=limits, worker=worker)
        node.report_started(parsed.duration)
        node.arm_watchdog()
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop("three_ground_global_wall_time_limit")
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop("three_ground_axis_cleanup")
            node.close_watchdog()
            node.destroy_node()
        worker.close(timeout_s=.5)
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == "__main__":
    main()
