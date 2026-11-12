"""Axis three-ground-point candidate runtime with an explicit reverse segment.

Frozen v25/v26 image, sensor and transport gates remain authoritative. Software
verification is not evidence of actual reverse response or official scoring.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass, replace
import math
import os
import time

from .bounded_ground_probe import MotionDecision, MotionLimits, create_bounded_node, finite, spin_control_loop
from .ground_observation_route import RouteCallbackTimingRecorder
from .ground_observation_survey import angle_wrap
from .ground_single_point import GroundSinglePointCoordinator
from .ground_three_point import GroundThreePointGuard, ThreePointLimits, _RouteView
from .ground_three_point_axis_plan import plan_ground_leg, plan_ground_entry
from .ground_three_point_plan import _turn_clearance, RING_THRESHOLD_M


DRIVES = frozenset(("DRIVE_TO_XY", "DRIVE_REVERSE_TO_XY"))


@dataclass(frozen=True)
class AxisThreePointLimits(ThreePointLimits):
    """A new pre-run contract; all caps except total angle remain unchanged."""
    max_rotation_rad: float = 28.

    def __post_init__(self):
        caps = {**asdict(ThreePointLimits()), "target_distance_m": 3., "max_rotation_rad": 28.}
        for key, cap in caps.items():
            if not finite(getattr(self, key)) or not 0 < getattr(self, key) <= cap:
                raise ValueError(key+" exceeds axis-three-ground limit")
        if (self.minimum_yaw_rate > self.maximum_yaw_rate
                or self.scan_anchor_motion_limit_m+self.local_stop_reserve_m > self.local_swept_motion_budget_m+1e-9):
            raise ValueError("inconsistent local reserve or angular bounds")


class GroundThreePointAxisGuard(GroundThreePointGuard):
    mode = "ground_three_point_axis"

    def __init__(self, *, started_wall_s, route_limits=None, session_id=None):
        limits = route_limits or AxisThreePointLimits()
        if not isinstance(limits, AxisThreePointLimits):
            raise ValueError("AxisThreePointLimits required for independent axis contract")
        self.reverse_segment = False
        super().__init__(started_wall_s=started_wall_s, route_limits=limits, session_id=session_id)

    def measured_forward(self):
        progress = super().measured_forward()
        return -progress if self.reverse_segment and progress is not None else progress

    def _turn_tolerance(self):
        if self.operation_index+1 < len(self.operations) and self.odom is not None:
            following = self.operations[self.operation_index+1]
            if following.kind in DRIVES:
                distance = math.dist(self.odom["xyz"][:2], following.target_xy_m)
                lateral = .003 if following.kind == "DRIVE_REVERSE_TO_XY" else .01
                return min(.025, math.atan2(lateral, max(distance, .01)))
        return .025

    def _desired_body_yaw(self, op):
        yaw = math.atan2(op.target_xy_m[1]-self.odom["xyz"][1],
                         op.target_xy_m[0]-self.odom["xyz"][0])
        return angle_wrap(yaw+math.pi) if op.kind == "DRIVE_REVERSE_TO_XY" else yaw

    def _plan_point(self, previous, target, wall):
        try:
            plan = plan_ground_leg(previous, target, self._current_pose())
        except ValueError as exc:
            return self.stop("three_ground_candidate_plan_rejected:"+str(exc)[:256], wall)
        fault = self._remaining_plan_fault(plan.path_m, plan.rotation_rad)
        if fault:
            return self.stop(fault, wall)
        self.point_plan = plan
        self.target_id, self.target_xy = target, plan.endpoint_xy_m
        self.route_plan = _RouteView(target, plan.initial_pose, plan.endpoint_xy_m,
            (plan.observation_xy_m,), plan.observation_yaw_rad, self.frozen_limits)
        self._load_operations(plan.to_observation_operations, "observation", wall)
        return MotionDecision(0., self.state, self.reason)

    def _start_operation(self, wall):
        # Reset before every operation, including TURN, empty lists and STOP.
        self.reverse_segment = False
        if self.state in ("STOP", "FINISHED"):
            return
        if self.operation_index == len(self.operations):
            return super()._start_operation(wall)
        op = self.operations[self.operation_index]
        reverse_turn = (op.kind == "TURN_ONLY" and self.operation_index+1 < len(self.operations)
            and self.operations[self.operation_index+1].kind == "DRIVE_REVERSE_TO_XY")
        if op.kind != "DRIVE_REVERSE_TO_XY" and not reverse_turn:
            return super()._start_operation(wall)
        self.drive_origin = self.drive_yaw = None
        self.operation_records.append({"point_index": self.point_index, "target_id": self.target_id,
            "role": self.current_operations_role, "operation_index": self.operation_index,
            "operation": asdict(op), "actual_start_pose": self._current_pose(), "started_wall_s": wall,
            "path_origin_m": self.total_path_m, "rotation_origin_rad": self.accumulated_rotation_rad})
        if reverse_turn:
            if (math.dist(self.odom["xyz"][:2], op.anchor_xy_m) > .02+1e-9
                    or abs(angle_wrap(self._current_pose()[2]-op.expected_start_yaw_rad)) > .05+1e-9
                    or not 0 < abs(op.signed_subturn_rad) <= math.pi/2+1e-9):
                return self.stop("three_ground_turn_anchor_outside_contract", wall)
            target = self._desired_body_yaw(self.operations[self.operation_index+1])
            error = angle_wrap(target-self._current_pose()[2])
            if abs(error) > self._turn_tolerance() and error*op.signed_subturn_rad <= 0:
                return self.stop("three_ground_turn_direction_mismatch", wall)
            if _turn_clearance(self.odom["xyz"][:2], self._current_pose()[2], error) < RING_THRESHOLD_M-1e-12:
                return self.stop("three_ground_actual_turn_candidate_unsafe", wall)
            self.target_yaw, self.initial_turn_error = target, error
            self.operation_records[-1]["runtime_target_yaw_rad"] = target
            self.turn_origin, self.turn_started_wall = self.odom["xyz"], wall
            self._enter_phase("TURN")
            return
        self.leg_target_xy = tuple(op.target_xy_m)
        self.target_yaw = self._desired_body_yaw(op)
        distance = math.dist(self.odom["xyz"][:2], self.leg_target_xy)
        if distance <= .002:
            self.operation_records[-1]["within_2mm_no_motion"] = True
            self.operation_index += 1
            return self._start_operation(wall)
        if (distance <= .02 or abs(angle_wrap(self.target_yaw-self._current_pose()[2]))
                > min(.025, math.atan2(.01, distance))+1e-9):
            return self.stop("three_ground_drive_heading_outside_contract", wall)
        self.reverse_segment = True
        self.drive_origin, self.drive_yaw = self.odom["xyz"], self.target_yaw
        self.leg_target_distance_m = distance
        self.approach_limits = replace(self.approach_limits, target_distance_m=distance-.002)
        self.leg_stop_path_m = None
        self.drive_started_wall = wall
        self.leg_records.append({"point_index": self.point_index, "leg_index": len(self.leg_records),
            "origin": self.drive_origin, "yaw_rad": self.drive_yaw, "reverse": True,
            "waypoint_xy_m": self.leg_target_xy, "target_projection_m": distance, "started_wall_s": wall})
        self._enter_phase("DRIVE")

    def _begin_entry(self, wall):
        if (self.state != "REPORT_WAIT" or self.phase != "REPORT_WAIT"
                or not self.attempts.get("recognition", {}).get("sent") or self.recognition is None
                or self.recognition.get("first_id") != self.target_id
                or self.recognition.get("stationary_epoch") != self.stationary_epoch
                or len(self.match_results) != 3 or self._stationary_epoch_fault()
                or self._current_recognition_condition(wall) or self._condition(wall)):
            return self.stop("three_ground_entry_visual_gate_missing", wall)
        try:
            operations = plan_ground_entry(self.target_id, self._current_pose())
        except ValueError as exc:
            return self.stop("three_ground_candidate_entry_rejected:"+str(exc)[:256], wall)
        fault = self._remaining_plan_fault(sum(op.length_m for op in operations),
                                           sum(abs(op.signed_subturn_rad) for op in operations))
        if fault:
            return self.stop(fault, wall)
        self.point_visual = {"stationary_epoch": self.stationary_epoch,
            "recognition": copy.deepcopy(self.recognition), "match_results": copy.deepcopy(self.match_results),
            "capture_frames": list(self.capture_frames)}
        self.in_entry = True
        self._load_operations(operations, "entry", wall)
        return MotionDecision(0., self.state, self.reason)

    def _handoff(self, wall):
        self._archive_point()
        if self.point_index == 2:
            return self.stop("three_ground_transport_complete_official_score_unverified", wall)
        target = self.mission_draw[self.point_index+1]
        try:
            plan = plan_ground_leg(self.target_id, target, self._current_pose())
        except ValueError as exc:
            return self.stop("three_ground_candidate_plan_rejected:"+str(exc)[:256], wall)
        fault = self._remaining_plan_fault(plan.path_m, plan.rotation_rad)
        if fault:
            return self.stop(fault, wall)
        previous = self.target_id
        self._enter_phase("POINT_HANDOFF")
        self.point_index += 1
        self.attempts = {}
        self.matcher_failures = []
        self.public_status = []
        self.recapture_count = 0
        self.in_entry = False
        self.point_visual = None
        return self._plan_point(previous, target, wall)

    def _linear_request(self, wall):
        if not self.initial_slow_validated:
            if self.initial_slow_anchor is None:
                self.initial_slow_anchor = (wall, self.odom["stamp"])
            if min(wall-self.initial_slow_anchor[0], self.odom["stamp"]-self.initial_slow_anchor[1]) < 1.-1e-9:
                magnitude = min(.03, self.approach_limits.linear_command_mps)
                return -magnitude if self.reverse_segment else magnitude
            self.initial_slow_validated = True
            self.speed_validation.update(slow_validated_wall_s=wall,
                slow_validated_odom_stamp_s=self.odom["stamp"],
                slow_continuous_wall_s=wall-self.initial_slow_anchor[0],
                slow_continuous_sim_s=self.odom["stamp"]-self.initial_slow_anchor[1])
        final_drive = not any(op.kind in DRIVES for op in self.operations[self.operation_index+1:])
        magnitude = min(.03 if final_drive and self.leg_target_distance_m-self.measured_forward() <= .03+1e-9 else .10,
                        self.approach_limits.linear_command_mps)
        return -magnitude if self.reverse_segment else magnitude

    def step(self, *, wall_s):
        with self._lock:
            # Parent handles forward re-aim. Reverse must retain body yaw,
            # rather than aiming the body at the destination behind it.
            if (self.state not in ("STOP", "FINISHED") and self.phase == "TURN_SETTLE"
                    and self.operation_index+1 < len(self.operations)
                    and self.operations[self.operation_index+1].kind == "DRIVE_REVERSE_TO_XY"
                    and self.odom is not None):
                desired = self._desired_body_yaw(self.operations[self.operation_index+1])
                delta = angle_wrap(desired-self._current_pose()[2])
                if _turn_clearance(self.odom["xyz"][:2], self._current_pose()[2], delta) < RING_THRESHOLD_M-1e-12:
                    return self.stop("three_ground_actual_turn_candidate_unsafe", wall_s)
                self.target_yaw = desired
            return super().step(wall_s=wall_s)

    def approach_evidence(self):
        with self._lock:
            data = super().approach_evidence()
            data["revision"] = "ground_three_point_axis_v1"
            data["three_point"].update(planner="ground_three_point_axis_plan",
                reverse_segment=self.reverse_segment, reverse_response_verified_on_platform=False)
            return data


def create_axis_three_point_node(reporter=None, *, wall_clock=time.monotonic, route_limits=None, worker=None):
    limits = route_limits or AxisThreePointLimits()
    def factory(_limits, *, started_wall_s):
        return GroundThreePointAxisGuard(started_wall_s=started_wall_s, route_limits=limits)
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
    parser = argparse.ArgumentParser(description="Axis three-point ground practice; reverse response unverified")
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
        node = create_axis_three_point_node(reporter, route_limits=limits, worker=worker)
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
