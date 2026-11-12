"""v40: three ground points and a real return on the same permanent ledger.

This is an independently bounded candidate, not evidence of platform success.
Frozen v38/v36 sources and all sensor, geometry and output gates are retained.
There is no takeoff, illumination or full-mission completion declaration.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
import math
import os
import time

from .bounded_ground_probe import MotionDecision, finite, spin_control_loop
from .ground_observation_survey import angle_wrap
from .ground_return_plan import plan_ground_return
from .ground_three_point_axis import AxisThreePointLimits
from .ground_three_point_camera_recovery import create_camera_recovery_three_point_node
from .ground_three_point_recovery_budget import GroundThreePointRecoveryBudgetGuard


@dataclass(frozen=True)
class ReturnThreePointLimits(AxisThreePointLimits):
    max_path_m: float = 15.
    max_rotation_rad: float = 40.
    global_wall_s: float = 1800.
    turn_total_wall_s: float = 1000.
    drive_wall_s: float = 650.

    def __post_init__(self):
        # New pre-run scope; never mutate a frozen parent's caps at runtime.
        caps = {**asdict(AxisThreePointLimits()), 'target_distance_m': 3., 'max_path_m': 15.,
                'max_rotation_rad': 40., 'global_wall_s': 1800.,
                'turn_total_wall_s': 1000., 'drive_wall_s': 650.}
        for key, cap in caps.items():
            if not finite(getattr(self, key)) or not 0 < getattr(self, key) <= cap:
                raise ValueError(key + ' exceeds independent ground-return limit')
        if (self.minimum_yaw_rate > self.maximum_yaw_rate
                or self.scan_anchor_motion_limit_m + self.local_stop_reserve_m
                > self.local_swept_motion_budget_m + 1e-9):
            raise ValueError('inconsistent ground-return local reserve or angular bounds')


class GroundThreePointReturnGuard(GroundThreePointRecoveryBudgetGuard):
    mode = 'ground_three_point_return'
    ACTIVE = GroundThreePointRecoveryBudgetGuard.ACTIVE | {'RETURN_SETTLE'}
    EXTRA_ZERO = GroundThreePointRecoveryBudgetGuard.EXTRA_ZERO | {'RETURN_SETTLE'}
    ZERO_PHASES = GroundThreePointRecoveryBudgetGuard.ZERO_PHASES | {'RETURN_SETTLE'}

    def __init__(self, *, started_wall_s, route_limits=None, session_id=None):
        limits = route_limits or ReturnThreePointLimits()
        if not isinstance(limits, ReturnThreePointLimits):
            raise ValueError('ReturnThreePointLimits required for independent return contract')
        self.return_started = self.return_ready = self.return_completed = False
        self.return_plan = self.return_started_wall = self.return_ready_wall = None
        self.return_start_ledger = self.return_completion = None
        super().__init__(started_wall_s=started_wall_s, route_limits=limits, session_id=session_id)

    def _handoff(self, wall):
        if self.state in ('STOP', 'FINISHED') or self.return_started:
            return MotionDecision(0., self.state, self.reason)
        if self.point_index != 2:
            return super()._handoff(wall)
        event = self.attempts.get('declaration', {})
        if (self.phase != 'DECLARE_WAIT' or self.state != 'DECLARE_WAIT'
                or not event.get('sent') or not finite(event.get('wall_s'))
                or wall - event['wall_s'] < 2. - 1e-9):
            return self.stop('return_third_declaration_gate_missing', wall)
        self._archive_point()
        fault = self._budget_fault(wall) or self.observed_motion_fault() or self._condition(wall)
        if fault:
            self.temporal_fault(fault, wall)
            return MotionDecision(0., self.state, self.reason)
        if not self._stable_dwell(wall):
            return MotionDecision(0., self.state, 'return_third_point_stability_dwell')
        try:
            plan = plan_ground_return(self._current_pose())
        except (ValueError, TypeError, KeyError) as exc:
            return self.stop('return_candidate_plan_rejected:' + str(exc)[:256], wall)
        fault = self._remaining_plan_fault(plan.path_m, plan.rotation_rad)
        if fault:
            return self.stop(fault, wall)
        self.return_plan, self.return_started, self.return_started_wall = plan, True, wall
        self.return_start_ledger = {
            'session_id': self.session_id, 'started_wall_s': self.started_wall,
            'global_origin': self.global_origin, 'total_path_m': self.total_path_m,
            'accumulated_rotation_rad': self.accumulated_rotation_rad,
            'turn_wall_s': self._elapsed_group('turn', wall),
            'drive_wall_s': self._elapsed_group('drive', wall),
            'resume_count': self.resume_count, 'pause_wall_s': self.completed_pause_wall_s,
        }
        self.in_entry = False
        self.point_visual = None  # All three points and original frames are archived.
        self._load_operations(plan.operations, 'return', wall)
        return MotionDecision(0., self.state, self.reason)

    def _start_operation(self, wall):
        if (self.return_started and self.current_operations_role == 'return'
                and self.operation_index == len(self.operations)
                and self.state not in ('STOP', 'FINISHED')):
            # The parent's non-observation completion branch would declare the
            # third point again. A new dwell starts only after actual operations.
            self.reverse_segment = False
            self.drive_origin = self.drive_yaw = None
            self.target_yaw = 0.
            self.return_ready = False
            self.return_ready_wall = None
            self._enter_phase('RETURN_SETTLE')
            return
        return super()._start_operation(wall)

    def _home_pose_valid(self):
        return (self.odom is not None and math.hypot(*self.odom['xyz'][:2]) <= .02 + 1e-9
                and abs(angle_wrap(self._current_pose()[2])) <= .025 + 1e-9)

    def temporal_fault(self, reason, wall):
        if self.return_started:
            self.return_ready = False
            self.return_ready_wall = None
        return super().temporal_fault(reason, wall)

    def step(self, *, wall_s):
        with self._lock:
            if (not self.return_started or self.phase != 'RETURN_SETTLE'
                    or self.state in ('STOP', 'FINISHED', 'PAUSE')):
                return super().step(wall_s=wall_s)
            self.return_ready = False
            self.return_ready_wall = None
            if not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall):
                return self.stop('wall_clock_reversed_or_invalid', wall_s)
            self.last_wall = wall_s
            fault = self._budget_fault(wall_s) or self.observed_motion_fault() or self._condition(wall_s)
            if fault:
                self.temporal_fault(fault, wall_s)
            elif not self._home_pose_valid():
                self.stable_anchor = None
                self.reason = 'return_home_pose_outside_contract'
            elif self._stable_dwell(wall_s):
                self.return_ready = True
                self.return_ready_wall = wall_s
                self.reason = 'return_waiting_for_successful_zero_publish'
            return MotionDecision(0., self.state, self.reason)

    def confirm_return_zero(self, attempt, *, wall_s):
        """Called only after the reviewed adapter's actual publish returned."""
        with self._lock:
            if (not self.return_ready or self.state != 'RETURN_SETTLE' or self.phase != 'RETURN_SETTLE'
                    or self.return_completed or not isinstance(attempt, dict)
                    or not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall)):
                return False
            # Publish may have blocked. Expired live control evidence must
            # clear readiness before rejecting an old/unsuitable attempt row.
            fault = self._budget_fault(wall_s) or self.observed_motion_fault() or self._condition(wall_s)
            if fault:
                self.temporal_fault(fault, wall_s)
                return False
            started = attempt.get('wall_elapsed_s')
            ended = attempt.get('returned_wall_elapsed_s')
            if (attempt.get('returned') is not True or attempt.get('error') is not None
                    or attempt.get('linear_x') != 0. or attempt.get('angular_z') != 0.
                    or attempt.get('phase') != 'RETURN_SETTLE' or attempt.get('state') != 'RETURN_SETTLE'
                    or not finite(started) or not finite(ended) or ended < started
                    or self.started_wall + started < self.return_ready_wall - 1e-9
                    or not 0 <= wall_s - self.started_wall - ended <= self.limits.sensor_age_s + 1e-9):
                return False
            if not self._home_pose_valid() or not self._stable_dwell(wall_s):
                self.return_ready = False
                self.return_ready_wall = None
                return False
            anchor = self.stable_anchor
            self.return_completion = {
                'wall_s': wall_s, 'odom_stamp_s': self.odom['stamp'],
                'pose': self._current_pose(), 'stationary_wall_s': wall_s - anchor[0],
                'stationary_source_s': self.odom['stamp'] - anchor[1],
                'linear_speed_mps': self.odom['speed'], 'angular_speed_rps': self.odom['yaw_speed'],
                'zero_publish_attempt': copy.deepcopy(attempt),
                'total_path_m': self.total_path_m, 'accumulated_rotation_rad': self.accumulated_rotation_rad,
            }
            self.return_completed = True
            self.stop('three_ground_return_complete_official_score_unverified', wall_s)
            return True

    def dispatch_one(self, publish, *, wall_s):
        if self.return_started:
            return None
        return super().dispatch_one(publish, wall_s=wall_s)

    def register_prepared_jobs(self, jobs, *, epoch):
        if self.return_started:
            return False
        return super().register_prepared_jobs(jobs, epoch=epoch)

    def accept_match_result(self, result, *, wall_s):
        if self.return_started:
            return False
        return super().accept_match_result(result, wall_s=wall_s)

    def observe_image(self, topic, msg, *, wall_s):
        if self.return_started:
            return None
        return super().observe_image(topic, msg, wall_s=wall_s)

    def approach_evidence(self):
        with self._lock:
            data = super().approach_evidence()
            data['revision'] = 'ground_three_point_return_v40'
            data['ground_return'] = {
                'started': self.return_started, 'ready': self.return_ready, 'complete': self.return_completed,
                'plan': None if self.return_plan is None else asdict(self.return_plan),
                'started_wall_s': self.return_started_wall,
                'start_ledger': copy.deepcopy(self.return_start_ledger),
                'completion': copy.deepcopy(self.return_completion),
                'official_acceptance': False, 'full_mission_complete': False,
                'interpretation': 'three_ground_transport_and_local_return_only_pending_platform_validation',
            }
            return data

    def additional_events(self):
        events = super().additional_events()
        evidence = self.approach_evidence()['ground_return']
        return events + [('probe_snapshot', {
            'mode': self.mode, 'revision': 'ground_three_point_return_v40',
            'section': 'ground_return_core', 'read_only': False, 'official_scoring': 'unverified',
            'session_id': self.session_id, 'state': self.state, 'reason': self.reason, **evidence,
        })]


def create_return_three_point_node(reporter=None, *, wall_clock=time.monotonic,
                                   route_limits=None, worker=None):
    limits = route_limits or ReturnThreePointLimits()
    node = create_camera_recovery_three_point_node(
        reporter, wall_clock=wall_clock, route_limits=limits, worker=worker,
        guard_factory=lambda *, started_wall_s, route_limits: GroundThreePointReturnGuard(
            started_wall_s=started_wall_s, route_limits=route_limits))
    original_tick = node.tick

    def tick():
        decision = original_tick()
        with node.command_lock, node.guard._lock:
            if node.axis_publish_attempts:
                node.guard.confirm_return_zero(node.axis_publish_attempts[-1], wall_s=wall_clock())
            if node.guard.state in ('STOP', 'FINISHED', 'PAUSE'):
                return MotionDecision(0., node.guard.state, node.guard.reason)
        return decision

    node.tick = tick
    return node


def main(args=None):
    import rclpy
    from .ground_matcher_worker import GroundMatcherWorker
    from .ground_single_point import load_ground_templates
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=1800.)
    parser.add_argument('--template-dir', default=None)
    parser.add_argument('--report-url', default=os.environ.get('INSPECTION_DIAGNOSTIC_URL', ''))
    parsed, ros_args = parser.parse_known_args(args)
    limits = ReturnThreePointLimits(global_wall_s=parsed.duration)
    worker = GroundMatcherWorker(load_ground_templates(parsed.template_dir))
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_return_three_point_node(reporter, route_limits=limits, worker=worker)
        node.report_started(parsed.duration)
        node.arm_watchdog()
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop('three_ground_return_global_wall_time_limit')
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop('three_ground_return_cleanup')
            node.close_watchdog()
            node.destroy_node()
        worker.close(timeout_s=.5)
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == '__main__':
    main()
