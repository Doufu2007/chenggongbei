"""Finite retry resources for a continuous ground route, declared before run.

This changes how many fully qualified stationary recoveries may occur. It
never changes the camera, control-source, geometry or command permission gates.
Frozen v36 and its parents retain their original defaults.
"""
from __future__ import annotations

import argparse
import copy
import os
import time

from .bounded_ground_probe import MotionDecision, finite, spin_control_loop
from .ground_three_point_axis import AxisThreePointLimits
from .ground_three_point_camera_recovery import (
    GroundThreePointCameraRecoveryGuard, create_camera_recovery_three_point_node,
)


MAX_RESUMES = 32
SINGLE_PAUSE_WALL_S = 5.
CUMULATIVE_PAUSE_WALL_S = 60.


class GroundThreePointRecoveryBudgetGuard(GroundThreePointCameraRecoveryGuard):
    mode = 'ground_three_point_recovery_budget'

    def __init__(self, *, started_wall_s, route_limits=None, session_id=None):
        self.pause_started_wall = None
        self.completed_pause_wall_s = 0.
        self.pause_records = []
        self.pause_records_dropped = 0
        super().__init__(started_wall_s=started_wall_s, route_limits=route_limits, session_id=session_id)

    def _pause_elapsed(self, wall):
        return 0. if self.pause_started_wall is None else max(0., wall-self.pause_started_wall)

    def _begin_pause_account(self, wall):
        if self.pause_started_wall is not None or not finite(wall):
            return
        self.pause_started_wall = wall
        self.pause_records.append({
            'started_wall_s': wall, 'started_wall_elapsed_s': wall-self.started_wall,
            'phase': self.phase, 'reason': self.reason, 'point_index': self.point_index,
            'target_id': self.target_id, 'epoch': self.stationary_epoch,
            'resume_count_before': self.resume_count, 'outcome': 'waiting',
            'entry_timing': self.timing_snapshot(wall_s=wall),
        })
        if len(self.pause_records) > 40:
            self.pause_records.pop(1)
            self.pause_records_dropped += 1

    def _finish_pause_account(self, wall, outcome):
        if self.pause_started_wall is None:
            return
        elapsed = self._pause_elapsed(wall)
        self.completed_pause_wall_s += elapsed
        self.pause_records[-1].update(
            ended_wall_s=wall, elapsed_wall_s=elapsed, outcome=outcome,
            resume_count_after=self.resume_count, epoch_after=self.stationary_epoch,
            cumulative_pause_wall_s=self.completed_pause_wall_s,
            exit_timing=self.timing_snapshot(wall_s=wall))
        self.pause_started_wall = None

    def _budget_fault(self, wall):
        fault = super()._budget_fault(wall)
        if fault or not finite(wall):
            return fault
        current = self._pause_elapsed(wall)
        if self.completed_pause_wall_s+current >= CUMULATIVE_PAUSE_WALL_S-1e-9:
            return 'three_ground_cumulative_pause_wall_limit'
        if self.pause_started_wall is not None and current >= SINGLE_PAUSE_WALL_S-1e-9:
            return 'three_ground_single_pause_wall_limit'
        return ''

    def temporal_fault(self, reason, wall):
        with self._lock:
            result = super().temporal_fault(reason, wall)
            if self.state == 'PAUSE':
                self._begin_pause_account(wall)
            return result

    def stop(self, reason, wall_s):
        with self._lock:
            # Preserve the original first-stop latch and wall validation.
            result = super().stop(reason, wall_s)
            if self.state in ('STOP', 'FINISHED') and self.pause_started_wall is not None:
                ended = max(self.last_wall, self.last_observation_wall,
                            wall_s if finite(wall_s) else self.last_wall)
                self._finish_pause_account(ended, 'stopped:'+self.reason)
            return result

    def _camera_event(self, event, reason, wall):
        super()._camera_event(event, reason, wall)
        self.camera_recovery_events[-1]['max_resumes'] = MAX_RESUMES

    def step(self, *, wall_s):
        with self._lock:
            if self.state != 'PAUSE':
                result = super().step(wall_s=wall_s)
                if self.state == 'PAUSE':
                    self._begin_pause_account(wall_s)
                return result
            if not finite(wall_s) or wall_s < max(self.last_wall, self.last_observation_wall):
                return self.stop('wall_clock_reversed_or_invalid', wall_s)
            self.last_wall = wall_s
            self._begin_pause_account(wall_s)
            fault = self._budget_fault(wall_s) or self.observed_motion_fault()
            if fault:
                return self.stop(fault, wall_s)
            control_fault = self._condition(wall_s)
            if control_fault:
                self.temporal_fault(control_fault, wall_s)
                return MotionDecision(0., self.state, self.reason)
            if self.phase in ('CAPTURE', 'RECOGNIZE_WAIT', 'REPORT_WAIT'):
                camera_fault = self._current_recognition_condition(wall_s)
                if camera_fault:
                    self.temporal_fault(camera_fault, wall_s)
                    self.stable_anchor = None
                    return MotionDecision(0., self.state, self.reason)
            if not self._stable_dwell(wall_s):
                return MotionDecision(0., self.state, 'pause_stability_dwell')
            if self.resume_count >= MAX_RESUMES:
                return self.stop('three_ground_resume_resource_limit', wall_s)
            recapture = (self.phase in ('CAPTURE', 'RECOGNIZE_WAIT')
                         and self.epoch_invalid == 'sensor_freshness_lost')
            if recapture and self.recapture_count >= 1:
                return self.stop('single_point_recapture_limit', wall_s)
            self.resume_count += 1
            if recapture:
                self.recapture_count += 1
                self.state = 'CAPTURE'
                self._enter_phase('CAPTURE')
                if self.camera_recovery_pending:
                    self.camera_recovery_pending = False
                    self._camera_event('recapture', self.reason, wall_s)
            else:
                self.state, self.reason = self.phase, 'three_ground_'+self.phase.lower()
                self.stable_anchor = None
            self._finish_pause_account(wall_s, 'recovered')
            # A recovery is a zero slot. A later tick must pass the original
            # motion/geometry gates again after its last real DDS query.
            return MotionDecision(0., self.state, self.reason)

    def approach_evidence(self):
        with self._lock:
            data = super().approach_evidence()
            now = max(self.last_wall, self.last_observation_wall)
            data['revision'] = 'ground_three_point_recovery_budget_v1'
            data['max_resumes'] = MAX_RESUMES
            data['camera_recovery']['max_resumes'] = MAX_RESUMES
            data['recovery_resource'] = {
                'max_resumes': MAX_RESUMES, 'resume_count': self.resume_count,
                'single_pause_wall_limit_s': SINGLE_PAUSE_WALL_S,
                'cumulative_pause_wall_limit_s': CUMULATIVE_PAUSE_WALL_S,
                'completed_pause_wall_s': self.completed_pause_wall_s,
                'active_pause_wall_s': self._pause_elapsed(now),
                'total_pause_wall_s': self.completed_pause_wall_s+self._pause_elapsed(now),
                'max_recaptures_per_point': 1,
                'records': copy.deepcopy(self.pause_records),
                'records_capacity': 40, 'records_dropped': self.pause_records_dropped,
                'contract': 'predeclared_finite_retry_resources_not_motion_permission',
                'official_acceptance': False,
            }
            return data


def create_recovery_budget_three_point_node(reporter=None, *, wall_clock=time.monotonic,
                                           route_limits=None, worker=None):
    return create_camera_recovery_three_point_node(
        reporter, wall_clock=wall_clock, route_limits=route_limits, worker=worker,
        guard_factory=lambda *, started_wall_s, route_limits: GroundThreePointRecoveryBudgetGuard(
            started_wall_s=started_wall_s, route_limits=route_limits))


def main(args=None):
    import rclpy
    from .ground_matcher_worker import GroundMatcherWorker
    from .ground_single_point import load_ground_templates
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=1200.)
    parser.add_argument('--template-dir', default=None)
    parser.add_argument('--report-url', default=os.environ.get('INSPECTION_DIAGNOSTIC_URL', ''))
    parsed, ros_args = parser.parse_known_args(args)
    limits = AxisThreePointLimits(global_wall_s=parsed.duration)
    worker = GroundMatcherWorker(load_ground_templates(parsed.template_dir))
    reporter = DiagnosticReporter(parsed.report_url)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_recovery_budget_three_point_node(reporter, route_limits=limits, worker=worker)
        node.report_started(parsed.duration)
        node.arm_watchdog()
        spin_control_loop(node, rclpy, duration_s=parsed.duration)
        node.stop('three_ground_global_wall_time_limit')
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=.02)
        node.report_final()
    finally:
        if node is not None:
            node.stop('three_ground_recovery_budget_cleanup')
            node.close_watchdog()
            node.destroy_node()
        worker.close(timeout_s=.5)
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == '__main__':
    main()
