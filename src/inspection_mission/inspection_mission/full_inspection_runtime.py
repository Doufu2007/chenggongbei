"""One official-platform attempt: actual three-point return, then all flight tasks.

This supervisor never turns a diagnostic stop into a completed competition.
The ground and flight executors own their live evidence and control gates.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import time


def _prepare_retired_report(node, stage, diagnostic_errors):
    """Keep reporting on retired Python state, without a destroyed ROS lookup.

    Called after control/watchdog cleanup. No sensor stamp or success evidence
    is changed. The retired ground clock excludes the later flight's duration.
    """
    try:
        get_logger = getattr(node, 'get_logger', None)
        logger = get_logger() if callable(get_logger) else None
        clock = getattr(node, 'wall_clock', None)
        retired_wall = clock() if callable(clock) else None
        report = node.report_final
        if logger is not None:
            node.get_logger = lambda: logger
        if callable(clock):
            node.wall_clock = lambda: retired_wall
        return report
    except Exception as exc:
        diagnostic_errors.append({'stage': stage, 'operation': 'prepare_report',
                                  'error_type': type(exc).__name__})
        return None


def _dispose(node, ros, reason, *, stage, pending_reports, diagnostic_errors):
    """Finish all safety cleanup now; diagnostics run after all control ends."""
    errors = []
    try:
        try:
            node.stop(reason)
        except Exception as exc:
            errors.append(exc)
        for _ in range(5):
            try:
                attempts = getattr(node, 'axis_publish_attempts', None)
                previous = attempts[-1] if attempts else None
                if node.publish_zero() is not True:
                    raise RuntimeError('zero_publish_not_confirmed')
                if attempts is not None:
                    current = attempts[-1] if attempts else None
                    if (current is None or current is previous or current.get('returned') is not True
                            or current.get('error') is not None or current.get('linear_x') != 0.
                            or current.get('angular_z') != 0.):
                        raise RuntimeError('zero_publish_attempt_not_current')
                ros.spin_once(node, timeout_sec=.02)
            except Exception as exc:
                errors.append(exc)
    finally:
        try:
            try:
                node.close_watchdog()
            except Exception as exc:
                errors.append(exc)
        finally:
            report = _prepare_retired_report(node, stage, diagnostic_errors)
            if report is not None:
                pending_reports.append((stage, report))
            try:
                node.destroy_node()
            except Exception as exc:
                errors.append(exc)
    if errors:
        raise errors[0]


def _emit_stage_observation(node, ros, reporter, stage, *, section, started,
                            wall_clock, diagnostic_errors, spin_outcome=None,
                            exception_type=None):
    """Queue a compact observation before cleanup changes state; never send now."""
    if reporter is None:
        return
    operation = 'stage_exit' if section == 'full_inspection_stage_exit' else 'first_stop'
    try:
        guard = node.guard
        progress = {
            'mode': 'full_inspection', 'section': section, 'stage': stage,
            'session_id': guard.session_id, 'elapsed_wall_s': wall_clock()-started,
            'state': str(getattr(guard, 'state', '')),
            'reason': str(getattr(guard, 'reason', '')),
            'phase': str(getattr(guard, 'phase', '')), 'ros_ok': ros.ok(),
            'return_completed': getattr(guard, 'return_completed', False) is True,
            'flight_completed': getattr(node, 'mission_completed', False) is True,
            'target_id': getattr(guard, 'target_id', None),
            'point_index': getattr(guard, 'point_index', None),
            'completed_count': len(getattr(guard, 'completed_points', ())),
            'spin_outcome': spin_outcome, 'exception_type': exception_type,
            'official_score': None,
        }
        reporter.emit('probe_snapshot', progress)
    except Exception as exc:
        diagnostic_errors.append({'stage': stage, 'operation': operation,
                                  'error_type': type(exc).__name__})


def _spin_with_progress(node, ros, reporter, stage, spin, *, duration_s, wall_clock,
                        diagnostic_errors):
    original = getattr(node, 'tick', None)
    started = wall_clock()
    last = -float('inf')
    first_stop_reported = False
    def tick():
        nonlocal last, first_stop_reported
        decision = original()
        now = wall_clock()
        if decision.state == 'STOP' and not first_stop_reported:
            first_stop_reported = True
            _emit_stage_observation(node, ros, reporter, stage,
                section='full_inspection_first_stop', started=started,
                wall_clock=wall_clock, diagnostic_errors=diagnostic_errors)
        if now-last >= 5.:
            last = now
            try:
                guard = node.guard
                odom = getattr(guard, 'odom', None)
                progress = {'mode': 'full_inspection', 'stage': stage,
                    'session_id': guard.session_id, 'elapsed_wall_s': now-started,
                    'state': str(decision.state), 'reason': str(decision.reason),
                    'phase': str(getattr(guard, 'phase', '')),
                    'xyz': list(odom['xyz']) if isinstance(odom, dict) and 'xyz' in odom else None,
                    'official_score': None}
                if stage == 'GROUND':
                    completed = getattr(guard, 'completed_points', ())
                    progress.update(target_id=getattr(guard, 'target_id', None),
                                    point_index=getattr(guard, 'point_index', None),
                                    completed_count=len(completed))
                reporter.emit('heartbeat', progress)
            except Exception:
                pass
        return decision
    wrapped = reporter is not None and callable(original)
    if wrapped:
        node.tick = tick
    outcome, exception_type = 'returned', None
    try:
        return spin(node, ros, duration_s=duration_s, wall_clock=wall_clock)
    except BaseException as exc:
        # Preserve interruptions exactly; only the exception class is observed.
        outcome, exception_type = 'exception', type(exc).__name__
        raise
    finally:
        if wrapped:
            node.tick = original
        _emit_stage_observation(node, ros, reporter, stage,
            section='full_inspection_stage_exit', started=started,
            wall_clock=wall_clock, diagnostic_errors=diagnostic_errors,
            spin_outcome=outcome, exception_type=exception_type)


def run_ground_and_flight(ros, reporter, *, worker, ground_factory=None, flight_factory=None,
                          spin=None, wall_clock=time.monotonic,
                          ground_duration=1800., flight_duration=900.):
    """Hand off only original successful ground-return evidence, after zeroing.

    Dependency injection supports transport-free integration tests. It never
    changes a controller's success state or creates a referee acknowledgement.
    """
    if ground_factory is None:
        from .ground_three_point_return import create_return_three_point_node
        ground_factory = create_return_three_point_node
    if spin is None:
        from .bounded_ground_probe import spin_control_loop
        spin = spin_control_loop
    ground = ground_factory(reporter, wall_clock=wall_clock, worker=worker)
    pending_reports, diagnostic_errors = [], []
    result = {'completed': False, 'failed_stage': 'GROUND', 'official_score': None,
              'session_id': ground.guard.session_id, 'ground_return_completed': False,
              'flight_completed': False, 'diagnostic_errors': diagnostic_errors}
    proof = None
    try:
        try:
            ground.report_started(ground_duration)
            ground.arm_watchdog()
            _spin_with_progress(ground, ros, reporter, 'GROUND', spin,
                                duration_s=ground_duration, wall_clock=wall_clock,
                                diagnostic_errors=diagnostic_errors)
            if ground.guard.return_completed:
                proof = copy.deepcopy(ground.guard.return_completion)
                result['ground_return_completed'] = isinstance(proof, dict) and bool(proof)
        finally:
            _dispose(ground, ros, 'full_pipeline_ground_stage_exit', stage='GROUND',
                     pending_reports=pending_reports, diagnostic_errors=diagnostic_errors)
        if not result['ground_return_completed']:
            return result
        # The old ground publishers/watchdog are gone before flight creation.
        if flight_factory is None:
            from .full_flight_inspection import create_flight_inspection_node
            flight_factory = create_flight_inspection_node
        result['failed_stage'] = 'FLIGHT'
        flight = flight_factory(reporter, session_id=result['session_id'],
                                ground_return_evidence=proof, wall_clock=wall_clock)
        try:
            flight.report_started(flight_duration)
            flight.arm_watchdog()
            _spin_with_progress(flight, ros, reporter, 'FLIGHT', spin,
                                duration_s=flight_duration, wall_clock=wall_clock,
                                diagnostic_errors=diagnostic_errors)
            result['flight_completed'] = flight.mission_completed is True
        finally:
            _dispose(flight, ros, 'full_pipeline_flight_stage_exit', stage='FLIGHT',
                     pending_reports=pending_reports, diagnostic_errors=diagnostic_errors)
        result['completed'] = result['flight_completed']
        result['failed_stage'] = None if result['completed'] else 'FLIGHT'
        return result
    finally:
        # Final reports are observation only, never a referee or score source.
        # Flight is reported first so a large ground bundle cannot delay it.
        for stage, report in reversed(pending_reports):
            try:
                report()
            except Exception as exc:
                diagnostic_errors.append({'stage': stage, 'operation': 'report_final',
                                          'error_type': type(exc).__name__})


def main(args=None):
    import rclpy
    from .ground_matcher_worker import GroundMatcherWorker
    from .ground_match_result_recorder import GroundMatchResultRecorder
    from .ground_template_catalog import load_complete_ground_templates
    from .telemetry import DiagnosticReporter
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ground-duration', type=float, default=1800.)
    parser.add_argument('--flight-duration', type=float, default=900.)
    parser.add_argument('--report-url', default=os.environ.get('INSPECTION_DIAGNOSTIC_URL', ''))
    parsed, ros_args = parser.parse_known_args(args)
    if not 30 <= parsed.ground_duration <= 1800 or not 30 <= parsed.flight_duration <= 900:
        parser.error('ground duration must be 30..1800s and flight duration 30..900s')
    # Both execution components must exist and compile before the attempt begins.
    from .full_flight_inspection import create_flight_inspection_node
    reporter = DiagnosticReporter(parsed.report_url)
    worker = GroundMatchResultRecorder(GroundMatcherWorker(load_complete_ground_templates()), reporter)
    rclpy.init(args=ros_args)
    try:
        result = run_ground_and_flight(rclpy, reporter, worker=worker,
            flight_factory=create_flight_inspection_node, ground_duration=parsed.ground_duration,
            flight_duration=parsed.flight_duration)
        print('FULL_INSPECTION_RESULT ' + json.dumps(result, ensure_ascii=False), flush=True)
        reporter.send_now('probe_snapshot', {'mode': 'full_inspection',
            'section': 'full_inspection_result', **result}, timeout_s=5.)
        return 0 if result['completed'] else 1
    except Exception as exc:
        detail = {'mode': 'full_inspection', 'section': 'full_inspection_exception',
                  'completed': False, 'official_score': None,
                  'error': type(exc).__name__ + ': ' + str(exc)[:500]}
        print('FULL_INSPECTION_EXCEPTION ' + json.dumps(detail, ensure_ascii=False), flush=True)
        reporter.send_now('probe_snapshot', detail, timeout_s=5.)
        return 1
    finally:
        worker.close(timeout_s=.5)
        if rclpy.ok():
            rclpy.shutdown()
        reporter.close()


if __name__ == '__main__':
    raise SystemExit(main())
