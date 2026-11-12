import types


class Node:
    def __init__(self, events, name, complete):
        self.events, self.name = events, name
        self.guard = types.SimpleNamespace(session_id='one-session', return_completed=complete,
                                           return_completion={'pose': [0., 0., 0.], 'stationary_wall_s': 1.2})
        self.mission_completed = complete

    def publish_zero(self):
        self.events.append((self.name, 'publish_zero'))
        return True

    def __getattr__(self, action):
        if action in ('report_started', 'arm_watchdog', 'stop', 'publish_zero', 'report_final', 'close_watchdog', 'destroy_node'):
            return lambda *args: self.events.append((self.name, action))
        raise AttributeError(action)


class Ros:
    def spin_once(self, *args, **kwargs):
        pass


def test_failed_ground_never_creates_flight_and_stops_before_exit():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events = []
    def flight(*args, **kwargs):
        raise AssertionError('incomplete ground cannot enter flight')
    result = run_ground_and_flight(Ros(), None, worker=object(),
        ground_factory=lambda *a, **kw: Node(events, 'ground', False),
        flight_factory=flight, spin=lambda *a, **kw: None)
    assert result['completed'] is False and result['failed_stage'] == 'GROUND'
    assert events.index(('ground', 'publish_zero')) < events.index(('ground', 'destroy_node'))


def test_ground_is_stopped_and_disposed_before_flight_receives_original_proof():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events = []
    proof = {}
    def flight(*args, **kwargs):
        events.append(('flight', 'created'))
        proof.update(kwargs)
        return Node(events, 'flight', True)
    result = run_ground_and_flight(Ros(), None, worker=object(),
        ground_factory=lambda *a, **kw: Node(events, 'ground', True),
        flight_factory=flight, spin=lambda *a, **kw: None)
    assert events.index(('ground', 'destroy_node')) < events.index(('flight', 'created'))
    assert proof['session_id'] == 'one-session'
    assert proof['ground_return_evidence']['stationary_wall_s'] == 1.2
    assert result['completed'] is True
    assert result['official_score'] is None


def test_flight_failure_cannot_be_promoted_to_complete():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events = []
    result = run_ground_and_flight(Ros(), None, worker=object(),
        ground_factory=lambda *a, **kw: Node(events, 'ground', True),
        flight_factory=lambda *a, **kw: Node(events, 'flight', False), spin=lambda *a, **kw: None)
    assert result['completed'] is False and result['failed_stage'] == 'FLIGHT'


def test_ground_stop_exception_still_disposes_and_prevents_takeoff():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    import pytest
    events = []
    class Broken(Node):
        def publish_zero(self):
            raise RuntimeError('transport failed')
    with pytest.raises(RuntimeError, match='transport failed'):
        run_ground_and_flight(Ros(), None, worker=object(),
            ground_factory=lambda *a, **kw: Broken(events, 'ground', True),
            flight_factory=lambda *a, **kw: pytest.fail('must not start flight'),
            spin=lambda *a, **kw: None)
    assert ('ground', 'destroy_node') in events


def test_ground_zero_false_still_disposes_and_prevents_takeoff():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    import pytest
    events = []
    class Rejected(Node):
        def publish_zero(self):
            self.events.append((self.name, 'publish_zero'))
            return False
    with pytest.raises(RuntimeError, match='zero_publish_not_confirmed'):
        run_ground_and_flight(Ros(), None, worker=object(),
            ground_factory=lambda *a, **kw: Rejected(events, 'ground', True),
            flight_factory=lambda *a, **kw: pytest.fail('must not start flight'),
            spin=lambda *a, **kw: None)
    assert ('ground', 'destroy_node') in events


def test_old_zero_attempt_cannot_confirm_cleanup_zero():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    import pytest
    events = []
    class Stale(Node):
        axis_publish_attempts = [{'returned': True, 'error': None, 'linear_x': 0., 'angular_z': 0.}]
    with pytest.raises(RuntimeError, match='zero_publish_attempt_not_current'):
        run_ground_and_flight(Ros(), None, worker=object(),
            ground_factory=lambda *a, **kw: Stale(events, 'ground', True),
            flight_factory=lambda *a, **kw: pytest.fail('must not start flight'),
            spin=lambda *a, **kw: None)
    assert ('ground', 'destroy_node') in events


def test_progress_is_bounded_and_never_changes_control_or_failure_result():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events, reports = [], []
    wall = [0.]
    class ProgressNode(Node):
        def tick(self):
            return types.SimpleNamespace(state='DRIVE', reason='actual')
    class Reporter:
        def emit(self, event, data):
            reports.append((event, data))
            if len(reports) == 2:
                raise RuntimeError('diagnostics unavailable')
    node = ProgressNode(events, 'ground', False)
    original = node.tick
    def spin(actual, ros, **kwargs):
        for i in range(16):
            wall[0] = float(i)
            assert actual.tick().state == 'DRIVE'
    result = run_ground_and_flight(Ros(), Reporter(), worker=object(),
        ground_factory=lambda *a, **kw: node, spin=spin, wall_clock=lambda: wall[0])
    assert result['completed'] is False
    assert len(reports) == 4
    assert all(e == 'heartbeat' and d['stage'] == 'GROUND' and d['state'] == 'DRIVE' for e, d in reports)
    assert node.tick == original


def test_ground_final_report_waits_until_all_flight_control_is_disposed():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events = []
    result = run_ground_and_flight(Ros(), None, worker=object(),
        ground_factory=lambda *a, **kw: Node(events, 'ground', True),
        flight_factory=lambda *a, **kw: Node(events, 'flight', True), spin=lambda *a, **kw: None)
    assert result['completed'] is True
    assert events.index(('flight', 'destroy_node')) < events.index(('ground', 'report_final'))
    assert events.index(('ground', 'close_watchdog')) < events.index(('ground', 'destroy_node'))
    assert events.index(('ground', 'destroy_node')) < events.index(('flight', 'report_started'))


def test_ground_diagnostic_exception_cannot_block_successful_flight():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events = []
    class DiagnosticBroken(Node):
        def report_final(self):
            self.events.append((self.name, 'report_final'))
            raise RuntimeError('diagnostic transport only')
    result = run_ground_and_flight(Ros(), None, worker=object(),
        ground_factory=lambda *a, **kw: DiagnosticBroken(events, 'ground', True),
        flight_factory=lambda *a, **kw: Node(events, 'flight', True), spin=lambda *a, **kw: None)
    assert result['completed'] is True and result['ground_return_completed'] is True
    assert result['official_score'] is None
    assert result['diagnostic_errors'] == [{'stage': 'GROUND', 'operation': 'report_final', 'error_type': 'RuntimeError'}]


def test_failed_flight_diagnostic_cannot_skip_ground_report_or_change_result():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events = []
    class DiagnosticBroken(Node):
        def report_final(self):
            self.events.append((self.name, 'report_final'))
            raise ValueError('diagnostic serialization only')
    result = run_ground_and_flight(Ros(), None, worker=object(),
        ground_factory=lambda *a, **kw: Node(events, 'ground', True),
        flight_factory=lambda *a, **kw: DiagnosticBroken(events, 'flight', False), spin=lambda *a, **kw: None)
    assert result['completed'] is False and result['failed_stage'] == 'FLIGHT'
    assert ('ground', 'report_final') in events
    assert result['diagnostic_errors'] == [{'stage': 'FLIGHT', 'operation': 'report_final', 'error_type': 'ValueError'}]


def test_report_uses_retirement_wall_and_cached_logger_after_node_destruction():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events, snapshots, wall = [], [], [0.]
    logger = object()
    class Retired(Node):
        destroyed = False
        wall_clock = staticmethod(lambda: wall[0])
        def get_logger(self):
            assert not self.destroyed, 'destroyed ROS handle must not be queried'
            return logger
        def destroy_node(self):
            self.destroyed = True
            self.events.append((self.name, 'destroy_node'))
        def report_final(self):
            assert self.destroyed
            snapshots.append((self.wall_clock(), self.get_logger()))
    def spin(node, *args, **kwargs):
        wall[0] = 12. if node.name == 'ground' else 250.
    result = run_ground_and_flight(Ros(), None, worker=object(),
        ground_factory=lambda *a, **kw: Retired(events, 'ground', True),
        flight_factory=lambda *a, **kw: Node(events, 'flight', True), spin=spin)
    assert result['completed'] is True
    assert snapshots == [(12., logger)]


def test_watchdog_close_failure_remains_a_control_failure_and_prevents_flight():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    import pytest
    events = []
    class WatchdogBroken(Node):
        def close_watchdog(self):
            raise RuntimeError('watchdog closure failed')
    with pytest.raises(RuntimeError, match='watchdog closure failed'):
        run_ground_and_flight(Ros(), None, worker=object(),
            ground_factory=lambda *a, **kw: WatchdogBroken(events, 'ground', True),
            flight_factory=lambda *a, **kw: pytest.fail('unsafe handoff'), spin=lambda *a, **kw: None)
    assert ('ground', 'destroy_node') in events


def test_ground_progress_reports_only_compact_current_target_and_completion_count():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    events, reports = [], []
    class ProgressNode(Node):
        def tick(self):
            return types.SimpleNamespace(state='DRIVE', reason='actual')
    class Reporter:
        def emit(self, event, data):
            reports.append(data)
    ground = ProgressNode(events, 'ground', False)
    ground.guard.target_id = 'ground_4'
    ground.guard.point_index = 1
    ground.guard.completed_points = [{'target_id': 'ground_2', 'verbose': object()}]
    run_ground_and_flight(Ros(), Reporter(), worker=object(),
        ground_factory=lambda *a, **kw: ground,
        spin=lambda node, *a, **kw: node.tick())
    assert len(reports) == 1
    assert reports[0]['target_id'] == 'ground_4'
    assert reports[0]['point_index'] == 1 and reports[0]['completed_count'] == 1
    assert 'completed_points' not in reports[0]
    assert ground.guard.completed_points[0]['target_id'] == 'ground_2'


def test_actual_return_factory_reports_retired_state_without_querying_destroyed_ros(monkeypatch):
    from test_bounded_ground_probe import install_ros_transport
    from inspection_mission.ground_three_point_return import create_return_three_point_node
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    install_ros_transport(monkeypatch)
    events, reports, wall = [], [], [0.]
    class Reporter:
        def emit(self, event, data):
            return True
        def send_now(self, event, data, timeout_s):
            reports.append(data)
            return True
    reporter = Reporter()
    ground = create_return_three_point_node(reporter, wall_clock=lambda: wall[0])
    # Synthetic proof isolates post-control reporting; it is not a physical run.
    ground.guard.return_completed = True
    ground.guard.return_completion = {'pose': [0., 0., 0.], 'stationary_wall_s': 1.2}
    ground.arm_watchdog = lambda: None
    destroyed = []
    original_get_logger = ground.get_logger
    def get_live_logger():
        assert not destroyed, 'destroyed ROS lookup'
        return original_get_logger()
    ground.get_logger = get_live_logger
    ground.destroy_node = lambda: destroyed.append(True)
    def spin(node, *args, **kwargs):
        wall[0] = 12. if node is ground else 250.
    result = run_ground_and_flight(Ros(), reporter, worker=object(),
        ground_factory=lambda *a, **kw: ground,
        flight_factory=lambda *a, **kw: Node(events, 'flight', True), spin=spin)
    assert destroyed == [True] and result['completed'] is True
    assert result['diagnostic_errors'] == []
    core = next(row for row in reports if row.get('section') == 'ground_motion_core')
    assert core['elapsed_s'] == 12.
    assert core['timing']['clock'] is None


def test_stop_and_destroy_errors_still_fail_closed_before_flight():
    from inspection_mission.full_inspection_runtime import run_ground_and_flight
    import pytest
    for failing_operation in ('stop', 'destroy_node'):
        events = []
        node = Node(events, 'ground', True)
        def fail(*args):
            raise RuntimeError('control cleanup failed')
        setattr(node, failing_operation, fail)
        with pytest.raises(RuntimeError, match='control cleanup failed'):
            run_ground_and_flight(Ros(), None, worker=object(),
                ground_factory=lambda *a, **kw: node,
                flight_factory=lambda *a, **kw: pytest.fail('unsafe handoff'), spin=lambda *a, **kw: None)
        assert events.count(('ground', 'publish_zero')) == 5
