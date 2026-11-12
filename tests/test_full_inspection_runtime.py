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
