import sys
from types import SimpleNamespace

import inspection_mission.mission_node as mission_node
from inspection_mission.mission_node import (
    RunOutcome, SimulationClockWatchdog, completion_grace_elapsed,
    missing_stop_subscribers,
    completion_shutdown_due, completion_status_payload,
)


def test_completion_status_uses_documented_forum_payload():
    assert completion_status_payload() == "mission_completed"


def test_completion_grace_uses_wall_clock_delta():
    assert completion_grace_elapsed(10.0, 10.5)
    assert not completion_grace_elapsed(10.0, 10.49)


def test_completion_shutdown_due_is_independent_of_ros_clock():
    assert completion_shutdown_due(20.0, 20.5)
    assert not completion_shutdown_due(20.0, 20.49)


def test_simulation_clock_watchdog_uses_monotonic_time_after_clock_starts():
    watchdog = SimulationClockWatchdog(timeout_s=1.0)
    assert not watchdog.observe(0.0, 10.0)
    assert not watchdog.observe(0.1, 10.1)
    assert not watchdog.observe(0.1, 11.09)
    assert watchdog.observe(0.1, 11.1)
    assert not watchdog.observe(0.2, 11.2)


def test_clock_reset_counts_as_progress_instead_of_a_stall():
    watchdog = SimulationClockWatchdog(timeout_s=1.0)
    watchdog.observe(4.0, 1.0)
    watchdog.observe(4.1, 1.1)
    assert not watchdog.observe(0.0, 2.0)
    assert not watchdog.observe(0.0, 2.9)
    assert watchdog.observe(0.0, 3.1)


def test_clock_that_never_leaves_zero_times_out_after_startup_grace():
    watchdog = SimulationClockWatchdog(timeout_s=1.0, startup_timeout_s=1.0)
    assert not watchdog.observe(0.0, 10.0)
    assert not watchdog.observe(0.0, 10.99)
    assert watchdog.observe(0.0, 11.0)


def test_safe_stop_requires_connected_consumers_for_each_stop_output():
    class Publisher:
        def __init__(self, count):
            self.count = count

        def get_subscription_count(self):
            return self.count

    publishers = {"ground_cmd": Publisher(1), "uav_cmd": Publisher(0), "light": Publisher(2)}
    assert missing_stop_subscribers(publishers, ("ground_cmd", "uav_cmd", "light")) == ["uav_cmd"]


def test_failed_safe_stop_never_publishes_success_and_exits_nonzero():
    outcome = RunOutcome()
    published = []
    assert not outcome.complete_after_stop(False, 10.0, lambda: published.append("mission_completed"))
    assert published == []
    assert outcome.shutdown_due(10.5)
    assert outcome.exit_code == 1


def test_successful_completion_publishes_only_after_safe_stop_and_exits_zero():
    outcome = RunOutcome()
    published = []
    assert outcome.complete_after_stop(True, 10.0, lambda: published.append("mission_completed"))
    assert published == ["mission_completed"]
    assert not outcome.shutdown_due(10.49)
    assert outcome.shutdown_due(10.5)
    assert outcome.exit_code == 0


def test_runtime_failure_overrides_pending_success_before_shutdown():
    outcome = RunOutcome()
    outcome.complete_after_stop(True, 10.0, lambda: None)
    outcome.fail("late runtime failure", 10.2)
    assert not outcome.shutdown_due(10.69)
    assert outcome.shutdown_due(10.7)
    assert outcome.exit_code == 1


def test_main_returns_nonzero_when_spin_ends_with_runtime_failure(monkeypatch):
    class FakeNode:
        def __init__(self):
            self._outcome = RunOutcome()

        def _fail_runtime(self, detail):
            self._outcome.fail(detail, 5.0)

        def destroy_node(self):
            pass

    node = FakeNode()
    fake_rclpy = SimpleNamespace(init=lambda args: None, spin=lambda current: current._fail_runtime("fault"),
                                 ok=lambda: True, shutdown=lambda: None)
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setattr(mission_node, "create_node", lambda: node)

    assert mission_node.main([]) == 1


def test_main_returns_zero_only_after_successful_outcome(monkeypatch):
    class FakeNode:
        def __init__(self):
            self._outcome = RunOutcome()

        def destroy_node(self):
            pass

    node = FakeNode()
    fake_rclpy = SimpleNamespace(init=lambda args: None,
                                 spin=lambda current: current._outcome.complete_after_stop(True, 5.0, lambda: None),
                                 ok=lambda: True, shutdown=lambda: None)
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setattr(mission_node, "create_node", lambda: node)

    assert mission_node.main([]) == 0
