import pytest

from inspection_mission.fsm import InspectionFSM, MissionState


def test_fsm_runs_ground_then_air_then_return():
    fsm = InspectionFSM(phase_timeout_s=10.0)
    assert fsm.state is MissionState.INIT

    fsm.tick(0.0, ready=True, pose_valid=True, control_ok=True)
    assert fsm.state is MissionState.GROUND_SCAN

    fsm.mark_ground_complete(2.0)
    assert fsm.state is MissionState.AERIAL_SCAN

    fsm.mark_aerial_complete(4.0)
    assert fsm.state is MissionState.RETURN

    fsm.mark_return_complete(5.0)
    assert fsm.state is MissionState.COMPLETE


def test_fsm_enters_failsafe_after_phase_timeout():
    fsm = InspectionFSM(phase_timeout_s=3.0)
    fsm.tick(0.0, ready=True, pose_valid=True, control_ok=True)
    fsm.tick(3.1, ready=True, pose_valid=True, control_ok=True)
    assert fsm.state is MissionState.FAILSAFE
    assert "timeout" in fsm.fault_reason


def test_fsm_rejects_missing_pose_or_control():
    fsm = InspectionFSM(phase_timeout_s=10.0)
    fsm.tick(0.0, ready=True, pose_valid=False, control_ok=True)
    assert fsm.state is MissionState.INIT
    assert fsm.fault_reason == ""
    fsm.tick(10.1, ready=True, pose_valid=False, control_ok=True)
    assert fsm.state is MissionState.FAILSAFE
    assert fsm.fault_reason == "timeout:init"

    fsm = InspectionFSM(phase_timeout_s=10.0)
    fsm.tick(0.0, ready=True, pose_valid=True, control_ok=False)
    assert fsm.state is MissionState.FAILSAFE
    assert fsm.fault_reason == "control_unavailable"


def test_fsm_repeated_completion_events_are_idempotent():
    fsm = InspectionFSM(phase_timeout_s=10.0)
    fsm.tick(0.0, ready=True, pose_valid=True, control_ok=True)
    fsm.mark_ground_complete(1.0)
    fsm.mark_ground_complete(1.5)
    assert fsm.state is MissionState.AERIAL_SCAN
    fsm.mark_aerial_complete(2.0)
    fsm.mark_aerial_complete(2.5)
    assert fsm.state is MissionState.RETURN


def test_fsm_waits_for_initial_pose_instead_of_failing_immediately():
    fsm = InspectionFSM(phase_timeout_s=10)
    fsm.tick(0.0, ready=False, pose_valid=False, control_ok=True)
    assert fsm.state is MissionState.INIT
    fsm.tick(0.1, ready=True, pose_valid=True, control_ok=True)
    assert fsm.state is MissionState.GROUND_SCAN

