"""Behavior tests for the complete controller, with no ROS or referee."""

import math

import pytest

from inspection_mission.control import Pose2D
from tools.mission_scenario import demonstration_plan


def controller(plan=None, **settings):
    from inspection_mission.mission import MissionController
    return MissionController(plan or demonstration_plan(), settings)


def at_ground(point=0, now=0.0):
    target = demonstration_plan()["ground_points"][point]["pose"]
    return Pose2D(x=target[0], y=target[1], yaw=target[2], stamp=now)


def detection(now, target="ground-one", source="ground", **overrides):
    result = {"target_id": target, "source": source, "stamp": now,
              "center": [0.5, 0.5], "confidence": 0.95}
    result.update(overrides)
    return result


def first_inspection(plan=None, **settings):
    mission = controller(plan, **settings)
    mission.step(0.0, at_ground(), None, [], ground_order=["g1", "g2", "g3"])
    mission.step(0.1, at_ground(now=0.1), None, [])
    assert mission.state == "GROUND_RECOGNIZE_1"
    return mission


def test_missing_or_unsafe_map_never_receives_invented_defaults():
    from inspection_mission.mission import MissionController
    with pytest.raises(ValueError):
        MissionController({})
    for change in ("window_clearance", "descent", "aerial_height", "count", "nonfinite"):
        plan = demonstration_plan()
        if change == "window_clearance":
            plan["windows"][0]["crossing_z"] = 0.17
        elif change == "descent":
            plan["windows"][0]["approach"][2] = 0.5
        elif change == "aerial_height":
            plan["aerial_points"][0]["pose"][2] = 0.7
        elif change == "count":
            plan["ground_points"].pop()
        else:
            plan["home_ground"][0] = float("nan")
        with pytest.raises(ValueError):
            MissionController(plan)


def test_legacy_global_uav_altitude_cannot_silently_override_measured_stage_geometry():
    with pytest.raises(ValueError, match="uav_altitude_m"):
        controller(uav_altitude_m=1.2)


def test_waits_without_moving_until_public_ground_order_arrives():
    mission = controller()
    command = mission.step(0.0, Pose2D(), None, [])
    assert command.ground == (0.0, 0.0)
    assert command.uav == (0.0, 0.0, 0.0, 0.0)
    assert command.state == "WAIT_ORDER" and not command.done
    command = mission.step(0.1, Pose2D(stamp=0.1), None, [], ground_order=["g3", "g1", "g2"])
    assert mission.active_vehicle == "ground"
    assert command.state == "GROUND_NAV_1"


def test_initial_missing_odom_is_diagnostic_wait_and_detection_cannot_be_disabled():
    mission = controller(phase_timeout_s=5.0)
    command = mission.step(0.0, None, None, [], ground_order=["g1", "g2", "g3"])
    assert command.state == "WAIT_READY"
    assert not command.fault and not command.done
    command = mission.step(0.2, None, None, [], ground_order=None)
    assert command.state == "WAIT_READY"
    assert not command.fault and not command.done
    with pytest.raises(ValueError, match="require_detection"):
        controller(require_detection=False)


def test_public_ground_order_must_be_permutation_and_cannot_change_mid_mission():
    mission = controller()
    command = mission.step(0, Pose2D(), None, [], ground_order=["g1", "g1", "g3"])
    assert command.fault and command.ground == (0, 0)
    mission = first_inspection()
    command = mission.step(0.2, at_ground(now=0.2), None, [], ground_order=["g3", "g2", "g1"])
    assert command.fault and not command.done


def test_ground_order_selects_three_points_from_five_point_candidate_map():
    """The official巡检任务随机选择 3 of the 5 ground candidates each run."""
    import copy

    plan = demonstration_plan()
    for index, point in enumerate((
        {"id": "g4", "target_id": "ground-four", "pose": [0.8, 0.8, 0.0]},
        {"id": "g5", "target_id": "ground-five", "pose": [-0.8, 0.8, 3.14]},
    )):
        plan["ground_points"].append(copy.deepcopy(point))
    mission = controller(plan)
    command = mission.step(
        0.0, Pose2D(stamp=0.0), None, [],
        ground_order=["g4", "g2", "g1"],
    )
    assert command.state == "GROUND_NAV_1"
    assert not command.fault


def test_arriving_is_insufficient_and_ack_must_follow_a_report():
    mission = first_inspection(session_id="run-test")
    for tick in range(2, 15):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None, [detection(now)])
    report = command.recognition
    assert report is not None and report["target_id"] == "ground-one"
    assert report["source"] == "ground" and "run-test" in report["event_id"]
    assert mission.state == "GROUND_RECOGNIZE_1"
    command = mission.step(1.5, at_ground(now=1.5), None, [detection(1.5)], recognition_ack={report["event_id"]})
    assert command.state == "GROUND_NAV_2" and not command.done


@pytest.mark.parametrize("bad", [
    {"target_id": "wrong"}, {"source": "uav"}, {"stamp": -10.0},
    {"center": [0.1, 0.5]}, {"confidence": 0.2}, {"stamp": 100.0},
])
def test_wrong_old_offcenter_or_other_camera_resets_continuous_recognition(bad):
    mission = first_inspection(recognition_stable_s=0.5)
    for tick in range(2, 6):
        now = tick / 10
        assert mission.step(now, at_ground(now=now), None, [detection(now)]).recognition is None
    assert mission.step(0.6, at_ground(now=0.6), None, [detection(0.6, **bad)]).recognition is None
    for tick in range(7, 11):
        now = tick / 10
        assert mission.step(now, at_ground(now=now), None, [detection(now)]).recognition is None
    assert mission.state == "GROUND_RECOGNIZE_1"


def test_recognition_loss_invalidates_report_and_old_ack():
    mission = first_inspection(recognition_stable_s=0.2)
    first = None
    for tick in range(2, 7):
        now = tick / 10
        first = mission.step(now, at_ground(now=now), None, [detection(now)]).recognition or first
    assert first
    mission.step(0.7, at_ground(now=0.7), None, [])
    second = None
    for tick in range(8, 14):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None, [detection(now)], recognition_ack={first["event_id"]})
        second = command.recognition or second
    assert second and second["event_id"] != first["event_id"]
    assert mission.state == "GROUND_RECOGNIZE_1"


@pytest.mark.parametrize("failure", ["stale_pose", "lost_pose", "control", "time_reversal", "nonfinite_pose"])
def test_runtime_invalid_feedback_stops_instead_of_faking_completion(failure):
    mission = first_inspection()
    pose, now, ready = at_ground(now=0.2), 0.2, True
    if failure == "stale_pose":
        now = 2.0
    elif failure == "lost_pose":
        pose = None
    elif failure == "control":
        ready = False
    elif failure == "time_reversal":
        now = 0.05
    elif failure == "nonfinite_pose":
        pose.x = float("nan")
    command = mission.step(now, pose, None, [], controls_ready=ready)
    assert command.fault and command.ground == (0.0, 0.0)
    assert command.uav == (0.0, 0.0, 0.0, 0.0) and not command.done


def test_obstacle_blocks_then_faults_with_zero_velocity():
    mission = controller(obstacle_timeout_s=0.5)
    mission.step(0.0, Pose2D(), None, [], ground_order=["g1", "g2", "g3"])
    for tick in range(1, 8):
        now = tick / 10
        command = mission.step(now, Pose2D(stamp=now), None, [], ground_clear=False)
        assert command.ground == (0.0, 0.0)
    assert command.fault and not command.done


def test_fresh_but_frozen_odometry_cannot_run_forever():
    mission = controller(no_progress_timeout_s=0.5)
    mission.step(0.0, Pose2D(), None, [], ground_order=["g1", "g2", "g3"])
    for tick in range(1, 9):
        now = tick / 10
        command = mission.step(now, Pose2D(stamp=now), None, [])
    assert command.fault and "progress" in command.fault


def test_phase_timeout_also_covers_unacknowledged_identification():
    mission = first_inspection(phase_timeout_s=0.5)
    for tick in range(2, 9):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None, [detection(now)])
    assert command.fault and "timeout" in command.fault


def test_independent_kinematics_completes_every_action_in_public_order():
    from tools.mission_scenario import run_scenario
    result = run_scenario(ground_order=["g3", "g1", "g2"])
    assert result.completed, result.fault
    assert [event["target_id"] for event in result.reports] == [
        "ground-three", "ground-one", "ground-two", "window-one", "air-one", "air-two", "window-two"]
    states = result.states
    assert states.index("RETURN_GROUND") < states.index("TAKEOFF")
    assert states.index("WINDOW_1_RECOGNIZE") < states.index("WINDOW_1_DESCEND") < states.index("WINDOW_1_CROSS")
    assert states.index("WINDOW_1_CROSS") < states.index("RECOVER_AFTER_WINDOW_1") < states.index("AIR_NAV_1")
    assert states.index("AIR_INSPECT_2") < states.index("WINDOW_2_APPROACH")
    assert states.index("RETURN_UAV") < states.index("LAND") < states.index("COMPLETE")
    assert result.ground.x == pytest.approx(0, abs=0.09)
    assert result.uav.x == pytest.approx(0, abs=0.09)
    assert result.uav.z == pytest.approx(0, abs=0.04)
    for frame in result.trace:
        if "CROSS" in frame["state"]:
            assert frame["uav"].z == pytest.approx(0.45, abs=0.07)
        if frame["state"].startswith("AIR_INSPECT"):
            assert frame["uav"].z >= 0.8
        if frame["state"] == "RETURN_UAV":
            assert frame["command"].uav[2] == pytest.approx(0, abs=1e-8)


def test_light_without_hardware_feedback_never_completes_aerial_point():
    from tools.mission_scenario import run_scenario
    result = run_scenario(light_feedback=False, settings={"phase_timeout_s": 30})
    assert not result.completed and result.fault
    assert "AIR_INSPECT_1" in result.states and "AIR_NAV_2" not in result.states
    assert not any(report["target_id"] == "air-one" for report in result.reports)


def test_no_recognition_ack_never_advances_ground_task():
    from tools.mission_scenario import run_scenario
    result = run_scenario(acknowledge=False, settings={"phase_timeout_s": 15})
    assert not result.completed and "GROUND_RECOGNIZE_1" in result.states
    assert "GROUND_NAV_2" not in result.states


def test_no_detection_never_completes_a_ground_inspection():
    """A pose at the point is insufficient without fresh visual evidence."""
    from tools.mission_scenario import run_scenario
    def no_camera(context):
        context["detections"].clear()
    result = run_scenario(settings={"phase_timeout_s": 15.0}, feedback_hook=no_camera)
    assert not result.completed
    assert "GROUND_RECOGNIZE_1" in result.states
    assert not result.reports


def test_lost_target_outside_observation_pose_reacquires_instead_of_parking():
    from tools.mission_scenario import run_scenario

    displaced = False

    def lose_view(context):
        nonlocal displaced
        if context["state"] == "GROUND_RECOGNIZE_1" and not displaced:
            context["ground"].x += 0.3
            context["detections"].clear()
            displaced = True

    result = run_scenario(
        demonstration_plan(protocol="inspection_v1"),
        settings={"phase_timeout_s": 30.0},
        feedback_hook=lose_view,
    )

    assert result.completed, result.fault
    assert displaced
    assert len(result.reports) == 7


def test_frozen_pose_during_target_reacquisition_faults_on_no_progress_timeout():
    plan = demonstration_plan(protocol="inspection_v1")
    mission = controller(plan, no_progress_timeout_s=0.5, phase_timeout_s=10.0)
    mission.step(0.0, Pose2D(stamp=0.0), None, [], ground_order=["g1", "g2", "g3"])
    approach = plan["ground_points"][0]["approach"]
    mission.step(0.1, Pose2D(x=approach[0], y=approach[1], yaw=approach[2], stamp=0.1), None, [])
    assert mission.state == "GROUND_RECOGNIZE_1"

    displaced = Pose2D(x=approach[0] + 0.3, y=approach[1], yaw=approach[2])
    for tick in range(2, 10):
        now = tick / 10
        displaced.stamp = now
        command = mission.step(now, displaced, None, [])
        if command.fault:
            break

    assert command.fault == "motion_no_progress:GROUND_RECOGNIZE_1"


def test_return_stage_publishes_motion_until_home_is_reached():
    """Completion cannot be declared by parking; return must command the route."""
    from tools.mission_scenario import run_scenario
    result = run_scenario()
    return_frames = [frame for frame in result.trace if frame["state"] == "RETURN_GROUND"]
    assert return_frames
    assert any(abs(frame["command"].ground[0]) > 1e-6 or
               abs(frame["command"].ground[1]) > 1e-6 for frame in return_frames)


def test_aerial_drift_or_darkness_restarts_full_illumination_interval():
    from tools.mission_scenario import run_scenario
    result = run_scenario(light_feedback="interrupt_once", settings={"illumination_s": 1.0})
    assert result.completed, result.fault
    inspection = [frame for frame in result.trace if frame["state"] == "AIR_INSPECT_1"]
    interrupted = next(frame["now"] for frame in inspection if frame["light_ready"] is False and frame["command"].light)
    report_time = next(report["stamp"] for report in result.reports if report["target_id"] == "air-one")
    assert report_time - interrupted >= 1.0


def test_a_cached_single_camera_frame_cannot_satisfy_continuous_recognition():
    mission = first_inspection(recognition_stable_s=0.2)
    for tick in range(2, 7):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None, [detection(0.2)])
        assert command.recognition is None


def test_navigation_follows_configured_via_points_before_recognizing_terminal():
    from tools.mission_scenario import run_scenario
    plan = demonstration_plan()
    plan["ground_points"][0]["via"] = [[0.0, -0.3, -math.pi / 2]]
    plan["aerial_points"][0]["via"] = [[1.0, -0.3, 1.0, 0.0]]
    plan["windows"][0]["via"] = [[0.0, -0.3, 1.2, 0.0]]
    plan["home_ground_via"] = [[-0.3, 0.0, 0.0]]
    plan["home_uav_via"] = [[0.0, 0.9, 1.2, 0.0]]
    result = run_scenario(plan)
    assert result.completed, result.fault
    for expected in ("GROUND_NAV_1_VIA_1", "AIR_NAV_1_VIA_1", "WINDOW_1_APPROACH_VIA_1", "RETURN_GROUND_VIA_1", "RETURN_UAV_VIA_1"):
        assert expected in result.states
    assert result.states.index("GROUND_NAV_1_VIA_1") < result.states.index("GROUND_RECOGNIZE_1")
    assert result.states.index("AIR_NAV_1_VIA_1") < result.states.index("AIR_INSPECT_1")
    assert [r["target_id"] for r in result.reports] == [
        "ground-one", "ground-two", "ground-three", "window-one", "air-one", "air-two", "window-two"]
    via_frames = [f for f in result.trace if f["state"] == "GROUND_NAV_1_VIA_1"]
    assert min(f["ground"].y for f in via_frames) < -0.2


@pytest.mark.parametrize("invalid", ["ground_dimensions", "aerial_low", "window_low", "home_uav_descent"])
def test_route_validation_rejects_invalid_or_unsafe_via_points(invalid):
    plan = demonstration_plan()
    if invalid == "ground_dimensions":
        plan["ground_points"][0]["via"] = [[0, 0]]
    elif invalid == "aerial_low":
        plan["aerial_points"][0]["via"] = [[1, 0, 0.7, 0]]
    elif invalid == "window_low":
        plan["windows"][0]["via"] = [[1, 0, 0.7, 0]]
    else:
        plan["home_uav_via"] = [[0, 0.5, 0.3, 0]]
    with pytest.raises(ValueError):
        controller(plan)


def test_actual_aerial_drift_resets_pending_report_until_new_dwell():
    from tools.mission_scenario import run_scenario
    displaced_at = []
    first_lit_at = []

    def displace(context):
        if context["state"] == "AIR_INSPECT_1":
            if not first_lit_at:
                first_lit_at.append(context["now"])
            if not displaced_at and context["now"] - first_lit_at[0] >= 0.7:
                context["uav"].x += 0.12
                displaced_at.append(context["now"])

    result = run_scenario(settings={"illumination_s": 1.0}, feedback_hook=displace)
    assert result.completed, result.fault
    report = next(r for r in result.reports if r["target_id"] == "air-one")
    assert report["stamp"] - displaced_at[0] >= 1.0


def test_crossing_cannot_continue_after_losing_low_altitude():
    from tools.mission_scenario import run_scenario

    def lift(context):
        if context["state"] == "WINDOW_1_CROSS":
            context["uav"].z += 0.2

    result = run_scenario(feedback_hook=lift)
    assert not result.completed and "crossing_altitude" in result.fault
    assert result.trace[-1]["command"].uav == (0, 0, 0, 0)


def test_window_descent_plan_accounts_for_recognition_and_arrival_tolerances():
    plan = demonstration_plan(protocol="inspection_v1")
    window = plan["windows"][0]
    window["approach"][2] = 1.11
    window["crossing_z"] = 1.0
    window["exit"][2] = 1.0
    window["bottom"] = 0.7
    window["top"] = 1.3

    with pytest.raises(ValueError, match="descent margin"):
        controller(plan)


def test_window_descent_uses_observed_downward_motion_without_unpublished_20cm_floor():
    from tools.mission_scenario import run_scenario

    plan = demonstration_plan(protocol="inspection_v1")
    window = plan["windows"][0]
    window["approach"][2] = 0.85
    window["crossing_z"] = 0.70
    window["exit"][2] = 0.70
    window["top"] = 0.85

    result = run_scenario(plan)

    assert result.completed, result.fault
    recognized = next(frame["uav"].z for frame in result.trace if frame["state"] == "WINDOW_1_RECOGNIZE")
    crossed = next(frame["uav"].z for frame in result.trace if frame["state"] == "WINDOW_1_CROSS")
    observed_descent = recognized - crossed
    assert observed_descent > 0.005
    assert observed_descent < 0.2


def test_below_minimum_aerial_height_cannot_be_counted_as_inspection():
    from tools.mission_scenario import run_scenario

    def lower(context):
        if context["state"] == "AIR_INSPECT_1":
            context["uav"].z = 0.7

    result = run_scenario(feedback_hook=lower)
    assert not result.completed and "minimum_height" in result.fault
    assert not any(r["target_id"] == "air-one" for r in result.reports)


def test_sensor_updates_may_be_slower_than_controller_without_resetting_stability():
    mission = first_inspection(recognition_stable_s=0.4)
    report = None
    for tick in range(4, 21):
        now = tick / 20
        sensor_time = math.floor(tick / 2) / 10
        command = mission.step(now, at_ground(now=sensor_time), None, [detection(now)])
        report = command.recognition or report
    assert report is not None


def test_visual_servo_requires_an_explicit_camera_direction():
    with pytest.raises(ValueError):
        controller(visual_servo_enabled=True)
    with pytest.raises(ValueError):
        controller(visual_servo_enabled=True, camera_yaw_sign=0)


@pytest.mark.parametrize("sign", [1, -1])
def test_opt_in_visual_servo_corrects_only_yaw_for_a_fresh_matching_target(sign):
    mission = first_inspection(visual_servo_enabled=True, camera_yaw_sign=sign)
    command = mission.step(0.2, at_ground(now=0.2), None, [detection(0.2, center=[0.8, 0.5])])
    assert command.ground[0] == 0 and command.ground[1] * sign > 0
    assert abs(command.ground[1]) <= 0.15 and command.recognition is None
    command = mission.step(0.3, at_ground(now=0.3), None, [detection(0.3, target="wrong", center=[0.8, 0.5])])
    assert command.ground == (0, 0) and command.recognition is None
    command = mission.step(0.4, at_ground(now=0.4), None, [])
    assert command.ground == (0, 0) and command.recognition is None


def test_visual_servo_stays_within_heading_bound_and_can_recognize_locked_heading():
    mission = first_inspection(visual_servo_enabled=True, camera_yaw_sign=1, visual_max_yaw_offset_rad=0.3)
    pose = at_ground(now=0.2)
    pose.yaw = 0.3
    command = mission.step(0.2, pose, None, [detection(0.2, center=[0.8, 0.5])])
    assert command.ground[1] <= 0
    report = None
    for tick in range(3, 15):
        now = tick / 10
        pose = at_ground(now=now)
        pose.yaw = 0.25
        command = mission.step(now, pose, None, [detection(now)])
        assert command.ground == (0, 0)
        report = command.recognition or report
    assert report and report["pose"][3] == pytest.approx(0.25)


def test_aerial_visual_servo_holds_position_and_altitude_while_locking():
    from tools.mission_scenario import run_scenario
    corrections = []

    def camera(context):
        if context["state"] == "AIR_INSPECT_1":
            yaw = context["uav"].yaw
            for item in context["detections"]:
                if item["target_id"] == "air-one":
                    item["center"] = [0.5 + (0.28 - yaw), 0.5]

    result = run_scenario(settings={"visual_servo_enabled": True, "camera_yaw_sign": 1}, feedback_hook=camera)
    assert result.completed, result.fault
    for frame in result.trace:
        command = frame["command"]
        if frame["state"] == "AIR_INSPECT_1" and command.uav[3] > 0:
            corrections.append(command)
            assert command.uav[:3] == (0, 0, 0)
            assert command.light is False
    assert corrections


@pytest.mark.parametrize("sign", [1, -1])
def test_visual_servo_never_commands_further_outward_at_either_heading_limit(sign):
    mission = first_inspection(visual_servo_enabled=True, camera_yaw_sign=sign, visual_max_yaw_offset_rad=0.3)
    pose = at_ground(now=0.2)
    pose.yaw = sign * 0.3
    command = mission.step(0.2, pose, None, [detection(0.2, center=[0.8, 0.5])])
    assert command.ground == (0, 0)


def test_unacknowledged_aerial_report_does_not_leave_light_on_past_three_seconds():
    from tools.mission_scenario import run_scenario

    def drop_aerial_ack(context):
        if context["state"] == "AIR_INSPECT_1":
            context["recognition_ack"] = set()

    result = run_scenario(settings={"phase_timeout_s": 15}, feedback_hook=drop_aerial_ack)
    assert not result.completed
    lit = [f for f in result.trace if f["state"] == "AIR_INSPECT_1" and f["command"].light]
    assert lit and lit[-1]["now"] - lit[0]["now"] <= 3.1
    assert result.trace[-1]["command"].light is False


@pytest.mark.parametrize("duration", [1.0, 2.0, 3.0])
def test_both_aerial_points_receive_the_configured_feedback_confirmed_illumination(duration):
    from tools.mission_scenario import run_scenario
    result = run_scenario(settings={"illumination_s": duration})
    assert result.completed, result.fault
    for point in (1, 2):
        frames = [f for f in result.trace if f["state"] == f"AIR_INSPECT_{point}"]
        first_feedback = next(f["now"] for f in frames if f["light_ready"])
        report = next(r for r in result.reports if r["target_id"] == ("air-one" if point == 1 else "air-two"))
        assert duration <= report["stamp"] - first_feedback + 1e-8 <= duration + 0.11
        assert frames[-1]["command"].light is False


def test_each_window_has_a_measured_vertical_descent_before_horizontal_crossing():
    from tools.mission_scenario import run_scenario
    result = run_scenario()
    assert result.completed
    for index in (1, 2):
        approach = next(f for f in result.trace if f["state"] == f"WINDOW_{index}_RECOGNIZE")
        crossing = next(f for f in result.trace if f["state"] == f"WINDOW_{index}_CROSS")
        assert approach["uav"].z - crossing["uav"].z > 0
        for frame in result.trace:
            if frame["state"] == f"WINDOW_{index}_DESCEND":
                assert frame["command"].uav[:2] == (0, 0)


def test_windows_are_declared_once_before_recognition_and_landing_before_descent():
    from tools.mission_scenario import run_scenario

    result = run_scenario()
    assert result.completed, result.fault
    declaration_frames = [frame for frame in result.trace if getattr(frame["command"], "declaration", None)]
    assert [frame["command"].declaration["value"] for frame in declaration_frames] == [
        "window_1", "window_2", "landing_marker",
    ]
    for frame in declaration_frames:
        declaration = frame["command"].declaration
        assert set(declaration) == {"channel", "value", "stamp", "state", "point_id"}
        assert declaration["channel"] == "cruise_event"
        assert declaration["stamp"] == frame["now"]
        assert declaration["state"] == frame["state"]
        assert frame["command"].ground == (0, 0)
        assert frame["command"].uav == (0, 0, 0, 0)
        assert frame["command"].recognition is None
    for index in (1, 2):
        declared = next(frame for frame in declaration_frames if frame["command"].declaration["value"] == f"window_{index}")
        first_recognition = next(frame for frame in result.trace if frame["state"] == f"WINDOW_{index}_RECOGNIZE")
        report = next(report for report in result.reports if report["point_id"] == f"w{index}")
        assert declared["now"] < first_recognition["now"] <= report["stamp"]
        assert declared["command"].declaration["point_id"] == f"w{index}"
    landing = declaration_frames[-1]
    first_downward_motion = next(frame for frame in result.trace if frame["state"] == "LAND" and frame["command"].uav[2] < 0)
    assert landing["now"] < first_downward_motion["now"]
    assert landing["command"].declaration["point_id"] == ""


def test_recognition_reports_include_ground_aerial_or_window_kind():
    from tools.mission_scenario import run_scenario

    result = run_scenario()
    assert result.completed, result.fault
    assert [report.get("kind") for report in result.reports] == [
        "ground", "ground", "ground", "window", "aerial", "aerial", "window",
    ]


def test_official_mode_requires_distinct_preentry_observation_poses_and_safe_speeds():
    import copy

    plan = demonstration_plan()
    plan["protocol"] = "inspection_v1"
    with pytest.raises(ValueError, match="approach"):
        controller(plan)
    for point in plan["ground_points"]:
        x, y, yaw = point["pose"]
        point["approach"] = [x - 0.24 * math.cos(yaw), y - 0.24 * math.sin(yaw), yaw]
    assert not controller(plan).completed
    near = copy.deepcopy(plan)
    near["ground_points"][0]["approach"] = list(near["ground_points"][0]["pose"])
    with pytest.raises(ValueError, match="approach"):
        controller(near)
    for name in ("max_ground_speed_mps", "max_uav_speed_mps"):
        with pytest.raises(ValueError, match="0.5"):
            controller(plan, **{name: 0.51})
    with pytest.raises(ValueError, match="ground order"):
        controller(plan, require_ground_order=False)
    missing = copy.deepcopy(plan)
    missing.pop("protocol")
    with pytest.raises(ValueError, match="protocol"):
        controller(missing)


def test_official_ground_identification_precedes_entry_and_arrival_declaration():
    from tools.mission_scenario import run_scenario

    plan = demonstration_plan(protocol="inspection_v1")
    result = run_scenario(plan, ground_order=["g3", "g1", "g2"])
    assert result.completed, result.fault
    for index, point_id in enumerate(("g3", "g1", "g2"), 1):
        point = next(point for point in plan["ground_points"] if point["id"] == point_id)
        report = next(report for report in result.reports if report["point_id"] == point_id)
        entering = next(frame for frame in result.trace if frame["state"] == f"GROUND_ENTER_{index}" and any(frame["command"].ground))
        declaration = next(frame for frame in result.trace if frame["command"].declaration and frame["command"].declaration["value"] == point_id)
        assert math.dist(report["pose"][:2], point["pose"][:2]) > 0.16
        assert report["stamp"] < entering["now"] < declaration["now"]
        assert declaration["command"].declaration["channel"] == "current_target"
        assert declaration["command"].declaration["point_id"] == point_id
        assert math.dist((declaration["ground"].x, declaration["ground"].y), point["pose"][:2]) <= 0.08


def test_official_mode_never_enters_a_point_without_recognition_acceptance():
    from tools.mission_scenario import run_scenario

    result = run_scenario(demonstration_plan(protocol="inspection_v1"), acknowledge=False,
                          settings={"phase_timeout_s": 15.0})
    assert not result.completed and result.reports
    assert "GROUND_RECOGNIZE_1" in result.states and "GROUND_ENTER_1" not in result.states
    assert not any(frame["command"].declaration for frame in result.trace)


def test_official_aerial_report_is_accepted_before_any_light_and_is_not_repeated():
    from tools.mission_scenario import run_scenario

    result = run_scenario(demonstration_plan(protocol="inspection_v1"))
    assert result.completed, result.fault
    for index, target in ((1, "air-one"), (2, "air-two")):
        reports = [report for report in result.reports if report["target_id"] == target]
        assert len(reports) == 1
        lights = [frame for frame in result.trace if frame["state"] == f"AIR_INSPECT_{index}" and frame["command"].light]
        assert lights and reports[0]["stamp"] < lights[0]["now"]
        assert reports[0]["state"] == f"AIR_RECOGNIZE_{index}"
        assert all(frame["command"].recognition is None for frame in lights)


def test_official_aerial_loss_of_identity_restarts_recognition_before_relighting():
    from tools.mission_scenario import run_scenario

    plan = demonstration_plan(protocol="inspection_v1")
    plan["aerial_points"][0]["target_id"] = "*"
    switched = []

    def camera(context):
        if context["state"] == "AIR_INSPECT_1" and not switched:
            switched.append(context["now"])
        for item in context["detections"]:
            if item["target_id"] == "*":
                item["target_id"] = "camera-b" if switched else "camera-a"

    result = run_scenario(plan, feedback_hook=camera)
    assert result.completed, result.fault
    reports = [report for report in result.reports if report["point_id"] == "a1"]
    assert [report["target_id"] for report in reports] == ["camera-a", "camera-b"]
    assert reports[0]["event_id"] != reports[1]["event_id"]
    first_relight = next(frame for frame in result.trace if frame["state"] == "AIR_INSPECT_1" and frame["command"].light)
    assert reports[1]["stamp"] < first_relight["now"]


def test_official_aerial_missing_acceptance_never_turns_on_light():
    from tools.mission_scenario import run_scenario

    def drop_air_ack(context):
        if context["state"] == "AIR_RECOGNIZE_1":
            context["recognition_ack"] = set()

    result = run_scenario(demonstration_plan(protocol="inspection_v1"), settings={"phase_timeout_s": 15.0},
                          feedback_hook=drop_air_ack)
    assert not result.completed
    assert "AIR_RECOGNIZE_1" in result.states and "AIR_INSPECT_1" not in result.states
    assert not any(frame["command"].light for frame in result.trace)


@pytest.mark.parametrize("failure", ["blank", "offcenter", "ambiguous"])
def test_official_aerial_invalid_evidence_requires_a_new_report_before_relighting(failure):
    from tools.mission_scenario import run_scenario

    plan = demonstration_plan(protocol="inspection_v1")
    plan["aerial_points"][0]["target_id"] = "*"
    interrupted = []

    def camera(context):
        for item in context["detections"]:
            if item["target_id"] == "*":
                item["target_id"] = "actual-target"
        if context["state"] == "AIR_INSPECT_1" and context["light_ready"] and not interrupted:
            interrupted.append(context["now"])
            if failure == "blank":
                context["detections"].clear()
            elif failure == "offcenter":
                context["detections"][0]["center"] = [0.9, 0.5]
            else:
                context["detections"].append(detection(context["now"], source="uav", target="another-target"))

    result = run_scenario(plan, feedback_hook=camera)
    assert result.completed, result.fault
    reports = [report for report in result.reports if report["point_id"] == "a1"]
    assert len(reports) == 2 and reports[0]["event_id"] != reports[1]["event_id"]
    interrupted_frame = next(frame for frame in result.trace if frame["now"] == interrupted[0])
    assert interrupted_frame["state"] == "AIR_RECOGNIZE_1" and not interrupted_frame["command"].light
    relit = next(frame for frame in result.trace if frame["now"] > interrupted[0] and frame["command"].light)
    assert reports[1]["stamp"] < relit["now"]


@pytest.mark.parametrize("feedback_delay_s", [0.1, 0.4])
def test_official_total_light_command_is_capped_at_three_seconds_including_feedback_delay(feedback_delay_s):
    from tools.mission_scenario import run_scenario

    on_started = []

    def delayed_light(context):
        if context["state"] == "AIR_INSPECT_1" and context["light_ready"]:
            if not on_started:
                on_started.append(context["now"] - 0.1)
            context["light_ready"] = context["now"] - on_started[0] + 1e-9 >= feedback_delay_s

    result = run_scenario(demonstration_plan(protocol="inspection_v1"),
                          settings={"illumination_s": 3.0}, feedback_hook=delayed_light)
    first_on = next(frame["now"] for frame in result.trace if frame["command"].light)
    first_off = next(frame["now"] for frame in result.trace if frame["now"] > first_on and not frame["command"].light)
    assert first_off - first_on <= 3.0 + 1e-9
    assert not result.completed and result.fault == "illumination_time_limit"


@pytest.mark.parametrize("case", ["same_stamp_changed_pose", "reversed_stamp"])
def test_inconsistent_or_reversed_pose_samples_stop_actuation(case):
    mission = first_inspection()
    pose = at_ground(now=0.1 if case == "same_stamp_changed_pose" else 0.05)
    if case == "same_stamp_changed_pose":
        pose.x += 0.01
    command = mission.step(0.2, pose, None, [detection(0.2)])
    assert command.fault and command.ground == (0, 0) and not command.done


def test_evidence_settings_cannot_require_more_than_three_seconds_of_illumination():
    with pytest.raises(ValueError):
        controller(recognition_stable_s=4.0)


def test_all_physical_observation_points_may_use_wildcard_without_storing_answers():
    plan = demonstration_plan()
    for group in ("ground_points", "aerial_points", "windows"):
        for point in plan[group]:
            point["target_id"] = "*"
    mission = controller(plan)
    command = mission.step(0.0, Pose2D(), None, [], ground_order=["g1", "g2", "g3"])
    assert command.state == "GROUND_NAV_1" and not command.fault


def test_wildcard_report_contains_camera_decoded_identity_and_physical_point_id():
    import cv2
    from inspection_mission.vision_core import detect_targets

    # The identity is decoded from actual pixels; the controller's map has no
    # matching answer and only describes the measured observation location.
    image = cv2.QRCodeEncoder_create().encode("runtime-random-7259")
    image = cv2.resize(image, (232, 232), interpolation=cv2.INTER_NEAREST)
    image = cv2.copyMakeBorder(image, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    observed = detect_targets(image, enable_aruco=False)
    assert len(observed) == 1 and observed[0]["target_id"] == "runtime-random-7259"
    plan = demonstration_plan()
    plan["ground_points"][0]["target_id"] = "*"
    mission = first_inspection(plan)
    report = None
    for tick in range(2, 13):
        now = tick / 10
        camera = [{**item, "source": "ground", "stamp": now} for item in observed]
        command = mission.step(now, at_ground(now=now), None, camera)
        report = command.recognition or report
    assert report and report["target_id"] == "runtime-random-7259"
    assert report["point_id"] == "g1" and report["target_id"] != "*"


def test_wildcard_does_not_accumulate_when_two_identities_are_centered():
    plan = demonstration_plan()
    plan["ground_points"][0]["target_id"] = "*"
    mission = first_inspection(plan)
    for tick in range(2, 15):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None,
                               [detection(now, target="actual-a"), detection(now, target="actual-b")])
        assert command.recognition is None and command.ground == (0, 0)
    report = None
    for tick in range(15, 23):
        now = tick / 10
        report = mission.step(now, at_ground(now=now), None, [detection(now, target="actual-b")]).recognition or report
    assert report and report["target_id"] == "actual-b"


def test_wildcard_identity_change_restarts_the_entire_stable_interval():
    plan = demonstration_plan()
    plan["ground_points"][0]["target_id"] = "*"
    mission = first_inspection(plan, recognition_stable_s=0.5)
    for tick in range(2, 7):
        now = tick / 10
        assert mission.step(now, at_ground(now=now), None, [detection(now, target="actual-a")]).recognition is None
    for tick in range(7, 12):
        now = tick / 10
        assert mission.step(now, at_ground(now=now), None, [detection(now, target="actual-b")]).recognition is None
    report = mission.step(1.2, at_ground(now=1.2), None, [detection(1.2, target="actual-b")]).recognition
    assert report and report["target_id"] == "actual-b" and report["point_id"] == "g1"


def test_identity_change_cancels_pending_report_so_its_ack_cannot_advance():
    plan = demonstration_plan()
    plan["ground_points"][0]["target_id"] = "*"
    mission = first_inspection(plan, recognition_stable_s=0.2)
    previous = None
    for tick in range(2, 7):
        now = tick / 10
        previous = mission.step(now, at_ground(now=now), None, [detection(now, target="actual-a")]).recognition or previous
    assert previous
    newer = None
    for tick in range(7, 14):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None, [detection(now, target="actual-b")], recognition_ack={previous["event_id"]})
        newer = command.recognition or newer
        assert command.state == "GROUND_RECOGNIZE_1"
    assert newer and newer["target_id"] == "actual-b" and newer["event_id"] != previous["event_id"]


@pytest.mark.parametrize("group,index,point_id", [("windows", 0, "w1"), ("aerial_points", 1, "a2")])
def test_window_and_aerial_wildcards_report_observed_identity_after_full_behavior(group, index, point_id):
    from tools.mission_scenario import run_scenario
    plan = demonstration_plan()
    plan[group][index]["target_id"] = "*"

    def camera(context):
        for item in context["detections"]:
            if item["target_id"] == "*":
                item["target_id"] = "camera-runtime-3941"

    result = run_scenario(plan, feedback_hook=camera)
    assert result.completed, result.fault
    report = next(r for r in result.reports if r["target_id"] == "camera-runtime-3941")
    assert report["point_id"] == point_id and report["source"] == "uav"


def test_explicitly_assigned_identity_still_reports_its_physical_point_id():
    mission = first_inspection()
    report = None
    for tick in range(2, 12):
        now = tick / 10
        report = mission.step(now, at_ground(now=now), None, [detection(now)]).recognition or report
    assert report and report["point_id"] == "g1" and report["target_id"] == "ground-one"


@pytest.mark.parametrize("bad", [
    {"source": "uav"}, {"stamp": -10.0}, {"center": [0.1, 0.5]},
    {"confidence": 0.1}, {"target_id": "*"}, {"target_id": ""},
])
def test_wildcard_keeps_camera_freshness_identity_and_center_gates(bad):
    plan = demonstration_plan()
    plan["ground_points"][0]["target_id"] = "*"
    mission = first_inspection(plan)
    for tick in range(2, 15):
        now = tick / 10
        item = {**detection(now, target="actual-visible"), **bad}
        command = mission.step(now, at_ground(now=now), None, [item])
        assert command.recognition is None and command.ground == (0, 0)


def test_wildcard_ambiguity_cancels_a_report_before_acknowledgement():
    plan = demonstration_plan()
    plan["ground_points"][0]["target_id"] = "*"
    mission = first_inspection(plan, recognition_stable_s=0.2)
    old = None
    for tick in range(2, 7):
        now = tick / 10
        old = mission.step(now, at_ground(now=now), None, [detection(now, target="actual-a")]).recognition or old
    assert old
    command = mission.step(0.7, at_ground(now=0.7), None,
                           [detection(0.7, target="actual-a"), detection(0.7, target="actual-b")],
                           recognition_ack={old["event_id"]})
    assert command.state == "GROUND_RECOGNIZE_1" and command.recognition is None
    new = None
    for tick in range(8, 14):
        now = tick / 10
        command = mission.step(now, at_ground(now=now), None, [detection(now, target="actual-a")], recognition_ack={old["event_id"]})
        assert command.state == "GROUND_RECOGNIZE_1"
        new = command.recognition or new
    assert new and new["event_id"] != old["event_id"]


def test_wildcard_identity_change_requires_new_full_aerial_illumination():
    from tools.mission_scenario import run_scenario
    plan = demonstration_plan()
    plan["aerial_points"][0]["target_id"] = "*"
    first_seen, changed = [], []

    def camera(context):
        if context["state"] == "AIR_INSPECT_1" and not first_seen:
            first_seen.append(context["now"])
        switched = bool(first_seen) and context["now"] - first_seen[0] >= 0.7
        if context["state"] == "AIR_INSPECT_1" and switched and not changed:
            changed.append(context["now"])
        for item in context["detections"]:
            if item["target_id"] == "*":
                item["target_id"] = "actual-b" if switched else "actual-a"

    result = run_scenario(plan, settings={"illumination_s": 1.0}, feedback_hook=camera)
    assert result.completed, result.fault
    assert not any(r["target_id"] == "actual-a" for r in result.reports)
    report = next(r for r in result.reports if r["target_id"] == "actual-b")
    assert report["point_id"] == "a1" and report["stamp"] - changed[0] >= 1.0
