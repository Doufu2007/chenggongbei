"""Pure-plan contract and independent sampled checks of fixed candidate geometry."""
from dataclasses import FrozenInstanceError
import importlib
import itertools
import math

import pytest


def module():
    return importlib.import_module("inspection_mission.ground_three_point_plan")


ENDPOINTS = {"ground_0": (1.115, -1.35, 0.), "ground_1": (0., -1.35, math.pi),
    "ground_2": (0., -2.69, math.pi), "ground_3": (1.115, -1.925, -math.pi/2),
    "ground_4": (1.115, -.775, -math.pi/2)}


def test_frozen_tuples_candidate_provenance_and_ground0_scored_axis():
    m = module()
    p = m.plan_ground_leg(None, "ground_0", (0., 0., 0.))
    assert p.initial_pose == (0., 0., 0.)
    assert p.observation_xy_m == (.6, -.45)
    assert p.endpoint_xy_m == (1.115, -1.35)
    assert p.endpoint_yaw_rad == 0.
    assert p.path_m == pytest.approx(2.465)
    assert p.rotation_rad == pytest.approx(4.197741550442512)
    assert tuple(x.target_xy_m for x in p.entry_operations if x.kind == "DRIVE_TO_XY") == ((.6, -1.35), (1.115, -1.35))
    assert isinstance(p.entry_operations, tuple) and isinstance(p.to_observation_operations, tuple)
    assert isinstance(m.BOXES, tuple) and len(m.BOXES) == 10
    assert m.CANDIDATE_WORLD_SHA256 == "595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1"
    assert p.candidate_mapping_status == "candidate_unverified"
    with pytest.raises(FrozenInstanceError):
        p.path_m = 0.
    with pytest.raises(FrozenInstanceError):
        p.entry_operations[0].kind = "DRIVE_TO_XY"


def test_all_sixty_orders_have_original_geometry_caps_and_small_explicit_turns():
    m = module()
    totals = []
    for order in itertools.permutations(ENDPOINTS, 3):
        previous, pose, path, rotation, drive_count, turn_count = None, (0., 0., 0.), 0., 0., 0, 0
        for target in order:
            p = m.plan_ground_leg(previous, target, pose)
            assert p.minimum_laser_distance_m >= .65-1e-12
            for op in p.to_observation_operations+p.entry_operations:
                assert op.settle_required_after
                if op.kind == "TURN_ONLY":
                    assert 0. < abs(op.signed_subturn_rad) <= math.pi/2+1e-12
                    assert op.rotation_rad == abs(op.signed_subturn_rad)
                    assert op.turn_direction == ("CCW" if op.signed_subturn_rad > 0 else "CW")
                    turn_count += 1
                else:
                    assert op.kind == "DRIVE_TO_XY"
                    drive_count += 1
            path += p.path_m
            rotation += p.rotation_rad
            pose, previous = ENDPOINTS[target], target
        totals.append((path, rotation, order))
        assert path <= 11.114290096809475+1e-10
        assert rotation <= 23.304866760825774+1e-10
        assert drive_count <= 12 and turn_count <= 20
    assert max(totals)[2] == ("ground_2", "ground_0", "ground_4")
    assert max(row[1] for row in totals) == pytest.approx(23.304866760825774)


@pytest.mark.parametrize("start_yaw", [math.pi, -math.pi, 3*math.pi])
def test_ground2_departure_uses_north_half_circle_for_equivalent_yaws(start_yaw):
    p = module().plan_ground_leg("ground_2", "ground_0", (0., -2.69, start_yaw))
    turns = []
    for op in p.to_observation_operations:
        if op.kind == "DRIVE_TO_XY":
            break
        turns.append(op)
    assert len(turns) == 2
    assert sum(abs(x.signed_subturn_rad) for x in turns) == pytest.approx(math.pi)
    for op in turns:
        for i in range(51):
            yaw = op.expected_start_yaw_rad+op.signed_subturn_rad*i/50
            assert -.08*math.sin(yaw) <= 1e-12  # laser stays on north half.


def test_actual_pose_changes_exit_and_unsafe_allowed_endpoint_error_is_rejected():
    m = module()
    p = m.plan_ground_leg("ground_2", "ground_3", (.01, -2.68, math.pi-.01))
    assert p.initial_pose == (.01, -2.68, math.pi-.01)
    drive = next(x for x in p.to_observation_operations if x.kind == "DRIVE_TO_XY")
    assert drive.anchor_xy_m == (.01, -2.68)
    assert drive.target_xy_m == (.6, -2.68)
    with pytest.raises(ValueError, match="unsafe"):
        m.plan_ground_leg("ground_2", "ground_3", (0., -2.71, math.pi))


def test_entry_replans_from_actual_observation_and_ground4_new_observation_is_used():
    m = module()
    p = m.plan_ground_leg(None, "ground_4", (0., 0., 0.))
    assert p.observation_xy_m == (.35, -.10)
    assert p.observation_yaw_rad == pytest.approx(-.7165416740964037)
    actual = (.343, -.103, p.observation_yaw_rad+.01)
    ops = m.plan_ground_entry("ground_4", actual)
    assert isinstance(ops, tuple)
    assert ops[0].anchor_xy_m == actual[:2]
    assert ops[0].expected_start_yaw_rad == pytest.approx(actual[2])
    assert next(x for x in ops if x.kind == "DRIVE_TO_XY").anchor_xy_m == actual[:2]
    assert ops[-1].kind == "TURN_ONLY"
    assert ops[-1].target_yaw_rad == pytest.approx(-math.pi/2)


@pytest.mark.parametrize("previous,target,pose", [
    ("bad", "ground_0", (0., 0., 0.)), (None,"bad",(0.,0.,0.)),
    ([], "ground_0", (0.,0.,0.)), (None, [], (0.,0.,0.)),
    (None,"ground_0",(0.,0.)), (None,"ground_0",(False,0.,0.)),
    (None,"ground_0",(0.,float("nan"),0.)), (None,"ground_0",(0.,0.,float("inf"))),
    (None,"ground_0",(0.,.63,0.)),
])
def test_invalid_or_statically_unsafe_inputs_fail_closed(previous,target,pose):
    with pytest.raises(ValueError):
        module().plan_ground_leg(previous,target,pose)


def test_analytic_clearance_agrees_with_independent_dense_pose_sampling():
    m = module()
    for previous, target, pose in [(None,"ground_4",(0.,0.,0.)),
        ("ground_2","ground_0",ENDPOINTS["ground_2"]),
        ("ground_4","ground_2",ENDPOINTS["ground_4"])]:
        p = m.plan_ground_leg(previous,target,pose)
        for op in p.to_observation_operations+p.entry_operations:
            sampled = math.inf
            for i in range(401):
                u = i/400
                yaw = op.expected_start_yaw_rad+u*op.signed_subturn_rad if op.kind == "TURN_ONLY" else op.target_yaw_rad
                if op.kind == "TURN_ONLY":
                    x,y = op.anchor_xy_m
                else:
                    x = op.anchor_xy_m[0]+u*(op.target_xy_m[0]-op.anchor_xy_m[0])
                    y = op.anchor_xy_m[1]+u*(op.target_xy_m[1]-op.anchor_xy_m[1])
                x,y = x+.08*math.cos(yaw), y+.08*math.sin(yaw)
                for _, (x0,y0,x1,y1) in m.BOXES:
                    sampled = min(sampled,math.hypot(max(x0-x,0.,x-x1),max(y0-y,0.,y-y1)))
            assert sampled >= .641-1e-12
            assert op.minimum_laser_distance_m <= sampled+1e-12
            assert sampled-op.minimum_laser_distance_m < 2e-5
