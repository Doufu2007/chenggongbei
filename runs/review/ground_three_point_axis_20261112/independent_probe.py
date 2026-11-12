"""Independent perturbed-pose chain review; no ROS and no platform claim."""
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/inspection_mission"))
from inspection_mission import ground_three_point_axis_plan as axis

END = {"ground_0": (1.115, -1.35, 0.), "ground_1": (0., -1.35, math.pi),
       "ground_2": (0., -2.69, math.pi), "ground_3": (1.115, -1.925, -math.pi/2),
       "ground_4": (1.115, -.775, -math.pi/2)}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def check_chain(plan, pose):
    xy, yaw = pose[:2], wrap(pose[2])
    path = angle = 0.
    reverse = 0
    for group in (plan.to_observation_operations, plan.entry_operations):
        for op in group:
            assert math.dist(xy, op.anchor_xy_m) < 1e-12
            assert abs(wrap(yaw-op.expected_start_yaw_rad)) < 1e-12
            if op.kind == "TURN_ONLY":
                assert op.target_xy_m is None and op.length_m == 0.
                assert abs(wrap(yaw+op.signed_subturn_rad-op.target_yaw_rad)) < 1e-12
                assert 0. < abs(op.signed_subturn_rad) <= math.pi/2+1e-12
                angle += abs(op.signed_subturn_rad)
                yaw = op.target_yaw_rad
            else:
                assert op.kind in ("DRIVE_TO_XY", "DRIVE_REVERSE_TO_XY")
                dx, dy = op.target_xy_m[0]-xy[0], op.target_xy_m[1]-xy[1]
                assert min(abs(dx), abs(dy)) < 1e-12
                assert abs(wrap(yaw-op.target_yaw_rad)) < 1e-12
                sign = dx*math.cos(yaw)+dy*math.sin(yaw)
                assert sign < 0. if op.kind == "DRIVE_REVERSE_TO_XY" else sign > 0.
                assert math.isclose(math.hypot(dx,dy), op.length_m, abs_tol=1e-12)
                path += math.hypot(dx,dy)
                reverse += int(op.kind == "DRIVE_REVERSE_TO_XY")
                xy = op.target_xy_m
        if group is plan.to_observation_operations:
            assert math.dist(xy, plan.observation_xy_m) < 1e-12
            assert abs(wrap(yaw-plan.observation_yaw_rad)) < 1e-12
    assert math.dist(xy, plan.endpoint_xy_m) < 1e-12
    assert abs(wrap(yaw-plan.endpoint_yaw_rad)) < 1e-12
    assert math.isclose(path, plan.path_m, abs_tol=1e-12)
    assert math.isclose(angle, plan.rotation_rad, abs_tol=1e-12)
    return reverse


def main():
    source = ROOT/"src/inspection_mission/inspection_mission/ground_three_point_axis_plan.py"
    sha = hashlib.sha256(source.read_bytes()).hexdigest()
    assert sha == "e1bb65499f24eba7389b645dd29c10539bd7942fa51675d57912d95454deba8e"
    accepted = rejected = reverse = 0
    # New perturbed actual poses exercise anchors rather than rerunning the
    # author's nominal sixty-order and 501-point geometry checks.
    for previous in (None, *END):
        original = (0.,0.,0.) if previous is None else END[previous]
        for target in END:
            if target == previous:
                continue
            for dx,dy,da in itertools.product((-.003,0.,.003),(-.003,0.,.003),(-.008,0.,.008)):
                pose = (original[0]+dx, original[1]+dy, original[2]+da)
                try:
                    plan = axis.plan_ground_leg(previous,target,pose)
                except ValueError:
                    rejected += 1
                    continue
                assert plan.initial_pose == pose
                reverse += check_chain(plan,pose)
                assert (target == "ground_2") == bool(any(
                    op.kind == "DRIVE_REVERSE_TO_XY" for op in plan.entry_operations))
                accepted += 1
    # The <.641 initial ring counterexample must be rejected before planning.
    try:
        axis.plan_ground_leg("ground_2","ground_0",(0.,-2.71,math.pi))
    except ValueError:
        south_rejected = True
    else:
        raise AssertionError("unsafe south departure accepted")
    result = {"source_sha256":sha,"accepted_perturbed_chains":accepted,
        "rejected_perturbed_chains":rejected,"explicit_reverse_ops":reverse,
        "unsafe_south_departure_rejected":south_rejected,"all_checks_passed":True,
        "runtime_integrated":False,"official_score_verified":False}
    out = Path(__file__).with_name("independent_metrics.json")
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
