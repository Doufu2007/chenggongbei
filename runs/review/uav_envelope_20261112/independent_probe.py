"""Read-only independent geometry/runtime review probes; no ROS or network."""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src/inspection_mission"), str(ROOT), str(ROOT / "tests")]

from inspection_mission.envelope import uav_envelope_from_plan
from inspection_mission.mission import MissionController
from inspection_mission.mission_node import motion_path_clear
from inspection_mission.perception import yaw_level_rotation
from inspection_mission.runtime import RuntimeInputs
from test_runtime import configured_profile, odom
from tools.mission_scenario import demonstration_plan


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def plan():
    p = demonstration_plan()
    p.update(robot_radius_m=.447, clearance_m=.03,
             uav_body_envelope={"below_origin_m": .068, "above_origin_m": .166,
                                "max_tilt_rad": .05})
    for w in p["windows"]:
        w.update(width_m=1., bottom=.4, top=1.15, crossing_z=.75)
        w["exit"][2] = .75
        w["exit"][0] = w["approach"][0] + 1.2*math.cos(w["approach"][3])
    return p


def runtime(q, level_points, stamp_ns=0):
    p = configured_profile()
    p["uav_obstacle_sensor"] = "pointcloud"
    p["max_planar_tilt_rad"] = .1
    del p["channels"]["uav_scan"]
    p["channels"]["uav_cloud"] = {
        "topic": "/uav/lidar/points", "type": "sensor_msgs/msg/PointCloud2",
        "frame_id": "review/lidar", "translation_m": [.12, 0., .18],
        "rotation_rpy_rad": [0., 0., 0.], "horizontal_fov_rad": math.pi,
        "vertical_fov_rad": 1.1, "range_min_m": .05, "range_max_m": 4.}
    e = uav_envelope_from_plan(plan())
    data = RuntimeInputs(p, {"g1", "g2", "g3"}, uav_envelope=e)
    payload = odom(frame="uav/odom")
    payload["pose"]["pose"]["orientation"] = dict(zip(("x", "y", "z", "w"), q))
    data.update_pose("uav", payload, now=5)
    yaw = data.poses["uav"].yaw
    rot = yaw_level_rotation(q, yaw)
    # Test oracle uses row-vector inverse of an orthonormal rigid rotation.
    sensor_points = np.asarray(level_points) @ rot - np.array([.12, 0., .18])
    data.update_cloud({"frame_id": "review/lidar", "stamp": {"sec": 5, "nanosec": stamp_ns}},
                      sensor_points.tolist(), now=5)
    return data, rot


def run():
    results = {}
    e = uav_envelope_from_plan(plan())
    # Independent exact rotated cylinder surface, rather than the implementation formula.
    angles = np.linspace(-math.pi, math.pi, 721)
    ring = np.array([[.447*math.cos(a), .447*math.sin(a), z]
                     for a in angles for z in (-.068, .166)])
    count = 0
    extrema = [0., 0., 0.]
    for axis in np.linspace(-math.pi, math.pi, 37):
        for tilt in np.linspace(0., .05, 11):
            u = np.array([math.cos(axis), math.sin(axis), 0.])
            cross = np.array([[0., -u[2], u[1]], [u[2], 0., -u[0]], [-u[1], u[0], 0.]])
            rot = math.cos(tilt)*np.eye(3)+(1-math.cos(tilt))*np.outer(u,u)+math.sin(tilt)*cross
            tilted = ring @ rot.T
            max_r = float(np.max(np.linalg.norm(tilted[:, :2], axis=1)))
            low, high = float(-tilted[:,2].min()), float(tilted[:,2].max())
            assert max_r <= e.horizontal_radius_m-.03+1e-12
            assert low <= e.below_origin_m-.03+1e-12
            assert high <= e.above_origin_m-.03+1e-12
            extrema = [max(extrema[0],max_r), max(extrema[1],low), max(extrema[2],high)]
            count += len(tilted)
    results["exact_cylinder_boundary"] = {"points_checked": count, "maxima": extrema, "passed": True}

    far = [[2., 0., 0.], [2.2, 0., 0.], [2.4, 0., 0.]]
    combined = []
    for roll, pitch, yaw in ((.025,.025,1.1), (-.03,.02,-1.7), (.01,-.04,2.8), (0.,0.,-.9)):
        q = quaternion(roll,pitch,yaw)
        data, rot = runtime(q, far+[[.8,0.,.26]])
        assert np.max(np.abs(rot.T @ rot-np.eye(3))) < 1e-12
        velocity = (.2*math.cos(data.poses["uav"].yaw), .2*math.sin(data.poses["uav"].yaw), 0., 0.)
        clear = data.motion_clear("uav", velocity, 5, stop_distance=1.2,
                                  corridor_radius=e.horizontal_radius_m)
        assert clear
        blocked, _ = runtime(q, far+[[.8,0.,.20]])
        assert not blocked.motion_clear("uav", velocity, 5, stop_distance=1.2,
                                        corridor_radius=e.horizontal_radius_m)
        skewed, _ = runtime(q, far, stamp_ns=1)
        assert not skewed.motion_clear("uav", velocity, 5, stop_distance=1.2,
                                       corridor_radius=e.horizontal_radius_m)
        combined.append({"rpy": [roll,pitch,yaw], "clear":clear,
                         "inside_obstacle_rejected": True, "one_ns_skew_rejected": True})
    results["combined_attitudes"] = combined

    accepted = MissionController(plan(), {"position_tolerance_m": .01})
    try:
        MissionController(plan())
    except ValueError as exc:
        assert "horizontal clearance" in str(exc)
        results["default_tolerance_rejected"] = str(exc)
    else:
        raise AssertionError("default tolerance was accepted")
    invalid = []
    for key, value in (("below_origin_m",True), ("above_origin_m",float("inf")),
                       ("max_tilt_rad",float("nan")), ("above_origin_m",-.01)):
        bad = plan()
        bad["uav_body_envelope"][key] = value
        try:
            MissionController(bad, {"position_tolerance_m":.01})
        except ValueError:
            invalid.append(key)
        else:
            raise AssertionError((key,value))
    results["invalid_configuration_rejected"] = invalid
    data, _ = runtime(quaternion(0.,0.,0.), far)
    assert not motion_path_clear(accepted,data,"uav",(.2,0.,0.,0.),5)
    # At the real tick distance R+v*timeout=.5853, front FOV cannot cover this candidate.
    results["real_tick_front_fov_still_refused"] = True
    files = ["envelope.py","mission.py","mission_node.py","runtime.py","perception.py"]
    results["source_sha256"] = {name: hashlib.sha256((ROOT / "src/inspection_mission/inspection_mission" / name).read_bytes()).hexdigest() for name in files}
    results["platform_verified"] = False
    target = Path(__file__).with_name("independent_metrics.json")
    target.write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({"passed":True,"cylinder_points":count,"combined_attitudes":len(combined),
                      "output":str(target.relative_to(ROOT))}))


if __name__ == "__main__":
    run()
