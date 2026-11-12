"""Read-only FOV diagnosis using public geometry and production perception."""
from pathlib import Path
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/inspection_mission"))
from inspection_mission.envelope import uav_envelope_from_plan
from inspection_mission.perception import pointcloud_sensor_envelope_path_clear

WORLD = ROOT / "external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
tree = ET.parse(WORLD)
sensor = tree.find(".//model[@name='amphibious_drone']/link[@name='base_footprint']/sensor[@name='uav_3d_lidar']")
assert sensor is not None
offset = list(map(float, sensor.findtext("pose").split()[:3]))
scan = sensor.find("ray/scan")
spec = {
    "translation_m": offset,
    "horizontal_fov_rad": float(scan.findtext("horizontal/max_angle")) - float(scan.findtext("horizontal/min_angle")),
    "vertical_fov_rad": float(scan.findtext("vertical/max_angle")) - float(scan.findtext("vertical/min_angle")),
    "range_min_m": float(sensor.findtext("ray/range/min")),
    "range_max_m": float(sensor.findtext("ray/range/max")),
}
# The production profile's <= pi contract conservatively clips SDF's 3.1416.
spec["horizontal_fov_rad"] = min(spec["horizontal_fov_rad"], math.pi)
candidate = json.loads((ROOT / "docs/candidates/uav_body_envelope_20261112.json").read_text(encoding="utf-8"))
e = uav_envelope_from_plan(candidate["plan_fragment"])
far = np.array([[2.,0.,0.], [2.2,0.,0.], [2.4,0.,0.]]) - offset

def gate(distance, rotation=np.eye(3), points=far):
    return pointcloud_sensor_envelope_path_clear(points, (.2,0.,0.), spec,
        stop_distance=distance, corridor_radius=e.horizontal_radius_m,
        envelope=e, body_to_level=rotation)

threshold = offset[0] + (offset[2]+e.below_origin_m) / math.tan(spec["vertical_fov_rad"]/2)
real_distance = e.horizontal_radius_m + .2*.5
angle = math.atan2(offset[2]+e.below_origin_m, real_distance-offset[0])
assert not gate(real_distance)
assert not gate(threshold-1e-6)
assert gate(threshold+1e-6)

tilts = []
for axis_angle in np.linspace(-math.pi, math.pi, 73):
    axis = np.array([math.cos(axis_angle), math.sin(axis_angle), 0.])
    cross = np.array([[0.,0.,axis[1]], [0.,0.,-axis[0]], [-axis[1],axis[0],0.]])
    a = e.max_tilt_rad
    rotation = math.cos(a)*np.eye(3) + (1-math.cos(a))*np.outer(axis,axis) + math.sin(a)*cross
    lo, hi = .3, 1.5
    for _ in range(50):
        mid = (lo+hi)/2
        if gate(mid, rotation, (far+offset) @ rotation-offset):
            hi = mid
        else:
            lo = mid
    tilts.append({"axis_angle_rad": float(axis_angle), "minimum_cap_distance_m": hi})

# A point just ahead of the present rotor envelope, below the visible near field.
# A longer distal cap passes even though the new swept strip is unobserved.
hidden_level = np.array([.52, 0., -.11])
hidden_sensor = hidden_level-offset
hidden_elevation = abs(math.atan2(hidden_sensor[2], math.hypot(*hidden_sensor[:2])))
assert 0 < hidden_level[0] < real_distance
assert hidden_level[0] > e.horizontal_radius_m
assert -e.below_origin_m < hidden_level[2] < e.above_origin_m
assert hidden_elevation > spec["vertical_fov_rad"]/2
assert gate(1.2)
assert gate(1.2, points=np.vstack([far, hidden_sensor]))

payload = {
    "schema": "independent_front_fov_diagnosis_v1",
    "platform_verified": False,
    "world_sha256": hashlib.sha256(WORLD.read_bytes()).hexdigest(),
    "source_sha256": {name: hashlib.sha256((ROOT/"src/inspection_mission/inspection_mission"/name).read_bytes()).hexdigest()
                      for name in ("envelope.py", "perception.py", "mission_node.py", "runtime.py")},
    "public_sensor_spec": spec,
    "envelope": vars(e),
    "real_tick_distance_m": real_distance,
    "real_tick_required_lower_elevation_rad": angle,
    "real_tick_gate": gate(real_distance),
    "level_minimum_cap_distance_m": threshold,
    "level_threshold_boundary_checked_m": 1e-6,
    "tilt_grid": tilts,
    "tilt_grid_max": max(tilts, key=lambda x:x["minimum_cap_distance_m"]),
    "longer_cap_blind_strip_counterexample": {
        "level_point_m": hidden_level.tolist(),
        "sensor_point_m": hidden_sensor.tolist(),
        "elevation_rad": hidden_elevation,
        "outside_sensor_aperture": True,
        "within_new_required_swept_strip": True,
        "gate_with_longer_cap_and_no_hidden_point": gate(1.2),
        "gate_with_longer_cap_and_out_of_aperture_point": gate(1.2, points=np.vstack([far, hidden_sensor])),
        "interpretation": "Longer cap inclusion cannot certify unobserved near-field free space.",
    },
}
out = Path(__file__).with_name("metrics.json")
out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps({k:payload[k] for k in ("real_tick_distance_m", "real_tick_required_lower_elevation_rad", "level_minimum_cap_distance_m", "tilt_grid_max")}, ensure_ascii=True))
