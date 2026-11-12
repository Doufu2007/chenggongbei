"""Read-only v30 geometry diagnosis; no robot, ROS, network or production edit."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
WORLD = ROOT / "external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
EVIDENCE = ROOT / "runs/analysis/20261112_reverse_v30/evidence.json"
EVENTS = ROOT / "runs/analysis/20261112_reverse_v30/events_without_parts.json"
GATE = .641
OFFSET = .08
Z = .135


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose(el):
    return tuple(map(float, el.findtext("pose", default="0 0 0 0 0 0").split()))


def parse_boxes():
    result = []
    for model in ET.parse(WORLD).findall("./world/model"):
        if model.findtext("static") != "true":
            continue  # Robot body is not an external static scan obstacle.
        mp = pose(model)
        for link in model.findall("link"):
            lp = pose(link)
            for collision in link.findall("collision"):
                text = collision.findtext("geometry/box/size")
                if text is None:
                    raise AssertionError("unexpected non-box collision")
                cp = pose(collision)
                if any(abs(p[i]) > 1e-12 for p in (mp, lp, cp) for i in (3, 4, 5)):
                    raise AssertionError("diagnosis requires explicitly axis-aligned candidate boxes")
                xyz = tuple(sum(p[i] for p in (mp, lp, cp)) for i in range(3))
                size = tuple(map(float, text.split()))
                low = tuple(x - s / 2 for x, s in zip(xyz, size))
                high = tuple(x + s / 2 for x, s in zip(xyz, size))
                if low[2] <= Z <= high[2]:
                    result.append({"name": model.attrib["name"] + "/" + collision.attrib["name"],
                                   "rect": [low[0], low[1], high[0], high[1]], "z": [low[2], high[2]]})
    return result


BOXES = parse_boxes()


def point_distance(point, rect):
    return math.hypot(max(rect[0] - point[0], point[0] - rect[2], 0.),
                      max(rect[1] - point[1], point[1] - rect[3], 0.))


def laser_xy(position, yaw):
    return position[0] + OFFSET * math.cos(yaw), position[1] + OFFSET * math.sin(yaw)


def point_clearance(position, yaw):
    laser = laser_xy(position, yaw)
    distance, name = min((point_distance(laser, b["rect"]), b["name"]) for b in BOXES)
    return {"minimum_m": distance, "obstacle": name, "laser_xy_m": laser}


def sampled_arc(position, start, delta):
    # Independent dense sampler. Distance is 1-Lipschitz in laser position;
    # the true arc minimum is within OFFSET * angular_step / 2 of this result.
    count = max(1, math.ceil(abs(delta) / .0001))
    best = None
    for index in range(count + 1):
        yaw = start + delta * index / count
        value = point_clearance(position, yaw)
        if best is None or value["minimum_m"] < best["minimum_m"]:
            best = {**value, "yaw_rad": yaw}
    return {"anchor_xy_m": position, "start_yaw_rad": start, "delta_rad": delta,
            **best, "sample_count": count + 1,
            "minimum_error_bound_m": OFFSET * abs(delta) / count / 2,
            "passes_gate_with_sampling_error": best["minimum_m"] - OFFSET * abs(delta) / count / 2 > GATE}


def sampled_drive(start, end, yaw):
    count = max(1, math.ceil(math.dist(start, end) / .0001))
    best = None
    for index in range(count + 1):
        xy = tuple(a + (b - a) * index / count for a, b in zip(start, end))
        value = point_clearance(xy, yaw)
        if best is None or value["minimum_m"] < best["minimum_m"]:
            best = {**value, "base_xy_m": xy}
    return {"start_xy_m": start, "end_xy_m": end, "yaw_rad": yaw,
            **best, "sample_count": count + 1,
            "minimum_error_bound_m": math.dist(start, end) / count / 2,
            "passes_gate_with_sampling_error": best["minimum_m"] - math.dist(start, end) / count / 2 > GATE}


def stamp(v):
    s = v["header"]["stamp"]
    return s["sec"] + s["nanosec"] * 1e-9


def raycast(origin, angle):
    direction = math.cos(angle), math.sin(angle)
    best = (math.inf, None)
    for box in BOXES:
        rect = box["rect"]
        near, far = 0., math.inf
        for axis in (0, 1):
            if abs(direction[axis]) < 1e-15:
                if not rect[axis] <= origin[axis] <= rect[axis + 2]:
                    far = -1.
                    break
            else:
                a = (rect[axis] - origin[axis]) / direction[axis]
                b = (rect[axis + 2] - origin[axis]) / direction[axis]
                near, far = max(near, min(a, b)), min(far, max(a, b))
        if near <= far and far >= 0. and near < best[0]:
            best = near, box["name"]
    return best


data = json.loads(EVIDENCE.read_text(encoding="utf-8"))
events = json.loads(EVENTS.read_text(encoding="utf-8"))


def normalized_odoms(rows):
    result = []
    for row in rows:
        p = row["payload"]["pose"]["pose"]
        q = [p["orientation"][k] for k in ("x", "y", "z", "w")]
        norm = math.hypot(*q)
        x, y, z, w = [v / norm for v in q]
        result.append({"source_stamp_s": stamp(row["payload"]),
                       "wall_elapsed_s": row["wall_elapsed_s"],
                       "xyz_m": [p["position"][k] for k in ("x", "y", "z")],
                       "yaw_rad": math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))})
    return result


odoms = normalized_odoms(data["raw_odometry"])


def scan_diagnosis(payload, label, arrival=None):
    source = stamp(payload)
    previous = [o for o in odoms if o["source_stamp_s"] <= source]
    if not previous:
        raise AssertionError("no actual prior odometry")
    odom = previous[-1]
    skew = source - odom["source_stamp_s"]
    if skew > .05 + 1e-12:
        raise AssertionError("not a permitted actual-prior pairing")
    finite_ranges = [(v, i) for i, v in enumerate(payload["ranges"]) if math.isfinite(v)]
    value, beam = min(finite_ranges)
    body_angle = payload["angle_min"] + beam * payload["angle_increment"]
    yaw = odom["yaw_rad"]
    base_xy = odom["xyz_m"][:2]
    laser = laser_xy(base_xy, yaw)
    angle = yaw + body_angle
    expected, obstacle = raycast(laser, angle)
    end = laser[0] + value * math.cos(angle), laser[1] + value * math.sin(angle)
    errors = []
    for i, value_i in enumerate(payload["ranges"]):
        if math.isfinite(value_i):
            r, _ = raycast(laser, yaw + payload["angle_min"] + i * payload["angle_increment"])
            if math.isfinite(r):
                errors.append(abs(r - value_i))
    return {"label": label, "source_stamp_s": source, "scan_arrival_wall_s": arrival,
            "prior_odom_source_s": odom["source_stamp_s"], "actual_prior_skew_s": skew,
            "prior_odom_wall_s": odom["wall_elapsed_s"], "pose_xy_m": base_xy,
            "yaw_rad": yaw, "laser_xy_m": laser, "minimum_range_m": value,
            "minimum_beam_index": beam, "minimum_body_angle_rad": body_angle,
            "minimum_world_angle_rad": angle, "minimum_beam_endpoint_m": end,
            "candidate_minimum_beam_obstacle": obstacle, "candidate_expected_beam_range_m": expected,
            "minimum_beam_range_abs_error_m": abs(expected - value),
            "candidate_full_scan_ray_errors_m": {"count": len(errors), "mean": sum(errors) / len(errors),
                                                   "max": max(errors)},
            "candidate_continuous_ring": point_clearance(base_xy, yaw)}


all_scan_diagnoses = [scan_diagnosis(s["payload"], f"accepted_raw_{i}", s["wall_elapsed_s"])
                      for i, s in enumerate(data["raw_scans"])]
snapshot_last = None
for event in events:
    sample = event.get("payload", {}).get("data", {}).get("public_messages", {}).get("/scan", {}).get("last")
    if sample:
        snapshot_last = sample
if snapshot_last is None:
    raise AssertionError("public last scan unexpectedly absent")

preposition = {
    "initial_clockwise_quarter": sampled_arc((0., 0.), 0., -math.pi / 2),
    "south_drive_150mm": sampled_drive((0., 0.), (0., -.15), -math.pi / 2),
    "south_counterclockwise_quarter_1": sampled_arc((0., -.15), -math.pi / 2, math.pi / 2),
    "south_counterclockwise_quarter_2": sampled_arc((0., -.15), 0., math.pi / 2),
    "reverse_south_max_3_pulses_40mm_each": sampled_drive((0., -.15), (0., -.27), math.pi / 2),
}

# Import only the immutable candidate geometry, never a ROS runtime.
sys.path.insert(0, str(ROOT / "src/inspection_mission"))
from inspection_mission.ground_three_point_axis_plan import plan_ground_leg
axis = plan_ground_leg(None, "ground_2", (0., 0., 0.))
axis_ops = []
for op in (*axis.to_observation_operations, *axis.entry_operations):
    row = asdict(op)
    if op.kind == "TURN_ONLY":
        row["independent_arc"] = sampled_arc(op.anchor_xy_m, op.expected_start_yaw_rad, op.signed_subturn_rad)
    axis_ops.append(row)

historical = {}
for label, dirname in [("v28_negative_turn", "20261112_direction_v28"), ("v30_positive_turn", "20261112_reverse_v30")]:
    d = json.loads((ROOT / "runs/analysis" / dirname / "evidence.json").read_text(encoding="utf-8"))
    commands = d["commands"]
    moving = [c for c in commands if c.get("sent") and c["angular_z"] != 0.]
    first, last = moving[0], moving[-1]
    first_zero = next(c for c in commands if c["wall_elapsed_s"] > last["wall_elapsed_s"] and c.get("sent") and c["angular_z"] == 0.)
    sampled = [o for o in normalized_odoms(d["raw_odometry"]) if first["wall_elapsed_s"] <= o["wall_elapsed_s"] <= first_zero["wall_elapsed_s"]]
    yaw_change = sampled[-1]["yaw_rad"] - sampled[0]["yaw_rad"]
    dt = sampled[-1]["wall_elapsed_s"] - sampled[0]["wall_elapsed_s"]
    historical[label] = {"first_nonzero_wall_s": first["wall_elapsed_s"], "first_zero_wall_s": first_zero["wall_elapsed_s"],
                         "command_interval_s": first_zero["wall_elapsed_s"] - first["wall_elapsed_s"],
                         "actual_observed_yaw_change_rad": yaw_change, "actual_odometry_interval_s": dt,
                         "mean_observed_rate_rad_s": abs(yaw_change / dt),
                         "command_min_abs_rad_s": min(abs(c["angular_z"]) for c in moving),
                         "command_max_abs_rad_s": max(abs(c["angular_z"]) for c in moving),
                         "quarter_turn_constant_observed_rate_projection_s": math.pi / 2 / abs(yaw_change / dt),
                         "projection_is_not_a_verified_prediction": True}

result = {
    "purpose": "independent read-only v30 laser clearance and next-experiment geometry diagnosis",
    "platform_id": "e5d26785-01cd-41a6-acb8-02098e088649",
    "world_path": str(WORLD.relative_to(ROOT)), "world_sha256": sha(WORLD),
    "evidence_sha256": sha(EVIDENCE), "snapshots_sha256": sha(EVENTS),
    "source_status": "public candidate world; actual platform mapping not independently authenticated",
    "gate_m_unchanged": GATE, "assumed_forward_laser_offset_m": OFFSET, "candidate_scan_plane_z_m": Z,
    "parsed_collision_boxes": BOXES,
    "accepted_scan_count": len(all_scan_diagnoses), "accepted_scans": all_scan_diagnoses,
    "post_stop_latest_scan": scan_diagnosis(snapshot_last, "post_stop_public_snapshot_latest_not_first_rejected"),
    "first_rejected_scan_preserved": False,
    "origin_ccw_quarter": sampled_arc((0., 0.), 0., math.pi / 2),
    "east_200mm_ccw_quarter": sampled_arc((.2, 0.), 0., math.pi / 2),
    "origin_north_wall_analytic_gate_crossing_yaw_rad": math.asin((.65 - GATE) / OFFSET),
    "minimum_south_preposition_for_north_facing_gate_m_strictly_greater_than": GATE - (.65 - OFFSET),
    "preposition_candidate_150mm": preposition,
    "preposition_turn_total_rad": 3 * math.pi / 2,
    "candidate_path_notes": {"nominal_preposition_plus_requested_3x10mm_m": .18,
                             "preposition_plus_3x40mm_hard_motion_limits_m": .27,
                             "radius_250mm_incompatible_with_reaching_all_3x40mm_hard_limits": True,
                             "budget_policy": "abort at radius/path/rotation/time limit; limits must not be reset after preposition"},
    "axis_ground2_first_leg_operations": axis_ops,
    "axis_ground2_first_leg_minimum_reported_m": axis.minimum_laser_distance_m,
    "historical_turn_time": historical,
    "conclusion": "north wall plus forward laser offset explains v30; east shift cannot resolve; south preposition is geometry-feasible but adds unverified motion/time contract",
}
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"output": str(OUT / "metrics.json"), "origin_arc": result["origin_ccw_quarter"],
                  "last_accepted_scan": all_scan_diagnoses[-1], "post_stop_scan": result["post_stop_latest_scan"],
                  "preposition": preposition, "historical_turn_time": historical,
                  "axis_operations": [(o["kind"], o["anchor_xy_m"], o["target_xy_m"],
                                        o["expected_start_yaw_rad"], o["signed_subturn_rad"],
                                        o["minimum_laser_distance_m"]) for o in axis_ops]}, ensure_ascii=False, indent=2))
