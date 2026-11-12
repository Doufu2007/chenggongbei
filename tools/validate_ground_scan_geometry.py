"""Compare captured public scan endpoints with a fixed candidate SDF world.

This is an offline ray/box calculation, without ROS, control or referee imports.
It tests a named geometry hypothesis, not an official map or task contract.
No fitting, timestamp substitution, interpolation or sensor-data publication.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from dataclasses import dataclass
import xml.etree.ElementTree as ET


def finite(value, label):
    if type(value) not in (int, float):
        raise ValueError(label + " must be finite")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(label + " is too large") from exc
    if not math.isfinite(value):
        raise ValueError(label + " must be finite")
    return value


def mv(matrix, vector):
    return tuple(sum(row[i] * vector[i] for i in range(3)) for row in matrix)


def transpose(matrix):
    return tuple(zip(*matrix))


def mm(left, right):
    columns = transpose(right)
    return tuple(tuple(sum(a * b for a, b in zip(row, col)) for col in columns) for row in left)


def add(left, right):
    return tuple(a + b for a, b in zip(left, right))


def rpy_rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def pose_transform(element):
    if element.get("relative_to"):
        raise ValueError("relative_to SDF poses need explicit frame resolution")
    fields = [finite(float(v), "SDF pose") for v in (element.text or "0 0 0 0 0 0").split()]
    if len(fields) != 6:
        raise ValueError("SDF pose must contain xyz/rpy")
    return tuple(fields[:3]), rpy_rotation(*fields[3:])


def element_transform(parent):
    pose = parent.find("pose")
    return ((0., 0., 0.), rpy_rotation(0., 0., 0.)) if pose is None else pose_transform(pose)


def compose(left, right):
    return add(left[0], mv(left[1], right[0])), mm(left[1], right[1])


@dataclass(frozen=True)
class Box:
    name: str
    center_m: tuple
    half_size_m: tuple
    rotation: tuple


@dataclass(frozen=True)
class CandidateGeometry:
    boxes: tuple[Box, ...]
    sensor_translation_m: tuple
    sensor_rotation: tuple
    sensor_parent: str
    sensor_frame: str


def load_candidate_world(path, *, robot_model="amphibious_drone", sensor_name="rplidar"):
    """Read only static collision boxes and the selected public ray sensor pose."""
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("SDF world is missing")
    boxes = []
    sensor = None
    if world.findall("include"):
        raise ValueError("unresolved world includes cannot be silently omitted")
    for model in world.findall("model"):
        name = model.get("name", "")
        model_tf = element_transform(model)
        if model.findall("model") or model.findall("include"):
            raise ValueError("nested or included models require explicit geometry resolution")
        for link in model.findall("link"):
            link_tf = element_transform(link)
            if name == robot_model:
                for item in link.findall("sensor"):
                    if item.get("name") == sensor_name:
                        if sensor is not None or item.get("type") != "ray":
                            raise ValueError("selected ray sensor must be unique")
                        sensor_tf = compose(link_tf, element_transform(item))
                        frame = item.findtext("plugin/frame_name")
                        if not frame or not link.get("name"):
                            raise ValueError("sensor and parent frame names are required")
                        sensor = (*sensor_tf, link.get("name"), frame)
                continue  # Gazebo ray data excludes the sensor model's own geometry.
            collisions = link.findall("collision")
            if collisions and model.findtext("static") != "true":
                raise ValueError("dynamic environment collision cannot be treated as fixed")
            for collision in collisions:
                geometry = collision.find("geometry")
                size = geometry.findtext("box/size") if geometry is not None else None
                if size is None:
                    raise ValueError("unsupported non-box collision: " + name)
                values = tuple(finite(float(v), "box size") for v in size.split())
                if len(values) != 3 or min(values) <= 0:
                    raise ValueError("box size must contain three positive values")
                transform = compose(compose(model_tf, link_tf), element_transform(collision))
                boxes.append(Box(name + "/" + collision.get("name", ""), transform[0],
                                 tuple(v / 2 for v in values), transform[1]))
    if sensor is None or not boxes:
        raise ValueError("selected sensor or static collision boxes are missing")
    return CandidateGeometry(tuple(boxes), sensor[0], sensor[1], sensor[2], sensor[3])


def ray_box_distance(origin, direction, box):
    inverse = transpose(box.rotation)
    local_origin = mv(inverse, tuple(a-b for a, b in zip(origin, box.center_m)))
    local_direction = mv(inverse, direction)
    near, far = -math.inf, math.inf
    for coordinate, component, half in zip(local_origin, local_direction, box.half_size_m):
        if abs(component) < 1e-14:
            if not -half <= coordinate <= half:
                return None
            continue
        a, b = (-half-coordinate)/component, (half-coordinate)/component
        near, far = max(near, min(a, b)), min(far, max(a, b))
        if far < max(near, 0.):
            return None
    return near if near >= 0. else far


def quaternion_rotation(payload):
    x, y, z, w = (finite(payload[k], "pose quaternion") for k in ("x", "y", "z", "w"))
    if abs(math.hypot(x, y, z, w) - 1.) > 1e-8:
        raise ValueError("pose quaternion is not unit; no normalization is applied")
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def stamp(message):
    value = message["header"]["stamp"]
    sec, nano = value["sec"], value["nanosec"]
    if type(sec) is not int or sec < 0 or type(nano) is not int or not 0 <= nano < 1_000_000_000:
        raise ValueError("invalid raw sec/nanosec timestamp")
    return sec + nano / 1e9


def percentile(values, quantile):
    ordered = sorted(values)
    index = quantile * (len(ordered)-1)
    lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] + (ordered[hi]-ordered[lo])*(index-lo)


def compare_scan(scan, odom, geometry, *, tolerance_m, translation=(0., 0., 0.), yaw=0.):
    position = tuple(finite(odom["pose"]["pose"]["position"][key], "pose position") for key in ("x", "y", "z"))
    body_rotation = quaternion_rotation(odom["pose"]["pose"]["orientation"])
    hypothesis_rotation = rpy_rotation(0., 0., yaw)
    sensor_origin = add(translation, mv(hypothesis_rotation, add(position, mv(body_rotation, geometry.sensor_translation_m))))
    sensor_rotation = mm(mm(hypothesis_rotation, body_rotation), geometry.sensor_rotation)
    angle_min, increment = finite(scan["angle_min"], "angle_min"), finite(scan["angle_increment"], "angle_increment")
    range_min, range_max = finite(scan["range_min"], "range_min"), finite(scan["range_max"], "range_max")
    if increment == 0 or not 0 <= range_min < range_max or not isinstance(scan["ranges"], list) or not scan["ranges"]:
        raise ValueError("invalid scan layout or bounds")
    beams, residuals, hits, missing = [], [], Counter(), 0
    for index, observed in enumerate(scan["ranges"]):
        angle = angle_min + increment * index
        direction = mv(sensor_rotation, (math.cos(angle), math.sin(angle), 0.))
        predicted, hit = None, None
        for box in geometry.boxes:
            distance = ray_box_distance(sensor_origin, direction, box)
            if distance is not None and 0 <= distance <= range_max and (predicted is None or distance < predicted):
                predicted, hit = distance, box.name
        valid = type(observed) in (int, float) and math.isfinite(observed) and range_min <= observed <= range_max
        residual = predicted - observed if valid and predicted is not None and predicted >= range_min else None
        if residual is None:
            missing += 1
        else:
            residuals.append(residual)
            hits[hit] += 1
        beams.append({"index": index, "angle_rad": angle, "observed_m": observed, "predicted_m": predicted,
                      "residual_m": residual, "hit": hit})
    absolute = [abs(v) for v in residuals]
    metrics = {"beam_count": len(beams), "finite_observed_beams": sum(type(v) in (int, float) and math.isfinite(v) and range_min <= v <= range_max for v in scan["ranges"]),
               "missing_observed_beams": missing, "compared_beams": len(residuals),
               "mae_m": sum(absolute)/len(absolute) if absolute else None,
               "rmse_m": math.sqrt(sum(v*v for v in residuals)/len(residuals)) if residuals else None,
               "p95_absolute_error_m": percentile(absolute, .95) if absolute else None,
               "maximum_absolute_error_m": max(absolute) if absolute else None,
               "inlier_count": sum(v <= tolerance_m for v in absolute), "tolerance_m": tolerance_m,
               "hit_counts": dict(hits)}
    return {"sensor_origin_world_m": sensor_origin, "metrics": metrics, "beams": beams,
            "candidate_consistency": bool(absolute) and missing == 0 and max(absolute) <= tolerance_m}


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def no_constant(value):
    raise ValueError("nonfinite JSON value: " + value)


def finite_json_float(value):
    return finite(float(value), "JSON float")


def file_identity(path):
    raw = Path(path).read_bytes()
    return {"path": str(Path(path)), "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}


def validate_capture(evidence_path, world_path, *, tolerance_m=.001, max_pose_scan_skew_s=.15,
                     expected_odom_frame="odom", expected_mode="bounded_ground_motion"):
    tolerance_m = finite(tolerance_m, "tolerance_m")
    max_pose_scan_skew_s = finite(max_pose_scan_skew_s, "max_pose_scan_skew_s")
    if tolerance_m <= 0 or max_pose_scan_skew_s < 0:
        raise ValueError("positive tolerance and nonnegative time-pairing bound required")
    if expected_mode not in ("bounded_ground_motion", "ground_short_drive"):
        raise ValueError("expected_mode must name an explicitly supported ground diagnostic")
    full = []
    modes_seen = set()
    for line_number, line in enumerate(Path(evidence_path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line, object_pairs_hook=unique_object, parse_constant=no_constant, parse_float=finite_json_float)
            payload = row["payload"]
            event_data = payload.get("data", {})
            if "mode" in event_data:
                if event_data["mode"] != expected_mode:
                    raise ValueError("callback mode does not match expected_mode; mixed modes are rejected")
                modes_seen.add(event_data["mode"])
            if payload.get("event") == "probe_snapshot" and payload["data"].get("section") == "ground_motion_full":
                full.append((line_number, payload["data"]))
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            raise ValueError(f"invalid callback JSONL line {line_number}: {exc}") from exc
    if len(full) != 1:
        raise ValueError("exactly one ground_motion_full snapshot is required; runs are never merged")
    line_number, data = full[0]
    if data.get("mode") != expected_mode or data.get("read_only") is not False or data.get("partial") is not False:
        raise ValueError("expected one complete active ground diagnostic snapshot")
    geometry = load_candidate_world(world_path)
    report = {"schema_version": 1, "kind": "fixed_candidate_scan_geometry_validation",
              "mode": data["mode"], "source_modes_seen": sorted(modes_seen),
              "source_evidence": file_identity(evidence_path), "candidate_world": file_identity(world_path),
              "snapshot_line": line_number, "official_scene_version": "unverified", "point_id_mapping": "hypothesis_only",
              "task_completion": "unverified", "score": None, "scans": [],
              "assumptions": {"world_from_odom": "identity_hypothesis", "odom_frame": expected_odom_frame,
                  "sensor_pose_source": "selected public candidate world rplidar pose; not an independently calibrated TF",
                  "sensor_parent": geometry.sensor_parent, "sensor_frame": geometry.sensor_frame,
                  "sensor_translation_m": geometry.sensor_translation_m, "sensor_rotation": geometry.sensor_rotation,
                  "fit_parameters": [], "max_pose_scan_skew_s": max_pose_scan_skew_s},
              "limits": ["Only environment surfaces hit by recorded rays are tested; images, point IDs, unseen geometry and official versions remain separate.",
                  "First/last pose and scan are capture endpoints, not synchronized pose-at-scan samples. Original stamps and their skew are retained.",
                  "Static scans constrain the composite world-to-sensor transform, not map-to-odom and lidar extrinsics independently.",
                  "Recorded values and tiny mathematical residuals do not establish a physical localization error bound or collision-free path."]}
    alternatives = [("yaw_plus_90deg", (0., 0., 0.), math.pi/2), ("yaw_180deg", (0., 0., 0.), math.pi),
                    ("yaw_minus_90deg", (0., 0., 0.), -math.pi/2), ("x_plus_0.1m", (.1, 0., 0.), 0.),
                    ("y_plus_0.1m", (0., .1, 0.), 0.), ("yaw_plus_0.05rad", (0., 0., 0.), .05)]
    for phase in ("first", "last"):
        try:
            pose, scan = data["odometry"]["/odom"][phase], data["public_messages"]["/scan"][phase]
            if pose["header"]["frame_id"] != expected_odom_frame or pose["child_frame_id"] != geometry.sensor_parent:
                raise ValueError("odometry world/child frame does not match the explicit candidate hypothesis")
            if scan["header"]["frame_id"] != geometry.sensor_frame:
                raise ValueError("scan frame does not match candidate sensor")
            pose_stamp, scan_stamp = stamp(pose), stamp(scan)
            if abs(scan_stamp - pose_stamp) > max_pose_scan_skew_s:
                raise ValueError("pose/scan endpoint timestamp skew exceeds explicit bound")
            item = compare_scan(scan, pose, geometry, tolerance_m=tolerance_m)
            item.update(phase=phase, pose_message=pose, scan_header=scan["header"], pose_stamp_s=pose_stamp,
                        scan_stamp_s=scan_stamp, scan_minus_pose_stamp_s=scan_stamp-pose_stamp,
                        time_pairing="capture_endpoint_not_synchronized", alternative_hypotheses=[])
            for name, translation, yaw in alternatives:
                comparison = compare_scan(scan, pose, geometry, tolerance_m=tolerance_m, translation=translation, yaw=yaw)
                item["alternative_hypotheses"].append({"name": name, "translation_world_m": translation,
                                                       "yaw_rad": yaw, "metrics": comparison["metrics"]})
            report["scans"].append(item)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"invalid {phase} pose/scan: {exc}") from exc
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--tolerance-m", type=float, default=.001)
    parser.add_argument("--max-pose-scan-skew-s", type=float, default=.15)
    parser.add_argument("--expected-mode", choices=("bounded_ground_motion", "ground_short_drive"), default="bounded_ground_motion")
    args = parser.parse_args(argv)
    try:
        report = validate_capture(args.evidence, args.world, tolerance_m=args.tolerance_m,
                                  max_pose_scan_skew_s=args.max_pose_scan_skew_s, expected_mode=args.expected_mode)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        print(json.dumps({"output": str(args.output), "scans": [{"phase": s["phase"], "candidate_consistency": s["candidate_consistency"],
              "metrics": s["metrics"]} for s in report["scans"]], "official_scene_version": "unverified"}, ensure_ascii=True))
        return 0
    except (OSError, ValueError, ET.ParseError) as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
