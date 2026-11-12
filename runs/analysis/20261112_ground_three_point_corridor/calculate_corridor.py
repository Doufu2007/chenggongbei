"""Enumerate all ordered distinct ground triples on a candidate corridor graph.

Pure offline geometry. Does not import ROS, perception or control state machines.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
BASE = ROOT / "runs/analysis/20261112_ground23_geometry/calculate_geometry.py"
spec = importlib.util.spec_from_file_location("prior_candidate_geometry", BASE)
geo = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = geo
spec.loader.exec_module(geo)
from inspection_mission.ground_observation_route_plan import _POINTS, _candidate_projection

RING = .641
SPINE_X = .6
OBS = {key: value[2] for key, value in _POINTS.items()}
OBS["ground_4"] = (.35, -.10)
ENDS = {"ground_0": ((1.115, -1.35), 0.), "ground_1": ((0., -1.35), math.pi),
        "ground_2": ((0., -2.69), math.pi), "ground_3": ((1.115, -1.925), -math.pi/2),
        "ground_4": ((1.115, -.775), -math.pi/2)}
PHASE_CACHE = {}


def clearance(row):
    key = json.dumps(row, sort_keys=True)
    if key not in PHASE_CACHE:
        PHASE_CACHE[key] = geo.phase_clearance(dict(row))
    return dict(PHASE_CACHE[key])


def safe_turn(position, heading, target, label):
    """Evaluate both topological arcs. Never choose +/-pi by wrap alone."""
    principal = geo.wrap(target-heading)
    candidates = [principal]
    if abs(principal) > 1e-12:
        candidates.append(principal-math.copysign(2*math.pi, principal))
    options = []
    for delta in candidates:
        row = clearance({"kind": "turn", "label": label, "start": tuple(position),
            "start_yaw": heading, "end_yaw_unwrapped": heading+delta, "angle_rad": abs(delta)})
        row["passes_fixed_ring"] = row["clearance"]["laser_distance_m"] >= RING-1e-12
        options.append(row)
    passing = [row for row in options if row["passes_fixed_ring"]]
    if not passing:
        raise ValueError(("no safe candidate turn", position, heading, target, options))
    chosen = min(passing, key=lambda row: row["angle_rad"])
    chosen["arc_policy"] = "minimum_absolute_angle_among_both_ring_valid_arcs"
    chosen["rejected_shorter_arcs"] = [{"angle_rad": row["angle_rad"],
        "end_yaw_unwrapped": row["end_yaw_unwrapped"], "minimum_laser_distance_m": row["clearance"]["laser_distance_m"]}
        for row in options if row["angle_rad"] <= chosen["angle_rad"]+1e-12 and not row["passes_fixed_ring"]]
    return chosen


class SegmentBuilder:
    def __init__(self, position, heading):
        self.position, self.heading = tuple(position), heading
        self.rows = []

    def turn(self, target, label):
        row = safe_turn(self.position, self.heading, target, label)
        # Zero-angle phases are retained to make each transfer's interface clear.
        self.rows.append(row)
        self.heading = target

    def drive(self, target, label):
        target = tuple(target)
        distance = math.dist(self.position, target)
        if distance < 1e-12:
            return
        heading = math.atan2(target[1]-self.position[1], target[0]-self.position[0])
        self.turn(heading, label+"_aim")
        row = clearance({"kind": "drive", "label": label, "start": self.position,
            "end": target, "heading": self.heading, "length_m": distance})
        assert row["clearance"]["laser_distance_m"] >= RING-1e-12
        self.rows.append(row)
        self.position = target


def entry_points(target, scheme):
    if target == "ground_2":
        return ((.6, -2.62), ENDS[target][0])
    if target == "ground_0" and scheme == "preserve_ground0_axis":
        return ((.6, -1.35), ENDS[target][0])
    return (ENDS[target][0],)


def compile_operations(rows):
    operations = []
    continuous = None
    for row in rows:
        if row["kind"] == "turn":
            if continuous is None:
                continuous = row["start_yaw"]
            assert abs(geo.wrap(continuous-row["start_yaw"])) < 1e-12
            delta = row["end_yaw_unwrapped"]-row["start_yaw"]
            if abs(delta) < 1e-12:
                continue
            count = math.ceil(abs(delta)/(math.pi/2)-1e-12)
            for i in range(1, count+1):
                step = math.copysign(min(math.pi/2, abs(delta)-(i-1)*math.pi/2), delta)
                target = continuous+step
                operations.append({"op": "TURN_ONLY", "anchor_xy_m": row["start"],
                    "expected_start_yaw_rad": geo.wrap(continuous), "signed_subturn_rad": step,
                    "target_yaw_rad": geo.wrap(target), "target_yaw_unwrapped_rad": target,
                    "unwrapped_coordinate_scope": "this_operation_list_start_pose",
                    "direction": "CCW" if delta > 0 else "CW",
                    "maximum_subturn_rad": math.pi/2, "settle_required_after": True,
                    "source_phase": row["label"], "zero_translation": True})
                continuous = target
        else:
            if continuous is None:
                continuous = row["heading"]
            assert abs(geo.wrap(continuous-row["heading"])) < 1e-12
            operations.append({"op": "DRIVE_TO_XY", "target_xy_m": row["end"],
                "required_heading_rad": row["heading"], "settle_required_after": True,
                "source_phase": row["label"]})
    return operations


def leg(previous, target, scheme, actual_start_pose=None):
    if actual_start_pose is None:
        position, heading = ((0., 0.), 0.) if previous is None else ENDS[previous]
    else:
        if (len(actual_start_pose) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in actual_start_pose)):
            raise ValueError("actual pose must be three finite numeric coordinates")
        position, heading = tuple(actual_start_pose[:2]), actual_start_pose[2]
    b = SegmentBuilder(position, heading)
    obs = OBS[target]
    if previous is None:
        if target == "ground_4":
            b.drive(obs, "first_direct_observation")
        else:
            b.drive((.6, 0.), "first_east_gateway")
            b.drive(obs, "first_spine_to_observation")
    else:
        b.drive((SPINE_X, position[1]), "exit_to_spine")
        b.drive((SPINE_X, obs[1]), "spine_to_next_observation_row")
        b.drive(obs, "observation_branch")
    board = _POINTS[target][0]
    obs_yaw = math.atan2(board[1]-obs[1], board[0]-obs[0])
    b.turn(obs_yaw, "observation_visual_heading")
    exposure_boundary = len(b.rows)
    for i, point in enumerate(entry_points(target, scheme)):
        b.drive(point, f"entry_leg_{i+1}")
    b.turn(ENDS[target][1], "final_entry_alignment")
    assert b.position == ENDS[target][0]
    path = sum(row.get("length_m", 0.) for row in b.rows)
    rotation = sum(row.get("angle_rad", 0.) for row in b.rows)
    return {"previous": previous, "target": target, "scheme": scheme, "start_xy": position,
        "start_yaw": heading, "observation_xy": obs, "observation_yaw": obs_yaw,
        "end_xy": b.position, "end_yaw": b.heading, "path_m": path, "rotation_rad": rotation,
        "to_observation_path_m": sum(row.get("length_m", 0.) for row in b.rows[:exposure_boundary]),
        "to_observation_rotation_rad": sum(row.get("angle_rad", 0.) for row in b.rows[:exposure_boundary]),
        "minimum_laser_distance_m": min(row["clearance"]["laser_distance_m"] for row in b.rows),
        "minimum_ring_margin_m": min(row["clearance"]["ring_margin_m"] for row in b.rows),
        "longest_drive_m": max(row.get("length_m", 0.) for row in b.rows),
        "largest_single_turn_rad": max(row.get("angle_rad", 0.) for row in b.rows),
        "drive_leg_count": sum(row["kind"] == "drive" for row in b.rows),
        "to_observation_operations": compile_operations(b.rows[:exposure_boundary]),
        "entry_operations_after_current_visual_report": compile_operations(b.rows[exposure_boundary:]),
        "phase_rows": b.rows}


def plan_transfer_from_pose(previous_id, next_id, actual_pose, *, scheme="preserve_ground0_axis"):
    """Reference design API: data only, explicitly fails unsafe candidate arcs.

    It does not own mission counters or current sensor/freshness authority.
    A downstream executor must hold the same permanent mission ledger.
    """
    if previous_id not in ENDS or next_id not in ENDS or previous_id == next_id:
        raise ValueError("two distinct valid ground IDs required")
    return leg(previous_id, next_id, scheme, actual_start_pose=actual_pose)


def enumerate_scheme(scheme):
    ids = tuple(_POINTS)
    first = {key: leg(None, key, scheme) for key in ids}
    transfers = {f"{a}->{b}": leg(a, b, scheme) for a in ids for b in ids if a != b}
    sequences = []
    for order in itertools.permutations(ids, 3):
        legs = [first[order[0]], transfers[f"{order[0]}->{order[1]}"], transfers[f"{order[1]}->{order[2]}"]]
        rows = [row for item in legs for row in item["phase_rows"]]
        sequences.append({"order": order, "path_m": sum(item["path_m"] for item in legs),
            "path_with_0_30m_per_point_reserve": sum(item["path_m"] for item in legs)+.9,
            "rotation_rad": sum(item["rotation_rad"] for item in legs),
            "rotation_with_0_40rad_per_point_reserve": sum(item["rotation_rad"] for item in legs)+1.2,
            "minimum_laser_distance_m": min(row["clearance"]["laser_distance_m"] for row in rows),
            "longest_drive_m": max(row.get("length_m", 0.) for row in rows),
            "largest_single_turn_rad": max(row.get("angle_rad", 0.) for row in rows),
            "drive_leg_count": sum(row["kind"] == "drive" for row in rows),
            "max_origin_displacement_m": max(math.hypot(*point) for row in rows for point in
                ((row["start"], row["end"]) if row["kind"] == "drive" else (row["start"],)))})
    assert len(sequences) == 60
    all_legs = [*first.values(), *transfers.values()]
    summary = {"sequence_count": 60,
        "worst_path": max(sequences, key=lambda item: item["path_m"]),
        "worst_rotation": max(sequences, key=lambda item: item["rotation_rad"]),
        "longest_complete_transfer": max(transfers.values(), key=lambda item: item["path_m"])["path_m"],
        "longest_complete_transfer_pair": max(transfers, key=lambda key: transfers[key]["path_m"]),
        "longest_transfer_to_observation": max(item["to_observation_path_m"] for item in transfers.values()),
        "largest_single_turn_rad": max(item["largest_single_turn_rad"] for item in all_legs),
        "longest_drive_m": max(item["longest_drive_m"] for item in all_legs),
        "minimum_laser_distance_m": min(item["minimum_laser_distance_m"] for item in all_legs),
        "max_origin_displacement_m": max(item["max_origin_displacement_m"] for item in sequences),
        "max_drive_leg_count_per_sequence": max(item["drive_leg_count"] for item in sequences)}
    summary["max_turn_only_suboperations_per_sequence"] = max(sum(len([op for op in item[section] if op["op"] == "TURN_ONLY"])
        for item in (first[order[0]], transfers[f"{order[0]}->{order[1]}"], transfers[f"{order[1]}->{order[2]}"])
        for section in ("to_observation_operations", "entry_operations_after_current_visual_report"))
        for order in itertools.permutations(ids, 3))
    return {"summary": summary, "first_legs": first, "transfers": transfers, "sequences": sequences}


def verify_numerically():
    import numpy as np
    greatest = 0.
    for row in PHASE_CACHE.values():
        t = np.linspace(0., 1., 10001)
        if row["kind"] == "turn":
            h = row["start_yaw"]+t*(row["end_yaw_unwrapped"]-row["start_yaw"])
            x, y = row["start"][0]+.08*np.cos(h), row["start"][1]+.08*np.sin(h)
        else:
            h = row["heading"]
            x = row["start"][0]+t*(row["end"][0]-row["start"][0])+.08*math.cos(h)
            y = row["start"][1]+t*(row["end"][1]-row["start"][1])+.08*math.sin(h)
        dense = float("inf")
        for _, r in geo.BOXES:
            dx = np.maximum.reduce((r[0]-x, np.zeros_like(x), x-r[2]))
            dy = np.maximum.reduce((r[1]-y, np.zeros_like(y), y-r[3]))
            dense = min(dense, float(np.hypot(dx, dy).min()))
        exact = row["clearance"]["laser_distance_m"]
        assert exact <= dense+1e-12 and dense-exact < 1e-7
        greatest = max(greatest, dense-exact)
    return {"distinct_phase_checks": len(PHASE_CACHE), "samples_per_phase": 10001,
            "maximum_dense_minus_analytic_m": greatest, "all_agree_within_1e_7m": True}


def endpoint_envelopes():
    output = {}
    for key, (position, heading) in ENDS.items():
        p = {"first_id": key, "final_xy": position, "final_yaw_rad": heading}
        nominal = geo.footprint(p, "all_ground_collisions")
        error_margin = min(min(geo.footprint(dict(p, final_yaw_rad=heading+error), "all_ground_collisions")
            ["edge_margins_west_south_east_north_m"])-.02 for error in (-.025, 0., .025))
        output[key] = {"xy": position, "yaw": heading, "wheel_nominal": nominal,
            "wheel_margin_after_2cm_and_0_025rad_errors_m": error_margin,
            "full_visible": geo.footprint(p, "full_visible_envelope")}
        assert nominal["fully_inside_nominal"] and error_margin > .008
    return output


def main():
    schemes = {key: enumerate_scheme(key) for key in ("preserve_ground0_axis", "all_direct_except_ground2")}
    observations = {}
    for key, xy in OBS.items():
        center, size, _ = _POINTS[key]
        yaw = math.atan2(center[1]-xy[1], center[0]-xy[0])
        prediction = _candidate_projection(center, size, xy, yaw)
        assert prediction.all_corners_in_frame
        observations[key] = {"xy": xy, "yaw_rad": yaw, "candidate_projection": asdict(prediction),
            "same_as_current_single_point_observation": xy == _POINTS[key][2]}
    # Explicit hard example: normalizing -pi then naively choosing +pi goes
    # south at ground_2 and violates the existing ring. The other arc is safe.
    example = safe_turn(ENDS["ground_2"][0], -math.pi, 0., "ground2_departure_sign_example")
    assert example["rejected_shorter_arcs"] and example["end_yaw_unwrapped"] < -math.pi
    assert abs(example["rejected_shorter_arcs"][0]["minimum_laser_distance_m"]-.58) < 1e-12
    baseline = schemes["preserve_ground0_axis"]["summary"]
    assert baseline["minimum_laser_distance_m"] >= .65-1e-12
    assert baseline["longest_drive_m"] == 2.59
    actual_pose_example = plan_transfer_from_pose("ground_2", "ground_4", (0., -2.69, -math.pi))
    assert actual_pose_example["to_observation_operations"][0]["target_yaw_unwrapped_rad"] == -3*math.pi/2
    actual_pose_rejection = None
    try:
        plan_transfer_from_pose("ground_2", "ground_4", (0., -2.71, -math.pi))
    except ValueError:
        actual_pose_rejection = "2cm south endpoint error leaves laser distance .64m < .641m; no motion plan emitted"
    assert actual_pose_rejection is not None
    report = {"kind": "candidate_three_ground_point_corridor_design", "candidate_only": True,
        "production_modified": False, "ROS_executed": False, "uploaded": False,
        "world_sha256": hashlib.sha256(geo.WORLD.read_bytes()).hexdigest(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "previous_geometry_source_sha256": hashlib.sha256(BASE.read_bytes()).hexdigest(),
        "spine_x_m": SPINE_X, "ring_threshold_m": RING, "body_radius_m": .447,
        "actual_candidate_laser_offset_body_xy_m": [.08, 0.],
        "observations": observations, "endpoints": endpoint_envelopes(),
        "ground2_departure_arc_counterexample": example,
        "actual_pose_reference_example": actual_pose_example,
        "allowed_endpoint_error_rejection_example": actual_pose_rejection,
        "schemes": schemes, "numerical_crosscheck": verify_numerically()}
    (OUT/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    draw_map(report)
    print(json.dumps({key: value["summary"] for key, value in schemes.items()}, indent=2))


def draw_map(report):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, axes = plt.subplots(1, 2, figsize=(12, 8), constrained_layout=True)
    baseline = report["schemes"]["preserve_ground0_axis"]
    order = baseline["summary"]["worst_path"]["order"]
    for ax in axes:
        for _, (x0, y0, x1, y1) in geo.BOXES:
            ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, fc="#bbb", ec="#777", alpha=.65))
        for key, (center, size, _) in _POINTS.items():
            ax.add_patch(Rectangle((center[0]-size[0]/2, center[1]-size[1]/2), *size, fc="#eee7db", ec="#999"))
            ax.text(center[0], center[1]+size[1]/2+.035, key, ha="center", fontsize=8)
        ax.set(xlim=(-.85, 2.07), ylim=(-3.45, .75), aspect="equal", xlabel="candidate world x (m)", ylabel="candidate world y (m)")
        ax.grid(alpha=.15)
    axes[0].plot([.6, .6], [-2.69, 0.], color="#176b7c", lw=2, label="shared spine")
    for key, (end, yaw) in ENDS.items():
        axes[0].plot([end[0], .6], [end[1], end[1]], color="#888", lw=1, ls="--")
        axes[0].arrow(end[0], end[1], .11*math.cos(yaw), .11*math.sin(yaw), width=.008, color="#b96b28")
        axes[0].plot(*OBS[key], "o", color="#176b7c")
        for row in baseline["first_legs"][key]["phase_rows"]:
            if row["kind"] == "drive" and row["label"].startswith("entry"):
                axes[0].plot([row["start"][0], row["end"][0]], [row["start"][1], row["end"][1]], color="#b96b28", lw=1.2)
    axes[0].set_title("Reusable spine, observations and entry branches\nOrange arrows: declared endpoint headings", fontsize=11)
    legs = [baseline["first_legs"][order[0]], baseline["transfers"][f"{order[0]}->{order[1]}"], baseline["transfers"][f"{order[1]}->{order[2]}"]]
    colors = ("#176b7c", "#b96b28", "#805a99")
    for leg_data, color in zip(legs, colors):
        first = True
        for row in leg_data["phase_rows"]:
            if row["kind"] == "drive":
                axes[1].plot([row["start"][0], row["end"][0]], [row["start"][1], row["end"][1]], color=color, lw=2,
                             label=leg_data["target"] if first else None)
                first = False
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].set_title("Worst total path: "+" > ".join(order)+f"\n{baseline['summary']['worst_path']['path_m']:.6f}m before reserve", fontsize=10)
    fig.suptitle("Offline corridor for all 60 ordered ground triples\nNominal candidate geometry; no ROS or referee execution", fontsize=12)
    fig.savefig(OUT/"corridor_graph.png", dpi=170)
    fig.savefig(OUT/"corridor_graph.pdf")
    plt.close(fig)


if __name__ == "__main__":
    main()
