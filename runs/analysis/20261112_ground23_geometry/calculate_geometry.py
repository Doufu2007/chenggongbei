"""Offline candidate-map design only. No ROS, production edits or score I/O."""
from pathlib import Path
import hashlib
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src" / "inspection_mission"))
from inspection_mission.ground_observation_route_plan import plan_ground_observation_route, _POINTS

spec = importlib.util.spec_from_file_location("candidate_geometry_reader", ROOT / "tools/validate_ground_scan_geometry.py")
geom = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = geom
spec.loader.exec_module(geom)
WORLD = ROOT / "external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"
RADIUS, RING, LASER_OFFSET = .447, .641, .08
geometry = geom.load_candidate_world(WORLD)
assert geometry.sensor_translation_m == (.08, 0., .08)
assert hashlib.sha256(WORLD.read_bytes()).hexdigest() == "595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1"
BOXES = []
for box in geometry.boxes:
    assert box.rotation == ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    if box.center_m[2]-box.half_size_m[2] <= .135 <= box.center_m[2]+box.half_size_m[2]:
        BOXES.append((box.name, (box.center_m[0]-box.half_size_m[0], box.center_m[1]-box.half_size_m[1],
                                 box.center_m[0]+box.half_size_m[0], box.center_m[1]+box.half_size_m[1])))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def point_rect(p, r):
    return math.hypot(max(r[0]-p[0], 0., p[0]-r[2]), max(r[1]-p[1], 0., p[1]-r[3]))


def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def segment_rect(a, b, r):
    lo, hi = 0., 1.
    for coordinate, delta, minimum, maximum in zip(a, (b[0]-a[0], b[1]-a[1]), r[:2], r[2:]):
        if abs(delta) < 1e-15:
            if not minimum <= coordinate <= maximum:
                break
        else:
            t0, t1 = sorted(((minimum-coordinate)/delta, (maximum-coordinate)/delta))
            lo, hi = max(lo, t0), min(hi, t1)
            if lo > hi:
                break
    else:
        return 0.
    corners = [(x, y) for x in (r[0], r[2]) for y in (r[1], r[3])]
    return min(point_rect(a, r), point_rect(b, r), *(point_segment(p, a, b) for p in corners))


def arc_rect(center, a, b, r):
    """Exact finite extrema/boundary candidates for point-to-box distance."""
    low, high = sorted((a, b))
    candidates = [a, b]
    principal = [k*math.pi/2 for k in range(-4, 5)]
    principal += [math.atan2(y-center[1], x-center[0]) for x in (r[0], r[2]) for y in (r[1], r[3])]
    for x in (r[0], r[2]):
        u = (x-center[0])/LASER_OFFSET
        if -1. <= u <= 1.:
            principal += [math.acos(u), -math.acos(u)]
    for y in (r[1], r[3]):
        u = (y-center[1])/LASER_OFFSET
        if -1. <= u <= 1.:
            t = math.asin(u)
            principal += [t, math.pi-t]
    for t in principal:
        for k in range(-3, 4):
            q = t+2*k*math.pi
            if low-1e-12 <= q <= high+1e-12:
                candidates.append(q)
    rows = [(point_rect((center[0]+LASER_OFFSET*math.cos(t), center[1]+LASER_OFFSET*math.sin(t)), r), t)
            for t in candidates]
    return min(rows)


def phase_clearance(row):
    records = []
    for name, rect in BOXES:
        if row["kind"] == "turn":
            value, heading = arc_rect(row["start"], row["start_yaw"], row["end_yaw_unwrapped"], rect)
            center_gap = point_rect(row["start"], rect)
        else:
            h = row["heading"]
            a = tuple(row["start"][i]+LASER_OFFSET*(math.cos(h), math.sin(h))[i] for i in (0, 1))
            b = tuple(row["end"][i]+LASER_OFFSET*(math.cos(h), math.sin(h))[i] for i in (0, 1))
            value, heading = segment_rect(a, b, rect), h
            center_gap = segment_rect(row["start"], row["end"], rect)
        records.append({"obstacle": name, "laser_distance_m": value, "ring_margin_m": value-RING,
                        "critical_yaw_rad": heading, "body_circle_clearance_lower_bound_m": center_gap-RADIUS})
    row["clearance"] = min(records, key=lambda item: item["laser_distance_m"])
    row["minimum_body_circle_clearance_lower_bound_m"] = min(item["body_circle_clearance_lower_bound_m"] for item in records)
    return row


def plan(first, entry, final_yaw=None):
    observation = plan_ground_observation_route(first, (0., 0., 0.))
    rows, position, heading = [], (0., 0.), 0.
    def turn(target, label):
        nonlocal heading
        end = heading+wrap(target-heading)
        rows.append(phase_clearance({"kind": "turn", "label": label, "start": position,
            "start_yaw": heading, "end_yaw_unwrapped": end, "angle_rad": abs(end-heading)}))
        heading = target
    def drive(target, label):
        nonlocal position
        target_heading = math.atan2(target[1]-position[1], target[0]-position[0])
        turn(target_heading, label+"_turn")
        rows.append(phase_clearance({"kind": "drive", "label": label, "start": position, "end": target,
                                    "heading": heading, "length_m": math.dist(position, target)}))
        position = target
    for index, target in enumerate(observation.waypoints_xy_m):
        drive(target, f"observation_leg_{index+1}")
    turn(observation.final_yaw_rad, "observation_visual_heading")
    for index, target in enumerate(entry):
        drive(target, f"entry_leg_{index+1}")
    if final_yaw is not None:
        turn(final_yaw, "optional_final_alignment")
    path = sum(row.get("length_m", 0.) for row in rows)
    rotation = sum(row.get("angle_rad", 0.) for row in rows)
    return {"first_id": first, "observation": observation.waypoints_xy_m[-1],
        "entry_waypoints": entry, "final_xy": position, "final_yaw_rad": heading,
        "total_path_m": path, "path_with_0_30m_reserve": path+.30,
        "planned_rotation_rad": rotation, "rotation_with_0_40rad_reserve": rotation+.40,
        "max_origin_displacement_m": max(math.hypot(*row["start"]) for row in rows+[{'start': position}]),
        "minimum_laser_distance_m": min(row["clearance"]["laser_distance_m"] for row in rows),
        "minimum_ring_margin_m": min(row["clearance"]["ring_margin_m"] for row in rows),
        "minimum_body_circle_clearance_lower_bound_m": min(row["minimum_body_circle_clearance_lower_bound_m"] for row in rows),
        "phases": rows}


robot = ET.parse(WORLD).getroot().find("world/model[@name='amphibious_drone']")
SHAPES = []
for link in robot.findall("link"):
    for tag in ("collision", "visual"):
        for item in link.findall(tag):
            tf = geom.compose(geom.element_transform(link), geom.element_transform(item))
            SHAPES.append((tag, link.get("name")+"/"+item.get("name"), item.find("geometry"), tf))


def footprint(plan, category):
    h, xy = plan["final_yaw_rad"], plan["final_xy"]
    body_tf = ((xy[0], xy[1], .055), geom.rpy_rotation(0., 0., h))
    rows = []
    for tag, name, shape, local_tf in SHAPES:
        if category == "base_collision_box" and name != "base_footprint/base_collision":
            continue
        if category == "all_ground_collisions" and tag != "collision":
            continue
        tf = geom.compose(body_tf, local_tf)
        if shape.find("box") is not None:
            half = [float(v)/2 for v in shape.findtext("box/size").split()]
            support = [sum(abs(tf[1][axis][i])*half[i] for i in range(3)) for axis in (0, 1)]
        elif shape.find("cylinder") is not None:
            radius = float(shape.findtext("cylinder/radius"))
            half = float(shape.findtext("cylinder/length"))/2
            support = [radius*math.sqrt(max(0., 1.-tf[1][axis][2]**2))+half*abs(tf[1][axis][2]) for axis in (0, 1)]
        else:
            raise ValueError("unknown candidate primitive")
        rows.append((tf[0][0]-support[0], tf[0][1]-support[1], tf[0][0]+support[0], tf[0][1]+support[1]))
    bounds = (min(r[0] for r in rows), min(r[1] for r in rows), max(r[2] for r in rows), max(r[3] for r in rows))
    center, size, _ = _POINTS[plan["first_id"]]
    board = (center[0]-size[0]/2, center[1]-size[1]/2, center[0]+size[0]/2, center[1]+size[1]/2)
    margins = (bounds[0]-board[0], bounds[1]-board[1], board[2]-bounds[2], board[3]-bounds[3])
    return {"bounds_xyxy_m": bounds, "board_bounds_xyxy_m": board,
        "edge_margins_west_south_east_north_m": margins, "fully_inside_nominal": min(margins) >= -1e-12}


def main():
    plans = {
        "ground_2_original_axis_to_center": plan("ground_2", ((0., -1.8), (0., -2.7))),
        "ground_2_requested_axis": plan("ground_2", ((.6, -2.62), (0., -2.62))),
        "ground_3_existing_axis": plan("ground_3", ((.6, -1.925), (1.115, -1.925))),
        "ground_2_optional_diagonal": plan("ground_2", ((.6, -2.62), (0., -2.68)), -math.pi),
        "ground_2_recommended_diagonal": plan("ground_2", ((.6, -2.62), (0., -2.69)), -math.pi),
        "ground_3_recommended_diagonal": plan("ground_3", ((1.115, -1.925),), -math.pi/2),
    }
    for row in plans.values():
        row["entry_footprints"] = {kind: footprint(row, kind) for kind in
            ("base_collision_box", "all_ground_collisions", "full_visible_envelope")}
        row["passes_nominal_fixed_ring"] = row["minimum_laser_distance_m"] >= RING
        if row is plans["ground_2_original_axis_to_center"]:
            assert abs(row["minimum_laser_distance_m"]-.57) < 1e-12
        else:
            assert row["minimum_laser_distance_m"] >= .65-1e-12
        assert row["path_with_0_30m_reserve"] <= 4.2+1e-12
        assert row["rotation_with_0_40rad_reserve"] < 6.
    assert abs(plans["ground_2_requested_axis"]["total_path_m"]-3.82) < 1e-12
    assert abs(plans["ground_3_existing_axis"]["path_with_0_30m_reserve"]-3.34) < 1e-12
    assert not plans["ground_2_requested_axis"]["entry_footprints"]["all_ground_collisions"]["fully_inside_nominal"]
    assert plans["ground_2_optional_diagonal"]["entry_footprints"]["all_ground_collisions"]["fully_inside_nominal"]
    for name in ("ground_2_recommended_diagonal", "ground_3_recommended_diagonal"):
        p = plans[name]
        worst = []
        # Preserve the existing 2cm Euclidean endpoint and .025rad final-yaw
        # tolerances; +/- .02 translations bound each coordinate separately.
        for error in (-.025, 0., .025):
            variant = dict(p, final_yaw_rad=p["final_yaw_rad"]+error)
            fp = footprint(variant, "all_ground_collisions")
            worst.append(min(fp["edge_margins_west_south_east_north_m"])-.02)
        p["wheel_entry_margin_after_2cm_translation_and_0_025rad_yaw_m"] = min(worst)
        assert min(worst) > .008
    report = {"kind": "offline_candidate_ground23_geometry_design", "not_implemented": True,
        "candidate_world_path": str(WORLD.relative_to(ROOT)), "candidate_world_sha256": hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "initial_pose_nominal": [0., 0., 0.], "laser_body_xyz_m": geometry.sensor_translation_m,
        "laser_plane_world_z_m": .135, "fixed_ring_threshold_m": RING,
        "fixed_body_circle_radius_m": RADIUS, "plans": plans,
        "axis_ground2_infeasibility": {"south_approach_requires_center_y_at_least": -3.35+RING+LASER_OFFSET,
            "west_facing_wheel_entry_requires_center_y_at_most": -2.5-(plans["ground_2_requested_axis"]["entry_footprints"]["all_ground_collisions"]["bounds_xyxy_m"][3]+2.62),
            "no_intersection": True},
        "limitations": ["Candidate public geometry, not current official scene certification.",
            "Continuous nominal lines/arcs computed against all static boxes intersecting the actual candidate laser plane.",
            "No runtime tracking-error or unknown-obstacle guarantee; original scan/stop/speed/freshness gates remain required.",
            "Full collision footprint includes the wheels; full visible envelope additionally includes rotors and camera.",
            "No ROS or referee messages; no code or budget changes have been implemented."]}
    (OUT/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    draw(plans)
    for name, p in plans.items():
        print(name, json.dumps({key: p[key] for key in ("total_path_m", "path_with_0_30m_reserve", "planned_rotation_rad", "minimum_laser_distance_m")}))


def draw(plans):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    names = ("ground_2_requested_axis", "ground_3_existing_axis",
             "ground_2_recommended_diagonal", "ground_3_recommended_diagonal")
    fig, axes = plt.subplots(2, 2, figsize=(10, 13), constrained_layout=True)
    for ax, name in zip(axes.flat, names):
        p = plans[name]
        for obstacle, (x0, y0, x1, y1) in BOXES:
            ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, facecolor="#9aa0a6", edgecolor="#555", alpha=.6))
        for key, (center, size, _) in _POINTS.items():
            ax.add_patch(Rectangle((center[0]-size[0]/2, center[1]-size[1]/2), *size,
                                   facecolor="#ece7dc", edgecolor="#888", lw=.7))
            ax.text(*center, key, ha="center", va="center", fontsize=7)
        for row in p["phases"]:
            if row["kind"] == "drive":
                ax.plot([row["start"][0], row["end"][0]], [row["start"][1], row["end"][1]], color="#176b7c", lw=2)
        final = p["final_xy"]
        ax.plot(*final, "o", color="#176b7c")
        for category, color in (("all_ground_collisions", "#d38a21"), ("full_visible_envelope", "#ad4351")):
            b = p["entry_footprints"][category]["bounds_xyxy_m"]
            ax.add_patch(Rectangle((b[0], b[1]), b[2]-b[0], b[3]-b[1], fill=False, ec=color, lw=1.4))
        if name.startswith("ground_2"):
            laser = (.6, -2.70)
        elif name == "ground_3_existing_axis":
            laser = (1.195, -1.925)
        else:
            laser = (p["final_xy"][0]+.08*math.cos(p["final_yaw_rad"]),
                     p["final_xy"][1]+.08*math.sin(p["final_yaw_rad"]))
        ax.add_patch(Circle(laser, RING, fill=False, ec="#555", ls=":", lw=1))
        ax.plot(*laser, "+", color="#111")
        ax.text(laser[0]-.55, laser[1]-.40, "laser ring r=0.641m", fontsize=7)
        ax.set(xlim=(-.85, 2.05), ylim=(-3.45, .75), aspect="equal", xlabel="candidate world x (m)", ylabel="candidate world y (m)")
        ax.set_title(name.replace("_", " ")+f"\npath {p['total_path_m']:.3f}m; cap +0.30m", fontsize=10)
        ax.grid(alpha=.15)
    fig.suptitle("Offline candidate geometry; nominal path only\nOrange: all ground collisions, red: full visible envelope", fontsize=11)
    fig.savefig(OUT/"ground23_geometry.png", dpi=170)
    fig.savefig(OUT/"ground23_geometry.pdf")
    plt.close(fig)


if __name__ == "__main__":
    main()
