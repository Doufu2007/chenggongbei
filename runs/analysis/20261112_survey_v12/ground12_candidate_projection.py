"""Offline v12 team-image ROIs; candidate static geometry remains unverified."""
from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
WORLD = ROOT / "external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rotation(q):
    x, y, z, w = q
    assert abs(math.sqrt(sum(v*v for v in q))-1) < 1e-8
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    summary_path = OUT / "summary.json"
    d = json.loads(summary_path.read_text(encoding="utf-8"))
    image_record = next(i for i in d["native_images"]
                        if i["topic"] == "/uav/camera/image" and i["phase"] == "last_associated")
    assert image_record["exposure_validation"] == "valid_prior_pose_observation"
    original = OUT / image_record["file_name"]
    assert sha(original) == image_record["sha256"]
    exposure = image_record["exposure"]
    assert exposure["association_error"] is None
    pose = exposure["odom"]
    r = rotation(pose["quaternion"])
    infos = d["snapshots"]["assembled"]["public_messages"]["/uav/camera/camera_info"]
    first, last = infos["first"], infos["last"]
    assert first["k"] == last["k"] and first["d"] == last["d"] == [0.0]*5
    k = last["k"]
    world = ET.parse(WORLD).getroot().find("world")
    sensor = world.find("model[@name='amphibious_drone']/link[@name='base_footprint']/sensor[@name='uav_camera']")
    camera = [float(v) for v in sensor.findtext("pose").split()]
    assert camera[3:] == [0.0]*3
    image = Image.open(original).convert("RGB")
    frame = np.float32([[0, 0], [image.width-1, 0], [image.width-1, image.height-1], [0, image.height-1]])
    points = {}
    for n in range(5):
        model = world.find(f"model[@name='ground_task_point_{n}']")
        x, y, z, roll, pitch, yaw = [float(v) for v in model.findtext("pose").split()]
        assert [roll, pitch, yaw] == [0.0]*3
        w, h, height = [float(v) for v in model.findtext("link/visual/geometry/box/size").split()]
        corners = [[x-w/2, y-h/2, z+height/2], [x+w/2, y-h/2, z+height/2],
                   [x+w/2, y+h/2, z+height/2], [x-w/2, y+h/2, z+height/2]]
        pixels = []
        for corner in corners:
            local = r.T @ (np.array(corner)-pose["position"]) - np.array(camera[:3])
            assert local[0] > 0
            pixels.append([float(k[2]-k[0]*local[1]/local[0]), float(k[5]-k[4]*local[2]/local[0])])
        poly = np.float32(pixels)
        full_area = cv2.contourArea(poly)
        visible_area, visible_polygon = cv2.intersectConvexConvex(poly, frame)
        points[f"ground_{n}"] = {
            "world_corners_m": corners, "pixel_corners_xy": pixels,
            "all_corners_in_frame": all(0 <= p[0] < image.width and 0 <= p[1] < image.height for p in pixels),
            "source_polygon_area_px2": full_area,
            "visible_polygon_area_px2": float(visible_area),
            "visible_area_fraction": float(visible_area/full_area),
            "bbox_extent_px": [max(p[j] for p in pixels)-min(p[j] for p in pixels) for j in (0, 1)],
            "edge_lengths_px": [math.dist(pixels[i], pixels[(i+1)%4]) for i in range(4)],
            "visible_pixel_polygon_xy": [] if visible_polygon is None else visible_polygon.reshape(-1, 2).tolist()}

    annotated = image.copy()
    draw = ImageDraw.Draw(annotated)
    for name, color in [("ground_1", "orange"), ("ground_2", "lime")]:
        polygon = points[name]["pixel_corners_xy"]
        draw.line([tuple(p) for p in polygon] + [tuple(polygon[0])], fill=color, width=2)
        for i, p in enumerate(polygon):
            if 0 <= p[0] < image.width and 0 <= p[1] < image.height:
                draw.text((p[0]+3, p[1]+3), f"{name} C{i}", fill=color, stroke_fill="black", stroke_width=1)
    draw.rectangle((0, 0, image.width, 45), fill="black")
    draw.text((8, 8), "Candidate geometry: ground_2 full but very thin / ground_1 truncated", fill="white")
    draw.text((8, 25), "Unverified point-ID mapping and camera extrinsic; no class declaration", fill="white")
    annotated.save(OUT / "ground12_candidate_projection_overlay.png")

    product_names = ["ground12_candidate_projection_overlay.png"]
    for name in ("ground_1", "ground_2"):
        pixels = points[name]["pixel_corners_xy"]
        box = [max(0, math.floor(min(p[0] for p in pixels))),
               max(0, math.floor(min(p[1] for p in pixels))),
               min(image.width, math.ceil(max(p[0] for p in pixels))+1),
               min(image.height, math.ceil(max(p[1] for p in pixels))+1)]
        filename = name.replace("_", "") + ("_visible_fragment_bbox.png" if name == "ground_1" else "_candidate_bbox.png")
        image.crop(box).save(OUT / filename)
        points[name]["crop_box_ltrb"] = box
        points[name]["bbox_crop_file"] = filename
        product_names.append(filename)

    p2 = points["ground_2"]
    assert p2["all_corners_in_frame"]
    size = (352, 256)  # Physical plane ratio .55/.40, not new native detail.
    dst = np.float32([[0, 0], [size[0]-1, 0], [size[0]-1, size[1]-1], [0, size[1]-1]])
    homography = cv2.getPerspectiveTransform(np.float32(p2["pixel_corners_xy"]), dst)
    rectified = cv2.warpPerspective(np.array(image), homography, size, flags=cv2.INTER_LINEAR)
    filename = "ground2_candidate_rectified_world_axes.png"
    Image.fromarray(rectified).save(OUT / filename)
    product_names.append(filename)

    result = {
        "kind": "offline_candidate_ground12_projection_v12", "official_scoring": "unverified",
        "mapping_status": "candidate_unverified", "camera_extrinsic_status": "candidate_unverified",
        "source_image": {"path": original.relative_to(ROOT).as_posix(), "sha256": sha(original)},
        "source_summary": {"path": summary_path.relative_to(ROOT).as_posix(), "sha256": sha(summary_path)},
        "candidate_world": {"path": WORLD.relative_to(ROOT).as_posix(), "sha256": sha(WORLD)},
        "exposure": exposure, "camera_info_first": first, "camera_info_last": last,
        "camera_info_last_minus_image_stamp_s": last["header"]["stamp"]["sec"] + last["header"]["stamp"]["nanosec"]/1e9 - exposure["stamp_s"],
        "intrinsics_note": "Observed K/D agree; last CameraInfo header and image exposure stamp coincide in this run. Camera extrinsic remains candidate.",
        "camera_pose_body_xyz_rpy": camera,
        "projection_convention": "Camera forward body +X, image right -body Y, down -body Z; use K, not stereo P.",
        "corner_order": ["world_x_minus_y_minus", "world_x_plus_y_minus", "world_x_plus_y_plus", "world_x_minus_y_plus"],
        "points": points,
        "rectification": {"target": "ground_2", "size_wh": list(size),
                          "homography_source_pixel_to_output_pixel": homography.tolist(),
                          "orientation": "world +X output right / world +Y output down; semantic upright unknown",
                          "warning": "Ground_2 minimum source edge is about 7.9 pixels and bbox height about 11.8 pixels. Upsampling adds no detail. Ground_1 is truncated and was not rectified."},
        "products": [{"path": (OUT/p).relative_to(ROOT).as_posix(), "sha256": sha(OUT/p)} for p in product_names],
        "limitations": ["Predicted corners have not been independently measured.",
                        "No category, task progress or score evidence is inferred.",
                        "This script sends no network request, ROS command or report."]}
    (OUT / "ground12_candidate_projection.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({name: points[name] for name in ("ground_1", "ground_2")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
