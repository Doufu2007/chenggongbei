"""Offline candidate-map projection of the v13 already captured team image.

No ROS, network, task declarations or referee data. The candidate camera
extrinsic, flat target plane and point-ID mapping remain unverified contracts.
"""
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
    assert abs(math.sqrt(sum(v*v for v in q)) - 1) < 1e-8
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def project(pose, corners, camera_translation, intrinsics):
    r = rotation(pose["quaternion"])
    projected = []
    for point in corners:
        local = r.T @ (np.array(point) - pose["position"]) - camera_translation
        if local[0] <= 0:
            projected.append(None)
        else:
            projected.append([float(intrinsics[0] - intrinsics[2]*local[1]/local[0]),
                              float(intrinsics[1] - intrinsics[3]*local[2]/local[0])])
    return projected


def main():
    summary_path = OUT / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    native = next(i for i in summary["native_images"]
                  if i["topic"] == "/uav/camera/image" and i["phase"] == "last_associated")
    assert native["exposure_validation"] == "valid_prior_pose_observation"
    original = OUT / native["file_name"]
    assert sha(original) == native["sha256"]
    exposure = native["exposure"]
    assert exposure["association_error"] is None
    pose = exposure["odom"]

    public = summary["snapshots"]["assembled"]["public_messages"]
    info_series = public["/uav/camera/camera_info"]
    info_first, info_last = info_series["first"], info_series["last"]
    assert info_first["k"] == info_last["k"]
    assert info_first["d"] == info_last["d"] == [0.0]*5
    k = info_last["k"]
    intrinsics = (k[2], k[5], k[0], k[4])
    assert [info_last["width"], info_last["height"]] == [native["width"], native["height"]]

    world = ET.parse(WORLD).getroot().find("world")
    camera = world.find("model[@name='amphibious_drone']/link[@name='base_footprint']/sensor[@name='uav_camera']")
    camera_pose = [float(v) for v in camera.findtext("pose").split()]
    assert camera_pose[3:] == [0.0, 0.0, 0.0]
    camera_translation = np.array(camera_pose[:3])
    points = {}
    for n in range(5):
        model = world.find(f"model[@name='ground_task_point_{n}']")
        center = [float(v) for v in model.findtext("pose").split()]
        size = [float(v) for v in model.findtext("link/visual/geometry/box/size").split()]
        assert center[3:] == [0.0]*3
        x, y, z = center[:3]
        w, h, height = size
        corners = [[x-w/2, y-h/2, z+height/2], [x+w/2, y-h/2, z+height/2],
                   [x+w/2, y+h/2, z+height/2], [x-w/2, y+h/2, z+height/2]]
        points[f"ground_{n}"] = {"world_corners_m": corners,
                                 "pixel_corners_xy": project(pose, corners, camera_translation, intrinsics)}

    p4 = points["ground_4"]["pixel_corners_xy"]
    assert all(p is not None and 0 <= p[0] < native["width"] and 0 <= p[1] < native["height"] for p in p4)
    image = Image.open(original).convert("RGB")
    annotated = image.copy()
    draw = ImageDraw.Draw(annotated)
    colors = {"ground_0": "cyan", "ground_3": "yellow", "ground_4": "lime"}
    for name, color in colors.items():
        polygon = points[name]["pixel_corners_xy"]
        if all(p is not None for p in polygon):
            draw.line([tuple(p) for p in polygon] + [tuple(polygon[0])], fill=color, width=2)
            for i, p in enumerate(polygon):
                if 0 <= p[0] < image.width and 0 <= p[1] < image.height:
                    draw.text((p[0]+3, p[1]+3), f"{name} C{i}", fill=color, stroke_fill="black", stroke_width=1)
    draw.rectangle((0, 0, image.width, 44), fill="black")
    draw.text((8, 8), "Candidate geometry overlay; same captured image / no category assertion", fill="white")
    draw.text((8, 25), "Unverified extrinsic and mapping; this run first task is ground_4", fill="white")
    annotated.save(OUT / "ground4_candidate_projection_overlay.png")

    minx, miny = [math.floor(min(p[i] for p in p4)) for i in (0, 1)]
    maxx, maxy = [math.ceil(max(p[i] for p in p4)) for i in (0, 1)]
    image.crop((minx, miny, maxx+1, maxy+1)).save(OUT / "ground4_candidate_bbox.png")
    crop_size = (256, 352)
    destination = np.float32([[0, 0], [crop_size[0]-1, 0],
                              [crop_size[0]-1, crop_size[1]-1], [0, crop_size[1]-1]])
    homography = cv2.getPerspectiveTransform(np.float32(p4), destination)
    rectified = cv2.warpPerspective(np.array(image), homography, crop_size,
                                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    Image.fromarray(rectified).save(OUT / "ground4_candidate_rectified_world_axes.png")

    scenarios = {}
    for name, xyz, yaw in [("origin_facing_ground2", [0, 0, 0.055], -math.pi/2),
                            ("ground2_observation_candidate", [0.6, -1.55, 0.055], math.atan2(-1.15, -0.6))]:
        synthetic_pose = {"position": xyz, "quaternion": [0, 0, math.sin(yaw/2), math.cos(yaw/2)]}
        pixels = project(synthetic_pose, points["ground_2"]["world_corners_m"], camera_translation, intrinsics)
        scenarios[name] = {"status": "prediction_not_platform_observation", "body_xyz_m": xyz,
                           "yaw_rad": yaw, "pixel_corners_xy": pixels,
                           "bbox_extent_px": [max(p[i] for p in pixels)-min(p[i] for p in pixels) for i in (0, 1)]}

    products = ["ground4_candidate_projection_overlay.png", "ground4_candidate_bbox.png",
                "ground4_candidate_rectified_world_axes.png"]
    result = {"kind": "offline_candidate_ground_projection", "official_scoring": "unverified",
              "mapping_status": "candidate_unverified", "camera_extrinsic_status": "candidate_unverified",
              "source_summary": {"path": summary_path.relative_to(ROOT).as_posix(), "sha256": sha(summary_path)},
              "source_image": {"path": original.relative_to(ROOT).as_posix(), "sha256": sha(original)},
              "candidate_world": {"path": WORLD.relative_to(ROOT).as_posix(), "sha256": sha(WORLD)},
              "exposure": exposure, "camera_info_first": info_first, "camera_info_last": info_last,
              "intrinsics_note": "Observed K/D agree; last CameraInfo and selected image have matching exposure stamps in this run.",
              "camera_pose_body_xyz_rpy": camera_pose,
              "projection_convention": "SDF camera forward=body +X, image right=-body Y, image down=-body Z; K, not stereo P, is used.",
              "corner_order": ["world_x_minus_y_minus", "world_x_plus_y_minus", "world_x_plus_y_plus", "world_x_minus_y_plus"],
              "points": points, "scenarios": scenarios,
              "rectification": {"size_wh": list(crop_size), "homography_source_pixel_to_output_pixel": homography.tolist(),
                                "orientation": "world +X to output right, world +Y to output down; does not assert texture's semantic upright orientation",
                                "upsampling_note": "Resampling adds no detail; retain original quadrilateral and original image."},
              "products": [{"path": (OUT/p).relative_to(ROOT).as_posix(), "sha256": sha(OUT/p)} for p in products],
              "limitations": ["Projected polygons are hypothesis predictions, not independently measured target corners.",
                              "No category or task-ID declaration is sent. ground_4 is this run's first task, but only its geometry is analyzed.",
                              "No new platform motion or score is demonstrated."]}
    (OUT / "ground_next_path_projection.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"products": products, "ground4_corners": p4, "scenarios": scenarios}, ensure_ascii=False))


if __name__ == "__main__":
    main()
