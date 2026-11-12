"""Run the independent matcher on an explicit frozen offline-study manifest."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/inspection_mission"))
from inspection_mission.ground_plane_matcher import GroundPlaneMatcher, TemplateSpec


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, image):
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok: raise ValueError(f"Cannot encode {path}")
    path.write_bytes(encoded.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study-report", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    study = json.loads(args.study_report.read_text(encoding="utf-8"))
    args.output.mkdir(parents=True, exist_ok=True)
    specs = []
    for ref in study["references"]:
        specs.append(TemplateSpec(Path(ref["absolute_path"]), ref["class_id"], ref["label"],
                                  {"source_url": ref["source_url"], "git_blob_sha": ref["git_blob_sha"],
                                   "official_current_library_verified": False}, ref["sha256"]))
    cv2.setNumThreads(1)
    matcher = GroundPlaneMatcher(specs)
    frames = []
    for q in study["queries"]:
        source = ROOT/q["source_path"]
        assert sha(source) == q["source_sha256"]
        raw = cv2.imdecode(np.frombuffer(source.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        K = np.array(q["camera_info"]["k"], np.float64).reshape(3,3)
        result = matcher.match(raw, K, height_m=.173)
        bev = result.pop("bev_bgr")
        name = q["name"].removesuffix("_full_ground_bev")
        save(args.output/(name+"_bev.png"), bev)
        overlay = raw.copy()
        for item in result["candidates"]:
            if item["source_quad_xy"] is not None:
                points = np.rint(item["source_quad_xy"]).astype(np.int32)
                cv2.polylines(overlay, [points], True, (0,255,0), 2)
                cv2.putText(overlay, item["class_id"]+" (candidate)", tuple(points[0]),
                            cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1,cv2.LINE_AA)
        save(args.output/(name+"_source_candidate_overlay.png"),overlay)
        frames.append({"name": name, "source_path": q["source_path"], "source_sha256": q["source_sha256"],
                       "source_metadata": q["source_metadata"], "result": result})
        print(name, "candidates",[(x["class_id"], x["inliers"],x["query_distinct_xy"]) for x in result["candidates"]], flush=True)
    report = {"kind":"independent_ground_plane_matcher_development_check",
              "created_at":datetime.now(timezone.utc).isoformat(),
              "study_report_path":str(args.study_report.resolve()),"study_report_sha256":sha(args.study_report),
              "module_path":"src/inspection_mission/inspection_mission/ground_plane_matcher.py",
              "module_sha256":sha(ROOT/"src/inspection_mission/inspection_mission/ground_plane_matcher.py"),
              "class_ground_truth":None,"accuracy":None,"held_out":False,"official_acceptance":False,
              "frames":frames,"limitations":["Five fixed development frames, three correlated captures.",
              "Reference labels/provenance remain third-party annotations; no official class truth.",
              "Candidate camera extrinsics, no ROS/control/referee integration or deployment."]}
    (args.output/"report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding="utf-8")
    fields=["frame","template","class_id","good","inliers","template_unique_xy","query_unique_xy","template_hull_fraction","inlier_fraction","plane_area_m2","median_error_px","query_hull_over_projection","accepted_candidate","rejection_reasons"]
    with (args.output/"scores.csv").open("w",newline="",encoding="utf-8-sig") as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for frame in frames:
            for row in frame["result"]["diagnostics"]:
                writer.writerow({"frame":frame["name"],"template":Path(row["template_path"]).name,"class_id":row["class_id"],
                                 "good":row["good_matches"],"inliers":row["inliers"],"template_unique_xy":row["template_distinct_xy"],
                                 "query_unique_xy":row["query_distinct_xy"],"template_hull_fraction":row["template_hull_fraction"],
                                 "inlier_fraction":row["inlier_fraction"],"plane_area_m2":row["plane_area_m2"],
                                 "median_error_px":row["median_reprojection_error_px"],"query_hull_over_projection":row["query_hull_over_projection"],
                                 "accepted_candidate":row["accepted_candidate"],"rejection_reasons":";".join(row["rejection_reasons"])})
    print("all_template_diagnostics",sum(len(x["result"]["diagnostics"]) for x in frames),flush=True)


if __name__ == "__main__":
    main()
