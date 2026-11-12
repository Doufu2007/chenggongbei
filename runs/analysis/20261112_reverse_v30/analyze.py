"""Reuse frozen v28 integrity/numerical logic with explicit reverse mode identity."""
import importlib.util
import json
import math
from pathlib import Path

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
SPEC=importlib.util.spec_from_file_location("v28_readonly_analysis",OUT.parent/"20261112_direction_v28/analyze.py")
api=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(api)
# Exact same gzip/schema contract, but reject direction-mode evidence here.
api.MODE="ground_reverse_response"
api.OUT=OUT
api.DEFAULT_SOURCE=ROOT/"runs/evidence/20261112/callback_reverse_v30.jsonl"


def physical_reverse_summary():
    path=OUT/"metrics.json"
    data=json.loads(path.read_text(encoding="utf8"))
    rows=[]
    for p in data["pulses"]:
        r=p["reconstructed_response"]
        rows.append({"pulse":p["index"],"request_xy":p["requested_body_xy_mps"],
            "negative_request_actually_published":bool(p["request_count"] and p["requested_body_xy_mps"][0]<0),
            "actual_start_yaw_rad":r.get("start_yaw_rad"),
            "measured_delta_world_m":r.get("delta_world_m"),
            "measured_delta_start_body_xy_m":r.get("delta_start_body_xy_m"),
            "measured_bearing_rad":r.get("displacement_bearing_rad"),
            "bearing_error_from_negative_body_x_rad":p.get("measured_bearing_error_from_body_request_rad"),
            "measured_opposes_initial_heading":bool(r.get("available") and r["delta_start_body_xy_m"][0]<-.0005),
            "settled":p.get("settled_record_present"),
            "interpretation":"Measured at this posture and request; not a complete vehicle interface or arbitrary route proof"})
    api.write_json(OUT/"physical_reverse_summary.json",{"mode":api.MODE,"pulses":rows,
        "actual_reverse_motion_established_by_pose":bool(rows) and all(r["measured_opposes_initial_heading"] for r in rows),
        "all_three_pulses_observed":data["calibration_three_pulses_observed_complete"],
        "warning":"Negative publication alone does not establish reverse motion; signs here derive from actual source poses."})
    print(json.dumps({"measured_reverse_pulses":rows},ensure_ascii=False,indent=2))


if __name__=="__main__":
    status=api.main()
    if status==0:physical_reverse_summary()
    raise SystemExit(status)
