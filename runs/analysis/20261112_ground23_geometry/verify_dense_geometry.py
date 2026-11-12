"""Numerical cross-check of the analytic continuous geometry extrema."""
import hashlib
import json
from pathlib import Path
import numpy as np
import calculate_geometry as model

OUT = Path(__file__).resolve().parent
report = json.loads((OUT/"report.json").read_text(encoding="utf-8"))
checked, largest_gap, rows = 0, 0., []
for name, plan in report["plans"].items():
    for phase in plan["phases"]:
        t = np.linspace(0., 1., 25001)
        if phase["kind"] == "turn":
            heading = phase["start_yaw"]+t*(phase["end_yaw_unwrapped"]-phase["start_yaw"])
            x = phase["start"][0]+.08*np.cos(heading)
            y = phase["start"][1]+.08*np.sin(heading)
        else:
            x = phase["start"][0]+t*(phase["end"][0]-phase["start"][0])+.08*np.cos(phase["heading"])
            y = phase["start"][1]+t*(phase["end"][1]-phase["start"][1])+.08*np.sin(phase["heading"])
        dense = float("inf")
        for obstacle, rect in model.BOXES:
            dx = np.maximum.reduce((rect[0]-x, np.zeros_like(x), x-rect[2]))
            dy = np.maximum.reduce((rect[1]-y, np.zeros_like(y), y-rect[3]))
            dense = min(dense, float(np.hypot(dx, dy).min()))
        analytic = phase["clearance"]["laser_distance_m"]
        assert analytic <= dense+1e-12
        gap = dense-analytic
        assert gap < 1e-8, (name, phase["label"], gap)
        rows.append({"plan": name, "phase": phase["label"], "analytic_m": analytic, "dense_m": dense, "gap_m": gap})
        checked += 1
        largest_gap = max(largest_gap, gap)
result = {"kind": "independent_numerical_crosscheck_of_continuous_extrema", "phase_count": checked,
    "samples_per_phase": 25001, "largest_dense_minus_analytic_m": largest_gap,
    "input_report_sha256": hashlib.sha256((OUT/"report.json").read_bytes()).hexdigest(),
    "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    "all_extrema_agree_within_1e_8m": True, "rows": rows,
    "note": "Numerical cross-check of a candidate design, not sensor validation or official acceptance."}
(OUT/"dense_check.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
print(json.dumps({key:result[key] for key in ("phase_count", "samples_per_phase", "largest_dense_minus_analytic_m")}))
