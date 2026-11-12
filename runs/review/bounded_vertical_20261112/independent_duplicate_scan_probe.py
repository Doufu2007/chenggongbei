"""Read-only reproducer: changed duplicate header must not hide current danger."""
from pathlib import Path
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/"src/inspection_mission"),str(ROOT/"tests"),str(ROOT)]
from test_bounded_vertical_response import state, ready
from test_bounded_ground_probe import scan

rows = []
for label,value in (("near_return",.50),("nonfinite_return",float("nan"))):
    g = state()
    ready(g)
    before = {"state":g.state,"source_stamp":g.scan["stamp"],"minimum_m":g.scan["minimum_m"]}
    msg = scan(g.scan["stamp"])
    msg["ranges"][0] = value
    g.update_scan(msg,wall_s=1.15)
    d = g.step(wall_s=1.15)
    rows.append({"case":label,"before":before,"after_state":g.state,"after_reason":g.reason,
                 "output_z_mps":d.linear_z,"old_minimum_retained":g.scan["minimum_m"] if g.scan else None,
                 "hazard_hidden": d.linear_z>0.})
source = ROOT/"src/inspection_mission/inspection_mission/bounded_vertical_response.py"
data = {"source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"rows":rows,
        "all_expected_stops":all(r["output_z_mps"]==0 for r in rows),"platform_run":False}
target=Path(__file__).with_name("independent_duplicate_scan_metrics.json")
target.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps(data,ensure_ascii=True))
