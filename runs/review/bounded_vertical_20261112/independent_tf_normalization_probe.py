"""Read-only probe for angular TF comparison under accepted quaternion norm slack."""
from pathlib import Path
import hashlib
import json
import math
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/"src/inspection_mission"),str(ROOT/"tests"),str(ROOT)]
from test_bounded_vertical_response import state,ready,tf_static

g=state();ready(g)
msg=tf_static()
angle=.04
scale=1.0009
s,c=math.sin(angle/2),math.cos(angle/2)
k=math.sqrt(.5)
q=tuple(scale*x for x in (s*k,c*k,s*k,c*k))
msg["transforms"][0]["transform"]["rotation"]=dict(zip(("x","y","z","w"),q))
g.update_tf(msg,wall_s=1.15)
d=g.step(wall_s=1.15)
source=ROOT/"src/inspection_mission/inspection_mission/bounded_vertical_response.py"
result={"source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"quaternion_norm":math.hypot(*q),
        "normalized_angular_error_rad":angle,"comparison_threshold_rad":.005,
        "accepted":g.state=="ASCEND","output_z_mps":d.linear_z,"reason":g.reason,"platform_run":False}
target=Path(__file__).with_name("independent_tf_normalization_metrics.json")
target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps(result))
