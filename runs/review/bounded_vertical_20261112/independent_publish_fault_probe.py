"""Read-only transport failure injection; a broken ground channel must not skip UAV zero."""
from pathlib import Path
import hashlib
import json
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/"src/inspection_mission"),str(ROOT/"tests"),str(ROOT)]
from pytest import MonkeyPatch
from test_bounded_vertical_response import transport,feed_transport,tf_static

patch=MonkeyPatch()
wall=[0.]
try:
    node,callbacks,published,_=transport(patch,wall)
    callbacks["/tf_static"](tf_static())
    for i in range(12):
        wall[0]=i/10
        feed_transport(node,callbacks,wall)
        node.tick()
    before=[values for topic,values in published if topic=="/uav/cmd_vel"][-1]
    index=len(published)
    def broken(_):
        raise RuntimeError("independent injected ground publisher failure")
    node.publisher_by_topic["/cmd_vel"].publish=broken
    error=None
    try:
        node.stop("independent_test_stop")
    except Exception as exc:
        error=type(exc).__name__+":"+str(exc)
    following=published[index:]
    source=ROOT/"src/inspection_mission/inspection_mission/bounded_vertical_response.py"
    result={"source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"before_uav":before,
            "stop_state":node.guard.state,"stop_exception":error,"following_publications":following,
            "uav_zero_attempt_returned":any(topic=="/uav/cmd_vel" and values==(0.,)*6 for topic,values in following),
            "platform_run":False}
    target=Path(__file__).with_name("independent_publish_fault_metrics.json")
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(result,ensure_ascii=True))
finally:
    patch.undo()
