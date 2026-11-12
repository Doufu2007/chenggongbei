"""Local synthetic cost probe, never a ROS execution or task/score result."""
from __future__ import annotations
import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/"src/inspection_mission"),str(ROOT/"tests")]
from inspection_mission.ground_bounded_approach_sync import GroundSynchronizedApproachGuard
from test_ground_bounded_approach import drive_ready,image
from test_ground_observation_survey import posed_odom
from test_bounded_ground_probe import scan


def run():
    state=GroundSynchronizedApproachGuard(started_wall_s=0.)
    base,yaw=drive_ready(state)
    durations={}
    def measure(key,callback):
        w,c=time.perf_counter(),time.thread_time()
        callback()
        durations.setdefault(key,[]).append((time.perf_counter()-w,time.thread_time()-c))
    for i in range(1,2501):
        wall=base+i*.02;stamp=round(10.+wall,9)
        if i%5==0:measure("clock",lambda:state.update_clock(stamp,wall_s=wall))
        payload=posed_odom(stamp,yaw=yaw)
        measure("odom",lambda:state.update_odom(payload,wall_s=wall))
        uav=posed_odom(stamp,yaw=yaw);uav["child_frame_id"]="uav/base_link"
        measure("uav_odom",lambda:state.observe_public("/uav/odom",uav,wall_s=wall))
        if i%6==0:measure("scan",lambda:state.update_scan(scan(stamp),wall_s=wall))
        if i%17==0:
            msg=image(stamp)
            measure("ground_image",lambda:state.observe_image("/camera/image",msg,wall_s=wall))
            measure("uav_image",lambda:state.observe_image("/uav/camera/image",msg,wall_s=wall))
        if i%3==0:measure("step",lambda:state.step(wall_s=wall))
    def stats(rows,column):
        values=sorted(row[column] for row in rows)
        return {"total_s":sum(values),"median_s":statistics.median(values),
            "p99_s":values[min(len(values)-1,math.ceil(.99*len(values))-1)],"max_s":max(values)}
    return {"kind":"synthetic_local_pure_guard_cost_not_platform_evidence",
        "synthetic_duration_s":50.,"final_state":state.state,"reason":state.reason,
        "history_rows":len(state._observations.full_odom_history),
        "method":"unchanged v16 guard; existing deterministic test fixtures; stationary synthetic DRIVE; no ROS transport",
        "unmeasured":["ROS deserialize/take","message_to_ordereddict","RMW/DDS transport","platform CPU quota/steal","real executor scheduling","cross-thread lock wait"],
        "callbacks":{key:{"count":len(rows),"wall":stats(rows,0),"cpu":stats(rows,1)} for key,rows in durations.items()}}


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args();result=run();args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False))
