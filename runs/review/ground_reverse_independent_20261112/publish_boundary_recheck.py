"""Read-only independent verification of reverse's final query boundary fix."""
import hashlib
import json
from pathlib import Path
from independent_probe import (Transport, ROOT, actual_callback_all_three,
                               delayed_dds_cross_pulse_deadline)


def final_query_failure_or_stale(fault):
    f=Transport().first()
    try:
        f.wall[0]+=.01
        calls=[0]
        def query():
            calls[0]+=1
            if calls[0]==2:
                if fault=="exception":raise RuntimeError("independent DDS query failed")
                f.wall[0]+=.25
            return 1
        f.node.publisher_by_topic["/cmd_vel"].get_subscription_count=query
        before=len(f.rows)
        d=f.node.tick()
        assert f.rows[before:] and all(x==(0.,0.,0.) for x in f.rows[before:])
        assert d.linear_x==d.linear_y==d.angular_z==0.
        if fault=="exception": assert f.node.guard.state=="STOP"
        return {"fault":fault,"outputs":f.rows[before:],"state":f.node.guard.state,
                "phase":f.node.guard.phase,"reason":f.node.guard.reason,
                "last_command":f.node.guard.direction_commands[-1]}
    finally:f.close()


deadline=delayed_dds_cross_pulse_deadline()
assert deadline["wall_after_query"]>deadline["deadline"]
assert deadline["last_command"]["timing"]["odom"]["fresh"] is True
assert deadline["outputs"]==[(0.,0.,0.)]
assert deadline["phase"]=="PULSE_SETTLE"
normal=actual_callback_all_three()
result={"deadline_fresh_sources":deadline,
    "dds_exception":final_query_failure_or_stale("exception"),
    "dds_stale_sources":final_query_failure_or_stale("stale"),
    "normal_three_pulses":normal,
    "sha256":{p.as_posix():hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (
        Path("src/inspection_mission/inspection_mission/ground_reverse_response.py"),
        Path("scripts/run_ground_reverse_response.sh"),
        Path("src/inspection_mission/inspection_mission/ground_direction_response.py"))}}
Path(__file__).with_name("publish_boundary_recheck_metrics.json").write_text(
    json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
print(json.dumps({k:({kk:vv for kk,vv in v.items() if kk!="pulse_records"} if k=="normal_three_pulses" else v)
    for k,v in result.items()},ensure_ascii=False,indent=2))
