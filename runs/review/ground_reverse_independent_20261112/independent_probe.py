"""Independent reverse runtime callback/tick checks; synthetic ROS transport."""
from __future__ import annotations
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/"src/inspection_mission"), str(ROOT/"tests")]
import pytest
from inspection_mission import ground_reverse_response as api
from inspection_mission.ground_direction_response import DirectionLimits
from test_bounded_ground_probe import install_ros_transport, scan
from test_ground_observation_survey import posed_odom


class Transport:
    def __init__(self):
        self.patch = pytest.MonkeyPatch()
        self.callbacks, _, _ = install_ros_transport(self.patch)
        self.wall = [0.]
        self.node = api.create_reverse_response_node(wall_clock=lambda: self.wall[0])
        self.node.arm_watchdog = lambda: None
        self.rows = []
        self.others = []
        self.node.publisher_by_topic["/cmd_vel"].publish = lambda m:self.rows.append((m.linear.x,m.linear.y,m.angular.z))
        for t, pub in self.node.publisher_by_topic.items():
            if t != "/cmd_vel": pub.publish=lambda m,t=t:self.others.append(t)
        self.x = self.y = self.yaw = 0.

    def feed(self, t, integrate=True):
        dt = t-self.wall[0]
        vx, vy, wz = self.rows[-1] if self.rows else (0.,0.,0.)
        vx*=.2; vy*=.2; wz*=.15
        if integrate:
            self.x+=dt*(vx*math.cos(self.yaw)-vy*math.sin(self.yaw))
            self.y+=dt*(vx*math.sin(self.yaw)+vy*math.cos(self.yaw))
            self.yaw+=dt*wz
        else: vx=vy=wz=0.
        self.wall[0]=t
        source=round(10.+t,9); sec=int(source)
        self.callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((source-sec)*1e9)}})
        m=posed_odom(source,x=self.x,yaw=self.yaw)
        m["pose"]["pose"]["position"]["y"]=self.y
        m["twist"]["twist"]["linear"]["x"]=vx
        m["twist"]["twist"]["linear"]["y"]=vy
        m["twist"]["twist"]["angular"]["z"]=wz
        self.callbacks["/odom"](m); self.callbacks["/scan"](scan(source))
        return self.node.tick()

    def first(self):
        for i in range(1,1500):
            d=self.feed(round(i*.05,9))
            if d.linear_x<0:return self
            assert self.node.guard.state not in ("STOP","FINISHED"),self.node.guard.reason
        raise AssertionError("did not reach negative pulse")

    def close(self):
        self.node.close_watchdog();self.patch.undo()


def actual_callback_all_three():
    f=Transport()
    try:
        for i in range(1,2000):
            f.feed(round(i*.05,9))
            if f.node.guard.state=="STOP":break
        g=f.node.guard
        assert g.reason=="direction_response_observed_official_score_unverified",g.reason
        assert len(g.pulse_records)==3
        for r in g.pulse_records:
            assert abs(r["start"]["yaw_rad"]-math.pi/2)<=.025
            assert r["settled"]["stable_wall_s"]>=1.-1e-9
            assert r["settled"]["stable_source_s"]>=1.-1e-9
        actual=[(x["linear_x"],x["linear_y"],x["angular_z"]) for x in g.direction_commands if x["sent"]]
        assert f.rows==actual and not f.others
        assert all(x<=0 and y==0 and w>=0 for x,y,w in f.rows)
        original,reverse=asdict(DirectionLimits()),asdict(api.ReverseLimits())
        assert {k for k in original if reverse[k]!=original[k]}=={"max_rotation_rad"}
        return {"commands":len(actual),"negative_requests":sum(x<0 for x,y,w in actual),
            "turn_rotation_rad":g.accumulated_rotation_rad,"pulse_records":g.pulse_records,
            "initial_turn_error":g.initial_turn_error,"target_yaw":g.target_yaw,"other_outputs":f.others}
    finally:f.close()


def delayed_dds_cross_pulse_deadline():
    f=Transport().first()
    try:
        g=f.node.guard
        start=g.pulse_started_wall
        # Keep fresh valid sources but hold position. This isolates request
        # duration independently of whether an actuator responds to negative X.
        while f.wall[0]<start+1.90-1e-9:
            f.feed(round(f.wall[0]+.05,9),integrate=False)
        f.feed(start+1.95,integrate=False)
        f.wall[0]=start+1.97
        calls=[0]
        def query():
            calls[0]+=1
            if calls[0]==2:f.wall[0]=start+2.02
            return 1
        f.node.publisher_by_topic["/cmd_vel"].get_subscription_count=query
        before=len(f.rows)
        decision=f.node.tick()
        return {"pulse_start":start,"deadline":start+2.,"wall_after_query":f.wall[0],
            "outputs":f.rows[before:],"phase":g.phase,"state":g.state,"reason":g.reason,
            "decision":asdict(decision),"last_command":g.direction_commands[-1]}
    finally:f.close()


if __name__=="__main__":
    results={"all_three":actual_callback_all_three(),"delayed_dds":delayed_dds_cross_pulse_deadline()}
    results["sha256"]={p.as_posix():hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (
        Path("src/inspection_mission/inspection_mission/ground_reverse_response.py"),
        Path("scripts/run_ground_reverse_response.sh"))}
    Path(__file__).with_name("independent_metrics.json").write_text(json.dumps(results,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    print(json.dumps({"all_three":{k:v for k,v in results["all_three"].items() if k!="pulse_records"},
        "delayed_dds":results["delayed_dds"],"sha256":results["sha256"]},ensure_ascii=False,indent=2))
