"""Factory transport-level world/body and downward-trim checks."""
from pathlib import Path
import json,math,sys
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/inspection_mission'),str(ROOT/'tests')]
from pytest import MonkeyPatch
from test_full_flight_inspection import install_ros,feed_node,proof
from inspection_mission import full_flight_inspection as m

def run(stage_name,pose):
    patch=MonkeyPatch();_,published,_,_=install_ros(patch)
    node=m.create_flight_inspection_node(session_id='frame-probe',ground_return_evidence=proof(),wall_clock=lambda:0.)
    try:
        feed_node(node,0.,30.,x=pose[0],y=pose[1],z=pose[2],yaw=pose[3])
        g=node.guard;g.home_z=.055;c=g.controller=m.FlightMissionController('frame-probe',.055);c.start_flight(30.)
        c._stage_index=next(i for i,s in enumerate(c._stages) if s.name==stage_name);c.state=stage_name
        c._identified_window_z[0]=1.282
        node.tick()
        command=g.commands[-1]
        vx,vy,z=command['published_body_xyz'];yaw=pose[3]
        back=(math.cos(yaw)*vx-math.sin(yaw)*vy,math.sin(yaw)*vx+math.cos(yaw)*vy)
        return {'stage':stage_name,'fault':g.reason if g.stop_wall is not None else None,
            'requested_world_velocity':command['requested_world_velocity'],
            'published_body_xyz':command['published_body_xyz'],'reconstructed_world_xy':back,
            'light_commands':[p['data'] for t,p in published if t=='/uav/laser_fire']}
    finally:node.destroy_node();patch.undo()

data={'yaw_pi_over_2_translation':run('AIR_NAV_1',(2.65,-.3275,.9,math.pi/2)),
      'window_descent':run('WINDOW_1_DESCEND',(1.05,-.3275,1.282,0.)),
      'hover_trim':m.flight_z_request(.9,.9)}
(Path(__file__).parent/'frame_response_metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(data,ensure_ascii=False,indent=2))
