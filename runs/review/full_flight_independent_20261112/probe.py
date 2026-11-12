"""Read-only production probes through the actual FlightNode factory."""
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src'/'inspection_mission'), str(ROOT/'tests')]
from pytest import MonkeyPatch
from test_full_flight_inspection import install_ros, feed_node, proof
from inspection_mission import full_flight_inspection as m
from inspection_mission.mission import _Stage


def completion_late_zero():
    patch = MonkeyPatch()
    callbacks, published, pubs, _ = install_ros(patch)
    wall = [0.]
    node = m.create_flight_inspection_node(session_id='independent',
        ground_return_evidence=proof(), wall_clock=lambda: wall[0])
    try:
        for i in range(13):
            wall[0] = i/10
            feed_node(node, wall[0], 30.+wall[0])
            node.tick()
        c = node.guard.controller
        c._stages = [_Stage('LAND', 'uav', (0., 0., .055, 0.), 'vertical')]
        c._stage_index = 0
        c.state = 'LAND'
        c._phase_started = c._started = 31.2
        # A stationary LAND stage; preserve real factory source gating and
        # publish order. Only the final independent zero publish is delayed.
        calls = [0]
        base_publish = pubs['/uav/cmd_vel'].publish
        delayed = [False]
        def publish(msg):
            calls[0] += 1
            base_publish(msg)
            if calls[0] == 2:
                wall[0] += .30
                delayed[0] = True
        for i in range(13, 28):
            wall[0] = i/10
            feed_node(node, wall[0], 30.+wall[0])
            calls[0] = 0
            pubs['/uav/cmd_vel'].publish = publish
            node.tick()
            if delayed[0] or node.guard.stop_wall is not None:
                break
        return {'delayed_final_zero': delayed[0],
            'mission_completed': node.mission_completed,
            'state': node.guard.state, 'reason': node.guard.reason,
            'condition_at_return': node.guard.condition(wall[0]),
            'odom_wall_age_s': wall[0]-node.guard.odom['wall'],
            'clock_wall_age_s': wall[0]-node.guard.clock['wall'],
            'mission_status': [v['data'] for t,v in published if t == '/inspection/mission_status'],
            'landing_evidence': node.guard.landing_evidence}
    finally:
        node.destroy_node()
        patch.undo()


def vertical_return_veto():
    patch = MonkeyPatch()
    _, published, _, _ = install_ros(patch)
    wall = [0.]
    node = m.create_flight_inspection_node(session_id='independent',
        ground_return_evidence=proof(), wall_clock=lambda: wall[0])
    try:
        feed_node(node, 0., 30., z=.9)
        g = node.guard
        g.home_z = .055
        # Represent a physically visible front lidar return at body
        # (.32,0,.23), inside the 1s upward sweep ending .93m but outside
        # the current body envelope at z=.90m.
        # This is an unknown current return, separate from static geometry.
        import struct
        cloud = dict(node.last_cloud_raw)
        cloud['header']={'frame_id':'uav/lidar_link','stamp':{'sec':30,'nanosec':10000000}}
        cloud['data'] = b''.join(struct.pack('<fff', .20, 0., .05) for _ in range(3))
        node.receive('/clock',{'clock':{'sec':30,'nanosec':10000000}})
        node.receive('/uav/lidar/points', cloud)
        returned = g.path_clear((0., 0., .03, 0.), (0.,0.,1.282,0.), wall=wall[0])
        g.controller = m.FlightMissionController('independent',.055)
        g.controller.start_flight(30.)
        node.tick()
        return {'path_clear': returned, 'start_z': .9, 'end_z': .93,
            'obstacle_world_xyz': [.32,0.,1.13], 'BELOW': m.BELOW, 'ABOVE': m.ABOVE,
            'nonzero_published': [v for t,v in published if t == '/uav/cmd_vel' and any(v['xyz'])],
            'state':g.state,'reason':g.reason}
    finally:
        node.destroy_node()
        patch.undo()


if __name__ == '__main__':
    import hashlib
    data = {'source_sha256': hashlib.sha256((ROOT/'src/inspection_mission/inspection_mission/full_flight_inspection.py').read_bytes()).hexdigest(),
        'completion_late_zero': completion_late_zero(),
        'vertical_return_veto': vertical_return_veto()}
    print(json.dumps(data, ensure_ascii=False, indent=2))
    (Path(__file__).parent/'metrics.json').write_text(json.dumps(data, ensure_ascii=False, indent=2),encoding='utf-8')
