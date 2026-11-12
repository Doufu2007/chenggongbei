"""Offline synthetic control integration; never a platform/vision validation."""
import sys, math, json, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/inspection_mission')]
import pytest
from test_full_flight_inspection import install_ros, feed_node, proof
import inspection_mission.full_flight_inspection as m
from inspection_mission.target_geometry import TargetFix
from inspection_mission.ground_single_point import GROUND_LABELS

patch = pytest.MonkeyPatch(); _, published, _, _ = install_ros(patch); wall = [0.]
node = m.create_flight_inspection_node(session_id='offline_closed_loop', ground_return_evidence=proof(), wall_clock=lambda: wall[0])
x = y = yaw = 0.; z = .055; lamp = False; command = (0., 0., 0.); omega = 0.
states = []; illumination = []; last_state = None; dt = .1; started = time.perf_counter()
for i in range(9000):
    wall[0] = i*dt; stamp = 30.+wall[0]
    feed_node(node, wall[0], stamp, x=x, y=y, z=z, yaw=yaw)
    node.receive('/uav/laser_state', {'data': lamp})
    c = node.guard.controller
    if c is not None and c._stage_index < len(c._stages):
        stage = c._stages[c._stage_index]
        if stage.kind == 'recognize':
            label = next(iter(m.WINDOW_LABELS.values())) if stage.window_index >= 0 else next(iter(GROUND_LABELS.values()))
            d = {'target_id': label, 'confidence': .9, 'center': [.5, .5], 'stamp': stamp, 'source': 'uav', 'frame_id': m.CAMERA_FRAME}
            if stage.point_id in m.AIR_CENTERS:
                q = node.guard.odom['q']; inverse_q = (-q[0], -q[1], -q[2], q[3])
                relative = m._rotate(inverse_q, tuple(b-a for a,b in zip((x,y,z), m.AIR_CENTERS[stage.point_id])))
                optical = m._rotate((.5,-.5,.5,.5), tuple(a-b for a,b in zip(relative, m.CAMERA_OFFSET)))
                pixel = [400.+476.7*optical[0]/optical[2], 400.+476.7*optical[1]/optical[2]]
                d['target_fix'] = m.localize_floor_pixel(pixel, [476.7,0.,400.,0.,476.7,400.,0.,0.,1.],
                    node.guard.odom['message'], image_stamp=stamp, camera_frame=m.CAMERA_FRAME,
                    session_id='offline_closed_loop', target_id=label, point_id=stage.point_id, now=stamp)
            node.detections = [d]
    before = len(published); result = node.tick()
    for topic, p in published[before:]:
        if topic == '/uav/cmd_vel': command = p['xyz']; omega = p['yaw']
        if topic == '/uav/laser_fire': lamp = p['data']
    if node.guard.state != last_state:
        states.append({'wall_s': wall[0], 'state': node.guard.state, 'xyz': [x, y, z], 'yaw': yaw})
        last_state = node.guard.state
    if c is not None and c.state.startswith('AIR_INSPECT_'):
        illumination.append({'wall_s': wall[0], 'state': c.state, 'aim': c.aim.reason if c.aim else None,
            'geometry': c.geometric_aim.reason if c.geometric_aim else None, 'lamp': lamp,
            'feedback': node.guard.feedback, 'dwell_since': c._evidence_since,
            'light_started': c._light_command_started_at})
    if node.mission_completed or node.guard.stop_wall is not None:
        break
    vx = math.cos(yaw)*command[0]-math.sin(yaw)*command[1]
    vy = math.sin(yaw)*command[0]+math.cos(yaw)*command[1]
    vz = 2*(command[2]-.02695) if command[2] else -.0539
    x += vx*dt; y += vy*dt; z = max(.055, z+vz*dt)
    yaw = math.atan2(math.sin(yaw+omega*dt), math.cos(yaw+omega*dt))
node.close_watchdog(); node.destroy_node(); patch.undo()
result = {'kind': 'offline_synthetic_control_loop_not_platform_or_sensor_validation',
    'mission_completed': node.mission_completed, 'final_state': node.guard.state, 'reason': node.guard.reason,
    'synthetic_source_and_wall_elapsed_s': wall[0], 'integration_dt_s': dt,
    'z_model': 'vz=2*(nonzero_request-.02695);zero=-.0539;home_floor=.055',
    'visual_input': 'synthetic_valid_class_and_analytic_projection;production_localize_floor_pixel_for_original_fix',
    'actual_python_wall_s': time.perf_counter()-started, 'stages': states,
    'illumination': illumination, 'path_m': node.guard.total_path, 'landing_evidence': node.guard.landing_evidence}
output = Path(__file__).with_suffix('.json')
output.write_text(json.dumps(m._safe(result), ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: v for k,v in result.items() if k not in ('stages', 'landing_evidence', 'illumination')}, ensure_ascii=True))
print('stage_count', len(states))
