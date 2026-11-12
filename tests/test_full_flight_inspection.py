import dataclasses
import importlib
import importlib.util
import math
import pytest
from test_bounded_vertical_response import odom


def api():
    name = 'inspection_mission.full_flight_inspection'
    assert importlib.util.find_spec(name) is not None, 'complete flight executor missing'
    return importlib.import_module(name)


def proof():
    return {'wall_s': 20., 'odom_stamp_s': 30., 'pose': [0., 0., 0.],
            'stationary_wall_s': 1.1, 'stationary_source_s': 1.04,
            'linear_speed_mps': .001, 'angular_speed_rps': .002,
            'zero_publish_attempt': {'linear_x': 0., 'angular_z': 0., 'returned': True,
                'error': None, 'phase': 'RETURN_SETTLE', 'state': 'RETURN_SETTLE',
                'wall_elapsed_s': 19.9, 'returned_wall_elapsed_s': 19.901}}


def test_requires_original_ground_return_stationarity_and_actual_zero_return():
    m = api()
    assert m.validate_ground_return(proof()) == proof()
    for key, value in [('stationary_source_s', .9), ('linear_speed_mps', .02), ('pose', [.2, 0., 0.])]:
        bad = proof(); bad[key] = value
        with pytest.raises(ValueError): m.validate_ground_return(bad)
    bad = proof(); bad['zero_publish_attempt']['returned'] = False
    with pytest.raises(ValueError): m.validate_ground_return(bad)


def test_stage_order_separates_front_recognition_from_downlight_hover():
    m = api(); c = m.FlightMissionController('session', .055)
    c.start_flight(31.)
    stages = c._stages
    names = [s.name for s in stages]
    assert names[0] == 'TAKEOFF' and names[-1] == 'LAND'
    for i in (1, 2):
        assert names.index(f'WINDOW_{i}_DECLARE') < names.index(f'WINDOW_{i}_RECOGNIZE') < names.index(f'WINDOW_{i}_DESCEND') < names.index(f'WINDOW_{i}_CROSS')
        assert names.index(f'AIR_RECOGNIZE_{i}') < names.index(f'AIR_MOVE_TO_LIGHT_{i}') < names.index(f'AIR_INSPECT_{i}')
    assert names.index('WINDOW_1_CROSS') < names.index('AIR_RECOGNIZE_1') < names.index('AIR_INSPECT_2') < names.index('WINDOW_2_DECLARE')
    assert all(s.vehicle == 'uav' for s in stages)
    obs = next(s.target for s in stages if s.name == 'AIR_RECOGNIZE_1')
    assert math.dist(obs[:2], (3.08, -.32)) > 1.6


def test_public_static_sweep_can_supply_fov_blind_volume_without_ignoring_collision():
    m = api()
    assert m.static_swept_clear((1.05, -.3275, .75), (2.65, -.3275, .75))
    assert not m.static_swept_clear((1.05, -.3275, 1.282), (2.65, -.3275, 1.282))
    assert not m.static_swept_clear((1.05, -.29, .75), (2.65, -.29, .75))
    assert m.static_swept_clear((0., 0., .055), (0., 0., 1.282), support_floor=True)
    assert not m.static_swept_clear((math.nan, 0., 1.), (0., 0., 1.))


def test_ground_localization_uses_real_exposure_pose_frame_and_intrinsics():
    m = api()
    # Front optical ray reaches a floor-board at x=3.08. Never use that map
    # point as the visual answer; changing the measured pixel changes the fix.
    prior = odom(10., x=1.10, y=-.32, z=.9)
    # Independent public-world/launch values; never construct this input from
    # the implementation's own extrinsic constant (plane .012 is separate).
    camera_z = .9 + .130
    depth = 3.08 - 1.10 - .390
    K = [476.7, 0., 400., 0., 476.7, 400., 0., 0., 1.]
    pixel = [400., 400.+476.7*(camera_z-.012)/depth]
    fix = m.localize_floor_pixel(pixel, K, prior, image_stamp=10.02,
        camera_frame='uav/camera_link', session_id='s', target_id='被困人员', point_id='a1', now=10.03)
    assert fix.position_world_m == pytest.approx((3.08, -.32, .012))
    assert fix.stamp_s == 10.02 and fix.epoch_id == 's'
    with pytest.raises(ValueError):
        m.localize_floor_pixel(pixel, K, prior, image_stamp=10.02,
            camera_frame='wrong', session_id='s', target_id='被困人员', point_id='a1', now=10.03)
    with pytest.raises(ValueError):
        m.localize_floor_pixel(pixel, K, prior, image_stamp=10.2,
            camera_frame='uav/camera_link', session_id='s', target_id='被困人员', point_id='a1', now=10.21)


def test_hover_keeps_measured_z_feedback_and_landing_never_uses_trim_below_cutoff():
    m = api()
    assert m.flight_z_request(.9, .9, landed=False) == pytest.approx(.02695)
    assert .01 <= m.flight_z_request(.9, .8, landed=False) <= .05
    assert m.flight_z_request(.055, .079, landed=True) == 0.


def test_cached_fix_can_be_illuminated_without_front_image_but_not_when_expired():
    m = api()
    from inspection_mission.target_geometry import TargetFix, StampedBool, StampedRange
    target = TargetFix('被困人员', 'a1', (3.08, -.32, .012), 'odom', 'uav/base_link', 10.,
        'simulation', 's', .035, m.POSE_VERSION, 'K:live', m.PLANE_VERSION, m.CAMERA_VERSION)
    pose = m.geometry_pose(odom(20., x=3.08, y=-.32, z=.9), session_id='s')
    measured = StampedRange(m.range_extrinsic(), (1., 0., 0.), .818, .03, 5., .005, 20., 'simulation', 's')
    feedback = StampedBool(True, 20., 'simulation', 's', m.FEEDBACK_VERSION)
    assessment = m.assess_cached_target(target, pose, measured, feedback, now=20.)
    assert assessment.allowed, assessment.reason
    assert target.stamp_s == 10.
    expired_pose = m.geometry_pose(odom(80., x=3.08, y=-.32, z=.9), session_id='s')
    assert not m.assess_cached_target(target, expired_pose, measured, feedback, now=80.).allowed


def test_prelight_geometry_is_required_before_command_and_actual_feedback_for_dwell():
    m = api(); c = m.FlightMissionController('s', .055); c.start_flight(10.)
    from inspection_mission.mission import _Stage
    from inspection_mission.control import Pose2D
    stage = _Stage('AIR_INSPECT_1', 'uav', (3.08, -.32, .9, 0.), 'illuminate_reported', point_id='a1')
    c._stages = [stage]; c._stage_index = 0; c.state = stage.name
    p = Pose2D(3.08, -.32, .9, 0., 20.)
    previous = dataclasses.replace(p, stamp=19.9)
    c.aim = m.AimAssessment(True, 'ok', None, .05, 0.)
    c.geometric_aim = m.AimAssessment(False, 'bad range', None, None, None)
    assert c._illuminate_reported(20., p, previous, stage, [], True).light is False
    c.geometric_aim = m.AimAssessment(True, 'geometric', None, .05, 0.)
    assert c._illuminate_reported(20., p, previous, stage, [], False).light is True
    assert c._evidence_since is None


def install_ros(monkeypatch):
    import sys
    import types
    from types import SimpleNamespace as S
    callbacks = {}; published = []; pubs = {}; nodes = []
    class Message:
        def __init__(self):
            self.linear = S(x=0., y=0., z=0.); self.angular = S(x=0., y=0., z=0.); self.data = None
    class Node:
        def __init__(self, name): self.name = name; nodes.append(self)
        def create_publisher(self, cls, topic, qos):
            p = S(publish=lambda msg: published.append((topic, copy_msg(msg))), get_subscription_count=lambda: 1)
            pubs[topic] = p; return p
        def create_subscription(self, cls, topic, callback, qos): callbacks[topic] = callback; return callback
        def get_subscriptions_info_by_topic(self, topic):
            return [S(node_name='bridge', node_namespace='/', topic_type='geometry_msgs/msg/Twist')]
        def get_topic_names_and_types(self): return []
        def destroy_node(self): self.destroyed = True
    def copy_msg(msg): return {'xyz': (msg.linear.x, msg.linear.y, msg.linear.z), 'yaw': msg.angular.z, 'data': msg.data}
    modules = {name: types.ModuleType(name) for name in ('rclpy', 'rclpy.node', 'rclpy.qos',
        'rosidl_runtime_py', 'rosidl_runtime_py.convert', 'rosidl_runtime_py.utilities',
        'geometry_msgs', 'geometry_msgs.msg', 'std_msgs', 'std_msgs.msg')}
    modules['rclpy.node'].Node = Node
    modules['rclpy.qos'].QoSProfile = lambda **kw: S(**kw)
    modules['rclpy.qos'].ReliabilityPolicy = S(RELIABLE=1, BEST_EFFORT=2)
    modules['rclpy.qos'].DurabilityPolicy = S(VOLATILE=1, TRANSIENT_LOCAL=2)
    modules['rosidl_runtime_py.convert'].message_to_ordereddict = lambda v: v
    modules['rosidl_runtime_py.utilities'].get_message = lambda _: Message
    modules['geometry_msgs.msg'].Twist = Message
    modules['std_msgs.msg'].Bool = modules['std_msgs.msg'].String = Message
    for name, module in modules.items(): monkeypatch.setitem(sys.modules, name, module)
    return callbacks, published, pubs, nodes


def feed_node(node, wall, stamp, *, x=0., y=0., z=.055, yaw=0., cloud_points=None):
    import struct
    from test_bounded_ground_probe import scan
    from test_bounded_vertical_response import tf_static
    node.receive('/clock', {'clock': {'sec': int(stamp), 'nanosec': round((stamp-int(stamp))*1e9)}})
    p = odom(stamp, x=x, y=y, z=z)
    p['pose']['pose']['orientation'].update(z=math.sin(yaw/2), w=math.cos(yaw/2))
    node.receive('/uav/odom', p)
    p = odom(stamp, x=x, y=y, z=z, child='base_footprint')
    p['pose']['pose']['orientation'].update(z=math.sin(yaw/2), w=math.cos(yaw/2))
    node.receive('/odom', p)
    node.receive('/tf_static', tf_static())
    p = scan(stamp); p['ranges'] = [math.inf]*360; node.receive('/scan', p)
    down = scan(stamp); down['header']['frame_id'] = 'uav/laser_link'; down['ranges'] = [max(.03, z-.082)]*3
    down['range_min'] = .03
    node.receive('/uav/laser/scan', down)
    cloud = {'header': {'stamp': {'sec': int(stamp), 'nanosec': round((stamp-int(stamp))*1e9)}, 'frame_id': 'uav/lidar_link'},
        'width': 3, 'height': 1, 'point_step': 12, 'row_step': 36, 'is_bigendian': False,
        'fields': [{'name': k, 'offset': i*4, 'datatype': 7, 'count': 1} for i, k in enumerate(('x','y','z'))],
        'data': b''.join(struct.pack('<fff', *p) for p in (cloud_points or [(3., i*.1, .5) for i in range(3)]))}
    node.receive('/uav/lidar/points', cloud)
    node.receive('/uav/laser_state', {'data': False})


def test_real_factory_final_query_cannot_publish_nonzero_after_sources_age(monkeypatch):
    m = api(); _, published, pubs, _ = install_ros(monkeypatch); wall = [0.]
    node = m.create_flight_inspection_node(session_id='s', ground_return_evidence=proof(), wall_clock=lambda: wall[0])
    for i in range(13):
        wall[0] = i/10; feed_node(node, wall[0], 30.+wall[0]); node.tick()
    assert any(r[1]['xyz'][2] > 0 for r in published if r[0] == '/uav/cmd_vel')
    def late_match(): wall[0] += .6; return 1
    pubs['/uav/cmd_vel'].get_subscription_count = late_match
    start = len(published); node.tick()
    assert all(r[1]['xyz'] == (0., 0., 0.) for r in published[start:] if r[0] == '/uav/cmd_vel')
    assert not node.mission_completed
    node.close_watchdog(); node.destroy_node()


def test_factory_failure_disposes_already_created_node(monkeypatch):
    m = api(); _, published, pubs, nodes = install_ros(monkeypatch)
    Node = __import__('sys').modules['rclpy.node'].Node
    old = Node.create_subscription
    Node.create_subscription = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError('subscription refused'))
    with pytest.raises(RuntimeError, match='subscription refused'):
        m.create_flight_inspection_node(session_id='s', ground_return_evidence=proof())
    assert nodes[-1].destroyed
    assert any(t == '/uav/cmd_vel' and p['xyz'] == (0., 0., 0.) for t,p in published)


def test_complete_factory_sequence_lands_before_completion_and_sends_recognition_once(monkeypatch):
    m = api(); _, published, pubs, _ = install_ros(monkeypatch); wall = [0.]
    node = m.create_flight_inspection_node(session_id='s', ground_return_evidence=proof(), wall_clock=lambda: wall[0])
    for i in range(13):
        wall[0] = i/10; feed_node(node, wall[0], 30.+wall[0]); node.tick()
    assert node.guard.controller is not None
    landed_at = None; states = []
    for i in range(13, 800):
        wall[0] = i/10; stamp = 30.+wall[0]; c = node.guard.controller
        stage = c._stages[min(c._stage_index, len(c._stages)-1)]
        states.append(stage.name)
        feed_node(node, wall[0], stamp, x=stage.target[0], y=stage.target[1], z=stage.target[2], yaw=stage.target[3])
        if stage.name == 'LAND' and landed_at is None: landed_at = wall[0]
        if stage.kind == 'recognize':
            label = '飞鸟' if stage.window_index >= 0 else '被困人员'
            detection = {'target_id': label, 'confidence': .9, 'center': [.5,.5], 'stamp': stamp,
                'source': 'uav', 'frame_id': m.CAMERA_FRAME}
            if stage.point_id in m.AIR_CENTERS:
                from inspection_mission.target_geometry import TargetFix
                detection['target_fix'] = TargetFix(label, stage.point_id, m.AIR_CENTERS[stage.point_id],
                    'odom', 'uav/base_link', stamp, 'simulation', 's', .035, m.POSE_VERSION,
                    'K:live', m.PLANE_VERSION, m.CAMERA_VERSION)
            node.detections = [detection]
        elif stage.kind == 'illuminate_reported':
            node.receive('/uav/laser_state', {'data': True})
        result = node.tick()
        if node.guard.stop_wall is not None:
            pytest.fail(node.guard.reason)
        if node.mission_completed:
            assert result.state == 'FINISHED'
            assert wall[0]-landed_at >= 1.
            break
    assert node.mission_completed
    assert node.guard.landing_evidence['dwell_source_s'] >= 1.
    assert node.guard.landing_evidence['zero_publish_returned']
    assert all(name in states for name in ['WINDOW_1_CROSS','AIR_INSPECT_1','AIR_INSPECT_2','WINDOW_2_CROSS','LAND'])
    reports = [p['data'] for t,p in published if t == '/inspection/detections']
    assert len(reports) == 4
    completion = [i for i,(t,p) in enumerate(published) if t == '/inspection/mission_status']
    assert len(completion) == 1
    assert published[completion[0]-2][0] == '/uav/cmd_vel'
    assert published[completion[0]-2][1]['xyz'] == (0.,0.,0.)
    node.close_watchdog(); node.destroy_node()


def test_duplicate_source_cannot_refresh_wall_or_rewrite_one_exposure():
    m = api(); g = m.FlightEvidenceGuard('s', proof(), started_wall_s=0.)
    original = odom(30., z=.055)
    g.update('/uav/odom', original, wall=0.)
    g.update('/uav/odom', original, wall=.1)
    assert g.odom['wall'] == 0.
    changed = odom(30., x=.005, z=.055)
    with pytest.raises(ValueError, match='source_stamp_rewritten'):
        g.update('/uav/odom', changed, wall=.1)


def test_actual_landing_stationarity_also_requires_home_yaw():
    m = api(); g = m.FlightEvidenceGuard('s', proof(), started_wall_s=0.)
    g.home_z = .055; g.condition = lambda wall: ''
    for t in (0., 1.1):
        message = odom(30.+t, z=.055)
        message['pose']['pose']['orientation'].update(z=math.sin(.1/2), w=math.cos(.1/2))
        g.update('/uav/odom', message, wall=t)
        assert not g.landing_ready(wall=t)


def test_real_window_references_pass_the_actual_vision_job():
    import cv2
    import numpy as np
    from pathlib import Path
    from inspection_mission.vision_core import TemplateMatcher
    m = api(); references = m.load_flight_references()
    assert sum(len(paths) for paths in references.values()) == 14
    matcher = TemplateMatcher(references); matcher.labels = list(references)
    for label in m.WINDOW_LABELS.values():
        for path in references[label]:
            image = cv2.imdecode(np.frombuffer(Path(path).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            scale = min(600/image.shape[1], 600/image.shape[0])
            image = cv2.resize(image, None, fx=scale, fy=scale)
            canvas = np.full((800, 800, 3), 255, np.uint8)
            h, w = image.shape[:2]; top, left = (800-h)//2, (800-w)//2
            canvas[top:top+h, left:left+w] = image
            raw = canvas.tobytes()
            result = m._vision_job({'width': 800, 'height': 800, 'step': 2400, 'encoding': 'bgr8',
                'data': raw, 'frame_id': m.CAMERA_FRAME, 'stamp': 10., 'requested_clock_s': 10.},
                {'k': [476.7, 0., 400., 0., 476.7, 400., 0., 0., 1.]}, {},
                session_id='s', point_id='', kind='window', matcher=matcher)
            assert [d['target_id'] for d in result['detections']] == [label], Path(path).name
            assert result['detections'][0]['inlier_count'] >= 12
            assert result['source_sha256'] == m.hashlib.sha256(raw).hexdigest()
            assert result['image_jpeg'].startswith(b'\xff\xd8')


def test_report_roundtrip_keeps_real_cloud_and_fix_with_bounded_chunks(monkeypatch):
    import base64
    import gzip
    import hashlib
    import json
    import random
    from inspection_mission.telemetry import DiagnosticReporter
    from inspection_mission.target_geometry import TargetFix
    m = api(); install_ros(monkeypatch); records = []
    class Reporter:
        def emit(self, event, data):
            payload = DiagnosticReporter._encode(event, data)
            records.append((event, data, payload)); return True
        def send_now(self, event, data, **kwargs): return self.emit(event, data)
    node = m.create_flight_inspection_node(Reporter(), session_id='s', ground_return_evidence=proof(), wall_clock=lambda: 0.)
    feed_node(node, 0., 30.)
    raw_cloud = random.Random(20).randbytes(110000)
    node.last_cloud_raw = {'header': {'frame_id': 'uav/lidar_link'}, 'data': raw_cloud}
    fix = TargetFix('test', 'a1', (3.08, -.32, .012), 'odom', 'uav/base_link', 30.,
        'simulation', 's', .035, m.POSE_VERSION, 'K:live', m.PLANE_VERSION, m.CAMERA_VERSION)
    node.visual_records = [{'image_jpeg': b'\xff\xd8'+random.Random(21).randbytes(90000)+b'\xff\xd9',
        'detections': [{'target_id': 'test', 'target_fix': fix}], 'point_id': 'a1'}]
    node.report_started(900.); node.report_final(); count = len(records); node.report_final()
    assert len(records) == count and all(len(payload) < 65536 for _,_,payload in records)
    parts = [data for event,data,_ in records if event == 'probe_snapshot']
    assert len(parts) > 1
    compressed = base64.b64decode(''.join(p['base64_part'] for p in parts), validate=True)
    raw = gzip.decompress(compressed); result = json.loads(raw)
    assert len(compressed) == parts[0]['compressed_bytes'] and len(raw) == parts[0]['uncompressed_bytes']
    assert hashlib.sha256(compressed).hexdigest() == parts[0]['compressed_sha256']
    assert hashlib.sha256(raw).hexdigest() == parts[0]['uncompressed_sha256']
    assert base64.b64decode(result['last_cloud_raw']['data_b64']) == raw_cloud
    assert result['last_cloud_raw']['data_sha256'] == hashlib.sha256(raw_cloud).hexdigest()
    assert result['visual_records'][0]['detections'][0]['target_fix']['stamp_s'] == 30.
    node.close_watchdog(); node.destroy_node()


def test_vertical_current_return_sweep_rejects_an_obstacle_above_current_body(monkeypatch):
    m = api(); _, published, _, _ = install_ros(monkeypatch); wall = [0.]
    node = m.create_flight_inspection_node(session_id='s', ground_return_evidence=proof(), wall_clock=lambda: wall[0])
    feed_node(node, 0., 30., z=.9, cloud_points=[(.20, 0., .05)]*3)
    g = node.guard; g.home_z = .055; g.controller = m.FlightMissionController('s', .055)
    g.controller.start_flight(30.); g.state = g.controller.state
    assert not g.path_clear((0., 0., .03, 0.), (0., 0., 1.282, 0.), wall=0., support=True)
    node.tick()
    assert g.stop_wall is not None
    assert all(p['xyz'] == (0., 0., 0.) for t,p in published if t == '/uav/cmd_vel')
    node.close_watchdog(); node.destroy_node()


def test_final_zero_delay_cannot_send_completion_from_expired_landing_sources(monkeypatch):
    m = api(); _, published, _, _ = install_ros(monkeypatch); wall = [0.]
    node = m.create_flight_inspection_node(session_id='s', ground_return_evidence=proof(), wall_clock=lambda: wall[0])
    feed_node(node, 0., 30.)
    g = node.guard; g.home_z = .055; g.controller = m.FlightMissionController('s', .055)
    g.controller.start_flight(30.); g.controller._stage_index = len(g.controller._stages)-1
    g.controller.state = g.controller._stages[-1].name
    g.landing_anchor = (-1.1, 28.9, (0., 0., .055))
    original = node._zero
    def slow_zero():
        result = original(); wall[0] += .3; return result
    node._zero = slow_zero
    node.tick()
    assert not node.mission_completed
    assert not any(t == '/inspection/mission_status' for t,p in published)
    node._zero = original; node.close_watchdog(); node.destroy_node()


def test_empty_and_late_visual_results_remain_bounded_evidence_without_stage_leak(monkeypatch):
    from concurrent.futures import Future
    m = api(); install_ros(monkeypatch)
    node = m.create_flight_inspection_node(session_id='s', ground_return_evidence=proof(), wall_clock=lambda: 0.)
    c = node.guard.controller = m.FlightMissionController('s', .055); c.start_flight(30.)
    index = next(i for i,s in enumerate(c._stages) if s.name == 'WINDOW_1_RECOGNIZE')
    c._stage_index = index; point_id = c._stages[index].point_id
    for stamp in range(5):
        future = Future(); future.set_result({'point_id': point_id, 'stamp_s': stamp, 'detections': [],
            'image_jpeg': b'actual-worker-result', 'camera_info': {'k': [1.]*9}, 'prior_odom': {'stamp': stamp}})
        node.vision_future = future; node.vision_stage = (index, point_id); node._poll_vision()
    assert len(node.visual_records) == 3
    assert [r['stamp_s'] for r in node.visual_records] == [0, 1, 4]
    assert all(r['detections'] == [] for r in node.visual_records)
    sentinel = [{'target_id': 'current-stage-only'}]; node.detections = sentinel
    c._stage_index += 1
    future = Future(); future.set_result({'point_id': point_id, 'stamp_s': 5,
        'detections': [{'target_id': 'late-result'}], 'image_jpeg': b'actual-worker-result'})
    node.vision_future = future; node.vision_stage = (index, point_id); node._poll_vision()
    assert node.detections is sentinel
    assert len(node.visual_records) == 3
    assert node.visual_records[-1]['adopted_for_current_stage'] is False
    assert node.visual_records[-1]['original_stage'] == (index, point_id)
    node.close_watchdog(); node.destroy_node()


def test_matching_exception_keeps_real_exposure_and_calibration():
    m = api()
    class BrokenMatcher:
        labels = list(m.WINDOW_LABELS.values())
        def detect(self, image): raise RuntimeError('synthetic matcher failure')
    frame = {'width': 20, 'height': 20, 'step': 60, 'encoding': 'bgr8', 'data': bytes(1200),
        'frame_id': m.CAMERA_FRAME, 'stamp': 30., 'requested_clock_s': 30.}
    calibration = {'k': [1.]*9}; prior = {'original': 'exposure-pose'}
    result = m._vision_job(frame, calibration, prior, session_id='s', point_id='w1', kind='window', matcher=BrokenMatcher())
    assert result['detections'] == [] and 'synthetic matcher failure' in result['visual_error']
    assert result['camera_info'] == calibration and result['prior_odom'] == prior
    assert result['source_sha256'] == m.hashlib.sha256(frame['data']).hexdigest()
    assert result['image_jpeg'].startswith(b'\xff\xd8')


def test_real_floor_ray_error_budget_can_illuminate_after_observation_to_hover_travel():
    from inspection_mission.target_geometry import StampedRange, StampedBool
    m = api()
    p = odom(10., x=3.08, y=-2.15, z=.9)
    p['pose']['pose']['orientation'].update(z=math.sin(math.pi/4), w=math.cos(math.pi/4))
    K = [476.7, 0., 400., 0., 476.7, 400., 0., 0., 1.]
    fix = m.localize_floor_pixel([400., 400.+476.7*1.018/1.44], K, p, image_stamp=10.,
        camera_frame=m.CAMERA_FRAME, session_id='s', target_id='test', point_id='a1', now=10.)
    assert fix.position_error_m > .05  # Preserve the actual ray's error bound.
    current = m.geometry_pose(odom(30., x=3.08, y=-.32, z=.9), session_id='s')
    measured = StampedRange(m.range_extrinsic(), (1., 0., 0.), .818, .03, 5., .005, 30., 'simulation', 's')
    feedback = StampedBool(True, 30., 'simulation', 's', m.FEEDBACK_VERSION)
    assessment = m.assess_cached_target(fix, current, measured, feedback, now=30.)
    assert assessment.allowed, assessment.reason
    assert assessment.total_error_m >= fix.position_error_m
    assert fix.stamp_s == 10. and fix.position_world_m == pytest.approx((3.08, -.32, .012))
    assert not m.assess_cached_target(dataclasses.replace(fix, position_error_m=.11), current,
        measured, feedback, now=30.).allowed
    assert not m.assess_cached_target(fix, current, dataclasses.replace(measured, distance_m=1.05),
        feedback, now=30.).allowed
    assert not m.assess_cached_target(fix, current, measured, feedback, now=71.).allowed
    off_center = m.geometry_pose(odom(30., x=3.18, y=-.32, z=.9), session_id='s')
    assert not m.assess_cached_target(fix, off_center, measured, feedback, now=30.).allowed
    assert not m.assess_cached_target(dataclasses.replace(fix, position_world_m=(3.08, -.32, .04)),
        current, measured, feedback, now=30.).allowed


def test_aerial_worker_uses_floor_ipm_and_preserves_original_exposure_fix():
    import cv2
    import numpy as np
    from pathlib import Path
    from types import SimpleNamespace
    from inspection_mission.floor_plane_detector import FloorPlaneDetector
    from inspection_mission.ground_template_catalog import load_complete_ground_templates
    from test_floor_plane_detector import observation, synthetic
    m = api(); references = load_complete_ground_templates()
    image = cv2.imdecode(np.frombuffer(Path(references[0].path).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    bgr, _ = synthetic(image); calibration, prior = observation()
    matcher = SimpleNamespace(labels=list(m.load_flight_references()), floor_detector=FloorPlaneDetector())
    frame = {'width': 800, 'height': 800, 'step': 2400, 'encoding': 'bgr8', 'data': bgr.tobytes(),
        'frame_id': m.CAMERA_FRAME, 'stamp': 30., 'requested_clock_s': 30.}
    result = m._vision_job(frame, calibration, prior, session_id='s', point_id='a1', kind='aerial', matcher=matcher)
    assert result['visual_error'] is None and len(result['detections']) == 1
    detection = result['detections'][0]
    assert detection['target_id'] == references[0].label
    assert detection['stamp'] == detection['target_fix'].stamp_s == 30.
    assert detection['target_fix'].position_error_m > .05
    assert detection['target_fix'].position_world_m == pytest.approx((3.08, -.32, .012), abs=.025)
    assert result['detector_diagnostics']
    frame['data'] = bytes([170])*1920000
    assert m._vision_job(frame, calibration, prior, session_id='s', point_id='a1', kind='aerial', matcher=matcher)['detections'] == []
