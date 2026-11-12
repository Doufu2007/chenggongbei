"""Flight continuation for the explicitly declared public static ICAN scene.

The static model supplies the swept volume outside the front lidar aperture;
current returns can veto it. This is not unknown-space obstacle avoidance.
Classes always come from this run's images. A cached exposure fix, never a
retimestamped front image, supports the separate downlight hover stage.
"""
from __future__ import annotations

import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
from dataclasses import asdict, replace
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
import threading
import time

from .bounded_ground_probe import MotionDecision, finite
from .bounded_height_hold import feedback_command
from .bounded_vertical_response import STATIC_BOXES, WORLD_SHA256, WORLD_SOURCE, _pose, _stamp, _safe
from .control import Pose2D, angle_error
from .mission import MissionController, MissionCommand, _Stage
from .official_protocol import recognition_phrase
from .target_geometry import (RigidTransform, StampedPose3D, CameraObservation,
    LocalizationLimits, LightExtrinsics, StampedRange, StampedBool,
    IlluminationLimits, TargetFix, localize_target, hover_for_downlight,
    assess_illumination, AimAssessment, _rotate)

WORLD_FRAME = 'odom'
BODY_FRAME = 'uav/base_link'
CAMERA_FRAME = 'uav/camera_link'
# Public world/launch static-TF configuration; v34 synchronized body origins
# differ by <=.0005m. Camera TF was not observed in that run, so this remains
# an explicit configuration candidate. Plane z=.012 is applied only once.
CAMERA_OFFSET = (.390, 0., .130)
POSE_VERSION = 'raw_uav_odom:public_static_bound_0.01m_0.003rad'
CAMERA_VERSION = 'public_world:' + WORLD_SHA256 + ':camera_optical'
PLANE_VERSION = 'public_static_horizontal_board_top_z_0.012'
PLANE_HEIGHT_ERROR_M = .005  # Public static-plane engineering assumption.
RANGE_VERSION = 'observed_tf:uav/base_link->uav/laser_link'
LIGHT_VERSION = 'public_static_downlight_axis'
FEEDBACK_VERSION = 'laser_state:arrival_clock_bound_0.5s'
OPTICAL_Q = (-.5, .5, -.5, .5)
RADIUS = .447 + .166*math.sin(.05) + .02 + .01
BELOW = .068 + .447*math.sin(.05) + .02
ABOVE = .166 + .447*math.sin(.05) + .02
AIR_CENTERS = {'a1': (3.08, -.32, .012), 'a2': (3.08, -1.355, .012)}
WINDOW_LABELS = {'window_bird': '飞鸟', 'window_target': '救援无人机'}


def validate_ground_return(proof):
    if not isinstance(proof, dict):
        raise ValueError('actual ground return evidence required')
    try:
        pose = proof['pose']; zero = proof['zero_publish_attempt']
        values = [proof[k] for k in ('wall_s', 'odom_stamp_s', 'stationary_wall_s',
            'stationary_source_s', 'linear_speed_mps', 'angular_speed_rps')]
        good = (len(pose) == 3 and all(finite(v) for v in (*pose, *values))
            and math.hypot(*pose[:2]) <= .02 and abs(angle_error(0., pose[2])) <= .025
            and min(proof['stationary_wall_s'], proof['stationary_source_s']) >= 1.
            and 0 <= proof['linear_speed_mps'] <= .01 and 0 <= proof['angular_speed_rps'] <= .03
            and zero.get('returned') is True and zero.get('error') is None
            and zero.get('linear_x') == zero.get('angular_z') == 0.
            and zero.get('phase') == zero.get('state') == 'RETURN_SETTLE'
            and finite(zero.get('wall_elapsed_s')) and finite(zero.get('returned_wall_elapsed_s'))
            and zero['returned_wall_elapsed_s'] >= zero['wall_elapsed_s'])
    except (KeyError, TypeError):
        good = False
    if not good:
        raise ValueError('incomplete actual ground return/zero/stationarity contract')
    return copy.deepcopy(proof)


def static_swept_clear(start, end, *, support_floor=False):
    """Conservative disk × height envelope, sampled at <=5mm along a segment.

    Extra 5mm expansion covers the sampling interval. Floor contact is allowed
    only in the initial/home support column, never a general floor exception.
    """
    if len(start) != 3 or len(end) != 3 or not all(finite(v) for v in (*start, *end)):
        return False
    if min(start[2], end[2]) < .03 or max(start[2], end[2]) > 1.55:
        return False
    if min(start[2], end[2])-BELOW < 0 and not (support_floor
            and max(math.hypot(*start[:2]), math.hypot(*end[:2])) <= .04):
        return False
    n = max(1, math.ceil(math.dist(start, end)/.005))
    for i in range(n+1):
        p = tuple(a+(b-a)*i/n for a, b in zip(start, end))
        for _, low, high in STATIC_BOXES:
            if high[2] < p[2]-BELOW-.005 or low[2] > p[2]+ABOVE+.005:
                continue
            dx = max(low[0]-p[0], 0., p[0]-high[0])
            dy = max(low[1]-p[1], 0., p[1]-high[1])
            if math.hypot(dx, dy) <= RADIUS+.005:
                return False
    return True


def geometry_pose(payload, *, session_id):
    sample = _pose(payload, BODY_FRAME)
    return StampedPose3D(RigidTransform(WORLD_FRAME, BODY_FRAME, sample['xyz'], sample['q'],
        POSE_VERSION, .01, .003), sample['stamp'], 'simulation', session_id, .20, .30)


def camera_extrinsic():
    return RigidTransform(BODY_FRAME, CAMERA_FRAME, CAMERA_OFFSET, OPTICAL_Q, CAMERA_VERSION, .01, .003)


def range_extrinsic():
    return RigidTransform(BODY_FRAME, 'uav/laser_link', (0., 0., -.07),
        (0., math.sqrt(.5), 0., math.sqrt(.5)), RANGE_VERSION, .005, .002)


def light_extrinsic():
    return LightExtrinsics(RigidTransform(BODY_FRAME, 'downlight', (0., 0., -.068),
        (0., 0., 0., 1.), LIGHT_VERSION, .005, .002), (0., 0., -1.))


def illumination_limits():
    return IlluminationLimits(WORLD_FRAME, BODY_FRAME, POSE_VERSION, LIGHT_VERSION, RANGE_VERSION,
        FEEDBACK_VERSION, 60., .20, .30, .50, .12, .10, .12, .145, .12, .10,
        .80, 1.5, .0002)


def assess_cached_target(target, pose, measured, feedback, *, now):
    """Keep ray XY uncertainty separate from the independently fixed plane Z.

    The original TargetFix/stamp/position are never altered. Generic geometry
    treats its scalar error as uncertain plane height too, which double counts
    this module's lateral ray error. Validate all strict geometry with the
    explicit public-plane height bound, then add the complete original lateral
    bound and cache drift to the final spot error and keep the same hit limits.
    """
    limits = illumination_limits()
    try:
        if (not isinstance(target, TargetFix) or target.size_version != PLANE_VERSION
                or abs(target.position_world_m[2]-.012) > 1e-8):
            raise ValueError('horizontal_public_plane_provenance_required')
        if not finite(target.position_error_m) or target.position_error_m < 0:
            raise ValueError('invalid_original_lateral_error')
        drift = limits.localization_drift_mps*max(0., now-target.stamp_s,
            pose.stamp_s-target.stamp_s, measured.stamp_s-target.stamp_s)
        lateral = target.position_error_m+drift
        if not finite(lateral) or lateral > limits.max_target_error_m:
            raise ValueError('original_lateral_error_exceeded')
        plane_bound = replace(target, position_error_m=PLANE_HEIGHT_ERROR_M)
        base = assess_illumination(plane_bound, pose, light_extrinsic(), measured, feedback,
            replace(limits, localization_drift_mps=0.), now_s=now)
        if not base.allowed:
            return base
        total = base.total_error_m+lateral
        if total > limits.max_total_error_m:
            raise ValueError('horizontal_plane_total_error_exceeded')
        if math.dist(base.spot_world_m, target.position_world_m)+total > limits.hit_radius_m:
            raise ValueError('horizontal_plane_miss_distance_exceeded')
        return AimAssessment(True, 'ok_horizontal_plane_lateral_error_split', base.spot_world_m,
            total, base.range_residual_m)
    except (AttributeError, TypeError, ValueError) as exc:
        return AimAssessment(False, str(exc), None, None, None)


def prelight_geometry(target, pose, measured, *, now):
    """Geometric permission before lamp-on; it never supplies a feedback Bool."""
    try:
        if not isinstance(target, TargetFix) or pose.epoch_id != target.epoch_id or measured.epoch_id != target.epoch_id:
            raise ValueError('target_session_missing')
        if not (0 <= now-target.stamp_s <= 60. and -.12 <= now-pose.stamp_s <= .20
                and -.12 <= now-measured.stamp_s <= .30):
            raise ValueError('prelight_source_stale')
        p = pose.transform.translation_m
        light = light_extrinsic()
        origin = tuple(a+b for a, b in zip(p, _rotate(pose.transform.rotation_xyzw, light.transform.translation_m)))
        direction = _rotate(pose.transform.rotation_xyzw, light.direction_light)
        if direction[2] >= -math.cos(.05) or p[2]-target.position_world_m[2] < .8:
            raise ValueError('prelight_height_or_tilt')
        distance = (target.position_world_m[2]-origin[2])/direction[2]
        spot = tuple(a+distance*b for a, b in zip(origin, direction))
        error = target.position_error_m + .0002*(now-target.stamp_s) + .025 + distance*.007
        if error > .12 or math.dist(spot, target.position_world_m)+error > .145:
            raise ValueError('prelight_miss_distance')
        expected = p[2]-.07-target.position_world_m[2]
        if not measured.range_min_m <= measured.distance_m <= measured.range_max_m:
            raise ValueError('prelight_range_invalid')
        if abs(measured.distance_m-expected)+error+.005 > .12:
            raise ValueError('prelight_range_plane_mismatch')
        return AimAssessment(True, 'geometry_only_waiting_actual_feedback', spot, error,
            abs(measured.distance_m-expected))
    except (AttributeError, TypeError, ValueError) as exc:
        return AimAssessment(False, str(exc), None, None, None)


def localize_floor_pixel(pixel, K, prior, *, image_stamp, camera_frame,
                         session_id, target_id, point_id, now):
    if camera_frame != CAMERA_FRAME or len(K) != 9 or not all(finite(v) for v in K):
        raise ValueError('actual optical frame and pinhole K required')
    if K[0] <= 0 or K[4] <= 0 or K[6:] != [0., 0., 1.] and tuple(K[6:]) != (0., 0., 1.):
        raise ValueError('invalid actual K')
    pose = geometry_pose(prior, session_id=session_id)
    if not 0 <= image_stamp-pose.stamp_s <= .05+1e-9:
        raise ValueError('no bounded preceding exposure pose')
    ray = ((pixel[0]-K[2])/K[0], (pixel[1]-K[5])/K[4], 1.)
    origin = tuple(a+b for a, b in zip(pose.transform.translation_m,
        _rotate(pose.transform.rotation_xyzw, CAMERA_OFFSET)))
    direction = _rotate(pose.transform.rotation_xyzw, _rotate(OPTICAL_Q, ray))
    if direction[2] >= -1e-6:
        raise ValueError('floor ray not downward')
    depth = (.012-origin[2])/direction[2]
    if not 0 < depth <= 5.:
        raise ValueError('floor ray outside bounded range')
    observation = CameraObservation(target_id, point_id, tuple(depth*v for v in ray), CAMERA_FRAME,
        'opencv_optical', image_stamp, 'simulation', session_id, 'K:live', PLANE_VERSION, .025)
    limits = LocalizationLimits(WORLD_FRAME, BODY_FRAME, CAMERA_FRAME, POSE_VERSION, CAMERA_VERSION,
        'K:live', PLANE_VERSION, .05, .5, .55, .10, .12)
    return localize_target(observation, camera_extrinsic(), pose, limits, now_s=now)


def flight_z_request(target_z, z, *, landed=False):
    return 0. if landed and z <= target_z+.025 else feedback_command(target_z, z)


def candidate_plan(home_z):
    return {'protocol': 'inspection_v1', 'home_ground': [0., 0., 0.],
        'home_uav': [0., 0., home_z, 0.], 'robot_radius_m': .447, 'clearance_m': .02,
        'uav_body_envelope': {'below_origin_m': .068, 'above_origin_m': .166, 'max_tilt_rad': .05},
        'ground_points': [{'id': 'ground_'+str(i), 'target_id': '*', 'pose': [0., -i*.4, 0.],
            'approach': [-.3, -i*.4, 0.], 'via': []} for i in range(3)],
        'aerial_points': [
            {'id': 'a1', 'target_id': '*', 'pose': [3.08, -2.15, .9, math.pi/2], 'via': []},
            {'id': 'a2', 'target_id': '*', 'pose': [3.08, .45, .9, -math.pi/2], 'via': []}],
        'windows': [
            {'id': 'w1', 'target_id': '*', 'approach': [1.05, -.3275, 1.282, 0.],
                'exit': [2.65, -.3275, .75, 0.], 'crossing_z': .75, 'bottom': .4, 'top': 1.15, 'width_m': 1., 'via': []},
            {'id': 'w2', 'target_id': '*', 'approach': [2.75, -2.3725, 1.282, math.pi],
                'exit': [1.2, -2.3725, .75, math.pi], 'crossing_z': .75, 'bottom': .4, 'top': 1.15, 'width_m': 1., 'via': []}],
        'home_ground_via': [], 'home_uav_via': [[.6, -2.3725, 1.282, math.pi], [.6, 0., 1.282, 0.]]}


class FlightMissionController(MissionController):
    """Reuse task semantics while separating floor observation and light pose."""
    def __init__(self, session_id, home_z):
        super().__init__(candidate_plan(home_z), {'session_id': session_id,
            'recognition_confirmation_mode': 'transport_only', 'recognition_transport_wait_s': .2,
            'position_tolerance_m': .01, 'yaw_tolerance_rad': .025, 'window_z_tolerance_m': .015,
            'landing_z_tolerance_m': .005, 'max_uav_speed_mps': .12, 'max_uav_yaw_rate_rps': .20,
            'stable_speed_mps': .02, 'stable_yaw_rate_rps': .03, 'stable_radius_m': .015,
            'phase_timeout_s': 100., 'mission_timeout_s': 900., 'no_progress_timeout_s': 10.,
            'detection_timeout_s': .50, 'pose_timeout_s': .20, 'future_tolerance_s': .12,
            'minimum_confidence': .55, 'center_tolerance': .48, 'recognition_stable_s': .5})
        self.fixes = {}; self.aim = None; self.geometric_aim = None; self.stage_log = []

    def start_flight(self, now):
        # Build the reused internal stage representation, then discard ground
        # stages. This does not execute or acknowledge any ground task.
        self._build_stages([])
        self._stages = [s for s in self._stages if s.vehicle == 'uav']
        staged = []
        for stage in self._stages:
            if stage.name.startswith('AIR_INSPECT_'):
                i = stage.name.rsplit('_', 1)[1]
                staged.append(replace(stage, name='AIR_MOVE_TO_LIGHT_'+i, kind='navigate'))
            staged.append(stage)
        self._stages = staged
        self._started = self._phase_started = now
        return self._advance(now)

    def _advance(self, now):
        command = super()._advance(now)
        self.stage_log.append({'stamp_s': now, 'state': self.state})
        if self._stage_index < len(self._stages) and self.state.startswith('AIR_MOVE_TO_LIGHT_'):
            stage = self._stages[self._stage_index]
            fix = self.fixes.get(stage.point_id)
            if fix is None:
                return self._fail('aerial_world_fix_missing')
            yaw = stage.target[3]
            solution = hover_for_downlight(fix, light_extrinsic(), (0., 0., math.sin(yaw/2), math.cos(yaw/2)),
                .9, illumination_limits())
            target = (*solution.position_world_m, yaw)
            self._stages[self._stage_index] = replace(stage, target=target)
            self._stages[self._stage_index+1] = replace(self._stages[self._stage_index+1], target=target)
        return command

    def _inspect(self, now, pose, previous, stage, detections, light_ready, acknowledgements):
        if stage.point_id in AIR_CENTERS:
            usable = [d for d in detections if isinstance(d.get('target_fix'), TargetFix)
                and d['target_fix'].point_id == stage.point_id and d['target_fix'].target_id == d.get('target_id')]
            detection = self._target_detection(usable, stage, now)
            if detection is None:
                return super()._inspect(now, pose, previous, stage, [], light_ready, acknowledgements)
            self.fixes[stage.point_id] = detection['target_fix']
            detections = usable
        return super()._inspect(now, pose, previous, stage, detections, light_ready, acknowledgements)

    def _illuminate_reported(self, now, pose, previous, stage, detections, light_ready):
        # No front-image requirement at the downward hover pose. Actual pose,
        # independent downward range and lamp feedback assess the original fix.
        if not self._reached(pose, stage) or not self._stable(pose, previous, now):
            self._evidence_since = None
            return self._command(uav=self._velocity(pose, stage))
        if self.geometric_aim is None or not self.geometric_aim.allowed:
            self._evidence_since = None
            return self._command(light=False)
        if self._light_command_started_at is None:
            self._light_command_started_at = now
        elapsed = now-self._light_command_started_at
        if elapsed >= 3.:
            return self._fail('illumination_time_limit')
        good = self.aim is not None and self.aim.allowed and light_ready
        if not good:
            self._evidence_since = None
            return self._command(light=True)
        if self._evidence_since is None:
            self._evidence_since = now
        if now-self._evidence_since >= 2.:
            return self._advance(now)
        return self._command(light=True)


def decode_cloud(payload):
    """Decode the complete declared PointCloud2 rows, including row padding."""
    source = _stamp(payload, 'uav/lidar_link')
    width, height, step, row = (payload[k] for k in ('width', 'height', 'point_step', 'row_step'))
    if not all(type(v) is int for v in (width, height, step, row)) or not (0 < width <= 30000 and 0 < height <= 100 and 12 <= step <= 128 and row >= width*step):
        raise ValueError('cloud_layout')
    raw = bytes(payload['data'])
    if len(raw) != row*height or len(raw) > 2*1024*1024:
        raise ValueError('cloud_byte_length')
    offsets = {}
    for field in payload['fields']:
        if field['name'] in ('x', 'y', 'z'):
            if field['datatype'] != 7 or field['count'] != 1 or not 0 <= field['offset'] <= step-4:
                raise ValueError('cloud_xyz_field')
            offsets[field['name']] = field['offset']
    if set(offsets) != {'x', 'y', 'z'}:
        raise ValueError('cloud_xyz_missing')
    endian = '>' if payload['is_bigendian'] else '<'
    points = []
    for r in range(height):
        for c in range(width):
            xyz = tuple(struct.unpack_from(endian+'f', raw, r*row+c*step+offsets[k])[0] for k in ('x', 'y', 'z'))
            if all(finite(v) for v in xyz):
                points.append(xyz)
    if len(points) < 3:
        raise ValueError('cloud_no_finite_returns')
    return source, points


class FlightEvidenceGuard:
    """Current source gates and original evidence, independent of ROS calls."""
    def __init__(self, session_id, proof, *, started_wall_s):
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError('session required')
        self.session_id = session_id; self.ground_return = validate_ground_return(proof)
        self.started_wall = started_wall_s; self.state = 'WAIT'; self.reason = 'waiting_actual_flight_sources'
        self.clock = self.odom = self.ground = self.scan = self.cloud = self.down = self.feedback = None
        self.camera_info = None; self.laser_tf = None; self.last_stamp = {}; self.last_content = {}
        self.history = deque(maxlen=160); self.trajectory = deque(maxlen=2400)
        self.commands = deque(maxlen=2400); self.safety_checks = deque(maxlen=800)
        self.events = deque(maxlen=160); self.tf_records = deque(maxlen=64)
        self.total_path = 0.; self.last_pose = None; self.landing_anchor = None
        self.landing_evidence = None; self.mission_completed = False
        self.controller = None; self.home_z = None; self.stop_wall = None
        self.dropped = {'trajectory': 0, 'commands': 0, 'safety_checks': 0, 'events': 0}

    def retain(self, name, row):
        sequence = getattr(self, name)
        if len(sequence) == sequence.maxlen:
            self.dropped[name] = self.dropped.get(name, 0)+1
        sequence.append(row)

    def stop(self, reason, wall):
        if self.mission_completed:
            return
        if self.stop_wall is None:
            self.stop_wall = wall; self.reason = str(reason); self.state = 'STOP'

    def prior(self, source):
        return next((row for row in reversed(self.history) if 0 <= source-row['stamp'] <= .05+1e-9), None)

    def update(self, topic, payload, *, wall):
        if topic == '/clock':
            c = payload['clock']; stamp = c['sec']+c['nanosec']/1e9
            if type(c['sec']) is not int or type(c['nanosec']) is not int or c['sec'] < 0 or not 0 <= c['nanosec'] < 10**9:
                raise ValueError('clock_invalid')
            if self.clock and stamp < self.clock['stamp']:
                raise ValueError('clock_reversed')
            if self.clock is None or stamp > self.clock['stamp']:
                self.clock = {'stamp': stamp, 'wall': wall}
            return
        if topic in ('/uav/odom', '/odom'):
            sample = _pose(payload, BODY_FRAME if topic == '/uav/odom' else 'base_footprint')
            if not self._source(topic, sample['stamp'], sample):
                return
            sample.update(wall=wall, message=copy.deepcopy(payload))
            if topic == '/odom':
                self.ground = sample; return
            if sample['tilt'] > .05 or sample['speed'] > .25 or sample['angular_speed'] > .40 or not .025 <= sample['xyz'][2] <= 1.55:
                raise ValueError('actual_pose_motion_bound')
            if self.last_pose is not None:
                self.total_path += math.dist(sample['xyz'], self.last_pose)
            self.last_pose = sample['xyz']; self.odom = sample; self.history.append(sample)
            if not self.trajectory or sample['stamp']-self.trajectory[-1]['stamp'] >= .10-1e-9:
                self.retain('trajectory', {k: v for k, v in sample.items() if k != 'message'})
            return
        if topic in ('/tf', '/tf_static'):
            self.tf_records.append(copy.deepcopy(payload))
            for transform in payload['transforms']:
                if transform['header']['frame_id'] == BODY_FRAME and transform['child_frame_id'] == 'uav/laser_link':
                    translation = transform['transform']['translation']; q = transform['transform']['rotation']
                    if math.dist(tuple(translation[k] for k in ('x', 'y', 'z')), (0., 0., -.07)) > .005:
                        raise ValueError('downward_tf_translation_mismatch')
                    direction = _rotate(tuple(q[k] for k in ('x', 'y', 'z', 'w')), (1., 0., 0.))
                    if direction[2] > -.999:
                        raise ValueError('downward_tf_direction_mismatch')
                    self.laser_tf = copy.deepcopy(transform)
            return
        if topic == '/uav/camera/camera_info':
            if payload['header']['frame_id'] != CAMERA_FRAME or payload['width'] <= 0 or payload['height'] <= 0:
                raise ValueError('camera_info_frame_dimensions')
            K = list(payload['k'])
            if len(K) != 9 or not all(finite(v) for v in K) or K[0] <= 0 or K[4] <= 0 or any(abs(v) > 1e-8 for v in payload.get('d', [])):
                raise ValueError('camera_info_K_distortion')
            self.camera_info = {'message': copy.deepcopy(payload), 'wall': wall}; return
        if topic == '/uav/laser_state':
            if type(payload['data']) is not bool:
                raise ValueError('laser_feedback_bool')
            self.feedback = {'value': payload['data'], 'wall': wall,
                'stamp': self.clock['stamp'] if self.clock else None}; return
        if topic == '/uav/lidar/points':
            stamp, points = decode_cloud(payload)
            if not self._source(topic, stamp, payload):
                return
            self.cloud = {'stamp': stamp, 'wall': wall, 'points': points, 'prior': self.prior(stamp),
                'layout': {k: copy.deepcopy(payload[k]) for k in ('header', 'width', 'height', 'point_step', 'row_step', 'fields')}}
            return
        if topic in ('/scan', '/uav/laser/scan'):
            stamp = _stamp(payload, 'laser_link' if topic == '/scan' else 'uav/laser_link')
            if not self._source(topic, stamp, payload):
                return
            ranges = payload['ranges']; low, high = payload['range_min'], payload['range_max']
            if not 0 < low < high or not isinstance(ranges, (list, tuple)) or not ranges:
                raise ValueError('scan_layout')
            if any(not (finite(v) and low <= v <= high or type(v) in (int, float) and v == math.inf) for v in ranges):
                raise ValueError('scan_invalid_values')
            row = {'stamp': stamp, 'wall': wall, 'message': copy.deepcopy(payload), 'prior': self.prior(stamp)}
            if topic == '/scan':
                if len(ranges) < 300 or abs(payload['angle_increment'])*len(ranges) < 6.27:
                    raise ValueError('scan_ring_coverage')
                self.scan = row
            else:
                if len(ranges) != 3:
                    raise ValueError('downscan_three_beams')
                self.down = row

    def _source(self, topic, stamp, content=None):
        if topic in self.last_stamp and stamp < self.last_stamp[topic]:
            raise ValueError('source_stamp_reversed:'+topic)
        if topic in self.last_stamp and stamp == self.last_stamp[topic]:
            if content is not None and content != self.last_content.get(topic):
                raise ValueError('source_stamp_rewritten:'+topic)
            return False
        self.last_stamp[topic] = stamp
        self.last_content[topic] = copy.deepcopy(content)
        return True

    def fresh(self, row, wall, age):
        return (row is not None and self.clock is not None and 0 <= wall-row['wall'] <= age
            and -.12-1e-9 <= self.clock['stamp']-row['stamp'] <= age)

    def condition(self, wall):
        if wall-self.started_wall >= 900.:
            return 'flight_global_wall_limit'
        if self.total_path >= 30.:
            return 'flight_total_path_limit'
        if self.clock is None or not 0 <= wall-self.clock['wall'] <= .20:
            return 'flight_clock_stale'
        for name, age in (('odom', .20), ('scan', .35), ('cloud', .5), ('down', .30)):
            if not self.fresh(getattr(self, name), wall, age):
                return 'flight_'+name+'_stale'
        if self.cloud['prior'] is None or self.scan['prior'] is None:
            return 'flight_obstacle_prior_pose_missing'
        if self.laser_tf is None:
            return 'flight_actual_downward_tf_missing'
        return ''

    def measured_range(self):
        if self.down is None:
            return None
        p = self.down['message']; distance = p['ranges'][1]
        if not finite(distance):
            return None
        return StampedRange(range_extrinsic(), (1., 0., 0.), distance, p['range_min'], p['range_max'],
            .005, self.down['stamp'], 'simulation', self.session_id)

    def actual_feedback(self):
        if self.feedback is None or self.feedback['stamp'] is None:
            return None
        return StampedBool(self.feedback['value'], self.feedback['stamp'], 'simulation', self.session_id, FEEDBACK_VERSION)

    def path_clear(self, velocity, target, *, wall, support=False):
        """Check public static volume and all measured current-return intrusions."""
        if self.condition(wall):
            return False
        start = self.odom['xyz']
        # 1s bounded lookahead plus .02m response allowance at <=.12m/s.
        end = tuple(a+v*1. for a, v in zip(start, velocity[:3]))
        if not static_swept_clear(start, end, support_floor=support):
            return False
        if not static_swept_clear(start, target[:3], support_floor=support):
            return False
        from .perception import pointcloud_sensor_envelope_path_clear, yaw_level_rotation
        from .envelope import uav_envelope_from_plan
        yaw = math.atan2(2*(self.odom['q'][3]*self.odom['q'][2]+self.odom['q'][0]*self.odom['q'][1]),
            1-2*(self.odom['q'][1]**2+self.odom['q'][2]**2))
        vx, vy, vz = velocity[:3]; bx = math.cos(yaw)*vx+math.sin(yaw)*vy; by = -math.sin(yaw)*vx+math.cos(yaw)*vy
        spec = {'translation_m': [.12, 0., .18], 'rotation_rpy_rad': [0., 0., 0.],
            'horizontal_fov_rad': math.pi, 'vertical_fov_rad': 1.1, 'range_min_m': .08, 'range_max_m': 4.}
        fov_visible = False
        try:
            fov_visible = pointcloud_sensor_envelope_path_clear(self.cloud['points'], (bx, by, vz), spec,
                stop_distance=RADIUS+math.hypot(vx, vy, vz)*.5+.02, corridor_radius=RADIUS,
                envelope=uav_envelope_from_plan(candidate_plan(self.home_z or .055)),
                body_to_level=yaw_level_rotation(self.odom['q'], yaw)) if not velocity[3] else False
        except (KeyError, ValueError, TypeError):
            pass
        obstacles = []
        prior = self.cloud['prior']
        for point in self.cloud['points']:
            body = (point[0]+.12, point[1], point[2]+.18)
            rotated = _rotate(prior['q'], body)
            obstacles.append(tuple(a+b for a, b in zip(prior['xyz'], rotated)))
        scan = self.scan['message']; prior = self.scan['prior']
        for i, distance in enumerate(scan['ranges']):
            if not finite(distance):
                continue
            angle = scan['angle_min']+i*scan['angle_increment']
            rotated = _rotate(prior['q'], (.08+distance*math.cos(angle), distance*math.sin(angle), .08))
            obstacles.append(tuple(a+b for a, b in zip(prior['xyz'], rotated)))
        dx, dy, dz = (b-a for a, b in zip(start, end)); norm = dx*dx+dy*dy
        for point in obstacles:
            if support and point[2] <= .025:
                continue
            # Intersect the time interval of vertical envelope contact with
            # the horizontal cylinder sweep. An XY-only closest point misses
            # obstacles reached by pure vertical or combined XYZ motion.
            low_z, high_z = point[2]-ABOVE-.01, point[2]+BELOW+.01
            if abs(dz) < 1e-12:
                if not low_z <= start[2] <= high_z:
                    continue
                low_t, high_t = 0., 1.
            else:
                bounds = sorted(((low_z-start[2])/dz, (high_z-start[2])/dz))
                low_t, high_t = max(0., bounds[0]), min(1., bounds[1])
                if low_t > high_t:
                    continue
            closest = ((point[0]-start[0])*dx+(point[1]-start[1])*dy)/norm if norm else low_t
            alpha = max(low_t, min(high_t, closest))
            center = tuple(a+(b-a)*alpha for a, b in zip(start, end))
            if (center[2]-BELOW-.01 <= point[2] <= center[2]+ABOVE+.01
                    and math.hypot(point[0]-center[0], point[1]-center[1]) <= RADIUS):
                return False
        self.retain('safety_checks', {'wall_elapsed_s': wall-self.started_wall, 'source_stamp_s': self.clock['stamp'],
            'state': self.state, 'front_fov_measured_corridor_clear': bool(fov_visible),
            'static_blind_volume_source': WORLD_SHA256, 'static_sweep_clear': True,
            'measured_cloud_points': len(self.cloud['points']), 'scan_stamp_s': self.scan['stamp'],
            'cloud_stamp_s': self.cloud['stamp'], 'cloud_prior_skew_s': self.cloud['stamp']-self.cloud['prior']['stamp']})
        return True

    def landing_ready(self, *, wall):
        if self.odom is None or self.home_z is None or self.condition(wall):
            self.landing_anchor = None; return False
        p = self.odom
        q = p['q']
        yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
        good = (math.hypot(*p['xyz'][:2]) <= .02 and abs(p['xyz'][2]-self.home_z) <= .005
            and abs(angle_error(0., yaw)) <= .025 and p['speed'] <= .01 and p['angular_speed'] <= .03)
        if not good:
            self.landing_anchor = None; return False
        if self.landing_anchor is None:
            self.landing_anchor = (wall, p['stamp'], p['xyz'])
        anchor = self.landing_anchor
        if math.dist(p['xyz'], anchor[2]) > .005:
            self.landing_anchor = None; return False
        return wall-anchor[0] >= 1. and p['stamp']-anchor[1] >= 1.


def load_flight_references():
    """Hash every supplied reference, without mapping any run point to a class."""
    from .ground_template_catalog import load_complete_ground_templates
    mapping = {}
    for spec in load_complete_ground_templates():
        mapping.setdefault(spec.label, []).append(str(spec.path))
    root = Path(__file__).resolve().parent/'assets'/'flight_templates'
    manifest = json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    rows = manifest['templates']
    if len(rows) != 4 or {r['class_id'] for r in rows} != set(WINDOW_LABELS):
        raise ValueError('complete four-window references required')
    for row in rows:
        path = (root/row['file']).resolve()
        raw = path.read_bytes()
        if (path.parent != root.resolve() or len(raw) != row['size_bytes']
                or hashlib.sha256(raw).hexdigest() != row['sha256']
                or hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest() != row['git_blob_sha']):
            raise ValueError('flight reference provenance mismatch')
        mapping.setdefault(WINDOW_LABELS[row['class_id']], []).append(str(path))
    return mapping


def _vision_job(frame, calibration, prior, *, session_id, point_id, kind, matcher):
    import cv2
    import numpy as np
    width, height, step, encoding, raw = (frame[k] for k in ('width', 'height', 'step', 'encoding', 'data'))
    if encoding not in ('rgb8', 'bgr8') or step < width*3 or len(raw) != step*height:
        raise ValueError('actual image encoding or row length')
    pixels = np.frombuffer(raw, np.uint8).reshape(height, step)[:, :width*3].reshape(height, width, 3)
    bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else pixels.copy()
    visual_error = None
    try:
        if kind == 'aerial':
            detections = matcher.floor_detector.detect(bgr, calibration, prior, image_stamp=frame['stamp'])
        else:
            detections = matcher.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    except Exception as exc:
        # Keep the real exposure/K/pose even when matching itself fails.
        detections = []; visual_error = type(exc).__name__+':'+str(exc)[:160]
    expected_labels = set(WINDOW_LABELS.values()) if kind == 'window' else set(matcher.labels)-set(WINDOW_LABELS.values())
    detections = [d for d in detections if d['target_id'] in expected_labels]
    output = []
    for detected in detections:
        detected.update(source='uav', frame_id=frame['frame_id'], stamp=frame['stamp'])
        if kind == 'aerial':
            try:
                fix = localize_floor_pixel(detected['center_px'], list(calibration['k']), prior,
                    image_stamp=frame['stamp'], camera_frame=frame['frame_id'], session_id=session_id,
                    target_id=detected['target_id'], point_id=point_id, now=frame['requested_clock_s'])
            except ValueError:
                continue
            expected = AIR_CENTERS[point_id]
            if math.dist(fix.position_world_m, expected) > .20:
                continue
            detected['target_fix'] = fix
        output.append(detected)
    jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
    return {'detections': output, 'image_jpeg': jpeg, 'source_sha256': hashlib.sha256(raw).hexdigest(),
        'stamp_s': frame['stamp'], 'header_frame_id': frame['frame_id'], 'prior_odom': prior,
        'camera_info': calibration, 'point_id': point_id, 'kind': kind,
        'source_image': {'width': width, 'height': height, 'step': step, 'encoding': encoding,
            'data_bytes': len(raw), 'diagnostic_copy': 'full_resolution_lossy_jpeg85_not_raw_rgb'},
        'detector_diagnostics': copy.deepcopy(matcher.floor_detector.last_diagnostics) if kind == 'aerial' and hasattr(matcher, 'floor_detector') else [],
        'visual_error': visual_error}


def create_flight_inspection_node(reporter=None, *, session_id, ground_return_evidence,
                                   wall_clock=time.monotonic):
    """Create a real ROS node; a partially initialized node is zeroed/disposed."""
    validate_ground_return(ground_return_evidence)
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String
    inputs = {'/clock': 'rosgraph_msgs/msg/Clock', '/uav/odom': 'nav_msgs/msg/Odometry',
        '/odom': 'nav_msgs/msg/Odometry', '/scan': 'sensor_msgs/msg/LaserScan',
        '/uav/lidar/points': 'sensor_msgs/msg/PointCloud2', '/uav/laser/scan': 'sensor_msgs/msg/LaserScan',
        '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
        '/uav/camera/image': 'sensor_msgs/msg/Image', '/uav/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/uav/laser_state': 'std_msgs/msg/Bool'}

    class FlightNode(Node):
        def __init__(self):
            super().__init__('inspection_full_flight')
            self.command_lock = threading.RLock(); self.watchdog_closed = threading.Event()
            self.guard = FlightEvidenceGuard(session_id, ground_return_evidence, started_wall_s=wall_clock())
            self.publishers = {}; self.errors = []; self.subscriptions_keep = []
            self.last_tick_wall = wall_clock(); self.initial_anchor = None
            self.last_graph_wall = None; self.graph_descriptions = {}; self.publish_counts = {}
            self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='flight-visual')
            self.vision_future = None; self.vision_stage = None; self.matcher = None
            self.detections = []; self.visual_records = []; self.last_image = None
            self.last_vision_stamp = None; self.reported_final = False; self.status_sent = False
            self.image_submit_wall = -math.inf; self.last_cloud_raw = None
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            for topic, cls in (('/cmd_vel', Twist), ('/uav/cmd_vel', Twist), ('/uav/laser_fire', Bool),
                    ('/referee/cruise_event', String), ('/inspection/detections', String), ('/inspection/mission_status', String)):
                self.publishers[topic] = self.create_publisher(cls, topic, qos)
            for topic, kind in inputs.items():
                sensor = topic in ('/clock', '/odom', '/uav/odom', '/uav/lidar/points', '/uav/camera/image')
                qos = QoSProfile(depth=10 if topic.startswith('/tf') else 1,
                    reliability=ReliabilityPolicy.BEST_EFFORT if sensor else ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/tf_static' else DurabilityPolicy.VOLATILE)
                self.subscriptions_keep.append(self.create_subscription(get_message(kind), topic,
                    lambda msg, topic=topic: self.receive(topic, msg), qos))

        @property
        def mission_completed(self):
            return self.guard.mission_completed

        def _send(self, topic, message):
            try:
                self.publishers[topic].publish(message)
                self.publish_counts[topic] = self.publish_counts.get(topic, 0)+1
                return True
            except Exception as exc:
                self.errors.append('publish_failed:'+topic+':'+type(exc).__name__)
                self.guard.stop('flight_publish_failed:'+topic, wall_clock())
                return False

        def _zero(self):
            # Every channel is attempted independently even after another fails.
            results = [self._send('/cmd_vel', Twist()), self._send('/uav/cmd_vel', Twist())]
            off = Bool(); off.data = False; results.append(self._send('/uav/laser_fire', off))
            return all(results)

        def publish_zero(self):
            with self.command_lock:
                return self._zero()

        def stop(self, reason):
            with self.command_lock:
                self.guard.stop(reason, wall_clock()); self._zero()
                return MotionDecision(0., self.guard.state, self.guard.reason)

        def receive(self, topic, msg):
            wall = wall_clock()
            try:
                if topic == '/uav/camera/image':
                    self._image(msg, wall); return
                payload = message_to_ordereddict(msg)
                with self.command_lock:
                    self.guard.update(topic, payload, wall=wall)
                    if topic == '/uav/lidar/points':
                        self.last_cloud_raw = copy.deepcopy(payload)
                    if self.guard.stop_wall is not None:
                        return
                    if self.guard.controller is not None and topic == '/uav/odom':
                        if self.guard.odom['xyz'][2] >= 1.55:
                            self.stop('flight_absolute_height_limit')
            except Exception as exc:
                with self.command_lock:
                    self.errors.append('receive:'+topic+':'+type(exc).__name__+':'+str(exc)[:100])
                    self.guard.stop('flight_input_invalid:'+topic+':'+str(exc)[:100], wall)
                    self._zero()

        def _image(self, msg, wall):
            header = msg['header'] if isinstance(msg, dict) else message_to_ordereddict(msg.header)
            stamp = _stamp({'header': header}, CAMERA_FRAME)
            with self.command_lock:
                if not self.guard._source('/uav/camera/image', stamp):
                    return
                self.last_image = {'stamp': stamp, 'wall': wall}
                c = self.guard.controller
                if c is None or c._stage_index >= len(c._stages) or c._stages[c._stage_index].kind != 'recognize':
                    return
                if self.vision_future is not None or wall-self.image_submit_wall < .25:
                    return
                info = self.guard.camera_info; prior = self.guard.prior(stamp)
                if info is None or prior is None or self.guard.clock is None or wall-info['wall'] > 3.:
                    return
                frame = {k: msg[k] if isinstance(msg, dict) else getattr(msg, k) for k in ('width', 'height', 'step', 'encoding', 'data')}
                if frame['width'] != info['message']['width'] or frame['height'] != info['message']['height']:
                    return
                if frame['step']*frame['height'] > 4*1024*1024:
                    raise ValueError('image_buffer_limit')
                frame['data'] = bytes(frame['data'])
                frame.update(frame_id=header['frame_id'], stamp=stamp, requested_clock_s=self.guard.clock['stamp'])
                stage = c._stages[c._stage_index]; self.vision_stage = (c._stage_index, stage.point_id)
                self.image_submit_wall = wall
                calibration = copy.deepcopy(info['message']); raw_prior = copy.deepcopy(prior['message'])
            # Lazy matcher setup and all image work run outside the control lock.
            def run():
                if self.matcher is None:
                    from .vision_core import TemplateMatcher
                    references = load_flight_references()
                    self.matcher = TemplateMatcher(references); self.matcher.labels = list(references)
                if stage.window_index < 0 and not hasattr(self.matcher, 'floor_detector'):
                    from .floor_plane_detector import FloorPlaneDetector
                    self.matcher.floor_detector = FloorPlaneDetector()
                return _vision_job(frame, calibration, raw_prior, session_id=session_id,
                    point_id=stage.point_id, kind='window' if stage.window_index >= 0 else 'aerial', matcher=self.matcher)
            self.vision_future = self.executor.submit(run)

        def _poll_vision(self):
            future = self.vision_future
            if future is None or not future.done():
                return
            self.vision_future = None
            try:
                result = future.result()
            except Exception as exc:
                self.errors.append('visual_worker:'+type(exc).__name__+':'+str(exc)[:128]); return
            c = self.guard.controller
            adopted = (c is not None and c._stage_index < len(c._stages) and self.guard.stop_wall is None
                and self.vision_stage == (c._stage_index, c._stages[c._stage_index].point_id))
            result['original_stage'] = self.vision_stage
            result['adopted_for_current_stage'] = adopted
            if adopted:
                self.detections = copy.deepcopy(result['detections'])
            # Four physical points, at most three real exposures per point.
            # Keep the first two plus a latest/success slot. Empty candidates
            # and late results remain evidence and never become task answers.
            indices = [i for i,r in enumerate(self.visual_records) if r['point_id'] == result['point_id']]
            if len(indices) < 3:
                self.visual_records.append(result)
            elif (result['detections'] and adopted or not self.visual_records[indices[-1]]['detections']):
                self.visual_records[indices[-1]] = result

        def _graph(self):
            if self.last_graph_wall is not None and wall_clock()-self.last_graph_wall < .5:
                return
            descriptions = {}
            for topic in ('/cmd_vel', '/uav/cmd_vel', '/uav/laser_fire'):
                descriptions[topic] = [{'node': i.node_namespace.rstrip('/')+'/'+i.node_name,
                    'type': i.topic_type} for i in self.get_subscriptions_info_by_topic(topic)
                    if i.node_name != 'inspection_full_flight']
            self.graph_descriptions = descriptions; self.last_graph_wall = wall_clock()

        def _connected(self):
            for topic in ('/cmd_vel', '/uav/cmd_vel', '/uav/laser_fire'):
                if not self.graph_descriptions.get(topic) or self.publishers[topic].get_subscription_count() <= 0:
                    return False
            return True

        def _start(self, wall):
            g = self.guard
            if g.condition(wall) or g.ground is None or not g.fresh(g.ground, wall, .20):
                return
            p = g.odom
            if math.hypot(*p['xyz'][:2]) > .025 or abs(p['xyz'][2]-.055) > .02 or p['speed'] > .01 or p['angular_speed'] > .03:
                self.initial_anchor = None; return
            if self.initial_anchor is None:
                self.initial_anchor = (wall, p['stamp'])
            if wall-self.initial_anchor[0] < 1. or p['stamp']-self.initial_anchor[1] < 1.:
                return
            g.home_z = p['xyz'][2]
            g.controller = FlightMissionController(session_id, g.home_z)
            g.controller.start_flight(g.clock['stamp']); g.state = g.controller.state

        def _geometry(self, wall):
            g = self.guard; c = g.controller
            if c is None or c._stage_index >= len(c._stages):
                return
            stage = c._stages[c._stage_index]
            c.geometric_aim = c.aim = None
            fix = c.fixes.get(stage.point_id)
            if fix is None:
                return
            pose = geometry_pose(g.odom['message'], session_id=session_id)
            measured = g.measured_range()
            now = g.clock['stamp']
            c.geometric_aim = prelight_geometry(fix, pose, measured, now=now)
            feedback = g.actual_feedback()
            if feedback is not None and g.feedback is not None and wall-g.feedback['wall'] <= .5:
                c.aim = assess_cached_target(fix, pose, measured, feedback, now=now)

        def _publish_command(self, command):
            g = self.guard; c = g.controller; wall = wall_clock()
            if not self._send('/cmd_vel', Twist()):
                self._zero(); return
            # Actual DDS query can yield/block; every current condition is
            # evaluated after it and after the independent ground-zero publish.
            matches = self.publishers['/uav/cmd_vel'].get_subscription_count()
            fault = g.condition(wall_clock())
            if matches <= 0 or fault or g.stop_wall is not None:
                g.stop(fault or 'flight_final_dds_match_lost', wall_clock()); self._zero(); return
            stage = c._stages[c._stage_index] if c._stage_index < len(c._stages) else c._stages[-1]
            vx, vy, _, omega = command.uav
            landing = stage.name == 'LAND' or c.completed
            z_target = g.home_z+.02 if landing else stage.target[2]
            z_request = flight_z_request(z_target, g.odom['xyz'][2], landed=landing)
            if landing and g.odom['xyz'][2] <= g.home_z+.025:
                z_request = 0.
            physical_vz = max(-.03, min(.03, .5*(z_target-g.odom['xyz'][2]))) if z_request else 0.
            support = stage.name == 'TAKEOFF' or landing
            if not g.path_clear((vx, vy, physical_vz, omega), stage.target, wall=wall_clock(), support=support):
                g.stop('flight_static_or_current_return_path_rejected:'+stage.name, wall_clock()); self._zero(); return
            # Another source update may have happened inside a late transport
            # callback, so do not rely on the pre-query decision.
            if g.condition(wall_clock()) or g.stop_wall is not None:
                g.stop('flight_sources_changed_at_publish', wall_clock()); self._zero(); return
            if command.declaration is not None:
                msg = String(); msg.data = command.declaration['value']
                if not self._send('/referee/cruise_event', msg):
                    self._zero(); return
            if command.recognition is not None:
                report = command.recognition
                if report['event_id'] not in c._published_reports:
                    msg = String(); msg.data = recognition_phrase(report['kind'], report['target_id'])
                    if not self._send('/inspection/detections', msg):
                        self._zero(); return
                    c.mark_recognition_published(report['event_id'], g.clock['stamp'])
                    g.retain('events', {'type': 'recognition_publish_returned', 'report': report})
            if g.condition(wall_clock()) or g.stop_wall is not None:
                g.stop('flight_sources_changed_after_semantic_publish', wall_clock()); self._zero(); return
            yaw = math.atan2(2*(g.odom['q'][3]*g.odom['q'][2]+g.odom['q'][0]*g.odom['q'][1]),
                1-2*(g.odom['q'][1]**2+g.odom['q'][2]**2))
            msg = Twist()
            msg.linear.x = math.cos(yaw)*vx+math.sin(yaw)*vy
            msg.linear.y = -math.sin(yaw)*vx+math.cos(yaw)*vy
            msg.linear.z = z_request; msg.angular.z = omega
            if not self._send('/uav/cmd_vel', msg):
                self._zero(); return
            light = Bool(); light.data = False
            if command.light:
                light_matches = self.publishers['/uav/laser_fire'].get_subscription_count()
                self._geometry(wall_clock())
                light.data = bool(c.geometric_aim and c.geometric_aim.allowed and not g.condition(wall_clock())
                    and light_matches > 0 and g.stop_wall is None)
            if not self._send('/uav/laser_fire', light):
                self._zero(); return
            g.retain('commands', {'wall_elapsed_s': wall_clock()-g.started_wall, 'source_stamp_s': g.clock['stamp'],
                'state': c.state, 'requested_world_velocity': list(command.uav),
                'published_body_xyz': [msg.linear.x, msg.linear.y, msg.linear.z], 'yaw_rate': omega,
                'light': light.data, 'returned': True})
            if landing and z_request == 0. and g.landing_ready(wall=wall_clock()):
                # Physical stationarity and source/wall dwell precede real zeros.
                if self._zero():
                    final_wall = wall_clock()
                    if g.stop_wall is not None or not g.landing_ready(wall=final_wall):
                        g.stop('flight_landing_sources_changed_after_final_zero', final_wall)
                        self._zero(); return
                    feedback_off = (g.feedback is not None and g.feedback['value'] is False
                        and g.feedback['stamp'] is not None and g.fresh(g.feedback, final_wall, .5))
                    if feedback_off:
                        g.landing_evidence = {'pose': copy.deepcopy(g.odom),
                            'dwell_wall_s': final_wall-g.landing_anchor[0],
                            'dwell_source_s': g.odom['stamp']-g.landing_anchor[1],
                            'zero_publish_returned': True, 'zero_returned_wall_s': final_wall,
                            'lamp_feedback_off': True}
                        done = String(); done.data = 'mission_completed'
                        if self._send('/inspection/mission_status', done):
                            self.status_sent = True
                            completed_wall = wall_clock()
                            if (g.stop_wall is not None or not g.landing_ready(wall=completed_wall)
                                    or g.feedback is None or g.feedback['value'] is not False
                                    or g.feedback['stamp'] is None or not g.fresh(g.feedback, completed_wall, .5)):
                                g.stop('flight_landing_sources_changed_during_completion_publish', completed_wall)
                                self._zero(); return
                            g.landing_evidence.update(completion_publish_returned=True,
                                completion_returned_wall_s=completed_wall)
                            g.mission_completed = True
                            g.state = 'FINISHED'; g.reason = 'flight_actual_landing_complete_official_score_unverified'

        def tick(self):
            try:
                self._graph()  # outside command lock; watchdog remains independent
                with self.command_lock:
                    g = self.guard; wall = wall_clock(); self._poll_vision()
                    if g.mission_completed:
                        self._zero(); return MotionDecision(0., 'FINISHED', g.reason)
                    if g.stop_wall is not None:
                        self._zero()
                        if wall-g.stop_wall >= .5:
                            g.state = 'FINISHED'
                        return MotionDecision(0., g.state, g.reason)
                    if not self._connected():
                        if wall-g.started_wall >= 10.:
                            g.stop('flight_command_channels_unavailable', wall)
                        self._zero(); return MotionDecision(0., g.state, g.reason)
                    if g.controller is None:
                        self._start(wall)
                        if g.controller is None:
                            if wall-g.started_wall >= 15.:
                                g.stop('flight_initial_sources_or_stationarity_timeout:'+g.condition(wall), wall)
                            self._zero(); self.last_tick_wall = wall_clock()
                            return MotionDecision(0., g.state, g.reason)
                    fault = g.condition(wall)
                    if fault:
                        g.stop(fault, wall); self._zero(); return MotionDecision(0., g.state, g.reason)
                    c = g.controller; p = g.odom
                    yaw = math.atan2(2*(p['q'][3]*p['q'][2]+p['q'][0]*p['q'][1]), 1-2*(p['q'][1]**2+p['q'][2]**2))
                    pose = Pose2D(*p['xyz'], yaw, p['stamp'])
                    self._geometry(wall)
                    command = c.step(g.clock['stamp'], None, pose, self.detections,
                        light_ready=bool(c.aim and c.aim.allowed), controls_ready=True)
                    g.state = c.state
                    if command.fault:
                        g.stop(command.fault, wall); self._zero()
                    else:
                        self._publish_command(command)
                    self.last_tick_wall = wall_clock()
                    return MotionDecision(0., g.state, g.reason)
            except Exception as exc:
                self.errors.append('tick:'+type(exc).__name__+':'+str(exc)[:128])
                return self.stop('flight_tick_failed:'+type(exc).__name__+':'+str(exc)[:100])

        def arm_watchdog(self):
            if hasattr(self, 'watchdog_thread'):
                return
            def watch():
                while not self.watchdog_closed.wait(.05):
                    if wall_clock()-self.last_tick_wall > .25:
                        self.stop('flight_control_watchdog_stale')
            self.watchdog_thread = threading.Thread(target=watch, name='full-flight-watchdog', daemon=True)
            self.watchdog_thread.start()

        def close_watchdog(self):
            self.watchdog_closed.set()
            if hasattr(self, 'watchdog_thread') and self.watchdog_thread is not threading.current_thread():
                self.watchdog_thread.join(timeout=.5)

        def report_started(self, duration):
            if reporter is not None:
                reporter.emit('probe_started', {'mode': 'full_flight_inspection', 'session_id': session_id,
                    'read_only': False, 'duration_s': duration, 'world_sha256': WORLD_SHA256,
                    'official_scoring': 'unverified', 'ground_return_evidence': self.guard.ground_return,
                    'flight_contract': 'public_static_scene_plus_current_return_veto',
                    'z_feedback': 'v37_trim_actual_hold_0.85m;1.282m_unverified',
                    'velocity_frame': 'world_controller_to_body_twist_public_candidate'})

        def evidence_snapshot(self):
            with self.command_lock:
                g = self.guard; c = g.controller
                result = {'schema': 'full_flight_inspection_v1', 'session_id': session_id,
                    'mode': 'full_flight_inspection', 'state': g.state, 'reason': g.reason,
                    'full_mission_complete': g.mission_completed, 'official_score': None,
                    'ground_return_evidence': g.ground_return, 'landing_evidence': g.landing_evidence,
                    'trajectory': list(g.trajectory), 'commands': list(g.commands), 'total_path_m': g.total_path,
                    'safety_checks': list(g.safety_checks), 'events': list(g.events), 'dropped': g.dropped,
                    'stage_log': c.stage_log if c else [], 'plan': c.plan if c else None,
                    'fixes': {k: asdict(v) for k,v in c.fixes.items()} if c else {},
                    'last_odom': g.odom, 'last_scan': g.scan, 'last_downscan': g.down,
                    'last_cloud_layout': g.cloud['layout'] if g.cloud else None,
                    'last_cloud_raw': self.last_cloud_raw, 'actual_laser_tf': g.laser_tf,
                    'tf_records': list(g.tf_records), 'camera_info': g.camera_info,
                    'laser_feedback': g.feedback, 'publish_counts': self.publish_counts,
                    'errors': self.errors, 'endpoint_descriptions': self.graph_descriptions,
                    'assumptions': {'world_source': WORLD_SOURCE, 'world_sha256': WORLD_SHA256,
                        'static_scene_only': True, 'platform_map_equality_proven': False,
                        'full_fov_obstacle_coverage': False, 'body_cmd_frame_candidate': True,
                        'camera_offset_m': CAMERA_OFFSET, 'camera_extrinsic_candidate': True,
                        'camera_offset_source': 'public_world_uav_camera_pose_and_inspection_launch_static_tf_z0.130',
                        'geometry_error_bounds_are_engineering_assumptions': True,
                        'horizontal_plane_height_error_m': PLANE_HEIGHT_ERROR_M,
                        'illumination_error_policy': 'unchanged_ray_XY_bound_plus_cache_drift;independent_public_plane_Z_bound;original_hit_limits',
                        'scan_age_s': .35, 'cloud_age_s': .5,
                        'lookahead_s': 1., 'unmeasured_stop_response_allowance_m': .02,
                        'sensor_policy': 'source_and_wall_ages;public_static_sweep_plus_current_returns;no_unknown_space_claim'}}
                records = copy.deepcopy(self.visual_records)
            for record in records:
                record['image_jpeg_b64'] = base64.b64encode(record.pop('image_jpeg')).decode()
                record['detections'] = [{k: asdict(v) if isinstance(v, TargetFix) else v for k,v in d.items()}
                    for d in record['detections']]
            result['visual_records'] = records
            # PointCloud2 bytes need explicit encoding; no truncation is called
            # a complete cloud. All serialization/hash work is outside locks.
            if result['last_cloud_raw'] is not None:
                result['last_cloud_raw'] = copy.deepcopy(result['last_cloud_raw'])
                raw = bytes(result['last_cloud_raw'].pop('data'))
                result['last_cloud_raw'].update(data_b64=base64.b64encode(raw).decode(),
                    data_bytes=len(raw), data_sha256=hashlib.sha256(raw).hexdigest())
            return _safe(result)

        def report_final(self):
            if reporter is None or self.reported_final:
                return
            self.reported_final = True
            snapshot = self.evidence_snapshot()
            raw = json.dumps(snapshot, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode()
            compressed = gzip.compress(raw, mtime=0); encoded = base64.b64encode(compressed).decode()
            chunks = [encoded[i:i+40000] for i in range(0, len(encoded), 40000)]
            for i, chunk in enumerate(chunks):
                reporter.send_now('probe_snapshot', {'mode': 'full_flight_inspection',
                    'section': 'full_flight_gzip_part', 'session_id': session_id, 'encoding': 'gzip+base64',
                    'part_index': i, 'part_count': len(chunks), 'base64_part': chunk,
                    'compressed_bytes': len(compressed), 'uncompressed_bytes': len(raw),
                    'compressed_sha256': hashlib.sha256(compressed).hexdigest(),
                    'uncompressed_sha256': hashlib.sha256(raw).hexdigest()}, timeout_s=5.)
            reporter.send_now('probe_finished', {'mode': 'full_flight_inspection', 'session_id': session_id,
                'state': self.guard.state, 'reason': self.guard.reason,
                'full_mission_complete': self.mission_completed, 'official_scoring': 'unverified',
                'publish_counts': self.publish_counts, 'errors': self.errors[:32]}, timeout_s=5.)

        def destroy_node(self):
            self.close_watchdog()
            if hasattr(self, 'executor'):
                self.executor.shutdown(wait=False, cancel_futures=True)
            return super().destroy_node()

    node = FlightNode.__new__(FlightNode)
    try:
        node.__init__()
        return node
    except Exception:
        if hasattr(node, 'publishers'):
            try:
                node.publish_zero()
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        raise

