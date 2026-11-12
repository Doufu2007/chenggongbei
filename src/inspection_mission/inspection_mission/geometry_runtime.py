"""Lossless public-odometry evidence and bounded, preceding exposure matching.

This module provides no calibration, world fix, uncertainty bound or control.
Clock/frame provenance must be confirmed before a target_geometry adapter can
turn these raw observations into its stronger StampedPose3D contract.
"""
from __future__ import annotations

import copy
from collections import deque
from dataclasses import dataclass
import math

from .protocol import sensor_time


def finite_number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + " must be finite")
    return float(value)


def confirmed_text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(label + " must be explicitly confirmed")
    return value


@dataclass(frozen=True)
class OdometryEvidence:
    stamp_s: float
    position_m: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
    child_frame_id: str | None
    raw_payload: dict

    @property
    def header(self):
        return copy.deepcopy(self.raw_payload["header"])

    @property
    def twist(self):
        # Missing legacy fields stay missing; zero velocity is never invented.
        return copy.deepcopy(self.raw_payload.get("twist"))


def parse_odometry_evidence(payload, expected_frame, now, *, expected_child_frame=None):
    """Preserve accepted values exactly, without quaternion renormalization."""
    now = finite_number(now, "runtime time")
    header = payload["header"]
    if header["frame_id"] != expected_frame:
        raise ValueError("unconfirmed odometry world frame")
    sec, nano = header["stamp"]["sec"], header["stamp"]["nanosec"]
    if type(sec) is not int or type(nano) is not int or not 0 <= nano < 1_000_000_000:
        raise ValueError("invalid odometry header timestamp")
    stamp = sensor_time(sec, nano, now)
    child = payload.get("child_frame_id")
    if child is not None:
        confirmed_text(child, "odometry child_frame_id")
    if expected_child_frame is not None:
        confirmed_text(expected_child_frame, "expected child frame")
        if child != expected_child_frame:
            raise ValueError("unconfirmed odometry child frame")
    body_pose = payload["pose"]["pose"]
    position = tuple(finite_number(body_pose["position"][key], "position") for key in ("x", "y", "z"))
    quaternion = tuple(finite_number(body_pose["orientation"][key], "quaternion") for key in ("x", "y", "z", "w"))
    if abs(math.hypot(*quaternion) - 1.) > .01:
        raise ValueError("quaternion must be normalized")
    if "twist" in payload:
        twist = payload["twist"]["twist"]
        for vector in ("linear", "angular"):
            for axis in ("x", "y", "z"):
                finite_number(twist[vector][axis], "twist." + vector + "." + axis)
    return OdometryEvidence(stamp, position, quaternion, child, copy.deepcopy(payload))


class PoseHistory:
    """A short history; never interpolate, extrapolate or change source stamps."""

    def __init__(self, *, max_samples=128, max_span_s=2.):
        if type(max_samples) is not int or max_samples < 1:
            raise ValueError("pose history max_samples must be a positive integer")
        self.max_span_s = finite_number(max_span_s, "pose history max_span_s")
        if self.max_span_s <= 0:
            raise ValueError("pose history max_span_s must be positive")
        self._entries = deque(maxlen=max_samples)

    def clear(self):
        self._entries.clear()

    def append(self, evidence):
        if not isinstance(evidence, OdometryEvidence):
            raise ValueError("invalid odometry evidence")
        if self._entries:
            previous = self._entries[-1]
            if evidence.stamp_s < previous.stamp_s:
                self.clear()
                raise ValueError("odometry stamp reversed; geometry history cleared")
            if (evidence.header["frame_id"] != previous.header["frame_id"]
                    or evidence.child_frame_id != previous.child_frame_id):
                self.clear()
                raise ValueError("odometry frame changed; geometry history cleared")
            if evidence.stamp_s == previous.stamp_s:
                if evidence.raw_payload != previous.raw_payload:
                    self.clear()
                    raise ValueError("odometry same stamp changed; geometry history cleared")
                return
        self._entries.append(copy.deepcopy(evidence))
        while self._entries and evidence.stamp_s - self._entries[0].stamp_s > self.max_span_s:
            self._entries.popleft()

    def at_exposure(self, stamp, *, expected_child_frame, max_skew_s, now, max_age_s):
        confirmed_text(expected_child_frame, "expected child frame")
        stamp = finite_number(stamp, "exposure stamp")
        now = finite_number(now, "runtime time")
        skew = finite_number(max_skew_s, "maximum pose/image skew")
        age = finite_number(max_age_s, "maximum evidence age")
        if min(stamp, now, skew, age) < 0:
            raise ValueError("negative exposure matching bound")
        if stamp > now:
            raise ValueError("future exposure stamp")
        if now - stamp > age:
            raise ValueError("stale exposure stamp")
        if not self._entries:
            raise ValueError("empty geometry pose history")
        candidate = next((pose for pose in reversed(self._entries) if pose.stamp_s <= stamp), None)
        if candidate is None:
            raise ValueError("no preceding pose for exposure")
        if candidate.child_frame_id != expected_child_frame:
            raise ValueError("unconfirmed odometry child frame")
        if stamp - candidate.stamp_s > skew:
            raise ValueError("pose/image skew exceeds bound")
        if now - candidate.stamp_s > age:
            raise ValueError("stale exposure pose")
        return copy.deepcopy(candidate)


CAMERA_METADATA = frozenset({
    "frame_id", "calibration_version", "size_version", "coordinate_convention", "clock_id", "epoch_id",
    "position_error_m", "camera_info_stamp", "received_stamp", "image_width", "image_height",
})


def camera_metadata(payload, *, now):
    """Validate only explicitly supplied fields; absent provenance stays absent."""
    metadata = {key: copy.deepcopy(payload[key]) for key in CAMERA_METADATA if key in payload}
    for key in ("frame_id", "calibration_version", "size_version", "coordinate_convention", "clock_id", "epoch_id"):
        if key in metadata:
            confirmed_text(metadata[key], "camera " + key)
    for key in ("camera_info_stamp", "received_stamp"):
        if key in metadata:
            value = finite_number(metadata[key], "camera " + key)
            if value < 0 or value > now + .05:
                raise ValueError("invalid/future camera " + key)
    if "position_error_m" in metadata and finite_number(metadata["position_error_m"], "position_error_m") < 0:
        raise ValueError("negative camera position_error_m")
    for key in ("image_width", "image_height"):
        if key in metadata and (type(metadata[key]) is not int or metadata[key] <= 0):
            raise ValueError("camera " + key + " must be a positive integer")
    return metadata


def detection_with_metadata(item, metadata, *, source, stamp, now):
    """Carry actual camera provenance to each observation, rejecting conflicts."""
    explicit = camera_metadata(item, now=now)
    for key in explicit.keys() & metadata.keys():
        if explicit[key] != metadata[key]:
            raise ValueError("conflicting camera metadata: " + key)
    merged = {**copy.deepcopy(item), **metadata, **explicit, "source": source, "stamp": stamp}
    if "position_camera_m" in merged:
        position = merged["position_camera_m"]
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError("position_camera_m must contain three finite coordinates")
        for value in position:
            finite_number(value, "position_camera_m")
    return merged
