"""Bounded read-only PointCloud2 evidence for the separate v37 experiment.

The callback copies complete buffer bytes and bounded metadata only. Hashing and
base64 belong to ``snapshot(encode=True)`` after control has terminated. Clouds
and their associations never grant movement permission.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import math
import threading


def _get(value, name):
    return value[name] if isinstance(value, dict) else getattr(value, name)


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _stamp(header):
    stamp = _get(header, "stamp")
    sec, nano = _get(stamp, "sec"), _get(stamp, "nanosec")
    if type(sec) is not int or type(nano) is not int or not 0 <= sec <= 2**31-1 or not 0 <= nano < 10**9:
        raise ValueError("invalid_stamp")
    return {"sec": sec, "nanosec": nano}, sec*10**9+nano


def _text(value):
    if not isinstance(value, str) or not 0 < len(value) <= 256:
        raise ValueError("metadata_text")
    return value


def _small_copy(value, *, budget=None, depth=0):
    """Copy JSON-shaped recorded metadata without admitting large payloads."""
    budget = [1024] if budget is None else budget
    budget[0] -= 1
    if budget[0] < 0 or depth > 10:
        raise ValueError("metadata_limit")
    if value is None or type(value) is bool:
        return value
    if type(value) in (int, float):
        if not _finite(value) or abs(value) > 10**18:
            raise ValueError("metadata_numeric")
        return value
    if isinstance(value, str):
        if len(value) > 256:
            raise ValueError("metadata_limit")
        return value
    if isinstance(value, dict):
        if len(value) > 64:
            raise ValueError("metadata_limit")
        return {_text(key): _small_copy(item, budget=budget, depth=depth+1)
                for key, item in value.items()}
    if isinstance(value, (tuple, list)) and len(value) <= 128:
        return [_small_copy(item, budget=budget, depth=depth+1) for item in value]
    raise ValueError("metadata_type")


class CruiseCloudCapture:
    """Keep one whole cloud per stage, with honest recorded associations.

    ``trajectory`` accepts actual recorded odometry rows. A row's ``message``
    (raw odometry dictionary) is authoritative when present; legacy parsed rows
    remain usable but are explicitly marked ``raw_odom_unavailable``.
    ``tf_records`` uses the existing ``{message: {transforms: [...]}, ...}``
    records, with source and arrival fields retained when actually supplied.
    """

    SLOTS = ("WAIT", "ASCEND_CRUISE", "HOLD_CRUISE", "DESCEND_SUPPORT")
    SOURCE = "/uav/lidar/points"
    MAX_FRAME_BYTES = 2*1024*1024
    MAX_TOTAL_BYTES = 8*1024*1024
    MAX_FIELDS = 128
    MAX_TF_RECORDS = 128
    MAX_TF_TRANSFORMS = 128
    MAX_TRAJECTORY_ROWS = 8192
    MAX_ERRORS = 32
    _FIELD_BYTES = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    _COUNTERS = ("captured", "rejected", "slot_already_captured", "unsupported_phase",
                 "invalid_stamp", "invalid_layout", "invalid_arrival", "frame_oversize",
                 "total_oversize", "capture_exception", "no_prior_pose", "prior_pose_stale",
                 "raw_odom_unavailable", "missing_transform_chain", "tf_snapshot_truncated",
                 "tf_invalid_record", "trajectory_truncated")

    def __init__(self):
        self._frames = {}
        self._raw_bytes = 0
        self._counters = dict.fromkeys(self._COUNTERS, 0)
        self._errors, self._errors_dropped = [], 0
        self._lock = threading.RLock()

    def _record(self, reason, phase, wall_s, *, rejected=False):
        self._counters[reason] += 1
        if rejected:
            self._counters["rejected"] += 1
        error = {"reason": reason, "phase": phase if phase in self.SLOTS else None,
                 "arrival_wall_s": wall_s if _finite(wall_s) else None}
        if len(self._errors) < self.MAX_ERRORS:
            self._errors.append(error)
        else:
            self._errors_dropped += 1

    def _layout(self, msg):
        values = {name: _get(msg, name) for name in ("width", "height", "point_step", "row_step")}
        if any(type(value) is not int or not 0 < value <= 2**32-1 for value in values.values()):
            raise ValueError("invalid_layout")
        if values["row_step"] < values["width"]*values["point_step"]:
            raise ValueError("invalid_layout")
        for name in ("is_bigendian", "is_dense"):
            value = _get(msg, name)
            if type(value) is not bool:
                raise ValueError("invalid_layout")
            values[name] = value
        fields = _get(msg, "fields")
        if not isinstance(fields, (list, tuple)) or not 0 < len(fields) <= self.MAX_FIELDS:
            raise ValueError("invalid_layout")
        saved_fields, names = [], set()
        for field in fields:
            name = _text(_get(field, "name"))
            offset, datatype, count = (_get(field, key) for key in ("offset", "datatype", "count"))
            if (name in names or type(offset) is not int or offset < 0 or type(datatype) is not int
                    or datatype not in self._FIELD_BYTES or type(count) is not int or count <= 0
                    or offset+count*self._FIELD_BYTES[datatype] > values["point_step"]):
                raise ValueError("invalid_layout")
            names.add(name)
            saved_fields.append({"name": name, "offset": offset, "datatype": datatype, "count": count})
        values["fields"] = saved_fields
        return values

    def _prior_pose(self, trajectory, stamp_ns, phase, wall_s):
        if not isinstance(trajectory, (list, tuple)):
            trajectory = []
        omitted = max(0, len(trajectory)-self.MAX_TRAJECTORY_ROWS)
        if omitted:
            self._record("trajectory_truncated", phase, wall_s)
        best, best_ns = None, -1
        for row in trajectory[omitted:]:
            try:
                if not isinstance(row, dict):
                    continue
                if "message" in row:
                    message = row["message"]
                    _, candidate_ns = _stamp(message["header"])
                    if message["header"]["frame_id"] != "odom" or message["child_frame_id"] != "uav/base_link":
                        continue
                    pose, twist = message["pose"]["pose"], message["twist"]["twist"]
                    xyz = tuple(pose["position"][key] for key in ("x", "y", "z"))
                    quaternion = tuple(pose["orientation"][key] for key in ("x", "y", "z", "w"))
                    linear = tuple(twist["linear"][key] for key in ("x", "y", "z"))
                    angular = tuple(twist["angular"][key] for key in ("x", "y", "z"))
                    if (not all(_finite(value) for value in (*xyz, *quaternion, *linear, *angular))
                            or abs(math.hypot(*quaternion)-1.) > .001):
                        continue
                else:
                    value = row.get("stamp", row.get("stamp_s"))
                    if not _finite(value) or value < 0:
                        continue
                    candidate_ns = value*10**9
                if best_ns < candidate_ns <= stamp_ns:
                    best, best_ns = row, candidate_ns
            except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
                continue
        if best is None or stamp_ns-best_ns > 50_000_000:
            reason = "no_prior_pose" if best is None else "prior_pose_stale"
            self._record(reason, phase, wall_s)
            return {"status": "unavailable", "reason": reason, "trajectory_rows_omitted": omitted}
        try:
            saved = _small_copy(best)
        except (TypeError, ValueError, OverflowError):
            self._record("no_prior_pose", phase, wall_s)
            return {"status": "unavailable", "reason": "prior_pose_metadata_invalid",
                    "trajectory_rows_omitted": omitted}
        association = {"status": "valid_prior_pose", "reason": None, "record": saved,
                       "odom_stamp_s": best_ns/10**9, "skew_s": (stamp_ns-best_ns)/10**9,
                       "trajectory_rows_omitted": omitted}
        if "message" not in best:
            association["raw_odom_status"] = "raw_odom_unavailable"
            self._record("raw_odom_unavailable", phase, wall_s)
        return association

    def _tf_snapshot(self, records, frame_id, phase, wall_s):
        if not isinstance(records, (list, tuple)):
            records = []
        # Static extrinsics must survive a full dynamic history. Selection does
        # not infer static status from a zero stamp: the recorded source rules.
        start = max(0, len(records)-self.MAX_TRAJECTORY_ROWS)
        candidates = list(range(start, len(records)))
        static = [index for index in candidates
                  if isinstance(records[index], dict) and records[index].get("source") == "/tf_static"]
        static_set = set(static)
        dynamic = [index for index in candidates if index not in static_set]
        selected = static[-self.MAX_TF_RECORDS:]
        remaining = self.MAX_TF_RECORDS-len(selected)
        if remaining:
            selected.extend(dynamic[-remaining:])
        omitted = len(records)-len(selected)
        saved_records, edges, transform_count = [], [], 0
        invalid, transforms_omitted, source_unavailable = 0, 0, 0
        for index in selected:
            try:
                record = records[index]
                transforms = record["message"]["transforms"]
                if not isinstance(transforms, (list, tuple)) or len(record) > 64:
                    raise ValueError("tf_transforms")
                saved = {key: _small_copy(value) for key, value in record.items() if key != "message"}
                saved_transforms, record_edges = [], []
                room = self.MAX_TF_TRANSFORMS-transform_count
                transforms_omitted += max(0, len(transforms)-room)
                for transform in transforms[:room]:
                    header = transform["header"]
                    _stamp(header)
                    parent, child = _text(header["frame_id"]), _text(transform["child_frame_id"])
                    translation = transform["transform"]["translation"]
                    rotation = transform["transform"]["rotation"]
                    if not all(_finite(translation[key]) for key in ("x", "y", "z")) or not all(
                            _finite(rotation[key]) for key in ("x", "y", "z", "w")):
                        raise ValueError("tf_numeric")
                    if abs(math.hypot(*(rotation[key] for key in ("x", "y", "z", "w")))-1.) > .001:
                        raise ValueError("tf_quaternion")
                    saved_transforms.append(_small_copy(transform))
                    record_edges.append((parent, child))
                saved["message"] = {"transforms": saved_transforms}
                saved_records.append((index, saved))
                edges.extend(record_edges)
                transform_count += len(saved_transforms)
                source_unavailable += "source" not in saved
            except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
                invalid += 1
        if omitted or transforms_omitted:
            self._record("tf_snapshot_truncated", phase, wall_s)
        if invalid:
            self._record("tf_invalid_record", phase, wall_s)
        connected = {"uav/base_link"}
        for _ in range(len(edges)+1):
            previous = len(connected)
            for parent, child in edges:
                if parent in connected or child in connected:
                    connected.update((parent, child))
            if len(connected) == previous:
                break
        available = frame_id in connected
        if not available:
            self._record("missing_transform_chain", phase, wall_s)
        return {"status": "transform_chain_present" if available else "unavailable",
                "reason": None if available else "missing_transform_chain",
                "records": [saved for _, saved in sorted(saved_records, key=lambda item: item[0])],
                "records_omitted": omitted, "transforms_omitted": transforms_omitted,
                "invalid_records": invalid, "source_unavailable": source_unavailable}

    def capture(self, msg, *, phase, wall_s, trajectory, tf_records):
        """Copy a complete stage sample; malformed evidence never escapes here."""
        with self._lock:
            try:
                if phase not in self.SLOTS:
                    self._record("unsupported_phase", phase, wall_s, rejected=True)
                    return False
                if phase in self._frames:
                    self._counters["slot_already_captured"] += 1
                    return False
                if not _finite(wall_s) or wall_s < 0:
                    self._record("invalid_arrival", phase, wall_s, rejected=True)
                    return False
                header = _get(msg, "header")
                try:
                    stamp, stamp_ns = _stamp(header)
                    frame_id = _text(_get(header, "frame_id"))
                except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
                    self._record("invalid_stamp", phase, wall_s, rejected=True)
                    return False
                try:
                    layout = self._layout(msg)
                    data = memoryview(_get(msg, "data"))
                    expected_bytes = layout["row_step"]*layout["height"]
                    if not data.c_contiguous or data.itemsize != 1 or data.ndim != 1 or data.nbytes != expected_bytes:
                        raise ValueError("data_length_or_buffer")
                except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
                    self._record("invalid_layout", phase, wall_s, rejected=True)
                    return False
                if expected_bytes > self.MAX_FRAME_BYTES:
                    self._record("frame_oversize", phase, wall_s, rejected=True)
                    return False
                if self._raw_bytes+expected_bytes > self.MAX_TOTAL_BYTES:
                    self._record("total_oversize", phase, wall_s, rejected=True)
                    return False
                frame = {"phase": phase, "source": self.SOURCE,
                         "header": {"stamp": stamp, "frame_id": frame_id},
                         "source_stamp_s": stamp_ns/10**9, "arrival_wall_s": wall_s, **layout,
                         "data_bytes": expected_bytes,
                         "pose_association": self._prior_pose(trajectory, stamp_ns, phase, wall_s),
                         "tf_snapshot": self._tf_snapshot(tf_records, frame_id, phase, wall_s)}
                raw = data.tobytes()
                self._frames[phase] = (frame, raw)
                self._raw_bytes += expected_bytes
                self._counters["captured"] += 1
                return True
            except Exception:
                self._record("capture_exception", phase, wall_s, rejected=True)
                return False

    def snapshot(self, encode=False):
        """Return metadata, optionally encoding all retained bytes after stop.

        No control lock is held while hashing or encoding immutable raw buffers.
        The caller is responsible for invoking ``encode=True`` only after stop.
        """
        with self._lock:
            frames = [(copy.deepcopy(self._frames[phase][0]), self._frames[phase][1])
                      for phase in self.SLOTS if phase in self._frames]
            result = {"schema": "cruise_cloud_capture_v37", "encoded": bool(encode),
                      "frame_count": len(frames), "raw_bytes": self._raw_bytes,
                      "max_frame_bytes": self.MAX_FRAME_BYTES, "max_total_bytes": self.MAX_TOTAL_BYTES,
                      "missing_slots": [phase for phase in self.SLOTS if phase not in self._frames],
                      "counters": dict(self._counters), "errors": copy.deepcopy(self._errors),
                      "errors_dropped": self._errors_dropped}
        for metadata, raw in frames:
            if encode:
                metadata["data_sha256"] = hashlib.sha256(raw).hexdigest()
                metadata["data_b64"] = base64.b64encode(raw).decode("ascii")
                metadata["data_encoding"] = "base64"
        result["clouds"] = [metadata for metadata, _ in frames]
        return result
