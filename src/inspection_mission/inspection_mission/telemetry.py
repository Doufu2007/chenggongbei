"""Optional, bounded diagnostics to a team-owned temporary receiver.

The receiver is an observation channel. It is never a referee or score source.
Network failures cannot turn a probe into a success or block its ROS callbacks.
"""
from __future__ import annotations

import json
import base64
import math
import queue
import threading
import urllib.parse
import urllib.request
from datetime import datetime, timezone


EVENTS = frozenset(("launcher_started", "launcher_exit", "probe_started", "public_graph",
                   "subscription", "sample", "sample_error", "subscription_error",
                   "heartbeat", "probe_finished", "image_sample", "probe_snapshot"))


def validate_image_sample(data):
    """Accept only the bounded public-sensor JPEG snapshot contract."""
    if set(data) != {"topic", "stamp", "frame_id", "width", "height", "encoding", "jpeg_base64"}:
        raise ValueError("invalid image sample fields")
    topic = data["topic"]
    if (not isinstance(topic, str) or not topic.startswith("/") or len(topic) > 256
            or topic.startswith(("/gazebo/", "/ground_truth/", "/truth/", "/private/", "/referee/"))
            or any(part in topic.lower() for part in ("private", "truth", "hidden", "answer"))
            or not isinstance(data["frame_id"], str) or len(data["frame_id"]) > 256
            or type(data["width"]) is not int or not 1 <= data["width"] <= 480
            or type(data["height"]) is not int or not 1 <= data["height"] <= 360
            or type(data["stamp"]) not in (int, float) or not math.isfinite(data["stamp"])
            or data["stamp"] < 0 or data["encoding"] != "jpeg"
            or not isinstance(data["jpeg_base64"], str)):
        raise ValueError("invalid image sample metadata")
    try:
        encoded = base64.b64decode(data["jpeg_base64"], validate=True)
    except (ValueError, UnicodeError) as exc:
        raise ValueError("invalid image sample base64") from exc
    if not encoded.startswith(b"\xff\xd8\xff") or not encoded.endswith(b"\xff\xd9"):
        raise ValueError("image sample is not a complete JPEG")


def _validate(value, depth=0):
    if depth > 12:
        raise ValueError("Diagnostic payload is too deeply nested")
    if isinstance(value, dict):
        for key, child in value.items():
            if not isinstance(key, str) or any(word in key.lower() for word in ("password", "token", "cookie", "secret")):
                raise ValueError("Credentials are forbidden in diagnostic events")
            _validate(child, depth + 1)
    elif isinstance(value, list):
        for child in value:
            _validate(child, depth + 1)


class DiagnosticReporter:
    def __init__(self, url=""):
        self.url = url
        self._closed = False
        self._queue = queue.Queue(maxsize=64)
        self._thread = None
        self.delivered = 0
        self.failed = 0
        if url:
            parsed = urllib.parse.urlsplit(url)
            loopback = parsed.scheme == "http" and parsed.hostname in ("127.0.0.1", "localhost", "::1")
            if (not (parsed.scheme == "https" or loopback) or not parsed.hostname
                    or parsed.username or parsed.password or parsed.fragment):
                raise ValueError("Diagnostic endpoint must use HTTPS or loopback HTTP without credentials")
            self._thread = threading.Thread(target=self._send, daemon=True)
            self._thread.start()

    @staticmethod
    def _encode(event, data):
        if event not in EVENTS or not isinstance(data, dict):
            raise ValueError("Unknown diagnostic event or invalid data")
        if event == "image_sample":
            validate_image_sample(data)
        envelope = {"event": event, "timestamp": datetime.now(timezone.utc).isoformat(), "data": data}
        # Match the receiver's depth budget, including the event envelope.
        _validate(envelope, depth=1)
        payload = json.dumps(envelope, ensure_ascii=False, allow_nan=False).encode("utf-8")
        if len(payload) > 65536 or (event == "image_sample" and len(payload) >= 65536):
            raise ValueError("Diagnostic event exceeds 64 KB")
        return payload

    def emit(self, event, data):
        payload = self._encode(event, data)
        if not self.url or self._closed:
            return False
        try:
            self._queue.put_nowait(payload)
            return True
        except queue.Full:
            self.failed += 1
            return False

    def send_now(self, event, data, timeout_s=8):
        """Send final evidence directly; use after spinning, outside ROS callbacks.

        This independent request never waits for or locks the background queue.
        Network failure is diagnostic only, and does not fail a ROS probe.
        """
        if (type(timeout_s) not in (int, float) or not math.isfinite(timeout_s)
                or not 0 < timeout_s <= 30):
            raise ValueError("Synchronous diagnostic timeout must lie in (0,30] seconds")
        payload = self._encode(event, data)
        if not self.url or self._closed:
            return False
        return self._post(payload, timeout_s)

    def _post(self, payload, timeout_s):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        request = urllib.request.Request(self.url, data=payload, method="POST",
                    headers={"Content-Type": "application/json", "User-Agent": "inspection-readonly-probe/1"})
        try:
            with opener.open(request, timeout=timeout_s) as response:
                if response.status != 202:
                    raise ValueError("Receiver did not accept diagnostic event")
            self.delivered += 1
            return True
        except Exception:
            self.failed += 1
            return False

    def _send(self):
        while True:
            payload = self._queue.get()
            try:
                if payload is None:
                    return
                self._post(payload, 1.5)
            finally:
                self._queue.task_done()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._thread:
            try:
                self._queue.put_nowait(None)
            except queue.Full:
                return
            self._thread.join(timeout=3.0)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
