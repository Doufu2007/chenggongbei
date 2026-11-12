"""Temporary loopback-only callback receiver; callbacks are unverified evidence.

Example: python tools/diagnostic_receiver.py --output runs/evidence/callback.jsonl
Only the startup JSON prints the random URL. Never store that URL in source.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import secrets
import signal
import socket
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


MAX_BODY_BYTES = 64 * 1024
MAX_DEPTH = 12
MAX_DELIVERY_ROWS = 4096
MAX_DELIVERY_LINE_BYTES = 1024
EVENTS = {
    "launcher_started", "launcher_exit", "probe_started", "public_graph",
    "subscription", "sample", "sample_error", "subscription_error",
    "heartbeat", "probe_finished", "image_sample", "probe_snapshot",
}
FORBIDDEN_KEY_PARTS = ("password", "token", "cookie", "secret")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _check_tree(value, depth=1):
    if depth > MAX_DEPTH:
        raise ValueError("JSON depth exceeds limit")
    if isinstance(value, dict):
        for key, item in value.items():
            if any(part in key.lower() for part in FORBIDDEN_KEY_PARTS):
                raise ValueError("credential-like JSON key")
            _check_tree(item, depth + 1)
    elif isinstance(value, list):
        for item in value:
            _check_tree(item, depth + 1)
    elif isinstance(value, float) and not math.isfinite(value):
        raise ValueError("nonfinite JSON value")


def validate_event(raw):
    def nonfinite(_value):
        raise ValueError("nonfinite JSON value")

    payload = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object,
                         parse_constant=nonfinite)
    if (not isinstance(payload, dict) or set(payload) - {"event", "timestamp", "data"}
            or not {"event", "data"}.issubset(payload)
            or not isinstance(payload["event"], str) or payload["event"] not in EVENTS
            or not isinstance(payload["data"], dict)
            or ("timestamp" in payload and not isinstance(payload["timestamp"], str))):
        raise ValueError("invalid diagnostic event schema")
    _check_tree(payload)
    if payload["event"] == "image_sample":
        _check_image_sample(payload["data"])
        if len(raw) >= MAX_BODY_BYTES:
            raise ValueError("image sample must be smaller than 64 KiB")
    return payload


def _check_image_sample(data):
    """Keep image callbacks bounded without requiring an image-decoder runtime."""
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


class _Receiver(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, output, port, delivery_log=None):
        self.output_path = Path(output).resolve()
        self.delivery_path = Path(delivery_log).resolve() if delivery_log is not None else None
        if self.delivery_path == self.output_path:
            raise ValueError("delivery log and accepted output must be distinct")
        self.path_token = secrets.token_urlsafe(32)
        self.events_count = 0
        self._write_lock = threading.Lock()
        self._delivery_lock = threading.Lock()
        self._delivery_initialized = False
        self.delivery_count = 0
        self._delivery_bytes = 0
        self.delivery_dropped_count = 0
        self.delivery_write_errors = 0
        super().__init__(("127.0.0.1", port), _Handler)
        self.local_url = f"http://127.0.0.1:{self.server_address[1]}/{self.path_token}"
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def append_event(self, payload):
        record = {"received_at": datetime.now(timezone.utc).isoformat(),
                  "payload": payload, "source": "unverified_callback"}
        line = json.dumps(record, ensure_ascii=False, allow_nan=False)
        if self.path_token in line:
            raise ValueError("receiver URL must not appear in diagnostic data")
        with self._write_lock:
            with self.output_path.open("a", encoding="utf-8", newline="\n") as handle:
                handle.write(line + "\n")
                handle.flush()
            self.events_count += 1

    def record_delivery(self, *, method=None, path=None, status=None,
                        reason="handler_error", event="request_result",
                        response_completed=False, evidence_appended=False):
        """Best-effort bounded metadata; never persist raw request or exception data."""
        if getattr(self, "delivery_path", None) is None:
            return
        try:
            known_methods = {"POST", "GET", "PUT", "DELETE", "PATCH", "HEAD", "OPTIONS"}
            fingerprint = (hashlib.sha256(path.encode("utf-8", "surrogatepass")).hexdigest()[:16]
                           if path is not None else None)
            record = {"received_at": datetime.now(timezone.utc).isoformat(),
                      "event": event,
                      "method": method if method in known_methods else "OTHER",
                      "path_sha256": fingerprint,
                      "exact_path": path == "/" + self.path_token,
                      "http_status": status, "reason": reason,
                      "response_completed": response_completed,
                      "evidence_appended": evidence_appended}
            line = (json.dumps(record, ensure_ascii=True, allow_nan=False) + "\n").encode("ascii")
            with self._delivery_lock:
                if not self._delivery_initialized:
                    try:
                        with self.delivery_path.open("rb") as handle:
                            existing = handle.read(MAX_DELIVERY_ROWS * MAX_DELIVERY_LINE_BYTES + 1)
                    except FileNotFoundError:
                        existing = b""
                    self.delivery_count = existing.count(b"\n")
                    self._delivery_bytes = len(existing)
                    if existing and not existing.endswith(b"\n"):
                        self.delivery_count = MAX_DELIVERY_ROWS
                    self._delivery_initialized = True
                if (len(line) > MAX_DELIVERY_LINE_BYTES
                        or self.delivery_count >= MAX_DELIVERY_ROWS
                        or self._delivery_bytes + len(line) > MAX_DELIVERY_ROWS * MAX_DELIVERY_LINE_BYTES):
                    self.delivery_dropped_count += 1
                    return
                self.delivery_path.parent.mkdir(parents=True, exist_ok=True)
                with self.delivery_path.open("ab") as handle:
                    handle.write(line)
                    handle.flush()
                self.delivery_count += 1
                self._delivery_bytes += len(line)
        except Exception:
            # Optional diagnostics must not change accepted evidence or HTTP behavior.
            with self._delivery_lock:
                self.delivery_write_errors += 1

    def handle_error(self, request, client_address):
        self.record_delivery(event="handler_error")


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # BaseHTTPRequestHandler's access log would expose the secret path.
        pass

    def _reply(self, status, payload, *, reason):
        body = json.dumps(payload).encode("ascii")
        completed = False
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
            self.wfile.flush()
            completed = True
        finally:
            details = {"method": self.command, "path": self.path, "status": status,
                       "response_completed": completed,
                       "evidence_appended": getattr(self, "_evidence_appended", False)}
            self.server.record_delivery(reason=reason, **details)
            if status == 202 and not completed:
                self.server.record_delivery(event="response_delivery_failed",
                                            reason="response_write_error", **details)
        self.close_connection = True
        if status >= 400:
            self._drain_rejected_request()

    def _drain_rejected_request(self):
        # Closing with unread TCP data can erase the error response with a RST.
        # Discard a fixed budget; ambiguous Content-Length is never trusted.
        remaining = MAX_BODY_BYTES
        deadline = time.monotonic() + 0.1
        try:
            self.connection.shutdown(socket.SHUT_WR)
            while remaining:
                timeout = deadline - time.monotonic()
                if timeout <= 0:
                    break
                self.connection.settimeout(timeout)
                chunk = self.rfile.read1(min(8192, remaining))
                if not chunk:
                    break
                remaining -= len(chunk)
        except OSError:
            pass

    def do_POST(self):
        if self.path != "/" + self.server.path_token:
            return self._reply(404, {"accepted": False}, reason="path_mismatch")
        lengths = self.headers.get_all("Content-Length", [])
        if (len(lengths) != 1 or not lengths[0].isascii() or not lengths[0].isdigit()
                or self.headers.get("Transfer-Encoding") is not None):
            return self._reply(400, {"accepted": False}, reason="framing_invalid")
        normalized_length = lengths[0].lstrip("0") or "0"
        if len(normalized_length) > len(str(MAX_BODY_BYTES)):
            return self._reply(413, {"accepted": False}, reason="body_too_large")
        length = int(normalized_length)
        if length > MAX_BODY_BYTES:
            return self._reply(413, {"accepted": False}, reason="body_too_large")
        if self.headers.get_content_type() != "application/json":
            return self._reply(400, {"accepted": False}, reason="content_type_invalid")
        try:
            self.connection.settimeout(5)
            raw = self.rfile.read(length)
            if len(raw) != length:
                raise ValueError("incomplete request")
            self.server.append_event(validate_event(raw))
            self._evidence_appended = True
        except TimeoutError:
            return self._reply(400, {"accepted": False}, reason="body_timeout")
        except (ValueError, UnicodeError, RecursionError):
            return self._reply(400, {"accepted": False}, reason="event_invalid")
        except OSError:
            return self._reply(500, {"accepted": False}, reason="evidence_io_error")
        self._reply(202, {"accepted": True}, reason="accepted")

    def do_GET(self):
        exact = self.path == "/" + self.server.path_token
        self._reply(405 if exact else 404, {"accepted": False},
                    reason="method_not_allowed" if exact else "path_mismatch")

    do_PUT = do_GET
    do_DELETE = do_GET
    do_PATCH = do_GET
    do_HEAD = do_GET
    do_OPTIONS = do_GET


def create_receiver(output, port=0, *, delivery_log=None):
    """Bind a random or explicit port on 127.0.0.1, with no public-bind option."""
    if type(port) is not int or not 0 <= port <= 65535:
        raise ValueError("port must be an integer in [0,65535]")
    return _Receiver(output, port, delivery_log)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--delivery-log", type=Path, help="optional bounded request metadata JSONL")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--max-seconds", type=float, default=900)
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_seconds) or args.max_seconds <= 0:
        parser.error("--max-seconds must be finite and positive")
    try:
        server = create_receiver(args.output, args.port, delivery_log=args.delivery_log)
    except (ValueError, OSError):
        print("diagnostic_receiver: cannot initialize local receiver", file=sys.stderr)
        return 2
    timer = threading.Timer(args.max_seconds, server.shutdown)
    timer.daemon = True

    def terminate(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    previous_sigterm = signal.signal(signal.SIGTERM, terminate)
    print(json.dumps({"local_url": server.local_url, "port": server.server_address[1]}), flush=True)
    timer.start()
    try:
        server.serve_forever(poll_interval=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
        server.server_close()
        signal.signal(signal.SIGTERM, previous_sigterm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
