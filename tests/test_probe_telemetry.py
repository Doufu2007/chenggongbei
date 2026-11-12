import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread

import pytest

from inspection_mission import telemetry


def test_reporter_delivers_real_event_to_loopback_receiver():
    events = []

    class Receiver(BaseHTTPRequestHandler):
        def do_POST(self):
            events.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            self.send_response(202)
            self.end_headers()

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Receiver)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with telemetry.DiagnosticReporter(f"http://127.0.0.1:{server.server_port}/session") as reporter:
            reporter.emit("probe_started", {"read_only": True})
        assert len(events) == 1
        assert events[0]["event"] == "probe_started"
        assert events[0]["data"] == {"read_only": True}
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_reporter_is_optional_and_rejects_unsafe_endpoints():
    with telemetry.DiagnosticReporter("") as reporter:
        assert reporter.emit("probe_started", {"read_only": True}) is False
    for url in ("http://example.com/report", "https://user:pass@example.com/report", "file:///tmp/x"):
        with pytest.raises(ValueError):
            telemetry.DiagnosticReporter(url)


def test_reporter_cannot_send_credentials_or_unknown_events():
    with telemetry.DiagnosticReporter("") as reporter:
        with pytest.raises(ValueError):
            reporter.emit("probe_started", {"nested": {"password": "bad"}})
        with pytest.raises(ValueError):
            reporter.emit("referee_score", {"score": 90})


def test_launcher_reports_start_and_actual_failure_before_ros_is_available(tmp_path):
    import os
    import subprocess
    import sys
    from pathlib import Path
    from test_probe_launch import BASH, ROOT, shell_path
    if not Path(BASH).exists():
        pytest.skip("bash unavailable")
    events = []

    class Receiver(BaseHTTPRequestHandler):
        def do_POST(self):
            events.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            self.send_response(202)
            self.end_headers()

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Receiver)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        env = dict(os.environ, INSPECTION_DIAGNOSTIC_URL=f"http://127.0.0.1:{server.server_port}/session",
                   INSPECTION_DIAGNOSTIC_PYTHON=shell_path(sys.executable),
                   INSPECTION_ROS_SETUP=shell_path(tmp_path / "missing_ros.sh"))
        result = subprocess.run([BASH, str(ROOT / "scripts/run_interface_probe.sh")],
                                env=env, capture_output=True, timeout=10)
        assert result.returncode == 2
        assert [event["event"] for event in events] == ["launcher_started", "launcher_exit"]
        assert events[-1]["data"]["exit_code"] == 2
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
