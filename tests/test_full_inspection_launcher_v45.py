"""Run the actual launcher with harmless foreground child and real Bash signals."""
import contextlib
import http.server
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest
import test_full_inspection_launcher_v42 as shared

ROOT = Path(__file__).resolve().parents[1]
V45 = ROOT / 'scripts/run_full_inspection_v45.sh'
# Before v45 exists, exercise v44's real behavior to obtain a useful red test.
SCRIPT = V45 if V45.exists() else ROOT / 'scripts/run_full_inspection_v44.sh'
BASH = shared.BASH


@contextlib.contextmanager
def launcher_fixture(tmp_path, *, signal_child=False, task_status=0):
    if not Path(BASH).exists():
        pytest.skip('Bash required for actual launcher signal test')
    events = []
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            raw = self.rfile.read(int(self.headers['Content-Length']))
            events.append(shared.receiver_validate(raw))
            body = b'{"accepted":true}'
            self.send_response(202)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def log_message(self, *args):
            pass
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    workspace = tmp_path / 'workspace'
    (workspace / 'src/inspection_mission/inspection_mission').mkdir(parents=True)
    (workspace / 'src/inspection_mission/inspection_mission/full_inspection_runtime.py').write_text('# harmless fixture\n')
    (workspace / 'ros_setup.bash').write_text(':\n', newline='\n')
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    pidfile, child_events = tmp_path / 'child.pid', tmp_path / 'child.events'
    shim = bindir / 'python3'
    shim.write_text(
        '#!/usr/bin/env bash\n'
        'printf "%s\\n" "$@" > "$TASK_CAPTURE"\n'
        'if [[ "$SIGNAL_CHILD" != 1 ]]; then exit "$TASK_STATUS"; fi\n'
        'on_term() { printf "TERM\\n" >> "$CHILD_EVENTS"; exit 143; }\n'
        'on_int() { printf "INT\\n" >> "$CHILD_EVENTS"; exit 130; }\n'
        'trap on_term TERM\ntrap on_int INT\n'
        'printf "%s %s\\n" "$PPID" "$BASHPID" > "$CHILD_PIDFILE"\n'
        'for _ in {1..200}; do sleep .05; done\n'
        'printf "CONTINUED\\n" >> "$CHILD_EVENTS"\nexit 0\n', newline='\n')
    shim.chmod(0o755)
    script_copy = workspace / 'launcher.sh'
    script_copy.write_bytes(SCRIPT.read_bytes())
    capture = tmp_path / 'task_args.txt'
    env = os.environ.copy()
    env.update(INSPECTION_DIAGNOSTIC_URL=f'http://127.0.0.1:{server.server_port}/local-test',
        INSPECTION_DIAGNOSTIC_PYTHON=Path(sys.executable).as_posix(),
        INSPECTION_ROS_SETUP=(workspace / 'ros_setup.bash').as_posix(),
        TASK_CAPTURE=capture.as_posix(), TASK_STATUS=str(task_status),
        SIGNAL_CHILD='1' if signal_child else '0', CHILD_PIDFILE=pidfile.as_posix(),
        CHILD_EVENTS=child_events.as_posix())
    if os.name == 'nt':
        command = [BASH, '-c', 'export PATH="$(cygpath -u "$1"):$PATH"; exec bash "$2"',
                   'v45-launcher-fixture', bindir.as_posix(), script_copy.as_posix()]
    else:
        env['PATH'] = str(bindir) + os.pathsep + env['PATH']
        command = [BASH, str(script_copy)]
    process = subprocess.Popen(command, cwd=workspace, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        yield process, events, pidfile, child_events, capture
    finally:
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=15)
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.)


@pytest.mark.parametrize(('signal_name', 'status'), [('INT', 130), ('TERM', 143)])
def test_real_foreground_parent_and_child_signal_has_nonzero_code_and_signal_metadata(tmp_path, signal_name, status):
    with launcher_fixture(tmp_path, signal_child=True) as (process, events, pidfile, child_events, capture):
        deadline = time.monotonic() + 8.
        while not pidfile.exists() and time.monotonic() < deadline and process.poll() is None:
            time.sleep(.02)
        assert pidfile.exists(), 'foreground child did not start'
        parent_pid, child_pid = pidfile.read_text().strip().split()
        assert parent_pid.isdigit() and child_pid.isdigit() and parent_pid != child_pid
        # Real Bash PIDs are needed on Windows; Win32 Popen.pid is not a POSIX PID.
        delivered = subprocess.run([BASH, '-c', 'kill -s "$1" "$2" "$3"',
                                    'v45-signal-fixture', signal_name, parent_pid, child_pid], capture_output=True, text=True, timeout=3)
        assert delivered.returncode == 0, delivered.stderr
        stdout, stderr = process.communicate(timeout=8)
        assert process.returncode == status, stderr
        exits = [p for p in events if p['event'] == 'launcher_exit']
        assert len(exits) == 1
        assert exits[0]['data']['exit_code'] == status
        assert exits[0]['data'].get('termination_signal') == signal_name
        assert child_events.read_text().splitlines() == [signal_name]
        assert 'local-test' not in stdout + stderr and 'http://' not in stdout + stderr
        assert f'termination_signal={signal_name}' in stderr
        assert capture.read_text().splitlines() == ['-u', '-m', 'inspection_mission.full_inspection_runtime',
            '--ground-duration', '1800', '--flight-duration', '900']


@pytest.mark.parametrize('task_status', [0, 37])
def test_normal_foreground_status_is_unchanged_and_has_no_signal(tmp_path, task_status):
    with launcher_fixture(tmp_path, task_status=task_status) as (process, events, *rest):
        process.communicate(timeout=10)
        assert process.returncode == task_status
        exits = [p for p in events if p['event'] == 'launcher_exit']
        assert len(exits) == 1 and exits[0]['data']['exit_code'] == task_status
        assert 'termination_signal' not in exits[0]['data']


def test_signal_payload_is_accepted_by_unchanged_receiver(monkeypatch):
    monkeypatch.setattr(shared, 'SCRIPT', SCRIPT)
    monkeypatch.setattr(sys, 'argv', ['launcher-inline', 'launcher_exit', '143', 'TERM'])
    monkeypatch.setenv('INSPECTION_DIAGNOSTIC_URL', shared.CANARY_URL)
    attempts = []
    class Response:
        status = 202
        def __enter__(self): return self
        def __exit__(self, *args): return False
        def read(self, size): return b'{"accepted":true}'
    class Opener:
        def open(self, request, timeout):
            attempts.append(request)
            return Response()
    monkeypatch.setattr(shared.urllib.request, 'build_opener', lambda *args: Opener())
    exec(compile(shared.sender_source(), str(SCRIPT) + ':inline', 'exec'), {})
    assert len(attempts) == 1
    payload = shared.receiver_validate(attempts[0].data)
    assert payload['data']['termination_signal'] == 'TERM'
    assert payload['data']['exit_code'] == 143
