"""Execute the real embedded sender and harmless shell entrypoint fixtures."""
import contextlib
import hashlib
import http.server
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import urllib.error
import urllib.request

import pytest


ROOT = Path(__file__).resolve().parents[1]
V42 = ROOT/'scripts/run_full_inspection_v42.sh'
SCRIPT = Path(os.environ.get('INSPECTION_LAUNCHER_TEST_SCRIPT', V42))
BASH = shutil.which('bash') if os.name != 'nt' else r'C:\Program Files\Git\bin\bash.exe'
CANARY_URL = 'https://private-endpoint.invalid/PRIVATE_PATH_CANARY'


def sender_source():
    text = SCRIPT.read_text(encoding='utf-8')
    return text.split("<<'PY'", 1)[1].split('\n', 1)[1].split('\nPY\n', 1)[0]


def receiver_validate(raw):
    spec = importlib.util.spec_from_file_location('launcher_receiver_schema', ROOT/'tools/diagnostic_receiver.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module.validate_event(raw)


def run_sender(monkeypatch, outcomes, *, event='launcher_started', detail=''):
    attempts = []
    class Response:
        def __init__(self, status, body): self.status = status; self.body = body
        def __enter__(self): return self
        def __exit__(self, *args): return False
        def read(self, size=-1): return self.body if size < 0 else self.body[:size]
        def getcode(self): return self.status
    class Opener:
        def open(self, request, timeout):
            attempts.append({'request': request, 'timeout': timeout})
            outcome = outcomes[min(len(attempts)-1, len(outcomes)-1)]
            if isinstance(outcome, Exception): raise outcome
            return Response(*outcome)
    monkeypatch.setenv('INSPECTION_DIAGNOSTIC_URL', CANARY_URL)
    monkeypatch.setattr(urllib.request, 'build_opener', lambda *args: Opener())
    monkeypatch.setattr(sys, 'argv', ['launcher-inline', event, detail])
    stdout, stderr = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
        try:
            exec(compile(sender_source(), str(SCRIPT)+':inline', 'exec'), {})
        except SystemExit as exc:
            assert exc.code in (None, 0)
    return attempts, stdout.getvalue()+stderr.getvalue()


def assert_private_logs(logs):
    assert CANARY_URL not in logs and 'PRIVATE_PATH_CANARY' not in logs
    assert 'PRIVATE_EXCEPTION_CANARY' not in logs and 'private-endpoint' not in logs
    for line in logs.splitlines():
        assert re.fullmatch(r'event=(launcher_started|launcher_exit|heartbeat|invalid_event) attempt=[1-3] exception=[A-Za-z][A-Za-z0-9_]* status=(none|[0-9]{3})', line)


def test_timeout_retries_three_times_without_leaking_exception_text(monkeypatch):
    attempts, logs = run_sender(monkeypatch, [TimeoutError('PRIVATE_EXCEPTION_CANARY '+CANARY_URL)])
    assert len(attempts) == 3
    assert [a['timeout'] for a in attempts] == [3, 3, 3]
    assert len(logs.splitlines()) == 3 and 'exception=TimeoutError' in logs
    assert_private_logs(logs)


@pytest.mark.parametrize('rejected', [
    (200, b'{"accepted":true}'), (202, b'{"accepted":false}'),
    (202, b'{"accepted":1}'), (202, b'{"not_accepted":true}'),
    (202, b'not JSON PRIVATE_EXCEPTION_CANARY'), (503, b'private response'),
])
def test_only_202_and_json_true_are_accepted(monkeypatch, rejected):
    attempts, logs = run_sender(monkeypatch, [rejected, (202, b'{"accepted":true}')])
    assert len(attempts) == 2
    assert_private_logs(logs)


def test_accepted_response_stops_retry_and_matches_existing_receiver(monkeypatch):
    attempts, logs = run_sender(monkeypatch, [(202, b'{"accepted":true}')])
    assert len(attempts) == 1 and logs == ''
    request = attempts[0]['request']; payload = receiver_validate(request.data)
    assert request.method == 'POST' and request.get_header('Content-type') == 'application/json'
    assert payload['event'] == 'launcher_started'
    assert payload['data']['revision'] == 'full_inspection_v42'
    assert payload['data']['official_scoring'] == 'unverified'


def test_http_error_has_status_but_no_reason_url_or_body(monkeypatch):
    error = urllib.error.HTTPError(CANARY_URL, 502, 'PRIVATE_EXCEPTION_CANARY', {}, None)
    attempts, logs = run_sender(monkeypatch, [error])
    assert len(attempts) == 3 and 'exception=HTTPError status=502' in logs
    assert_private_logs(logs)


def test_heartbeat_stage_is_accepted_by_unchanged_receiver(monkeypatch):
    attempts, _ = run_sender(monkeypatch, [(202, b'{"accepted":true}')], event='heartbeat', detail='ros_setup_complete')
    payload = receiver_validate(attempts[0]['request'].data)
    assert payload['event'] == 'heartbeat'
    assert payload['data']['startup_stage'] == 'ros_setup_complete'


def test_exit_payload_keeps_original_task_status(monkeypatch):
    attempts, logs = run_sender(monkeypatch, [(400, b'no')], event='launcher_exit', detail='37')
    assert len(attempts) == 3
    assert all(receiver_validate(a['request'].data)['data']['exit_code'] == 37 for a in attempts)
    assert_private_logs(logs)


@pytest.mark.parametrize('task_status', [0, 37])
def test_real_shell_preserves_task_exit_after_failed_diagnostics(tmp_path, task_status):
    if not Path(BASH).exists(): pytest.skip('bash required for real launcher fixture')
    events = []
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            raw = self.rfile.read(int(self.headers['Content-Length']))
            events.append(receiver_validate(raw))
            body = b'{"accepted":false}'
            self.send_response(202); self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)
        def log_message(self, *args): pass
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    fixture = tmp_path/'workspace'; (fixture/'src/inspection_mission/inspection_mission').mkdir(parents=True)
    (fixture/'src/inspection_mission/inspection_mission/full_inspection_runtime.py').write_text('# never executed\n')
    (fixture/'ros_setup.bash').write_text(':\n', newline='\n')
    bindir = tmp_path/'bin'; bindir.mkdir(); capture = tmp_path/'task_args.txt'
    shim = bindir/'python3'; shim.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "$TASK_CAPTURE"\nexit "$TASK_STATUS"\n', newline='\n')
    shim.chmod(0o755)
    script_copy = fixture/'launcher.sh'; script_copy.write_bytes(SCRIPT.read_bytes())
    env = os.environ.copy(); env.update(INSPECTION_DIAGNOSTIC_URL=f'http://127.0.0.1:{server.server_port}/local-test',
        INSPECTION_DIAGNOSTIC_PYTHON=Path(sys.executable).as_posix(), INSPECTION_ROS_SETUP=(fixture/'ros_setup.bash').as_posix(),
        TASK_CAPTURE=capture.as_posix(), TASK_STATUS=str(task_status))
    try:
        if os.name == 'nt':
            command = ['-c', 'export PATH="$(cygpath -u "$1"):$PATH"; bash "$2"', 'launcher-fixture', bindir.as_posix(), script_copy.as_posix()]
        else:
            env['PATH'] = str(bindir)+os.pathsep+env['PATH']; command = [str(script_copy)]
        completed = subprocess.run([BASH, *command], cwd=fixture, env=env, capture_output=True, text=True, timeout=20)
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=1.)
    assert completed.returncode == task_status, completed.stderr
    assert capture.read_text().splitlines() == ['-u', '-m', 'inspection_mission.full_inspection_runtime', '--ground-duration', '1800', '--flight-duration', '900']
    assert [p['event'] for p in events].count('launcher_started') == 3
    exit_events = [p for p in events if p['event'] == 'launcher_exit']
    assert len(exit_events) == 3 and all(p['data']['exit_code'] == task_status for p in exit_events)
    assert {p['data'].get('startup_stage') for p in events if p['event'] == 'heartbeat'} == {'ros_setup_complete', 'source_located'}
    assert 'local-test' not in completed.stderr and 'http://' not in completed.stderr


def test_frozen_v41_submission_bytes_are_unchanged():
    expected_script = '7eeb6f05399bdfd82b538322065fa0fb9a2e4404ac5576ac986716a729b50309'
    script_hash = hashlib.sha256((ROOT/'dist/run_full_mission_v41.sh').read_bytes()).hexdigest()
    public_manifest = ROOT/'PUBLIC_SNAPSHOT_MANIFEST.json'
    if public_manifest.exists():
        # A public copy removes the private callback URL. Verify both provenances.
        records = json.loads(public_manifest.read_text(encoding='utf-8'))['files']
        entry = next(v for v in records if v['path'] == 'dist/run_full_mission_v41.sh')
        assert entry['source_sha256'] == expected_script
        assert entry['sanitized'] is True
        assert script_hash == entry['published_sha256']
    else:
        assert script_hash == expected_script
    assert hashlib.sha256((ROOT/'dist/inspection_full_mission_v41_src.zip').read_bytes()).hexdigest() == 'a2eaafc89bbfc8a185496e5df0f707f365652790d0747b548aa29cd8d9c6663d'
