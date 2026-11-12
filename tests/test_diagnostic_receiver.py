"""Local callback receipt never proves referee acceptance or an official score."""

import importlib
import base64
import http.client
import json
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.parse import urlsplit

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "diagnostic_receiver.py"


def api():
    assert TOOL.is_file(), "diagnostic receiver has not been implemented"
    return importlib.import_module("tools.diagnostic_receiver")


@pytest.fixture
def receiver(tmp_path):
    server = api().create_receiver(tmp_path / "callback.jsonl", port=0)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        thread.join(timeout=3)
        server.server_close()


def post(server, payload=None, *, path=None, raw=None, method="POST", headers=None):
    data = raw if raw is not None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(path or server.local_url, data=data, method=method,
                                     headers={"Content-Type": "application/json", **(headers or {})})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        response = opener.open(request, timeout=3)
    except urllib.error.HTTPError as exc:
        response = exc
    with response:
        return response.status, json.loads(response.read().decode("utf-8"))


def framed_post(server, headers, body=b'{}'):
    connection = http.client.HTTPConnection(*server.server_address, timeout=3)
    try:
        connection.putrequest("POST", urlsplit(server.local_url).path)
        connection.putheader("Content-Type", "application/json")
        for key, value in headers:
            connection.putheader(key, value)
        connection.endheaders(body)
        response = connection.getresponse()
        return response.status, json.loads(response.read().decode("utf-8"))
    finally:
        connection.close()


def test_receiver_binds_only_loopback_and_generates_unpredictable_distinct_paths(tmp_path):
    servers = [api().create_receiver(tmp_path / f"callback-{index}.jsonl") for index in range(2)]
    try:
        for server in servers:
            parsed = urlsplit(server.local_url)
            assert server.server_address[0] == parsed.hostname == "127.0.0.1"
            assert parsed.port == server.server_address[1] > 0
            assert len(parsed.path.strip("/")) >= 40
        assert urlsplit(servers[0].local_url).path != urlsplit(servers[1].local_url).path
    finally:
        for server in servers:
            server.server_close()


@pytest.mark.parametrize("port", [-1, 65536, True])
def test_invalid_port_is_rejected(tmp_path, port):
    with pytest.raises(ValueError, match="port"):
        api().create_receiver(tmp_path / "callback.jsonl", port=port)


def test_explicit_port_is_used(tmp_path):
    initial = api().create_receiver(tmp_path / "initial.jsonl")
    port = initial.server_address[1]
    initial.server_close()
    server = api().create_receiver(tmp_path / "callback.jsonl", port=port)
    try:
        assert server.server_address == ("127.0.0.1", port)
        assert urlsplit(server.local_url).port == port
    finally:
        server.server_close()


def test_only_exact_secret_path_accepts_posts(receiver, capsys):
    origin = "http://127.0.0.1:" + str(receiver.server_address[1])
    payload = {"event": "launcher_started", "data": {"pid": 123}}
    assert post(receiver, payload, path=origin + "/wrong")[0] == 404
    assert post(receiver, payload, path=receiver.local_url + "?extra=1")[0] == 404
    assert post(receiver, payload, method="GET")[0] == 405
    assert post(receiver, payload) == (202, {"accepted": True})
    captured = capsys.readouterr()
    assert urlsplit(receiver.local_url).path not in captured.out + captured.err


def test_accepted_event_appends_timestamped_unverified_jsonl_without_url_token(receiver):
    payload = {"event": "sample", "timestamp": "2026-11-11T01:00:00Z",
               "data": {"topic": "/odom", "fields": {"x": 0.25}}}
    assert post(receiver, payload) == (202, {"accepted": True})
    payload2 = {"event": "launcher_exit", "data": {"exit_code": 1}}
    assert post(receiver, payload2)[0] == 202
    lines = receiver.output_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == receiver.events_count == 2
    first = json.loads(lines[0])
    assert set(first) == {"received_at", "payload", "source"}
    assert first["payload"] == payload
    assert first["source"] == "unverified_callback"
    assert first["received_at"].endswith("+00:00")
    assert urlsplit(receiver.local_url).path.strip("/") not in "\n".join(lines)


def test_actual_receiver_url_or_path_token_cannot_be_written(receiver, capsys):
    for value in (receiver.local_url, receiver.path_token):
        payload = {"event": "sample", "data": {"nested": [{"note": value}]}}
        assert post(receiver, payload)[0] == 400
    assert receiver.events_count == 0
    assert not receiver.output_path.exists() or not receiver.output_path.read_bytes()
    captured = capsys.readouterr()
    assert receiver.path_token not in captured.out + captured.err


def test_concurrent_events_append_complete_distinct_jsonl_records(receiver):
    payloads = [{"event": "sample", "data": {"index": index}} for index in range(32)]
    with ThreadPoolExecutor(max_workers=8) as executor:
        statuses = list(executor.map(lambda payload: post(receiver, payload)[0], payloads))
    assert statuses == [202] * len(payloads)
    records = [json.loads(line) for line in receiver.output_path.read_text(encoding="utf-8").splitlines()]
    assert len(records) == receiver.events_count == len(payloads)
    assert sorted(record["payload"]["data"]["index"] for record in records) == list(range(32))
    assert all(record["source"] == "unverified_callback" for record in records)


@pytest.mark.parametrize("headers", [
    [],
    [("Content-Length", "2"), ("Content-Length", "2")],
    [("Content-Length", "2"), ("Transfer-Encoding", "chunked")],
    [("Content-Length", "-1")],
    [("Content-Length", "+2")],
    [("Content-Length", "2.0")],
    [("Content-Length", "\u00b2")],
])
def test_ambiguous_or_invalid_http_framing_is_rejected(receiver, headers):
    assert framed_post(receiver, headers)[0] == 400
    assert receiver.events_count == 0


@pytest.mark.parametrize("headers", [
    [("Content-Length", "2"), ("Content-Length", "2")],
    [("Content-Length", "2"), ("Transfer-Encoding", "chunked")],
])
def test_rejected_framing_delivers_400_with_unread_request_bytes(receiver, headers, monkeypatch):
    server_errors = []
    monkeypatch.setattr(receiver, "handle_error", lambda *_args: server_errors.append(sys.exc_info()[1]))
    for _ in range(20):
        # A separate TCP body write can race with header rejection on Windows.
        # It must not replace the promised 400 response with a connection reset.
        assert framed_post(receiver, headers, body=b"x" * 65536) == (400, {"accepted": False})
    assert server_errors == []
    assert receiver.events_count == 0
    assert post(receiver, {"event": "heartbeat", "data": {}})[0] == 202


def test_rejected_framing_does_not_wait_for_advertised_body_or_client_eof(receiver, monkeypatch):
    finished = threading.Event()
    shutdown_request = receiver.shutdown_request

    def observe_shutdown(request):
        try:
            shutdown_request(request)
        finally:
            finished.set()

    monkeypatch.setattr(receiver, "shutdown_request", observe_shutdown)
    path = urlsplit(receiver.local_url).path
    with socket.create_connection(receiver.server_address, timeout=1) as connection:
        request = (f"POST {path} HTTP/1.1\r\nHost: localhost\r\n"
                   "Content-Length: 999999999999999999999\r\n"
                   "Transfer-Encoding: chunked\r\n\r\n").encode("ascii")
        started = time.monotonic()
        connection.sendall(request)
        response = http.client.HTTPResponse(connection)
        response.begin()
        assert response.status == 400
        assert json.loads(response.read()) == {"accepted": False}
        # Keep our sending half open: cleanup must use its own bound.
        assert connection.recv(1) == b""
        assert finished.wait(timeout=0.5), "rejected request cleanup waited for the untrusted body"
        assert time.monotonic() - started < 0.75
    assert receiver.events_count == 0


def test_arbitrarily_large_numeric_content_length_is_rejected_without_integer_conversion(receiver):
    assert framed_post(receiver, [("Content-Length", "9" * 5000)])[0] == 413
    assert receiver.events_count == 0


@pytest.mark.parametrize("event", [
    "launcher_started", "launcher_exit", "probe_started", "public_graph", "subscription",
    "sample", "sample_error", "subscription_error", "heartbeat", "probe_finished",
    "probe_snapshot",
])
def test_all_documented_probe_events_are_accepted(receiver, event):
    assert post(receiver, {"event": event, "data": {}})[0] == 202


@pytest.mark.parametrize("payload", [
    [], {"event": "mission_completed", "data": {}},
    {"event": "sample", "data": []}, {"event": "sample"},
    {"event": "sample", "data": {}, "score": 90},
    {"event": "sample", "data": {}, "timestamp": 7},
    {"event": "sample", "data": {"password": "forbidden"}},
    {"event": "sample", "data": {"nested": [{"access_TOKEN": "forbidden"}]}},
    {"event": "sample", "data": {"cookie_jar": {}}},
    {"event": "sample", "data": {"aSecretValue": 1}},
])
def test_invalid_or_secret_bearing_schema_is_rejected_without_writing(receiver, payload):
    assert post(receiver, payload)[0] == 400
    assert receiver.events_count == 0
    assert not receiver.output_path.exists() or not receiver.output_path.read_bytes()


@pytest.mark.parametrize("raw", [
    b"{broken", b'"just-string"', b'{"event":"sample","event":"heartbeat","data":{}}',
    b'{"event":"sample","data":{"value":NaN}}',
    b'{"event":"sample","data":{"value":Infinity}}',
    b'\xff\xfe',
])
def test_malformed_duplicate_or_nonfinite_json_is_rejected(receiver, raw):
    assert post(receiver, raw=raw)[0] == 400
    assert receiver.events_count == 0


def test_oversized_payload_is_rejected_before_schema_or_storage(receiver):
    assert framed_post(receiver, [("Content-Length", str(64 * 1024 + 1))], body=b"")[0] == 413
    assert receiver.events_count == 0


def test_exact_64kb_valid_json_is_accepted(receiver):
    prefix = b'{"event":"sample","data":{"text":"'
    suffix = b'"}}'
    raw = prefix + b"x" * (65536 - len(prefix) - len(suffix)) + suffix
    assert len(raw) == 65536
    assert post(receiver, raw=raw)[0] == 202


def test_payload_depth_limit_is_enforced(receiver):
    data = {"leaf": 0}
    for _ in range(9):
        data = {"level": data}
    assert post(receiver, {"event": "sample", "data": data})[0] == 202
    assert post(receiver, {"event": "sample", "data": {"level": data}})[0] == 400
    assert receiver.events_count == 1


@pytest.mark.parametrize('delivery_enabled', [False, True])
def test_cli_emits_only_startup_url_then_exits_on_deadline(tmp_path, delivery_enabled):
    api()
    output = tmp_path / "callback.jsonl"
    optional_args = ['--delivery-log', str(tmp_path / 'delivery.jsonl')] if delivery_enabled else []
    result = subprocess.run([sys.executable, str(TOOL), "--output", str(output),
                             "--port", "0", "--max-seconds", "0.1", *optional_args],
                            cwd=ROOT, capture_output=True, text=True, encoding="utf-8", timeout=5)
    assert result.returncode == 0, result.stderr
    startup = json.loads(result.stdout)
    assert set(startup) == {"local_url", "port"}
    assert urlsplit(startup["local_url"]).hostname == "127.0.0.1"
    assert startup["port"] > 0
    assert not result.stderr
    assert not output.exists() or not output.read_bytes()


@pytest.mark.parametrize("interrupt", ["sigterm", "keyboard"])
def test_cli_releases_port_after_graceful_interrupt(tmp_path, interrupt):
    output = tmp_path / "callback.jsonl"
    script = (
        "import _thread,signal,sys,threading; "
        "from tools.diagnostic_receiver import main; "
        "interrupt = (lambda: signal.raise_signal(signal.SIGTERM)) "
        "if sys.argv[2] == 'sigterm' else _thread.interrupt_main; "
        "threading.Timer(0.3, interrupt).start(); "
        "raise SystemExit(main(['--output',sys.argv[1],'--max-seconds','30']))"
    )
    result = subprocess.run([sys.executable, "-c", script, str(output), interrupt],
                            cwd=ROOT, capture_output=True, text=True, encoding="utf-8", timeout=5)
    assert result.returncode == 0, result.stderr
    startup = json.loads(result.stdout)
    assert not result.stderr
    replacement = api().create_receiver(output, port=startup["port"])
    replacement.server_close()


def snapshot_event():
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    ok, encoded = cv2.imencode(".jpg", np.full((32, 64, 3), 180, dtype=np.uint8))
    assert ok
    return {"event": "image_sample", "data": {"topic": "/camera/image", "stamp": 7.25,
        "frame_id": "camera_optical", "width": 64, "height": 32, "encoding": "jpeg",
        "jpeg_base64": base64.b64encode(encoded.tobytes()).decode("ascii")}}


def test_real_jpeg_callback_is_stored_only_as_unverified_evidence(receiver):
    payload = snapshot_event()
    assert post(receiver, payload)[0] == 202
    record = json.loads(receiver.output_path.read_text(encoding="utf-8"))
    assert record["source"] == "unverified_callback"
    assert record["payload"] == payload


@pytest.mark.parametrize("changes", [
    {"score": 100}, {"width": 481}, {"height": 361}, {"width": True},
    {"width": 0}, {"stamp": -1}, {"encoding": "rgb8"},
    {"jpeg_base64": "not-base64"},
    {"jpeg_base64": base64.b64encode(b"not-jpeg").decode("ascii")},
    {"topic": "/gazebo/camera/image"}, {"topic": "/referee/hidden_image"},
    {"topic": "/ican_referee_private/camera"}, {"topic": "/robot/ground_truth/image"},
])
def test_image_callback_rejects_unbounded_or_nonimage_schema(receiver, changes):
    payload = snapshot_event()
    payload["data"].update(changes)
    assert post(receiver, payload)[0] == 400
    assert receiver.events_count == 0


def test_image_callback_requires_all_snapshot_fields(receiver):
    payload = snapshot_event()
    del payload["data"]["frame_id"]
    assert post(receiver, payload)[0] == 400
    assert receiver.events_count == 0


@pytest.fixture
def delivery_receiver(tmp_path):
    server = api().create_receiver(tmp_path / 'accepted.jsonl', port=0,
                                   delivery_log=tmp_path / 'delivery.jsonl')
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown(); thread.join(timeout=3); server.server_close()


def wait_for_delivery(predicate):
    deadline = time.monotonic() + 2
    while not predicate():
        assert time.monotonic() < deadline, 'delivery recording did not finish'
        time.sleep(.01)


def delivery_rows(server, expected=1):
    wait_for_delivery(lambda: server.delivery_count >= expected)
    return [json.loads(line) for line in server.delivery_path.read_text('utf-8').splitlines()]


def test_optional_delivery_ledger_separates_rejections_from_accepted_evidence(delivery_receiver):
    import hashlib
    server = delivery_receiver
    origin = 'http://127.0.0.1:' + str(server.server_address[1])
    payload = {'event': 'launcher_started', 'data': {}}
    assert post(server, payload, path=origin + '/wrong?private=value')[0] == 404
    assert post(server, payload, method='GET')[0] == 405
    assert post(server, raw=b'{broken')[0] == 400
    assert framed_post(server, [('Content-Length', '65537')], b'')[0] == 413
    assert post(server, payload)[0] == 202
    rows = delivery_rows(server, expected=5)
    assert [r['reason'] for r in rows] == ['path_mismatch', 'method_not_allowed',
                                         'event_invalid', 'body_too_large', 'accepted']
    assert [r['http_status'] for r in rows] == [404, 405, 400, 413, 202]
    assert rows[0]['exact_path'] is False and rows[-1]['exact_path'] is True
    assert rows[-1]['path_sha256'] == hashlib.sha256(('/' + server.path_token).encode()).hexdigest()[:16]
    assert all(r['received_at'].endswith('+00:00') for r in rows)
    assert all(len(line) <= 1024 for line in server.delivery_path.read_bytes().splitlines(keepends=True))
    text = server.delivery_path.read_text('utf-8')
    assert server.path_token not in text and '/wrong' not in text and 'private=value' not in text
    accepted = [json.loads(line) for line in server.output_path.read_text('utf-8').splitlines()]
    assert len(accepted) == server.events_count == 1
    assert set(accepted[0]) == {'received_at', 'payload', 'source'}
    assert accepted[0]['payload'] == payload


def test_delivery_log_is_opt_in_and_may_not_share_accepted_path(tmp_path, receiver):
    assert getattr(receiver, 'delivery_path', 'missing') is None
    target = tmp_path / 'same.jsonl'
    with pytest.raises(ValueError, match='distinct'):
        api().create_receiver(target, delivery_log=target)


def test_delivery_ledger_failure_does_not_change_accepted_post(delivery_receiver):
    server = delivery_receiver
    # Existing directory causes an OSError at ledger append; evidence remains writable.
    server.delivery_path.mkdir()
    assert post(server, {'event': 'heartbeat', 'data': {}})[0] == 202
    assert server.events_count == 1
    wait_for_delivery(lambda: server.delivery_write_errors == 1)
    assert server.delivery_write_errors == 1


def test_delivery_ledger_hard_row_bound_keeps_accepted_traffic(delivery_receiver, monkeypatch):
    server = delivery_receiver
    monkeypatch.setattr(api(), 'MAX_DELIVERY_ROWS', 3)
    for i in range(5):
        assert post(server, {'event': 'heartbeat', 'data': {'index': i}})[0] == 202
    wait_for_delivery(lambda: server.delivery_dropped_count == 2)
    assert len(delivery_rows(server, expected=3)) == 3
    assert server.delivery_dropped_count == 2
    assert server.events_count == 5


def test_accepted_202_response_failure_has_a_separate_failure_record(tmp_path):
    import io
    module = api()
    server = module.create_receiver(tmp_path / 'accepted.jsonl',
                                    delivery_log=tmp_path / 'delivery.jsonl')
    try:
        class BrokenStream(io.BytesIO):
            def write(self, value): raise BrokenPipeError('must-not-persist-exception-text')
        class Connection:
            def settimeout(self, value): pass
        raw = b'{"event":"launcher_started","data":{}}'
        path = '/' + server.path_token
        wire = (f'POST {path} HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\n'
                f'Content-Length: {len(raw)}\r\n\r\n').encode() + raw
        handler = module._Handler.__new__(module._Handler)
        handler.server = server; handler.connection = Connection()
        handler.rfile = io.BytesIO(wire); handler.wfile = BrokenStream()
        handler.close_connection = True
        with pytest.raises(BrokenPipeError): handler.handle_one_request()
        assert server.events_count == 1
        rows = delivery_rows(server)
        assert [r['event'] for r in rows] == ['request_result', 'response_delivery_failed']
        assert rows[0]['reason'] == 'accepted' and rows[0]['http_status'] == 202
        assert rows[0]['response_completed'] is False
        assert rows[1]['reason'] == 'response_write_error'
        text = server.delivery_path.read_text('utf-8')
        assert 'must-not-persist' not in text and server.path_token not in text
    finally:
        server.server_close()


def test_delivery_rejection_reason_codes_cover_framing_content_type_and_evidence_io(delivery_receiver):
    server = delivery_receiver
    assert framed_post(server, [('Content-Length', '2'), ('Content-Length', '2')])[0] == 400
    assert post(server, {'event': 'heartbeat', 'data': {}},
                headers={'Content-Type': 'text/plain'})[0] == 400
    server.output_path.mkdir()
    assert post(server, {'event': 'heartbeat', 'data': {}})[0] == 500
    rows = delivery_rows(server, expected=3)
    assert [r['reason'] for r in rows] == ['framing_invalid', 'content_type_invalid', 'evidence_io_error']
    assert all(r['evidence_appended'] is False for r in rows)
    assert server.events_count == 0


def test_delivery_concurrent_bound_preserves_all_accepted_events(delivery_receiver, monkeypatch):
    server = delivery_receiver
    monkeypatch.setattr(api(), 'MAX_DELIVERY_ROWS', 3)
    payloads = [{'event': 'sample', 'data': {'index': i}} for i in range(20)]
    with ThreadPoolExecutor(max_workers=8) as executor:
        statuses = list(executor.map(lambda payload: post(server, payload)[0], payloads))
    wait_for_delivery(lambda: server.delivery_dropped_count == 17)
    assert statuses == [202] * 20
    assert len(delivery_rows(server, expected=3)) == 3
    assert server.events_count == 20
    records = [json.loads(line) for line in server.output_path.read_text('utf-8').splitlines()]
    assert sorted(r['payload']['data']['index'] for r in records) == list(range(20))


def test_delivery_bound_counts_existing_rows(delivery_receiver, monkeypatch):
    server = delivery_receiver
    monkeypatch.setattr(api(), 'MAX_DELIVERY_ROWS', 3)
    existing = b'{}\n' * 3
    server.delivery_path.write_bytes(existing)
    assert post(server, {'event': 'heartbeat', 'data': {}})[0] == 202
    wait_for_delivery(lambda: server.delivery_dropped_count == 1)
    assert server.delivery_path.read_bytes() == existing
    assert server.events_count == 1


def test_delivery_handler_error_records_no_address_or_exception_text(delivery_receiver):
    server = delivery_receiver
    try:
        raise OSError('must-not-persist-runtime-exception')
    except OSError:
        server.handle_error(None, ('192.0.2.123', 54321))
    record, = delivery_rows(server)
    assert record['event'] == record['reason'] == 'handler_error'
    assert record['path_sha256'] is None and record['http_status'] is None
    assert record['exact_path'] is False and record['method'] == 'OTHER'
    text = server.delivery_path.read_text('utf-8')
    assert '192.0.2.123' not in text and 'must-not-persist' not in text
