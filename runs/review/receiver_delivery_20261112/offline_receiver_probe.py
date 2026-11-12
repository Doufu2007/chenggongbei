"""Real HTTP handler on in-memory streams. Never creates/binds a socket."""
from __future__ import annotations
import hashlib
import io
import json
from pathlib import Path
import sys
import threading
import uuid

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools import diagnostic_receiver as production

# Artificial fixture value, never a real receiver path or startup URL.
FIXTURE_TOKEN = 'offline-fixture-not-a-live-receiver-token'


class MemoryConnection:
    def settimeout(self, value): self.timeout = value
    def shutdown(self, how): pass


class ResponseWriteFails(io.BytesIO):
    def write(self, value): raise BrokenPipeError('fixture-client-disconnected')


class BodyReadTimesOut(io.BytesIO):
    def read(self, size=-1): raise TimeoutError('fixture-body-timeout')


def make_request(method='POST', path=None, raw=None, headers=None):
    raw = raw if raw is not None else b'{"event":"launcher_started","data":{"pid":123}}'
    path = '/' + FIXTURE_TOKEN if path is None else path
    pairs = [('Content-Type', 'application/json'), ('Content-Length', str(len(raw)))] if headers is None else headers
    return (f'{method} {path} HTTP/1.1\r\nHost: offline\r\n' +
            ''.join(f'{k}: {v}\r\n' for k,v in pairs) + '\r\n').encode('ascii') + raw


def execute(directory: Path, name: str, wire: bytes, *, sink_error=False,
            reply_error=False, body_timeout=False):
    directory = Path(directory)/name
    directory.mkdir(parents=True, exist_ok=True)
    server = production._Receiver.__new__(production._Receiver)
    server.output_path = directory/'accepted.jsonl'
    server.path_token = FIXTURE_TOKEN
    server.events_count = 0
    server._write_lock = threading.Lock()
    # Preserve the original default-off behavior without constructing a socket.
    candidate_delivery_path = directory/'accepted.jsonl.delivery.jsonl'
    server.delivery_path = None
    if sink_error:
        def refused(payload): raise OSError('fixture-evidence-sink-denied')
        server.append_event = refused
    handler = production._Handler.__new__(production._Handler)
    handler.server = server
    handler.connection = MemoryConnection()
    handler.request = handler.connection
    handler.client_address = ('offline-fixture',0)
    handler.rfile = io.BytesIO(wire)
    handler.wfile = ResponseWriteFails() if reply_error else io.BytesIO()
    handler.close_connection = True
    error_type = None
    if body_timeout:
        # Parse real HTTP headers first; only the POST body read times out.
        handler.raw_requestline = handler.rfile.readline(65537)
        assert handler.parse_request()
        handler.rfile = BodyReadTimesOut()
        invoke = handler.do_POST
    else:
        invoke = handler.handle_one_request
    try:
        invoke()
    except Exception as exc:
        error_type = type(exc).__name__
        server.handle_error(None, None)
    response = handler.wfile.getvalue()
    status = int(response.split(b' ',2)[1]) if response.startswith(b'HTTP/') else None
    accepted = [json.loads(line) for line in server.output_path.read_text('utf-8').splitlines()] if server.output_path.exists() else []
    deliveries = [json.loads(line) for line in candidate_delivery_path.read_text('utf-8').splitlines()] if candidate_delivery_path.exists() else []
    return {'case':name,'http_status':status,'events_count':server.events_count,
            'accepted_count':len(accepted),'accepted_event_names':[r['payload']['event'] for r in accepted],
            'uncaught_error_type':error_type,'delivery_count':len(deliveries),
            'accepted':accepted,'deliveries':deliveries,'directory':str(directory)}


def cases(directory):
    return [
        execute(directory,'accepted_launcher',make_request()),
        execute(directory,'wrong_path',make_request(path='/wrong-fixture-path')),
        execute(directory,'exact_token_query_rejected',make_request(path='/'+FIXTURE_TOKEN+'?extra=1')),
        execute(directory,'get_exact_path',make_request(method='GET')),
        execute(directory,'invalid_json',make_request(raw=b'{broken')),
        execute(directory,'content_type',make_request(headers=[('Content-Type','text/plain'),('Content-Length','2')],raw=b'{}')),
        execute(directory,'duplicate_length',make_request(headers=[('Content-Type','application/json'),('Content-Length','2'),('Content-Length','2')],raw=b'{}')),
        execute(directory,'too_large',make_request(headers=[('Content-Type','application/json'),('Content-Length','65537')],raw=b'')),
        execute(directory,'short_body',make_request(headers=[('Content-Type','application/json'),('Content-Length','99')],raw=b'{}')),
        execute(directory,'body_timeout',make_request(),body_timeout=True),
        execute(directory,'sink_error',make_request(),sink_error=True),
        execute(directory,'reply_write_error',make_request(),reply_error=True),
        execute(directory,'unsupported_method',make_request(method='TRACE')),
    ]


if __name__=='__main__':
    output = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent/'offline_cases'/uuid.uuid4().hex
    observed = cases(output)
    result={'source_sha256':hashlib.sha256((ROOT/'tools/diagnostic_receiver.py').read_bytes()).hexdigest(),
            'transport':'in_memory_real_handler_no_sockets',
            'cases':[{k:v for k,v in row.items() if k not in ('accepted','deliveries','directory')} for row in observed]}
    destination=Path(__file__).parent/'observed.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
