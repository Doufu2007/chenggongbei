#!/usr/bin/env bash
set -Eeuo pipefail
export PYTHONUNBUFFERED=1
export INSPECTION_DIAGNOSTIC_URL="${INSPECTION_DIAGNOSTIC_URL:-}"

printf '%s\n' 'FULL_INSPECTION launch_start read_only=false official_scoring=unverified'
diagnostic_event() {
  [[ -n "${INSPECTION_DIAGNOSTIC_URL:-}" ]] || return 0
  local event="${1:-invalid_event}"
  case "$event" in launcher_started|launcher_exit|heartbeat) ;; *) event=invalid_event ;; esac
  # Only intentionally sanitized Python output reaches stderr. Interpreter
  # errors/tracebacks are suppressed and replaced by the fixed message below.
  if ! "${INSPECTION_DIAGNOSTIC_PYTHON:-python3}" - "$event" "${2:-}" <<'PY' 1>&2 2>/dev/null
import datetime, json, os, sys, urllib.error, urllib.parse, urllib.request

event = sys.argv[1] if sys.argv[1] in ("launcher_started", "launcher_exit", "heartbeat") else "invalid_event"
detail = sys.argv[2]

def diagnostic_failure(attempt, exception, status=None):
    name = exception if exception.isascii() and exception.isidentifier() else "DiagnosticError"
    code = str(status) if type(status) is int and 100 <= status <= 599 else "none"
    print(f"event={event} attempt={attempt} exception={name} status={code}", flush=True)

url = os.environ.get("INSPECTION_DIAGNOSTIC_URL", "")
attempt_limit = 1 if event == "heartbeat" else 3
for attempt in range(1, attempt_limit + 1):
    status = None
    try:
        p = urllib.parse.urlsplit(url)
        loopback = p.scheme == "http" and p.hostname in ("localhost", "127.0.0.1", "::1")
        if (not (p.scheme == "https" or loopback) or not p.hostname
                or p.username or p.password or p.fragment or event == "invalid_event"):
            raise ValueError("invalid diagnostic configuration")
        data = {"read_only": False, "mode": "full_inspection_runtime", "revision": "full_inspection_v42", "official_scoring": "unverified"}
        if event == "launcher_exit":
            data["exit_code"] = int(detail)
        elif event == "heartbeat":
            if detail not in ("ros_setup_complete", "source_located"):
                raise ValueError("invalid startup stage")
            data["startup_stage"] = detail
        payload = json.dumps({"event": event, "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(), "data": data}).encode()
        request = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=3) as response:
            status = response.status
            if status != 202:
                raise ValueError("unexpected diagnostic status")
            body = response.read(4097)
            if len(body) > 4096:
                raise ValueError("diagnostic response too large")
            accepted = json.loads(body.decode("utf-8"))
            if not isinstance(accepted, dict) or accepted.get("accepted") is not True:
                raise ValueError("diagnostic response rejected")
        break
    except Exception as exc:
        if isinstance(exc, urllib.error.HTTPError):
            status = exc.code
            exc.close()
        diagnostic_failure(attempt, type(exc).__name__, status)
PY
  then
    printf 'event=%s attempt=1 exception=InterpreterError status=none\n' "$event" >&2
  fi
  return 0
}
on_exit() {
  local status=$?
  trap - EXIT
  diagnostic_event launcher_exit "$status"
  printf 'FULL_INSPECTION launch_exit exit_code=%s\n' "$status" >&2
  exit "$status"
}
trap on_exit EXIT
diagnostic_event launcher_started

ROS_SETUP="${INSPECTION_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'FULL_INSPECTION error=ros_setup_missing path=%s\n' "$ROS_SETUP" >&2
  exit 2
fi
set +u
source "$ROS_SETUP"
set -u
diagnostic_event heartbeat ros_setup_complete
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SCRIPT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
SOURCE_ROOT=""
for root in "$SCRIPT_DIR" "$SCRIPT_ROOT" "$PWD" "${INSPECTION_WORKSPACE:-}"; do
  [[ -n "$root" ]] || continue
  if [[ -f "$root/src/inspection_mission/inspection_mission/full_inspection_runtime.py" ]]; then
    SOURCE_ROOT="$(cd -- "$root" && pwd -P)"
    break
  fi
done
if [[ -z "$SOURCE_ROOT" ]]; then
  printf '%s\n' 'FULL_INSPECTION error=uploaded_source_missing expected=src/inspection_mission/inspection_mission/full_inspection_runtime.py' >&2
  exit 2
fi
diagnostic_event heartbeat source_located
if [[ -f "$SOURCE_ROOT/install/setup.bash" ]]; then
  set +u
  source "$SOURCE_ROOT/install/setup.bash"
  set -u
fi
export PYTHONPATH="$SOURCE_ROOT/src/inspection_mission${PYTHONPATH:+:$PYTHONPATH}"
cd -- "$SOURCE_ROOT"
# Full pipeline: three current visual ground points, actual return, both windows,
# both aerial light inspections, actual return and landing. No simulator/referee startup.
python3 -u -m inspection_mission.full_inspection_runtime --ground-duration 1800 --flight-duration 900
