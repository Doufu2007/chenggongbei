#!/usr/bin/env bash
set -Eeuo pipefail
export PYTHONUNBUFFERED=1

printf '%s\n' 'BOUNDED_CRUISE_HEIGHT launch_start read_only=false official_scoring=unverified revision=v37_cruise'
diagnostic_event() {
  [[ -n "${INSPECTION_DIAGNOSTIC_URL:-}" ]] || return 0
  "${INSPECTION_DIAGNOSTIC_PYTHON:-python3}" - "$1" "${2:-}" <<'PY' 2>/dev/null || true
import datetime, json, os, sys, urllib.parse, urllib.request
url = os.environ.get("INSPECTION_DIAGNOSTIC_URL", "")
p = urllib.parse.urlsplit(url)
if not (p.scheme == "https" or (p.scheme == "http" and p.hostname in ("localhost", "127.0.0.1", "::1"))) or p.username or p.password:
    raise SystemExit(0)
data = {"read_only": False, "mode": "bounded_cruise_height", "revision": "v37_cruise", "official_scoring": "unverified"}
if sys.argv[1] == "launcher_exit":
    data["exit_code"] = int(sys.argv[2])
payload = json.dumps({"event": sys.argv[1], "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(), "data": data}).encode()
request = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
try:
    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=3):
        pass
except Exception:
    pass
PY
}
on_exit() {
  local status=$?
  trap - EXIT
  diagnostic_event launcher_exit "$status"
  printf 'BOUNDED_CRUISE_HEIGHT launch_exit exit_code=%s\n' "$status" >&2
  exit "$status"
}
trap on_exit EXIT
diagnostic_event launcher_started

ROS_SETUP="${INSPECTION_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'BOUNDED_CRUISE_HEIGHT error=ros_setup_missing path=%s\n' "$ROS_SETUP" >&2
  exit 2
fi
set +u
source "$ROS_SETUP"
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SCRIPT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
SOURCE_ROOT=""
for root in "$SCRIPT_DIR" "$SCRIPT_ROOT" "$PWD" "${INSPECTION_WORKSPACE:-}"; do
  [[ -n "$root" ]] || continue
  if [[ -f "$root/src/inspection_mission/inspection_mission/bounded_cruise_height.py" ]]; then
    SOURCE_ROOT="$(cd -- "$root" && pwd -P)"
    break
  fi
done
if [[ -z "$SOURCE_ROOT" ]]; then
  printf '%s\n' 'BOUNDED_CRUISE_HEIGHT error=uploaded_source_missing expected=src/inspection_mission/inspection_mission/bounded_cruise_height.py' >&2
  exit 2
fi
if [[ -f "$SOURCE_ROOT/install/setup.bash" ]]; then
  set +u
  source "$SOURCE_ROOT/install/setup.bash"
  set -u
fi
export PYTHONPATH="$SOURCE_ROOT/src/inspection_mission${PYTHONPATH:+:$PYTHONPATH}"
cd -- "$SOURCE_ROOT"
# Independent .85m absolute cruise experiment in a declared fixed static column.
# No simulation/referee startup and no competition completion declaration.
python3 -u -m inspection_mission.bounded_cruise_height
