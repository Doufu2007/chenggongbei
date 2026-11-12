#!/usr/bin/env bash
set -Eeuo pipefail
export PYTHONUNBUFFERED=1
export INSPECTION_DIAGNOSTIC_URL="${INSPECTION_DIAGNOSTIC_URL:-}"

printf '%s\n' 'INTERFACE_PROBE launch_start read_only=true'
diagnostic_event() {
  [[ -n "${INSPECTION_DIAGNOSTIC_URL:-}" ]] || return 0
  "${INSPECTION_DIAGNOSTIC_PYTHON:-python3}" - "$1" "${2:-}" <<'PY' 2>/dev/null || true
import datetime, json, os, sys, urllib.parse, urllib.request
url = os.environ.get("INSPECTION_DIAGNOSTIC_URL", "")
parsed = urllib.parse.urlsplit(url)
loopback = parsed.scheme == "http" and parsed.hostname in ("127.0.0.1", "localhost", "::1")
if not (parsed.scheme == "https" or loopback) or parsed.username or parsed.password:
    raise SystemExit(0)
data = {"read_only": True}
if sys.argv[1] == "launcher_exit":
    data["exit_code"] = int(sys.argv[2])
payload = json.dumps({"event": sys.argv[1], "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                      "data": data}).encode()
request = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
try:
    with opener.open(request, timeout=1.5):
        pass
except Exception:
    pass
PY
}
on_exit() {
  local status=$?
  trap - EXIT
  diagnostic_event launcher_exit "$status"
  printf 'INTERFACE_PROBE launch_exit exit_code=%s\n' "$status" >&2
  exit "$status"
}
trap on_exit EXIT
diagnostic_event launcher_started

ROS_SETUP="${INSPECTION_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'INTERFACE_PROBE error=ros_setup_missing path=%s\n' "$ROS_SETUP" >&2
  exit 2
fi
set +u
source "$ROS_SETUP"
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SCRIPT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"

# The evaluator may invoke the uploaded script from an unrelated cwd. Resolve
# the exact uploaded source package before considering an installed overlay.
ROOT_CANDIDATES=(
  "$SCRIPT_DIR"
  "$SCRIPT_ROOT"
  "${PWD}"
  "${INSPECTION_WORKSPACE:-}"
)
SOURCE_ROOT=""
INSTALL_ROOT=""
for root in "${ROOT_CANDIDATES[@]}"; do
  [[ -n "$root" ]] || continue
  if [[ -f "$root/src/inspection_mission/inspection_mission/probe.py" ]]; then
    SOURCE_ROOT="$(cd -- "$root" && pwd -P)"
    break
  fi
  if [[ -z "$INSTALL_ROOT" && -f "$root/install/setup.bash" ]]; then
    INSTALL_ROOT="$root"
  fi
done

if [[ -n "$SOURCE_ROOT" ]]; then
  printf 'INTERFACE_PROBE source_root=%s\n' "$SOURCE_ROOT" >&2
  if [[ -f "$SOURCE_ROOT/install/setup.bash" ]]; then
    set +u
    source "$SOURCE_ROOT/install/setup.bash"
    set -u
  fi
  export PYTHONPATH="$SOURCE_ROOT/src/inspection_mission${PYTHONPATH:+:$PYTHONPATH}"
  cd -- "$SOURCE_ROOT"
  python3 -u -m inspection_mission.probe --duration 60 --compact-callback
  exit $?
fi

if [[ -n "$INSTALL_ROOT" ]]; then
  printf 'INTERFACE_PROBE install_root=%s\n' "$INSTALL_ROOT" >&2
  set +u
  source "$INSTALL_ROOT/install/setup.bash"
  set -u
  ros2 run inspection_mission interface_probe --duration 60 --compact-callback
  exit $?
fi

printf '%s\n' 'INTERFACE_PROBE error=uploaded_source_or_install_missing expected=src/inspection_mission/inspection_mission/probe.py_or_install/setup.bash' >&2
exit 2
