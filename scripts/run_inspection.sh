#!/usr/bin/env bash
set -Eeuo pipefail

printf '%s\n' 'INSPECTION_MISSION launch_start'
PIDS=()
LAUNCH_PREFIX=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    if [[ ${#LAUNCH_PREFIX[@]} -gt 0 ]]; then
      kill -- "-${pid}" 2>/dev/null || true
    fi
    kill "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
on_exit() {
  local status=$?
  trap - EXIT
  cleanup
  printf 'INSPECTION_MISSION launch_exit exit_code=%s\n' "$status" >&2
  exit "$status"
}
trap on_exit EXIT

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${INSPECTION_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 2
fi
set +u
source "${ROS_SETUP}"
set -u
# Prefer the requested or launcher workspace; the caller's cwd is a fallback.
for candidate in "${INSPECTION_WORKSPACE:-}" "${SCRIPT_DIR}/.." "${SCRIPT_DIR}" "${PWD}"; do
  if [[ -n "${candidate}" && -f "${candidate}/install/setup.bash" ]]; then
    set +u
    source "${candidate}/install/setup.bash"
    set -u
    if ros2 pkg prefix --share inspection_mission >/dev/null; then
      break
    fi
  fi
done
if PACKAGE_SHARE="$(ros2 pkg prefix --share inspection_mission)"; then
  :
else
  status=$?
  printf 'INSPECTION_MISSION inspection_mission package lookup failed exit_code=%s\n' "$status" >&2
  exit "$status"
fi
CONFIG_PATH="${INSPECTION_CONFIG:-${PACKAGE_SHARE}/config/inspection.yaml}"
if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "Inspection configuration not found: ${CONFIG_PATH}" >&2
  exit 2
fi
ros2 run inspection_mission validate_config --config "${CONFIG_PATH}" --default-config-dir "${PACKAGE_SHARE}/config"

if command -v setsid >/dev/null 2>&1; then
  LAUNCH_PREFIX=(setsid)
fi
trap 'exit 130' INT
trap 'exit 143' TERM

"${LAUNCH_PREFIX[@]}" ros2 run inspection_mission vision_node --ros-args --params-file "${CONFIG_PATH}" &
VISION_PID="$!"
PIDS+=("${VISION_PID}")
"${LAUNCH_PREFIX[@]}" ros2 run inspection_mission mission_node --ros-args --params-file "${CONFIG_PATH}" &
MISSION_PID="$!"
PIDS+=("${MISSION_PID}")

if wait -n -p EXITED_PID "${PIDS[@]}"; then
  NODE_STATUS=0
else
  NODE_STATUS=$?
fi
if [[ "${EXITED_PID:-}" == "${VISION_PID}" ]]; then
  printf 'INSPECTION_MISSION vision_node exited before mission_node exit_code=%s\n' "${NODE_STATUS}" >&2
  if [[ "${NODE_STATUS}" -eq 0 ]]; then
    exit 1
  fi
  exit "${NODE_STATUS}"
fi
if [[ "${EXITED_PID:-}" != "${MISSION_PID}" ]]; then
  echo 'INSPECTION_MISSION unknown child exited before mission_node' >&2
  exit 1
fi
exit "${NODE_STATUS}"

