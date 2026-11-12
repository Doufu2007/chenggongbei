#!/usr/bin/env bash
set -u
if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
printf '%s\n' '===PROBE START==='
printf '%s\n' '===TOPIC LIST==='
ros2 topic list 2>&1 | sort || true
for t in /referee/current_target /referee/ground_tasks /referee/recognition_status /referee/events_public /odom /uav/odom /tf /camera/image_raw /uav/camera/image_raw; do
  printf '\n===TYPE %s===\n' "$t"
  ros2 topic type "$t" 2>&1 || true
  printf '%s\n' "===INFO $t==="
  ros2 topic info "$t" -v 2>&1 || true
  printf '%s\n' "===ECHO $t==="
  timeout 3 ros2 topic echo --once "$t" 2>&1 || true
done
printf '%s\n' '===PROBE END==='
sleep 15
