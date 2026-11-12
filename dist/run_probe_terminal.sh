#!/usr/bin/env bash
set -e
SETUP_CMD="source /opt/ros/humble/setup.bash"
PROBE_CMD='echo ===PROBE_START===; pwd; ros2 topic list -t; for t in /referee/current_target /referee/ground_tasks /referee/recognition_status /referee/events_public /odom /uav/odom /tf; do echo ===TOPIC:$t===; ros2 topic info -v "$t"; timeout 3 ros2 topic echo --once "$t"; done; echo ===PROBE_END===; sleep 30'
gnome-terminal \
  --window --title="competition" --command="bash -lc '$SETUP_CMD; $PROBE_CMD; exec bash'"