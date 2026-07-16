#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"

ros2 launch central_bringup bringup_mock.launch.py &
launch_pid=$!
trap 'kill "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true' EXIT
sleep 3
ros2 run mission_manager send_demo_mission
