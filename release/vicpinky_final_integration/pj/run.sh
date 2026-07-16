#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"

EXECUTION_MODE="${EXECUTION_MODE:-hardware}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"

exec ros2 launch central_bringup final_system.launch.py \
  execution_mode:="$EXECUTION_MODE" \
  can_interface:="$CAN_INTERFACE" "$@"
