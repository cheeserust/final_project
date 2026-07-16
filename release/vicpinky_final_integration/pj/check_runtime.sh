#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"

required_actions=(
  /mission/execute /nav/go_to /dock/align /elevator/wait_door_open /elevator/board
  /elevator/exit /floor/check /map/switch /base/drive_straight
  /base/rotate /arm/homing /arm/execute /arm/press_button
  /mission/ready_and_approach /move_action
  /arm_controller/follow_joint_trajectory
  /gripper_controller/follow_joint_trajectory
)
required_services=(/arm_board/status /arm_board/enable /arm_board/home_all /arm_board/estop)

actions="$(ros2 action list)"
services="$(ros2 service list)"
failed=0

for name in "${required_actions[@]}"; do
  if ! grep -Fxq "$name" <<<"$actions"; then
    echo "MISSING action: $name"
    failed=1
  fi
done
for name in "${required_services[@]}"; do
  if ! grep -Fxq "$name" <<<"$services"; then
    echo "MISSING service: $name"
    failed=1
  fi
done

if (( failed )); then
  echo "Preflight failed. Mission Execute를 누르지 마세요."
  exit 1
fi
echo "Runtime action/service preflight: OK"
