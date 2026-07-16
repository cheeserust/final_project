#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"

if ! ros2 pkg prefix sllidar_ros2 >/dev/null 2>&1; then
  echo "기존 팀 sllidar_ros2가 현재 ROS 환경에서 보이지 않습니다." >&2
  echo "기존 Pinky workspace의 install/setup.bash를 먼저 source한 뒤 다시 실행하세요." >&2
  exit 2
fi

FRONT_VIDEO_DEVICE="${FRONT_VIDEO_DEVICE:-/dev/video0}"
REAR_VIDEO_DEVICE="${REAR_VIDEO_DEVICE:-/dev/video2}"

exec ros2 launch vicpinky_final_bringup final_robot.launch.py \
  front_video_device:="$FRONT_VIDEO_DEVICE" \
  rear_video_device:="$REAR_VIDEO_DEVICE" "$@"
