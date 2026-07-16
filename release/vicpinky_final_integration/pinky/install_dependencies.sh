#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$ROOT/src" --ignore-src -r -y \
  --skip-keys "ament_python pyseiral"

# The provided Pinky manifest spells this rosdep key as "pyseiral". Keep the
# original package untouched and install its intended runtime dependency here.
sudo apt-get install -y python3-serial
