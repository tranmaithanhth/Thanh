#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble not found at /opt/ros/humble. Requires Ubuntu 22.04 + Humble.' >&2
  exit 1
fi
# Avoid "unbound variable" from ROS setup scripts under set -u.
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH="$ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
if [[ ! -s "$ROOT/models/ackermann_matlab_terrain_v102/terrain_height.png" ]]; then
  echo 'Missing heightmap PNG. Run python3 tools/generate_terrain.py first.' >&2
  exit 1
fi
ros2 launch gazebo_ros gazebo.launch.py \
  world:="$ROOT/worlds/ackermann_matlab_terrain.world" verbose:=true