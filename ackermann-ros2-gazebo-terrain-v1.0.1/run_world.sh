#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The script expects a ROS 2 Humble terminal, not a mixed ROS Noetic shell.
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH="$ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
ros2 launch gazebo_ros gazebo.launch.py world:="$ROOT/worlds/ackermann_matlab_terrain.world" verbose:=true
