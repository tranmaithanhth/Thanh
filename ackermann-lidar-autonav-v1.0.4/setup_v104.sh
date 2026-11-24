#!/usr/bin/env bash
# v1.0.4 reuses the VERIFIED v1.0.3 C++ AWD force plugin.
# Do not change workspace symlinks, ROS distro, or rebuild Gazebo unnecessarily.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
FORCE_LIB="$WS/install/ackermann_force_control/lib/libackermann_force_drive.so"
RAY_LIB=/opt/ros/humble/lib/libgazebo_ros_ray_sensor.so
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
 echo "ERROR: ROS 2 Humble not found at /opt/ros/humble" >&2; exit 1
fi
if [[ ! -f "$RAY_LIB" ]]; then
 echo "[SETUP] Missing Gazebo Classic ROS ray plugin."
 echo "Install: sudo apt update && sudo apt install -y ros-humble-gazebo-plugins"
 exit 1
fi
if [[ ! -f "$FORCE_LIB" ]]; then
 echo "[SETUP] Force plugin not yet installed. Building the unchanged v1.0.3 dependency..."
 bash "$ROOT/../ackermann-force-trajectory-v1.0.3/setup_v103.sh"
fi
if [[ ! -f "$FORCE_LIB" ]]; then
 echo "ERROR: C++ force plugin still missing: $FORCE_LIB" >&2; exit 1
fi
echo "[OK] ROS 2 Humble and Gazebo Classic LiDAR plugin found."
echo "[OK] Existing AWD force plugin: $FORCE_LIB"
echo "[OK] v1.0.4 code uses the v1.0.3 force controller unchanged."
python3 -m unittest discover -s "$ROOT/tests" -v
