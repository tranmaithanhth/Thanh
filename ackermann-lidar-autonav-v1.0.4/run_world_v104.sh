#!/usr/bin/env bash
# Gazebo Classic 11 terrain with perimeter walls and v1.0.3 AWD force plugin.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE="$(cd "$ROOT/../ackermann-ros2-gazebo-terrain-v1.0.2" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
LIB="$WS/install/ackermann_force_control/lib"
# The default world adds four static collision walls at +/-19.5m.
# Set ACKERMANN_WALLS=0 for the original unmodified terrain world.
WALLS="${ACKERMANN_WALLS:-1}"
case "$WALLS" in
  1) WORLD_FILE="$ROOT/worlds/ackermann_matlab_terrain_walled.world" ;;
  0) WORLD_FILE="$BASE/worlds/ackermann_matlab_terrain.world" ;;
  *) echo 'ERROR: ACKERMANN_WALLS must be 1 or 0' >&2; exit 2 ;;
esac
[[ -s "$WORLD_FILE" ]] || { echo "ERROR: world not found: $WORLD_FILE" >&2; exit 1; }
[[ -f "$LIB/libackermann_force_drive.so" ]] || {
 echo 'ERROR: force plugin not built: run bash setup_v103.sh first' >&2; exit 1;
}
# Prevent accidentally testing the old plugin that skipped 90% of physics updates.
if [[ "$ROOT/../ackermann-force-trajectory-v1.0.3/ackermann_force_control/src/ackermann_force_drive.cpp" -nt "$LIB/libackermann_force_drive.so" ]]; then
 echo 'ERROR: Force plugin C++ source is newer than .so. Rebuild: bash setup_v103.sh' >&2
 exit 1
fi
[[ -s "$BASE/models/ackermann_matlab_terrain_v102/terrain_height.png" ]] || { echo 'terrain v1.0.2 missing' >&2; exit 1; }
[[ -f /opt/ros/humble/lib/libgazebo_ros_ray_sensor.so ]] || {
  echo 'ERROR: ray sensor plugin missing; install: sudo apt install ros-humble-gazebo-plugins' >&2
  exit 1
}
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" \
 DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
 XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" TERRAIN_ROOT="$BASE" FORCE_LIB="$LIB" WORLD_FILE="$WORLD_FILE" WALLS="$WALLS" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH="$TERRAIN_ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$FORCE_LIB${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
echo "[WORLD V1.0.4] 3D terrain 40x40m + physical walls=$WALLS | AWD force plugin=$FORCE_LIB | LiDAR 91x5Hz"
echo "[WORLD] file=$WORLD_FILE"
exec ros2 launch gazebo_ros gazebo.launch.py \
 world:="$WORLD_FILE" verbose:=false
'