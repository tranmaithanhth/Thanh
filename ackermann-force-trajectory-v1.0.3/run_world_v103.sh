#!/usr/bin/env bash
# Start the existing 3D terrain WITH new force-driven Gazebo model plugin path.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE="$(cd "$ROOT/../ackermann-ros2-gazebo-terrain-v1.0.2" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
LIB="$WS/install/ackermann_force_control/lib"
[[ -f "$LIB/libackermann_force_drive.so" ]] || {
 echo 'ERROR: force plugin not built: run bash setup_v103.sh first' >&2; exit 1;
}
[[ -s "$BASE/models/ackermann_matlab_terrain_v102/terrain_height.png" ]] || { echo 'terrain v1.0.2 missing' >&2; exit 1; }
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" \
 DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
 XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" TERRAIN_ROOT="$BASE" FORCE_LIB="$LIB" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH="$TERRAIN_ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$FORCE_LIB${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
echo "[WORLD] 3D terrain v1.0.2 (unchanged) | [PLUGIN] $FORCE_LIB"
exec ros2 launch gazebo_ros gazebo.launch.py \
 world:="$TERRAIN_ROOT/worlds/ackermann_matlab_terrain.world" verbose:=false
'