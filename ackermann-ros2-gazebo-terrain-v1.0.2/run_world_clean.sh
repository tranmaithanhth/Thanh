#!/usr/bin/env bash
# Start ROS 2 Humble / Gazebo Classic world in an isolated environment.
# Use: bash run_world_clean.sh [--software] [--server]
# This avoids contamination by previously sourced ROS Noetic workspaces.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE=gui
SOFTWARE=0
for arg in "$@"; do
  case "$arg" in
    --software) SOFTWARE=1 ;;
    --server) MODE=server ;;
    *) echo "Unknown argument: $arg (choose --software and/or --server)" >&2; exit 2 ;;
  esac
done
[[ -f /opt/ros/humble/setup.bash ]] || { echo "Missing /opt/ros/humble/setup.bash" >&2; exit 1; }
[[ -s "$ROOT/models/ackermann_matlab_terrain_v102/terrain_height.png" ]] || {
  echo "Missing terrain_height.png. Run git pull first." >&2; exit 1;
}
[[ -f "$ROOT/worlds/ackermann_matlab_terrain.world" ]] || {
  echo "Missing world file" >&2; exit 1;
}
# Pass only desktop/session variables. Never reuse ROS/Noetic, Python, Qt
# or Gazebo plugin search paths inherited from an interactive shell.
cmd=(env -i
 "HOME=$HOME"
 "USER=${USER:-$(id -un)}"
 "LOGNAME=${LOGNAME:-$(id -un)}"
 "PATH=/usr/local/bin:/usr/bin:/bin"
 "DISPLAY=${DISPLAY:-}"
 "XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}"
 "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-}"
 "DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-}"
 "TERM=${TERM:-xterm}"
 "LANG=${LANG:-C.UTF-8}"
 "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
 "TERRAIN_ROOT=$ROOT"
 "TERRAIN_MODE=$MODE"
 "TERRAIN_SOFTWARE=$SOFTWARE"
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH="$TERRAIN_ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
if [ "$TERRAIN_SOFTWARE" = 1 ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_X11_NO_MITSHM=1
fi
echo "[ROS] ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "[WORLD] $TERRAIN_ROOT/worlds/ackermann_matlab_terrain.world"
command -v ros2 >/dev/null || { echo "ros2 executable missing" >&2; exit 1; }
ros2 pkg prefix gazebo_ros >/dev/null || {
  echo "gazebo_ros is missing. Consider: sudo apt install ros-humble-gazebo-ros-pkgs" >&2
  exit 1
}
if [ "$TERRAIN_MODE" = server ]; then
  command -v gzserver >/dev/null || { echo "gzserver missing" >&2; exit 1; }
  echo "[MODE] Standalone Gazebo server only (no GUI; no ROS plugins)."
  exec gzserver --verbose "$TERRAIN_ROOT/worlds/ackermann_matlab_terrain.world"
fi
echo "[MODE] gazebo_ros launch with GUI. If GUI crashes, try --software or --server."
exec ros2 launch gazebo_ros gazebo.launch.py \
  world:="$TERRAIN_ROOT/worlds/ackermann_matlab_terrain.world" verbose:=true
')
exec "${cmd[@]}"
