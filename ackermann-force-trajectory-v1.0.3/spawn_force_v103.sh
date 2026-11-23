#!/usr/bin/env bash
# Spawn ONE stage3 robot, no LiDAR/camera, physical parameter mapping from MATLAB.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE="$(cd "$ROOT/../ackermann-ros2-gazebo-terrain-v1.0.2" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
SPAWN_X="${ACKERMANN_SPAWN_X:-0.0}"
SPAWN_Y="${ACKERMANN_SPAWN_Y:--2.0}"
SPAWN_Z="${ACKERMANN_SPAWN_Z:-0.055}"
[[ -f "$WS/install/ackermann_force_control/lib/libackermann_force_drive.so" ]] || { echo 'Run bash setup_v103.sh first.' >&2; exit 1; }
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
 V103_ROOT="$ROOT" BASE="$BASE" ACKERMANN_WS="$WS" SPAWN_X="$SPAWN_X" SPAWN_Y="$SPAWN_Y" SPAWN_Z="$SPAWN_Z" \
 bash --noprofile --norc -c '
set -eo pipefail
# ROS 2 environment scripts may inspect unset variables (e.g. AMENT_TRACE_SETUP_FILES).
# Enable nounset only after sourcing all ROS setups.
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
set -u
if ! ros2 service list | grep -qx /spawn_entity; then
 echo "ERROR: Gazebo ROS spawn service missing; start bash run_world_v103.sh" >&2; exit 1;
fi
if ros2 topic list | grep -q '^/ackermann_vehicle/odom$'; then
 echo "ERROR: existing Ackermann robot odom topic detected; restart Gazebo (avoid duplicate robot)" >&2; exit 1;
fi
p="$(ros2 pkg prefix ackermann_vehicle)" || { echo "No ackermann_vehicle installed"; exit 1; }
xacro_file="$p/share/ackermann_vehicle/urdf/ackermann_vehicle.urdf.xacro"
TMP="$(mktemp -d)"
trap '\''rm -rf "$TMP"'\'' EXIT
xacro "$xacro_file" > "$TMP/full.urdf"
python3 "$BASE/tools/build_robot_stage.py" --source "$TMP/full.urdf" --output "$TMP/stage3.urdf" --stage 3
python3 "$V103_ROOT/tools/patch_robot_v103.py" --input "$TMP/stage3.urdf" --output "$TMP/force.urdf" --params "$V103_ROOT/matlab_vehicle_params.json"
ros2 run gazebo_ros spawn_entity.py -entity ackermann_force_v103 -file "$TMP/force.urdf" -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z" -Y 0.0
printf "[OK] Spawned v1.0.3 force robot at X=%s Y=%s Z=%s (ground contact).\n" "$SPAWN_X" "$SPAWN_Y" "$SPAWN_Z"
'