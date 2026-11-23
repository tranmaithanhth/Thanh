#!/usr/bin/env bash
# Spawn the EXISTING ackermann_vehicle URDF into a running terrain Gazebo.
# Does not launch a second Gazebo instance.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
# Spawn root base_footprint only slightly above terrain at (0,-2).
# Override for a different vehicle geometry: ACKERMANN_SPAWN_Z=0.15 bash spawn_robot_on_terrain.sh
SPAWN_Z="${ACKERMANN_SPAWN_Z:-0.10}"
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "ERROR: ROS 2 workspace not built: $WS/install/setup.bash" >&2
  echo "First build ackermann_vehicle in $WS with colcon, then retry." >&2
  exit 1
fi
exec env -i \
 HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH="/usr/local/bin:/usr/bin:/bin" LANG="${LANG:-C.UTF-8}" \
 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
 ACKERMANN_WS="$WS" ACKERMANN_SPAWN_Z="$SPAWN_Z" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
echo "ROS_DISTRO=$ROS_DISTRO"
PKG_PREFIX="$(ros2 pkg prefix ackermann_vehicle)" || {
  echo "ackermann_vehicle was not found in $ACKERMANN_WS/install" >&2; exit 1;
}
XACRO="$PKG_PREFIX/share/ackermann_vehicle/urdf/ackermann_vehicle.urdf.xacro"
[[ -f "$XACRO" ]] || { echo "Missing installed XACRO: $XACRO" >&2;exit 1; }
command -v xacro >/dev/null || { echo "Missing xacro: sudo apt install ros-humble-xacro" >&2;exit 1; }
# A robot entity can be spawned only when gazebo_ros_factory has created /spawn_entity.
if ! ros2 service list | grep -qx "/spawn_entity";then
  echo "ERROR: /spawn_entity service absent. Start Gazebo with bash run_world_clean.sh (GUI)," >&2
  echo "not the standalone --server mode, and wait for Gazebo to load." >&2
  exit 1
fi
if gz model -l 2>/dev/null | grep -q "terrain_ackermann";then
 echo "Robot terrain_ackermann appears already spawned; not spawning twice."
 exit 0
fi
URDF="$(mktemp --suffix=.urdf)"
trap '\''rm -f "$URDF"'\'' EXIT
xacro "$XACRO" > "$URDF"
# Start in an unobstructed lane at y=-2 m so the robot can drive through
# both ramps without meeting the cylinders around y=0..4.
ros2 run gazebo_ros spawn_entity.py -entity terrain_ackermann -file "$URDF" \
  -x 0.0 -y -2.0 -z "$ACKERMANN_SPAWN_Z" -Y 0.0
printf "\nRobot spawned at z=%s m. IMPORTANT: press ▶ Play in Gazebo if paused.\n" "$ACKERMANN_SPAWN_Z"
printf "Gazebo paused = zero gravity/collision stepping; the robot will appear suspended.\n"
printf "Once wheels settle on terrain, use this separate terminal to drive:\n"
printf "  bash drive_robot_on_terrain.sh\n"
'