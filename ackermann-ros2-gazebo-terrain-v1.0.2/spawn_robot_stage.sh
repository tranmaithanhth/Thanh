#!/usr/bin/env bash
# Launch a partial Ackermann model in the already running Gazebo terrain.
# Usage: bash spawn_robot_stage.sh 0..5
# Optional: ACKERMANN_SAFE_FRICTION=1 bash spawn_robot_stage.sh 2
# Optional spawn pose: ACKERMANN_SPAWN_X=-4 ACKERMANN_SPAWN_Y=-2 bash spawn_robot_stage.sh 3
# Optional PID test: ACKERMANN_DRIVE_PROFILE=no_motors|gentle|moderate bash spawn_robot_stage.sh 3
set -eo pipefail
STAGE="${1:-}"
if [[ ! "$STAGE" =~ ^[0-5]$ ]]; then
  echo 'Usage: bash spawn_robot_stage.sh <0|1|2|3|4|5>' >&2
  exit 2
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
SPAWN_X="${ACKERMANN_SPAWN_X:-0.0}"
SPAWN_Y="${ACKERMANN_SPAWN_Y:--2.0}"
SPAWN_Z="${ACKERMANN_SPAWN_Z:-0.10}"
SAFE="${ACKERMANN_SAFE_FRICTION:-0}"
DRIVE_PROFILE="${ACKERMANN_DRIVE_PROFILE:-original}"
if [[ ! "$DRIVE_PROFILE" =~ ^(original|no_motors|gentle|moderate)$ ]]; then
  echo "Invalid ACKERMANN_DRIVE_PROFILE=$DRIVE_PROFILE (use original/no_motors/gentle/moderate)" >&2
  exit 2
fi
[[ -f "$WS/install/setup.bash" ]] || { echo "Missing ROS workspace: $WS/install/setup.bash" >&2; exit 1; }
exec env -i \
 HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH="/usr/local/bin:/usr/bin:/bin" LANG="${LANG:-C.UTF-8}" \
 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ACKERMANN_WS="$WS" \
 STAGE="$STAGE" ROOT="$ROOT" SPAWN_X="$SPAWN_X" SPAWN_Y="$SPAWN_Y" SPAWN_Z="$SPAWN_Z" SAFE="$SAFE" DRIVE_PROFILE="$DRIVE_PROFILE" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
echo "[ROS] ROS_DISTRO=${ROS_DISTRO:-?}"
ros2 service list | grep -qx /spawn_entity || {
  echo "ERROR: ROS spawn service is absent. Launch the terrain using bash run_world_clean.sh (GUI, not --server)." >&2
  exit 1
}
if [[ "$STAGE" == 3 ]]; then
  # A stale controller can publish positive speed as soon as a new robot appears.
  # No controller should publish until AFTER this robot has been spawned.
  PUB_COUNT="$(ros2 topic info /ackermann_vehicle/cmd_vel 2>/dev/null | sed -n "s/^Publisher count: //p" || true)"
  if [[ "$PUB_COUNT" =~ ^[0-9]+$ ]] && (( PUB_COUNT > 0 )); then
    echo "[ERROR] Existing cmd_vel publisher(s): $PUB_COUNT. Stop old controllers before spawning stage 3." >&2
    echo "Inspect: ros2 topic info /ackermann_vehicle/cmd_vel -v" >&2
    exit 1
  fi
fi
PKG_PREFIX="$(ros2 pkg prefix ackermann_vehicle)" || {
 echo "ERROR: ackermann_vehicle not found in workspace" >&2; exit 1;
}
XACRO="$PKG_PREFIX/share/ackermann_vehicle/urdf/ackermann_vehicle.urdf.xacro"
[[ -f "$XACRO" ]] || { echo "Missing XACRO: $XACRO" >&2; exit 1; }
TMP="$(mktemp -d)"
trap '\''rm -rf "$TMP"'\'' EXIT
xacro "$XACRO" > "$TMP/robot_full.urdf"
OPTS=()
if [[ "$SAFE" == 1 ]]; then OPTS+=(--safe-friction); fi
python3 "$ROOT/tools/build_robot_stage.py" \
 --source "$TMP/robot_full.urdf" --output "$TMP/robot_stage.urdf" \
 --stage "$STAGE" --drive-profile "$DRIVE_PROFILE" "${OPTS[@]}"
ENTITY="terrain_probe_s$STAGE"
echo "[SPAWN] $ENTITY at x=$SPAWN_X y=$SPAWN_Y z=$SPAWN_Z | Ackermann drive profile=$DRIVE_PROFILE"
echo "[NOTE] Restart Gazebo between stages, otherwise prior robot(s) remain in world."
ros2 run gazebo_ros spawn_entity.py \
 -entity "$ENTITY" -file "$TMP/robot_stage.urdf" \
 -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z" -Y 0.0
printf "\nStage %s spawned. Gazebo must be in PLAY state. Check: timeout 10s gz stats\n" "$STAGE"
'