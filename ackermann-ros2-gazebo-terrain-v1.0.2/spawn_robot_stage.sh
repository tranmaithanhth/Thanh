#!/usr/bin/env bash
# Gazebo Classic 11 + ROS 2 Humble Ackermann progressive debugging.
# Keeps world running; only spawns one URDF test variant per run.
# Usage: bash spawn_robot_stage.sh A|B|C|D|E|F
#       ACKERMANN_SPAWN_Z=0.12 bash spawn_robot_stage.sh B
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
STAGE="${1:-}"
SPAWN_Z="${ACKERMANN_SPAWN_Z:-0.10}"
if [[ ! "$STAGE" =~ ^[ABCDEF]$ ]]; then
  echo 'Usage: bash spawn_robot_stage.sh A|B|C|D|E|F' >&2
  exit 2
fi
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Missing $WS/install/setup.bash (build workspace first)" >&2
  exit 1
fi
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH="/usr/local/bin:/usr/bin:/bin" LANG="${LANG:-C.UTF-8}" \
 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
 ACKERMANN_WS="$WS" DIAG_ROOT="$ROOT" DIAG_STAGE="$STAGE" \
 ACKERMANN_SPAWN_Z="$SPAWN_Z" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
command -v xacro >/dev/null || { echo "Missing xacro" >&2; exit 1; }
ros2 pkg prefix gazebo_ros >/dev/null || { echo "Missing gazebo_ros" >&2;exit 1; }
if ! timeout 8s ros2 service list | grep -qx /spawn_entity; then
  echo "Missing /spawn_entity. Start GUI world using run_world_clean.sh (without --server)." >&2
  exit 1
fi
# A stale model from a prior test would invalidate the comparisons.
if timeout 4s gz model -l 2>/dev/null | grep -q "terrain_stage_"; then
  echo "Another terrain_stage model is present; restart world before each trial." >&2
  exit 1
fi
pkg="$(ros2 pkg prefix ackermann_vehicle)" || { echo "Missing ackermann_vehicle in current workspace" >&2;exit 1; }
xacro_file="$pkg/share/ackermann_vehicle/urdf/ackermann_vehicle.urdf.xacro"
[[ -s "$xacro_file" ]] || { echo "Missing XACRO: $xacro_file" >&2;exit 1; }
tmpdir="$(mktemp -d)"
trap '\''rm -rf "$tmpdir"'\'' EXIT
xacro "$xacro_file" -o "$tmpdir/full.urdf"
python3 "$DIAG_ROOT/tools/generate_robot_stage.py" "$DIAG_STAGE" "$tmpdir/full.urdf" "$tmpdir/stage.urdf"
echo "[TEST] stage=$DIAG_STAGE, entity=terrain_stage_$DIAG_STAGE, spawn z=$ACKERMANN_SPAWN_Z"
echo "[NOTE] Gazebo must be in Play mode; let the model settle 10-15 seconds."
timeout 50s ros2 run gazebo_ros spawn_entity.py -entity "terrain_stage_$DIAG_STAGE" \
  -file "$tmpdir/stage.urdf" -x 0.0 -y -2.0 -z "$ACKERMANN_SPAWN_Z" -Y 0.0
printf "\n[TEST] Monitor: gz stats\n"
printf "[TEST] Do not drive before testing stage C.\n"
'