#!/usr/bin/env bash
# Start bounded Gazebo nearby perception + unknown-obstacle A* + force local rollout.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
 V104_ROOT="$ROOT" ACKERMANN_WS="$WS" \
 bash --noprofile --norc -c '
set -eo pipefail
# ROS 2 environment scripts may inspect unset variables (e.g. AMENT_TRACE_SETUP_FILES).
# Enable nounset only after sourcing all ROS setups.
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
set -u
exec python3 "$V104_ROOT/tools/autonomous_nearby_nav.py" --config "$V104_ROOT/mission.json" "$@"
' bash "$@"