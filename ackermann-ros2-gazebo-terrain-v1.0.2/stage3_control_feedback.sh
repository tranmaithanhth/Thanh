#!/usr/bin/env bash
# No camera, no lidar: only ROS2 odometry and cmd_vel from ackermann stage 3.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
[[ -f "$WS/install/setup.bash" ]] || { echo "Missing ROS2 workspace setup: $WS/install/setup.bash" >&2; exit 1; }
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH="/usr/local/bin:/usr/bin:/bin" LANG="${LANG:-C.UTF-8}" ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
 ACKERMANN_WS="$WS" STAGE3_ROOT="$ROOT" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
exec python3 "$STAGE3_ROOT/tools/stage3_console.py" "$@"
' bash "$@"