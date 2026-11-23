#!/usr/bin/env bash
# Drive the already spawned terrain Ackermann robot using ROS2 Twist messages.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
[[ -f "$WS/install/setup.bash" ]] || { echo "ERROR: missing $WS/install/setup.bash" >&2;exit 1; }
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH="/usr/local/bin:/usr/bin:/bin" LANG="${LANG:-C.UTF-8}" \
 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ACKERMANN_WS="$WS" DRIVER_ROOT="$ROOT" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
exec python3 "$DRIVER_ROOT/tools/drive_slope_demo.py"
'