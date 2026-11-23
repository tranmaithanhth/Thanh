#!/usr/bin/env bash
# Controlled stage-3 reverse/forward diagnostic (no extra Gazebo plugins).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
[[ -f "$WS/install/setup.bash" ]] || { echo "ROS 2 workspace not found: $WS" >&2; exit 1; }
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
  PATH="/usr/local/bin:/usr/bin:/bin" LANG="${LANG:-C.UTF-8}" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ACKERMANN_WS="$WS" ROOT="$ROOT" \
  bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$ACKERMANN_WS/install/setup.bash"
exec python3 "$ROOT/tools/test_stage3_reverse.py" "$@"
' bash "$@"