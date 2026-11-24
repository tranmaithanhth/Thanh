#!/usr/bin/env bash
# Observe real Gazebo LaserScan without publishing driving commands.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
exec python3 "$1/tools/lidar_monitor.py"
' bash "$ROOT"
