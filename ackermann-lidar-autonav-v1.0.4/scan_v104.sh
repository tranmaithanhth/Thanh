#!/usr/bin/env bash
# Backward-compatible monitor name: observe Gazebo nearby state, never LiDAR.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
exec python3 "$1/tools/nearby_monitor.py"
' bash "$ROOT"
