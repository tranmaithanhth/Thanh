#!/usr/bin/env bash
# Build ROS2 Humble Gazebo Classic force-control plugin without sourcing ROS1/Noetic.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
mkdir -p "$WS/src"
PACKAGE_PATH="$WS/src/ackermann_force_control"
if [[ -e "$PACKAGE_PATH" && ! -L "$PACKAGE_PATH" ]]; then
  echo "ERROR: '$PACKAGE_PATH' already exists and is not a symlink. Won't overwrite." >&2
  exit 1
fi
if [[ -L "$PACKAGE_PATH" && "$(readlink -f "$PACKAGE_PATH")" != "$ROOT/ackermann_force_control" ]]; then
  echo "ERROR: existing '$PACKAGE_PATH' points to something else. Resolve manually." >&2
  exit 1
fi
if [[ ! -e "$PACKAGE_PATH" ]]; then ln -s "$ROOT/ackermann_force_control" "$PACKAGE_PATH"; fi
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
  PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" ACKERMANN_WS="$WS" \
  bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
cd "$ACKERMANN_WS"
colcon build --symlink-install --packages-select ackermann_force_control --event-handlers console_direct+
source "$ACKERMANN_WS/install/setup.bash"
test -f "$ACKERMANN_WS/install/ackermann_force_control/lib/libackermann_force_drive.so" || {
 echo "ERROR: plugin .so was not installed" >&2; exit 1;
}
echo "[OK] Force plugin available: $ACKERMANN_WS/install/ackermann_force_control/lib/libackermann_force_drive.so"
'