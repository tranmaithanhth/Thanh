#!/usr/bin/env bash
# ROS2 Humble + PyQt5 GUI. Run in a graphical Ubuntu desktop, not SSH without X11.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ACKERMANN_WS:-$HOME/ackermann_humble_ws}"
for m in PyQt5 matplotlib numpy; do
  if ! /usr/bin/python3 -c "import $m" 2>/dev/null; then
    echo "[HMI] Missing Ubuntu Python module: $m" >&2
    echo "Install: sudo apt install -y python3-pyqt5 python3-matplotlib python3-numpy" >&2
    exit 1
  fi
done
[[ -f /opt/ros/humble/setup.bash ]] || { echo '[HMI] ROS Humble not found' >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo '[HMI] Workspace not built' >&2; exit 1; }
exec bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$1/install/setup.bash"
exec /usr/bin/python3 "$2/tools/hmi_v104.py"
' bash "$WS" "$ROOT"
