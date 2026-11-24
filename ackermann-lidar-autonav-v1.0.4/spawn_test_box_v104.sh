#!/usr/bin/env bash
# Insert a NEW solid box into a running Gazebo world to verify LiDAR detects it.
# Use after the robot begins driving. Do not place directly on top of the robot.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
X="${BOX_X:-8.2}"; Y="${BOX_Y:--2.2}"; NAME="${BOX_NAME:-lidar_test_box_v104}"
Z="$(python3 - "$ROOT" "$X" "$Y" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);sys.path.insert(0,str(root/'tools'))
from planner_core import terrain
cfg=json.loads((root/'mission.json').read_text())
x,y=map(float,sys.argv[2:4])
if not(-18<x<18 and -17<y<17):raise SystemExit('BOX_X/Y out of test-area bounds')
print(f"{terrain(x,y)[0]+float(cfg['height_shift_gazebo_m'])+0.40:.6f}")
PY
)"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT
cat >"$TMP" <<'SDF'
<sdf version="1.6">
  <model name="lidar_test_box">
    <static>true</static>
    <link name="body">
      <collision name="solid"><geometry><box><size>0.7 0.7 0.8</size></box></geometry></collision>
      <visual name="visible">
        <geometry><box><size>0.7 0.7 0.8</size></box></geometry>
        <material><ambient>0.95 0.52 0.08 1</ambient><diffuse>0.95 0.52 0.08 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
SDF
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" \
 PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" \
 BOX_X="$X" BOX_Y="$Y" BOX_Z="$Z" BOX_NAME="$NAME" BOX_FILE="$TMP" \
 bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity "$BOX_NAME" -file "$BOX_FILE" \
 -x "$BOX_X" -y "$BOX_Y" -z "$BOX_Z"
'
