#!/usr/bin/env bash
# Static/environment diagnosis without starting or killing Gazebo.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="$ROOT/worlds/ackermann_matlab_terrain.world"
MODEL="$ROOT/models/ackermann_matlab_terrain_v102/model.sdf"
PNG="$ROOT/models/ackermann_matlab_terrain_v102/terrain_height.png"
echo "===== ACKERMANN GAZEBO DIAGNOSTICS ====="
echo "Project: $ROOT"
echo "ROS_DISTRO in current terminal: ${ROS_DISTRO:-<unset>}"
echo "DISPLAY: ${DISPLAY:-<unset>}"
echo "XDG_RUNTIME_DIR: ${XDG_RUNTIME_DIR:-<unset>}"
for c in python3 ros2 gzserver gzclient gazebo; do
  if command -v "$c" >/dev/null;then echo "OK executable: $c -> $(command -v "$c")";
  else echo "MISSING executable: $c";fi
done
for f in "$WORLD" "$MODEL" "$PNG" /opt/ros/humble/setup.bash; do
  if [[ -s "$f" ]];then echo "OK file: $f";else echo "MISSING file: $f";fi
done
python3 - "$WORLD" "$MODEL" "$PNG" <<'PY'
import os,sys,struct,xml.etree.ElementTree as ET
world,sdf,png=sys.argv[1:]
for path in [world,sdf]:
 try:
  root=ET.parse(path).getroot()
  assert root.tag=="sdf", f"Unexpected root tag {root.tag}"
  print("OK XML:",path)
 except Exception as e:print("INVALID XML:",path,repr(e));sys.exit(1)
with open(png,"rb") as f: header=f.read(24)
assert header[:8]==b"\x89PNG\r\n\x1a\n","Not a valid PNG header"
width,height=struct.unpack(">II",header[16:24])
assert (width,height)==(257,257),f"Unexpected heightmap shape {width}x{height}"
print("OK heightmap PNG:",width,"x",height)
model_name=ET.parse(sdf).getroot().find("model").attrib["name"]
world_text=ET.parse(world).getroot()
includes=[m.findtext("uri") for m in world_text.findall(".//include")]
assert "model://"+model_name in includes, "Model name mismatch in world include"
print("OK model reference:",model_name)
PY
if command -v ros2 >/dev/null;then
  ros2 pkg prefix gazebo_ros 2>&1 || echo "gazebo_ros missing or ROS environment not sourced"
else
  echo "INFO: run_world_clean.sh sources /opt/ros/humble/setup.bash before launch"
fi
echo "===== EXISTING GAZEBO PROCESSES ====="
pgrep -a -f '(^|/)(gzserver|gzclient)( |$)' || true
echo "===== NEXT ====="
echo "1) bash run_world_clean.sh --server   # Test terrain server"
echo "2) bash run_world_clean.sh            # Test ROS2 + GUI"
echo "3) bash run_world_clean.sh --software # Test Mesa software rendering"
echo "Copy the first ERROR or FATAL line if launch fails."
