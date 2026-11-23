# Spawn existing ROS 2 Humble Ackermann robot into running Gazebo terrain

The terrain world must already be running from `bash run_world_clean.sh` (normal GUI mode). Do not start a second Gazebo. These helpers reuse the existing, installed `ackermann_vehicle` XACRO and Gazebo Ackermann drive plugin.

From a SECOND terminal:

```bash
cd ~/ackermann_humble_ws/Thanh
# Pull once; if Git refuses due to modified local files, reconcile them first.
git pull --ff-only origin main
cd ackermann-ros2-gazebo-terrain-v1.0.2
bash spawn_robot_on_terrain.sh
```

Wait until the robot has settled on the terrain. Then in the same or another terminal:

```bash
bash drive_robot_on_terrain.sh
```

Spawn pose is `(x=0,y=-2,z=0.30,yaw=0)` to choose a straight +X corridor that avoids the four cylindrical obstacles. The auto-demo publishes `geometry_msgs/Twist` to `/ackermann_vehicle/cmd_vel` and uses `/ackermann_vehicle/odom` to stop after about 15 m. Its 0.45 m/s maximum speed is deliberately low. The world terrain ramps along X, so this test goes uphill and downhill. **It is not DWA/TEB** and does not try to reach the green goal marker `(16,0.6)`.

For manual testing (after the spawn):

```bash
source /opt/ros/humble/setup.bash
source ~/ackermann_humble_ws/install/setup.bash
ros2 topic pub /ackermann_vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.0}}' -r 10
# Ctrl+C to stop publishing, then issue zero speed:
ros2 topic pub --once /ackermann_vehicle/cmd_vel geometry_msgs/msg/Twist '{}'
```

If no `/spawn_entity` appears, make sure you launched the world with `run_world_clean.sh` without `--server`; the standalone `gzserver --server` diagnostic deliberately does not load the ROS spawn factory. If the robot appears but does not move, inspect:

```bash
ros2 topic list | grep ackermann
ros2 topic echo /ackermann_vehicle/odom --once
```

If a plugin is missing, capture the Gazebo terminal lines mentioning `libgazebo_ros_ackermann_drive.so`. The setup scripts use a clean ROS 2 Humble shell to avoid ROS Noetic contamination. This is a convenience demo that still requires testing on the user's actual Gazebo and robot package.