# Điều khiển Ackermann cấp 3 và phản hồi tư thế 3D (không LiDAR, không camera)

Đã thử nghiệm ở máy người dùng: cấp 0/1/2 và cấp 3 + tiến chậm bình thường; cấp 4 (LiDAR) gây nghẽn mô phỏng. **Giữ cố định cấp 3**. Không spawn cấp 4 hoặc 5, không chạy `gazebo.launch.py` của robot gốc vì nó mở thêm world. Các lệnh dưới đây chỉ dùng plugin Ackermann đang có.

## Chạy

**Terminal 1** – chạy địa hình, bắt đầu từ Gazebo sạch:

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-ros2-gazebo-terrain-v1.0.2
bash run_world_clean.sh
```

**Terminal 2** – spawn cấp 3 (đợi Gazebo nạp xong và nhấn Play):

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-ros2-gazebo-terrain-v1.0.2
bash spawn_robot_stage.sh 3
```

**Terminal 3** – mở console điều khiển và hiển thị phản hồi pose:

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-ros2-gazebo-terrain-v1.0.2
bash stage3_control_feedback.sh
```

Chương trình sẽ in mỗi 0,5 s: `XYZ` (m), `ROLL`, `PITCH`, `YAW` (độ), vận tốc tiến, tốc độ quay và lệnh gửi tới Ackermann Drive. Bấm phím trực tiếp trong Terminal console:

| Phím | Lệnh |
|---|---|
| `w` | Tiến 0,25 m/s |
| `s` | Lùi 0,10 m/s |
| `a` | Quay trái (yaw rate +0,25 rad/s) |
| `d` | Quay phải (yaw rate −0,25 rad/s) |
| `c` | Trả lệnh góc quay về 0 |
| `x` hoặc `Space` | Dừng |
| `q` | Dừng và thoát |

Giữ hoặc nhấn lại phím điều khiển để tiếp tục di chuyển: **sau 3 giây không có phím mới, chương trình tự gửi lệnh dừng** (deadman), vì robot không có LiDAR và chưa tự tránh vật cản. Nếu mất odometry quá 2 giây, console tự gửi lệnh dừng. Nếu Gazebo treo, lệnh dừng không thể được bảo đảm thực thi cho tới khi server hồi phục. Chỉ chạy ở môi trường mô phỏng.

## Phản hồi vị trí, tư thế và dữ liệu ROS 2

Các topic được sử dụng:

- Nhận: `/ackermann_vehicle/odom` (`nav_msgs/msg/Odometry`), chứa vị trí Gazebo world, quaternion và vận tốc do plugin Ackermann Drive đang có công bố.
- Gửi điều khiển: `/ackermann_vehicle/cmd_vel` (`geometry_msgs/msg/Twist`).
- Phát bổ sung: `/ackermann_vehicle/pose_feedback` (`geometry_msgs/msg/PoseStamped`), vị trí + quaternion với header giữ từ `odom`.
- Phát bổ sung: `/ackermann_vehicle/rpy_deg` (`geometry_msgs/msg/Vector3Stamped`), **x = roll, y = pitch, z = yaw**, đơn vị **độ**, cùng header.

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /ackermann_vehicle/rpy_deg
# hoặc xem chi tiết vị trí và quaternion:
ros2 topic echo /ackermann_vehicle/pose_feedback
```

ROS2 `gazebo_ros_ackermann_drive` tham chiếu `model_->WorldPose()` để lấy pose, rồi xuất lên `/ackermann_vehicle/odom`. Vì vậy, trong phiên bản plugin này, các số XYZ và góc tương ứng với **thế giới mô phỏng** (bao gồm cả pitch/roll khi leo dốc), mặc dù `header.frame_id` mặc định là `odom`. Chúng **không phải** ước lượng định vị GPS ngoài đời thật; cũng không thay thế IMU/LiDAR của robot vật lý. Cần kiểm tra thêm trên máy của bạn rằng plugin cài đặt vẫn theo hành vi này (chạy xe lên dốc xem Z/pitch thay đổi).

### Chỉ theo dõi, không điều khiển

```bash
bash stage3_control_feedback.sh --monitor-only
```

Dữ liệu được tự động ghi vào `~/ackermann_stage3_pose.csv` ở tốc độ tối đa 5 bản ghi/giây để mở bằng MATLAB, Python hoặc Excel; khi chạy nhiều lần file được nối thêm, không ghi đè. Đổi nơi lưu bằng `--csv /tmp/pose.csv` hoặc tắt ghi bằng `--csv ''`.

## Kiểm tra nếu không thấy trạng thái

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep ackermann
ros2 topic echo /ackermann_vehicle/odom --once
gz stats
```

Nếu không có `/ackermann_vehicle/odom`, kiểm tra Gazebo có spawn thành công `terrain_probe_s3` và plugin `libgazebo_ros_ackermann_drive.so` có nạp không. Nếu không thấy `pitch/roll` thay đổi khi xe đi trên dốc, kiểm tra bản plugin được cài và tọa độ gốc `base_footprint`/hệ quy chiếu pose. Không bật lại LiDAR để kiểm tra việc này.