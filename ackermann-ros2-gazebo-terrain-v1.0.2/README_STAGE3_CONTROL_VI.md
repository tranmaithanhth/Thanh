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
| `w` | Tiến 0,25 m/s, đồng thời hủy lệnh rẽ cũ |
| `s` | Lùi 0,10 m/s (hoặc `--reverse-speed`), đồng thời hủy lệnh rẽ cũ |
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
## Chẩn đoán xe đứng im dù lệnh khác 0 (23/11/2026)

Trong một thử nghiệm, xe đứng ở khoảng `(x=2.54, y=-3.52)` dù `cmd=(-0.10,0)`, odometry chỉ cho `v=-0.01 m/s`. Đó **không phải** giới hạn diện tích terrain (`x,y` đều thuộc khoảng -20..20 m). Vị trí `x=2.5` trùng điểm bắt đầu ramp, nhưng không chứng minh ramp là nguyên nhân.

Bản cập nhật `tools/stage3_console.py` in `[WARN] STALL` khi có lệnh tiến/lùi từ 0.08 m/s nhưng vận tốc thân xe < 0.025 m/s liên tục ít nhất 4 s. Đó là cảnh báo **điều kiện chuyển động**, không phải khẳng định bánh xe đã bị khóa.

**A/B test an toàn (không LiDAR, không camera, không sửa XACRO gốc):**

1. Đóng **cả** Gazebo và console cũ. Chạy `bash run_world_clean.sh` và chắc chắn không có robot cũ.
2. Thử cấu hình stage 3 gốc: `bash spawn_robot_stage.sh 3`, sau đó trong terminal riêng `bash stage3_control_feedback.sh --deadman 15`.
3. Lặp thử với một phiên Gazebo sạch, nhưng spawn bằng:
   ```bash
   ACKERMANN_SAFE_FRICTION=1 bash spawn_robot_stage.sh 3
   ```
   Tùy chọn này đặt hệ số ma sát 4 bánh từ 1000 về 1.0 và bỏ `fdir1=0 0 1` **chỉ ở URDF tạm**. Giữ mọi tham số khác như cũ. Cùng tốc độ và đường chạy, so sánh xe có vượt `x=3` được không, độ lệch ngang, RTF và log `[WARN] STALL`.
4. **Chỉ một bộ điều khiển:** kiểm tra `ros2 topic info /ackermann_vehicle/cmd_vel -v`; tránh chạy `drive_robot_on_terrain.sh` và `stage3_control_feedback.sh` đồng thời. `cmd=(...)` in trên console là lệnh nội bộ của console, chưa đủ xác nhận Gazebo nhận lệnh nếu có nhiều publisher.
5. Để đối chiếu tốc độ mô phỏng: `timeout 8s gz stats`. Nếu chỉ có cấu hình ma sát gốc bị dừng, nguyên nhân cần kiểm tra nằm trong tiếp xúc bánh–đường; nếu cả hai cùng bị dừng, cần xem tiếp lực kéo PID/khớp và vị trí từng bánh.

Lưu ý: không sử dụng `--deadman 60` cho các thử nghiệm gần chướng ngại; dùng 15 s và bấm X/Space để dừng sớm. Khi không cần lái, chạy `bash stage3_control_feedback.sh --monitor-only` (không phát lệnh, vẫn phản hồi tư thế).

## Phát hiện xe như chạm ranh giới khi lùi ở x≈2.4 m

Log tại (x=2.415,y=-1.357) cho thấy `cmd=(-0.25,+0.25)`: xe đang được yêu cầu lùi **đồng thời rẽ**, không phải lùi thẳng. Bản sửa console mới đặt `angular.z=0` khi nhấn `W` hoặc `S`; bạn vẫn có thể nhấn A/D sau đó để rẽ. Để thử ngay trên bản chưa cập nhật, nhấn X rồi S, hoặc nhấn C để xóa lệnh rẽ.

**Thử nghiệm tách địa hình và truyền động (không LiDAR):**

- Đóng console và Gazebo cũ, cập nhật GitHub.
- Mở Gazebo sạch bằng `bash run_world_clean.sh`.
- Spawn robot cấp 3 trên đoạn đất phẳng **x = -4 m**, y=-2 bằng `ACKERMANN_SPAWN_X=-4 ACKERMANN_SPAWN_Y=-2 bash spawn_robot_stage.sh 3`.
- Điều khiển bằng `bash stage3_control_feedback.sh --deadman 10 --reverse-speed 0.25`; nhấn S (không rẽ) và theo dõi x có giảm từ -4 xuống -5 không. Chỉ có một node gửi lệnh `cmd_vel`.
- Nếu lùi được trên vùng x=-4 nhưng không lùi được ở x≈2.4, cần kiểm tra tiếp tiếp xúc bánh với chân dốc x≈2.5 và lực kéo; **không có tường hay giới hạn vị trí** tại đó trong world SDF.
- Nếu lùi vẫn không được ở x=-4 nhưng tiến được, lỗi liên quan đến cơ cấu truyền động khi đảo chiều, điều khiển PID hoặc chiều quay khớp bánh. Kiểm tra thêm `ros2 topic info /ackermann_vehicle/cmd_vel -v` để loại trừ nhiều publisher.

Lưu ý: vùng terrain tồn tại x,y từ -20 đến 20 m. Kịch bản thử này chỉ thay đổi điểm spawn, **không thay đổi world, cảm biến hoặc plugin Ackermann**. Kết luận về lỗi truyền động cần dựa trên kết quả thử thực tế.

**Chú ý về ý nghĩa lệnh rẽ:** Trong phiên bản `gazebo_ros_ackermann_drive` này, giá trị `Twist.angular.z` được xử lý trực tiếp như **góc lái yêu cầu (rad)** để tính hai góc lái Ackermann, không phải tốc độ yaw (rad/s) theo cách hiểu thông thường của `geometry_msgs/Twist`. Vì vậy `cmd=(-0.25,+0.25)` nghĩa là lùi và yêu cầu góc lái xấp xỉ 0.25 rad; còn trường `wz` đo từ odometry mới là tốc độ quay thân xe (rad/s).
