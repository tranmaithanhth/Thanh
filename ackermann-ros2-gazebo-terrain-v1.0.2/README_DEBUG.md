# Chẩn đoán từng thành phần robot Ackermann / ROS 2 Humble / Gazebo Classic 11

**Không thay đổi file URDF/XACRO gốc**. Script tự đọc `ackermann_vehicle.urdf.xacro` trong workspace, dùng `xacro` sinh URDF tạm rồi lọc các thành phần theo mức thử nghiệm.

## Mức thử

| Mức | Nội dung | Kiểm tra |
|---|---|---|
| A | Chỉ `base_footprint` + `chassis` | Quán tính thân, trọng lực, va chạm thân–địa hình |
| B | A + 4 bánh + khớp lái/quay bánh | Tiếp xúc bánh–đường và ODE joint constraints; **không plugin** |
| C | B + `libgazebo_ros_ackermann_drive.so` | Plugin điều khiển Ackermann |
| D | C + LiDAR | ROS ray sensor |
| E | C + camera độ sâu (**không LiDAR**) | Gazebo depth camera |
| F | C + LiDAR + camera | Mô hình đầy đủ |

## Hướng dẫn

Mỗi mức **phải bắt đầu với một world mới**, không spawn chồng nhiều robot. Terminal 1 (đóng phiên Gazebo trước rồi chạy):

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-ros2-gazebo-terrain-v1.0.2
bash run_world_clean.sh
```

Xác nhận Gazebo ở **Play**, `gz stats` có `Paused[F]`, RTF gần 1 khi chưa có robot.

Terminal 2 (ví dụ mức B):

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-ros2-gazebo-terrain-v1.0.2
bash spawn_robot_stage.sh B
```

Terminal 3:

```bash
ps -C gzserver -o pid,stat,%cpu,%mem,etime,args
timeout 5s gz stats
```

Quan sát 10–15 giây rồi ghi lại RTF và `SimTime`. Nếu chậm nghiêm trọng, đừng tiếp tục spawn các mức khác; đánh dấu **mức đầu tiên gây lỗi**, giữ log. Muốn thử mức kế tiếp, đóng Gazebo và mở world lại từ Terminal 1.

Mức C có plugin: chỉ sau khi xác nhận đứng yên ổn định, thử vận tốc nhỏ (trong Terminal đã source Humble + workspace):

```bash
ros2 topic pub -r 5 /ackermann_vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'
```

Nhấn Ctrl+C rồi gửi lệnh dừng:

```bash
ros2 topic pub --once /ackermann_vehicle/cmd_vel geometry_msgs/msg/Twist '{}'
```

## Cách đọc kết quả

- **A đã lỗi**: kiểm tra inertial/collision chassis, physics world.
- **A bình thường, B lỗi**: tập trung vào collision của 4 bánh, joint rpy/axis, hệ số ma sát bánh `mu1,mu2=1000` trong `macros.xacro`, `kp,kd` và ODE (chưa kết luận nguyên nhân).
- **B bình thường, C lỗi**: plugin lái, tên joint, PID, callback cập nhật.
- **C bình thường, D lỗi**: LiDAR (ray sensor/plugin).
- **C bình thường, E lỗi**: camera sâu (GPU/rendering/plugin).
- **D,E độc lập bình thường, F lỗi**: xung đột/chi phí kết hợp cảm biến.

**Cảnh báo:** RTF thấp hơn 1 một chút không phải treo; xem `SimTime` có tăng không. RTF gần 0, CPU cao và thời gian đứng yên mới là tín hiệu bất thường. Khi thử phải giữ số bước `max_step_size` và world không đổi để so sánh.