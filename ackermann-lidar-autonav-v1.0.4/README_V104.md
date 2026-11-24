# Ackermann LiDAR Autonav v1.0.4 — ROS 2 Humble / Gazebo Classic 11

**Mục tiêu:** kế thừa toàn bộ robot AWD + lực phát động F + góc lái delta và terrain 3D/tường bao của v1.0.3, thêm cảm biến LiDAR 2D ROS `/scan`, phát hiện vật cản mới, phanh an toàn khi cần và thử lập lại đường A* theo bản đồ vật cản động. **Chỉ mô phỏng và nghiên cứu — chưa đủ điều kiện vận hành robot thật.**

**Không sử dụng Jazzy, không dùng `cmd_vel`, không thay đổi động lực học C++.** Robot không có camera. Có thể chạy chế độ quan sát LiDAR trước khi chạy tự hành.

## 1. Kết cấu

| Mô đun | Nội dung |
|---|---|
| World | Dùng lại terrain 40×40 m và 4 tường va chạm `worlds/ackermann_matlab_terrain_walled.world` |
| Truyền động | Dùng chính `libackermann_force_drive.so` của v1.0.3, AWD, tổng lực quy đổi ±300 N |
| LiDAR | Gazebo Classic `ray` + `libgazebo_ros_ray_sensor.so` |
| LiDAR frame | `lidar_link` gắn cứng lên `chassis`, x=0,20 m, z=0,33 m so với chassis |
| Quét | **360°, 91 tia, 5 Hz, tối đa 8 m**, topic `/scan` / `sensor_msgs/msg/LaserScan` |
| Cảm nhận | `tools/lidar_obstacles.py`: đổi tia quét sang bản đồ XY, lọc điểm của vật cản cố định, gom voxel, tối đa 30 điểm gần nhất |
| Lập đường | `tools/lidar_planner.py`: thêm vật cản LiDAR vào `Mission` A*, không thay A* gốc |
| An toàn | Dừng bằng lực phanh ngược chiều chuyển động khi thiếu dữ liệu `/scan` hoặc vật cản quá gần |
| Điều khiển | `tools/autonomous_lidar_nav.py`: xuất `Float64MultiArray [F_N,delta_rad]` trên `/ackermann_vehicle/force_steer` |

Lưu ý: 91 tia trên toàn bộ 360° cho độ phân giải góc ~4°. Cấu hình này cố ý nhẹ, chưa đủ tinh để nhận biết mọi vật nhỏ. Việc nhận biết chướng ngại thực tế cũng phụ thuộc chiều cao của mặt phẳng quét, độ phản xạ và góc robot khi lên dốc.

## 2. Chuẩn bị thư viện

Trên máy Ubuntu 22.04 đã có ROS 2 Humble và Gazebo Classic 11:

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

Khuyến nghị giữ nguyên plugin C++ v1.0.3 đã chạy thành công.

```bash
cd ~/ackermann_humble_ws/Thanh
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
bash setup_v104.sh
```

`setup_v104.sh` kiểm tra plugin LiDAR và plugin lực v1.0.3; nếu lực chưa được biên dịch, script sử dụng `../ackermann-force-trajectory-v1.0.3/setup_v103.sh`; sau đó chạy bài kiểm tra Python offline. Nếu có test báo `FAIL`/`ERROR`, chưa chạy tự hành thật.

## 3. Khởi động an toàn theo giai đoạn

Tắt **toàn bộ** Gazebo cũ, spawn robot cũ và controller manual/auto trước khi mở world mới.

**Terminal 1 — world có tường bao:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash run_world_v104.sh
```

**Terminal 2 — robot stage 3 + LiDAR:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash spawn_force_v104.sh
```

**Terminal 3 — kiểm tra cảm biến, KHÔNG điều khiển robot:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash scan_v104.sh
```

Log dự kiến: `[SCAN] packets=... beams=91 angle=[-180,180]deg front=... rear=... frame=lidar_link`. Có thể kiểm tra thêm trong terminal ROS 2 đã source:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '^/scan$|^/ackermann_vehicle/odom$'
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Nếu Gazebo bị lag hoặc thiếu topic `/scan`, không chạy auto. Kiểm tra `/opt/ros/humble/lib/libgazebo_ros_ray_sensor.so`, Gazebo terminal, và `ros2 topic info /scan -v`. LiDAR có thể dùng QoS Best Effort, hãy chọn Best Effort trong RViz nếu hiển thị không có dữ liệu.

**Terminal 4 — chỉ xem quỹ đạo, không cấp lực:**

```bash
bash auto_drive_v104.sh --dry-run
```

Nếu `/scan` và A* ổn định, Ctrl+C thoát dry-run và chạy **duy nhất 1 controller thật**:

```bash
bash auto_drive_v104.sh
```

Có thể xem log `[LIDAR]`, `[LIDAR HOLD]`, `[LIDAR REPLAN]`, `[LIDAR A*]` và `[AUTO]`. Nếu scan mất hơn 1,1s, controller chủ động phanh thay vì tiếp tục đi theo bản đồ tĩnh. `--dry-run` không phát lệnh tới plugin.

## 4. Kiểm tra phát hiện vật cản mới

Khi Gazebo đang chạy, bạn có thể dùng chức năng Insert của Gazebo để thêm hộp mới vào trước robot. Hoặc tạo hộp tĩnh màu cam trên đường đi ngay khi robot đang di chuyển:

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
BOX_X=8.2 BOX_Y=-2.2 bash spawn_test_box_v104.sh
```

Lưu ý chỉ đặt hộp ở vị trí **không trùng robot**. Không dùng `spawn_test_box_v104.sh` nếu đã có model cùng tên: đổi `BOX_NAME` nếu cần.

**Phản ứng dự kiến:**
1. `/scan` phát hiện hộp mới.
2. `lidar_obstacles.py` lấy các điểm trên hộp và lọc bỏ bốn trụ đã có trong bản đồ.
3. `DynamicMission` phát hiện đường A* bị chặn.
4. Xe phanh xuống gần dừng; A* thử tìm đường vòng tránh hộp.
5. Nếu tồn tại đường an toàn, xe tiếp tục theo đường mới. Nếu chưa có, controller giữ phanh và đợi; không ép robot xuyên vật cản.

Đây là **hành vi được lập trình**, chưa phải kết quả thực nghiệm Gazebo đã xác nhận. Xe có thể mắc kẹt tại vị trí quá hẹp, lỗi mô phỏng tiếp xúc, hoặc LiDAR nhìn nhầm mặt dốc; cần gửi log để hiệu chỉnh.

## 5. Các bảo vệ đang có

- `lidar_stale_timeout_s=1.1`: không lái khi dữ liệu quá hạn; nếu liên tục mất scan, chương trình kết thúc với `LIDAR UNAVAILABLE`.
- Quãng đường phanh tăng theo vận tốc: `d=margin+|v|*reaction+v^2/(2*a)`.
- Tự tăng lực leo dốc từ v1.0.3 **chỉ hoạt động khi không bị LiDAR chặn**.
- `lidar_obstacle_ttl_s=1.1`: loại điểm cũ khi không còn scan mới.
- A* chỉ lập lại đường khi tốc độ đủ thấp; không đổi quỹ đạo gấp lúc xe đang lao nhanh.
- Nếu không có đường tránh hợp lệ, giữ trạng thái chờ/phanh thay vì ép đi.
- Phanh bằng lực mô phỏng chưa phải phanh đỗ; **không đảm bảo giữ xe đứng yên trên dốc**.
- 2D LiDAR không phát hiện được hố, độ cao vật thể ngoài mặt phẳng quét hoặc vật quá nhỏ; chưa có dự báo chuyển động chướng ngại vật, SLAM/TF đầy đủ hoặc Hybrid A*.

## 6. So sánh / quay về v1.0.3

Phiên bản v1.0.3 giữ nguyên tại `../ackermann-force-trajectory-v1.0.3`. Để quay về bản đồ tĩnh ổn định, hãy đóng world và controller v1.0.4 rồi:

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-force-trajectory-v1.0.3
bash run_world_v103.sh
# Ở terminal khác:
bash spawn_force_v103.sh
bash auto_drive_v103.sh
```

Không trộn `spawn_force_v103.sh` với `auto_drive_v104.sh`, vì v1.0.3 không có topic LiDAR `/scan`.

## 7. Kiểm tra kỹ thuật và lỗi thường gặp

```bash
python3 -m compileall -q tools
python3 -m unittest discover -s tests -v
bash -n setup_v104.sh run_world_v104.sh spawn_force_v104.sh auto_drive_v104.sh scan_v104.sh spawn_test_box_v104.sh
```

Nếu **đã có topic nhưng chưa có data**, đọc lỗi plugin từ Terminal 1, kiểm tra sensor URL/TF khi xem RViz. Nếu `/scan` có nhiều publisher, đóng mọi phiên Gazebo trùng. Nếu `[LIDAR HOLD]` xuất hiện mà không có vật thể, kiểm tra phản xạ từ mặt đất dốc hoặc phần thân robot; đây là hạn chế cần hiệu chuẩn sensor pose. Nếu Gazebo đứng, giảm số tia từ 91 xuống 61 hoặc tần số từ 5 xuống 3 trong `tools/patch_robot_v104.py`, sau đó khởi động lại Gazebo.

**Tình trạng xác minh:** mã đã được tạo trong GitHub, cấu hình và kiểm tra offline đã được chuẩn bị; chưa chạy được Gazebo trên máy người dùng từ xa. Phải xác minh `/scan` và tác động trong Gazebo trước khi coi tránh vật cản thời gian thực là đạt.
