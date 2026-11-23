# Ackermann Terrain3D ROS 2 Humble / Gazebo Classic v1.0.3

**Đối chiếu MATLAB v1.0.4 + lực phát động và góc lái + A* tự tìm đường + bám quỹ đạo, KHÔNG LiDAR/camera.**

## 0. Phạm vi và khác biệt

Giữ nguyên terrain/world v1.0.2 (heightmap 40×40m, obstacle, cao độ MATLAB đã có z-shift ≈0,063m). Không chỉnh sửa robot gốc cài ở `ackermann_vehicle` và không chỉnh file gốc v1.0.2; `spawn_force_v103.sh` xử lý URDF tạm, thay plugin vận tốc bằng **Gazebo ModelPlugin C++ riêng** có đầu vào lực.

| Thông số | MATLAB Terrain3D v1.0.4 | Robot Gazebo gốc | v1.0.3 |
|---|---:|---:|---:|
| Tổng khối lượng | 20 kg | ≈12,002 kg | **20 kg** (chassis 15,998 + 4 bánh ×1 + 2 cụm lái ×0,001) |
| Wheelbase / track | 0,80 / 0,50 m | 0,80 / 0,50 m | 0,80 / 0,50 m |
| Bán kính bánh | 0,15m | 0,15m | 0,15m |
| Quán tính quay bánh | 0,03 kgm² | ~0,01125 kgm² quanh trục | **0,03 kgm²** trên trục quay quán tính bánh |
| Jroll / Jpitch / Jyaw | 0,85 / 1,45 / 1,50 kgm² | từ hộp 8 kg và bánh | 0,85 / 1,45 / 1,50 nhập ở `chassis` (tổng hệ gồm vị trí bánh sẽ khác) |
| Mômen phát động tổng | ±22 Nm | PID tốc độ không giới hạn mômen tương đương | **±22 Nm**, chia đều 11 Nm mỗi bánh sau |
| Góc lái / tốc độ lái | ±0,52rad / ±0,95rad/s | ±0,60 rad plugin | **±0,52rad / ±0,95rad/s** |
| Ma sát bánh | **Không có μ Coulomb tương đương**; MATLAB giả thiết lăn không trượt | μ bánh 1000; μ mặt 1,1 | **μ bánh 1,1** và mặt đường 1,1, là giả định cần nhận dạng thực nghiệm |
| Độ cứng, cản đứng | 1600 N/m, 72 N·s/m mỗi bánh (tính Fz) | Gazebo ODE collision kp 100000, kd10 | **Chưa tương đương**; cần mô hình lốp/giảm chấn riêng để khớp lực Fz |

Trong MATLAB `terrain3d_reduced_rhs`, lực tổng cầu sau được suy từ `tau / Rw`, với giới hạn `tau_max=22 N·m`; phương trình động lực học gồm quán tính, cản lăn, lực dốc và khớp lái. Gazebo là rigid-body contact solver và có trượt, **không thể đảm bảo cùng quỹ đạo chỉ do trùng tham số**. Bộ lập kế hoạch dùng `m=20`, `L=0.8`, `Rw=0.15`, mômen 22Nm (lực tương đương ±146,67N), độ dốc max0,50, các vật cản giống MATLAB. Trọng tâm ROS được giữ tại center của chassis; pose định vị P của MATLAB là giữa cầu sau, lệch `0.4m` so với pose model. Controller tự bù độ lệch này.

## 1. Cài đặt một lần

Ubuntu 22.04, ROS 2 Humble + Gazebo Classic 11 đã chạy world v1.0.2, workspace `~/ackermann_humble_ws`. Phụ thuộc Gazebo SDK, `gazebo_ros`, CMake, colcon:

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev \
  ros-humble-gazebo-plugins libgazebo-dev python3-colcon-common-extensions
cd ~/ackermann_humble_ws/Thanh
# chỉ cần sau khi có commit v1.0.3:
git pull --ff-only origin main
cd ackermann-force-trajectory-v1.0.3
bash setup_v103.sh
```

**Ubuntu 22.04 Jammy:** gói SDK đúng là `libgazebo-dev` (Gazebo Classic 11), **không phải** `libgazebo11-dev` (tên gói ở Ubuntu 20.04). Xác minh bằng `apt-cache policy libgazebo-dev` và `gazebo --version`. Không chạy `sudo apt upgrade` hàng trăm gói chỉ vì lỗi tên gói. Nếu `apt` chưa thấy `libgazebo-dev`, kiểm tra thành phần `universe` của Ubuntu (`apt-cache policy libgazebo-dev`) trước khi chỉnh nguồn APT. 

`setup_v103.sh` tạo symlink `~/ackermann_humble_ws/src/ackermann_force_control` và build `.so` bằng colcon. Nếu apt báo tên gói không tồn tại, dùng `apt-cache search ros-humble-gazebo` để xác nhận theo repo APT trên máy bạn. Không cần `colcon build` lại khi chỉ sửa JSON mission; C++ plugin cần build lại nếu sửa C++.

## 2. Chạy Gazebo / spawn / lái tự động

**Đóng hết Gazebo cũ và ROS controller cũ trước khi chạy một phiên mới.** Không mở stage3_console cũ, không mở teleop.

Terminal 1:
```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-force-trajectory-v1.0.3
bash run_world_v103.sh
```

Terminal 2:
```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-force-trajectory-v1.0.3
bash spawn_force_v103.sh
```

Terminal 3: **chỉ quan sát / lập tuyến** trước khi cấp lực:
```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-force-trajectory-v1.0.3
bash auto_drive_v103.sh --dry-run
```

Dùng Ctrl+C để thoát dry-run, rồi chạy thật:
```bash
bash auto_drive_v103.sh
```

Tự lấy vị trí và hướng từ `/ackermann_vehicle/odom`, đổi từ tâm thân sang điểm P giữa cầu sau. A* 8 hướng dùng chi phí 3D, tránh bốn chướng ngại tĩnh, độ dốc <=0,50 và footprint đã inflate 1,20m. Tracker chọn **lực cầu sau và góc lái** bằng dự đoán chuyển động cục bộ, có giới hạn ma sát giả định qua điều kiện an toàn hình học, giới hạn mômen/vận tốc và dừng khi không tìm được quỹ đạo hợp lệ. Mặc định đích `(16.0,0.6)` trong `mission.json`. Không có tự cập nhật bản đồ hay tránh vật cản động vì **không có LiDAR**.

## 3. Giao tiếp force/steer

`/ackermann_vehicle/force_steer` loại `std_msgs/msg/Float64MultiArray` với **2 phần tử theo thứ tự** `[F_drive_N, steer_rad]`. Đây là lệnh lực N, **KHÔNG phải tốc độ**, và có thể âm để phanh/lùi. Động cơ nhận `tau_rear_l = tau_rear_r = F * Rw / 2`. `|F| <= 146.67N`, `|delta|<=0.52rad`, tốc độ quay lái tham chiếu <=0.95rad/s. Nếu không nhận lệnh 0,6s, plugin đưa lực và góc đặt về 0 (trên dốc có thể trôi vì không tạo phanh giữ).

`/ackermann_vehicle/odom` loại `nav_msgs/Odometry`, vị trí world, quaternion pose; `/ackermann_vehicle/force_state` là mảng 6 số `[F_N, delta_ref_rad, delta_actual_avg_rad, omega_RL_rad_s, omega_RR_rad_s, v_m_s]`; `/ackermann_vehicle/planned_path` loại `nav_msgs/Path` (frame `world`). Có thể kiểm tra:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep ackermann
ros2 topic echo /ackermann_vehicle/force_state
```

### Điều khiển thủ công kiểm tra ngắn (không đồng thời chạy auto)
```bash
source /opt/ros/humble/setup.bash
# 30 N tiến với bánh thẳng, chỉ trong 1 giây; Ctrl+C rồi gửi lệnh 0 nếu cần
ros2 topic pub /ackermann_vehicle/force_steer std_msgs/msg/Float64MultiArray "{data: [30.0, 0.0]}" -r 10
```

**Chỉ chạy một publisher `force_steer` tại một thời điểm**. Ở nơi có dốc, gửi `0` không đảm bảo xe lập tức đứng yên; plugin không có phanh đỗ. Dùng Gazebo Pause hoặc đóng phiên nếu xe đi sai hướng.

## 4. Điều khiển và mô hình toán

Dùng state `[x_P, y_P, yaw, v]`, lực phát động `F`, góc lái `delta`:

```text
xP_dot = v cos(yaw)
yP_dot = v sin(yaw)
yaw_dot = v tan(delta) / L
v_dot = (F - c_roll*v - m*g*grade) / (m + sum(Jw)/Rw^2)
```

Đây là **mô hình dự báo rút gọn** của tracker; Gazebo tính động lực học tương tác bánh-đường thực tế và quán tính toàn thân. `A*` tìm tuyến toàn cục, `local_force_steer` lấy mẫu các cặp `[F,delta]`, tích phân thử trên mô hình rút gọn và loại các mẫu không an toàn, sau đó chọn mẫu có chi phí mục tiêu thấp. Đây không phải TEB hoặc DWA MATLAB nguyên bản, và không tối ưu thời gian 6-DoF.

**Hạn chế/khả năng gây thất bại:** Nếu robot trượt, không đủ lực, quay ngược trục bánh, ma sát chưa khớp hoặc dốc nghiêng, chương trình có thể dừng sớm để tránh chạy sai; cần xem `force_state` và log `[AUTO]`. Cần nhận dạng lực tiếp xúc riêng để so sánh định lượng với MATLAB. Bộ lập kế hoạch A* biết sẵn vật cản tĩnh; không phát hiện người/vật mới xuất hiện.

## 5. Kiểm thử và phát triển

- Chạy test offline bằng `python3 -m unittest discover -s tests -v` tại thư mục v1.0.3.
- Trong Gazebo kiểm tra tối thiểu 4 trường hợp: (1) spawn không command đứng yên hoặc trôi nhẹ; (2) cấp F=+30N, wheel velocities và v>0; (3) cấp F=-30N sau khi dừng, wheel velocities và v<0; (4) auto đến gần goal, tránh cylinder, không làm Gazebo treo. Bất kỳ điểm không đạt, ngừng kiểm thử và kiểm tra URDF/trục bánh trước khi tăng lực.
- Đề xuất tiếp theo: bộ PID lực–góc lái hiệu chỉnh từ phản hồi thực tế, đối chiếu lực Fz/ma sát bằng gazebo contact wrench, planner nonholonomic/hybrid A* có ràng buộc bán kính quay, DWA/TEB đúng công thức MATLAB 11 trạng thái.

### Các tệp

- `matlab_vehicle_params.json`: ánh xạ tham số vật lý.
- `mission.json`: đích, chướng ngại, ngưỡng planner.
- `ackermann_force_control`: package C++ Gazebo Force Drive.
- `tools/patch_robot_v103.py`: sửa mass, inertia, μ, plugin trong URDF **tạm**.
- `tools/planner_core.py`: A* + lực/góc dự báo cục bộ (không cần ROS để test).
- `tools/autonomous_force_nav.py`: ROS 2 node đọc odom, phát force/steer và Path.
- Các script `.sh`: setup, run world, spawn, auto.

## 6. Lái thử bằng **lực phát động** và góc lái (không qua plugin vận tốc)

Mở riêng Terminal điều khiển sau khi world và robot v1.0.3 đã spawn; **không chạy auto đồng thời**:

```bash
bash manual_drive_v103.sh --force-step 10 --steer-step 0.05 --deadman 5
```

W cộng **+10 N** mỗi lần, S giảm **−10 N** mỗi lần (qua 0 rồi mới lùi), A/D tăng giảm góc lái **0,05 rad** mỗi lần, C đưa góc lái về 0, X dừng phát động, Q thoát. Mọi lệnh bị chặn ở ±146,67 N và ±0,52 rad; hết 5 giây không nhấn sẽ trả về 0. Chương trình hiển thị liên tục vị trí, vận tốc thực và lực/góc ra lệnh. Dùng để xác định riêng khả năng xe lùi, trước khi khởi động A* tự hành.
## 7. Xác nhận phiên bản MATLAB dùng làm chuẩn và phép thử khối lượng–ma sát

**Nguồn so sánh chính:** tài liệu `Huong_dan_chi_tiet_code_Ackermann_Terrain3D_MATLAB_v1.0.4.docx`, phần `ackermann_cfg` / `terrain3d_reduced_rhs`. MATLAB Terrain3D **v1.0.4** dùng `c.m=20 kg`, `c.maxTorque=22 N.m`, `Jw=0.03 kg.m²/bánh`, `c.cv=0.8 N.s/m/bánh`, `tireK=1600 N/m` và `tireC=72 N.s/m`. Đừng nhầm với code `ackermann-simulink-v1.0.2/ackermann_cfg.m` cũ ở cùng repo, trong đó `c.maxTorque=7.5 N.m`. V1.0.3 chủ đích theo **Terrain3D v1.0.4**, không phải v1.0.2.

**Độ tương đương:** tổng mass và hình học được khớp; ma sát không thể gọi là khớp chính xác vì MATLAB áp ràng buộc lăn không trượt + cản nhớt, trong khi Gazebo áp ma sát Coulomb tiếp xúc. Giá trị `mu=1.1` là **giả thiết hiệu chỉnh khởi đầu**, không phải hệ số rút ra từ MATLAB. Cặp `tireK/tireC` chưa được mô phỏng tương đương bằng hệ lốp/treo riêng; nếu cần kết quả lực `Fz` trùng MATLAB, phải bổ sung mô hình độ võng và giảm chấn bánh. Các quán tính nhập trên `chassis` là quán tính riêng của link, không bằng quán tính tổng hợp toàn xe với 4 bánh.

**Chạy kiểm thử tĩnh trước khi điều khiển:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-force-trajectory-v1.0.3
python3 -m unittest discover -s tests -v
```

Lệnh kiểm thử Python đối chiếu tổng khối lượng URDF giả lập, giá trị lực cực đại, các đoạn đường tránh vật cản và bộ tracker trên mô hình dự báo rút gọn. Đây **không thay thế** kiểm thử C++ plugin và tương tác lốp–đường trên Gazebo thật. Kiểm tra thêm khi Gazebo chạy: `ros2 topic echo /ackermann_vehicle/force_state` và so sánh vận tốc/chiều quay hai bánh sau với lực âm/dương để phát hiện hướng trục sai trước khi tự hành.

## 8. Khắc phục lỗi `AMENT_TRACE_SETUP_FILES: unbound variable`

Nếu chạy `bash spawn_force_v103.sh` báo `/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable`, nguyên nhân là shell con của script bật `set -u` trước khi `source` ROS 2. Đã sửa trong `spawn_force_v103.sh`, `manual_drive_v103.sh`, `auto_drive_v103.sh`: chỉ bật `set -u` sau khi nạp `/opt/ros/humble/setup.bash` và `$ACKERMANN_WS/install/setup.bash`.

Cập nhật `cd ~/ackermann_humble_ws/Thanh && git pull --ff-only origin main`, rồi `cd ackermann-force-trajectory-v1.0.3 && bash spawn_force_v103.sh` (cần Gazebo đang chạy bằng `run_world_v103.sh`). Nếu đã cài và build plugin thành công, **không phải build lại C++** cho sửa lỗi shell này. Kiểm tra cú pháp: `for f in spawn_force_v103.sh manual_drive_v103.sh auto_drive_v103.sh; do bash -n "$f" || break; done`.
