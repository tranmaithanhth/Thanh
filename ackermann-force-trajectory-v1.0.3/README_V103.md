# Ackermann Terrain3D ROS 2 Humble / Gazebo Classic v1.0.3

**Đối chiếu MATLAB v1.0.4 + mô hình dẫn động cải tiến AWD 45 N·m / 300 N + A* tự hành bằng lực phát động và góc lái, KHÔNG LiDAR/camera.**

## 0. Phạm vi và khác biệt

Giữ nguyên terrain/world v1.0.2 (heightmap 40×40m, obstacle, cao độ MATLAB đã có z-shift ≈0,063m). Không chỉnh sửa robot gốc cài ở `ackermann_vehicle` và không chỉnh file gốc v1.0.2; `spawn_force_v103.sh` xử lý URDF tạm, thay plugin vận tốc bằng **Gazebo ModelPlugin C++ riêng** có đầu vào lực.

| Thông số | MATLAB Terrain3D v1.0.4 | Robot Gazebo gốc | v1.0.3 |
|---|---:|---:|---:|
| Tổng khối lượng | 20 kg | ≈12,002 kg | **20 kg** (chassis 15,998 + 4 bánh ×1 + 2 cụm lái ×0,001) |
| Wheelbase / track | 0,80 / 0,50 m | 0,80 / 0,50 m | 0,80 / 0,50 m |
| Bán kính bánh | 0,15m | 0,15m | 0,15m |
| Quán tính quay bánh | 0,03 kgm² | ~0,01125 kgm² quanh trục | **0,03 kgm²** trên trục quay quán tính bánh |
| Jroll / Jpitch / Jyaw | 0,85 / 1,45 / 1,50 kgm² | từ hộp 8 kg và bánh | 0,85 / 1,45 / 1,50 nhập ở `chassis` (tổng hệ gồm vị trí bánh sẽ khác) |
| Mômen phát động tổng | ±22 Nm, hai bánh sau | PID vận tốc chưa khớp mômen | **±45 Nm**, chia 4 bánh ở chế độ cải tiến AWD (11,25 Nm/bánh ở giới hạn) |
| Góc lái / tốc độ lái | ±0,52rad / ±0,95rad/s | ±0,60 rad plugin | **±0,52rad / ±0,95rad/s** |
| Ma sát bánh | **Không có μ Coulomb tương đương**; MATLAB giả thiết lăn không trượt | μ bánh 1000; μ mặt 1,1 | **μ bánh 1,1** và mặt đường 1,1, là giả định cần nhận dạng thực nghiệm |
| Độ cứng, cản đứng | 1600 N/m, 72 N·s/m mỗi bánh (tính Fz) | Gazebo ODE collision kp 100000, kd10 | **Chưa tương đương**; cần mô hình lốp/giảm chấn riêng để khớp lực Fz |

Trong MATLAB `terrain3d_reduced_rhs`, lực tổng cầu sau được suy từ `tau / Rw`, với giới hạn gốc `tau_max=22 N·m` (±146,67 N). **Chế độ Gazebo cải tiến này KHÔNG giữ giới hạn mômen MATLAB:** giới hạn tổng 45 N·m (tương đương ±300 N), phân bố lên bốn bánh chủ động (AWD) nhằm tăng độ bám khi leo dốc. Các thông số khối lượng, hình học, quán tính bánh và bản đồ vẫn tham chiếu MATLAB v1.0.4. Gazebo dùng vật rắn có trượt, nên chưa thể kỳ vọng quỹ đạo và lực tiếp xúc trùng MATLAB. Bộ lập kế hoạch dùng `m=20`, `L=0.8`, `Rw=0.15`, lực trần ±300 N và lực hỗ trợ khởi hành dốc 175 N, độ dốc max0,50. Trọng tâm ROS được giữ tại center của chassis; pose định vị P của MATLAB là giữa cầu sau, lệch `0.4m` so với pose model. Controller tự bù độ lệch này.

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

`/ackermann_vehicle/force_steer` loại `std_msgs/msg/Float64MultiArray` với **2 phần tử theo thứ tự** `[F_drive_N, steer_rad]`. Đây là lệnh lực N, **KHÔNG phải tốc độ**, và có thể âm để phanh/lùi. Động cơ nhận `tau_rear_l = tau_rear_r = F * Rw / 2`. `|F| <= 300N`, `|delta|<=0.52rad`, tốc độ quay lái tham chiếu <=0.95rad/s. Nếu không nhận lệnh 0,6s, plugin đưa lực và góc đặt về 0 (trên dốc có thể trôi vì không tạo phanh giữ).

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

W cộng **+20 N** mỗi lần (mặc định mới), S giảm **−20 N** mỗi lần (qua 0 rồi mới lùi), A/D tăng giảm góc lái **0,05 rad** mỗi lần, C đưa góc lái về 0, X dừng phát động, Q thoát. Mọi lệnh bị chặn ở ±300 N và ±0,52 rad; hết 5 giây không nhấn sẽ trả về 0. Chương trình hiển thị liên tục vị trí, vận tốc thực và lực/góc ra lệnh. Dùng để xác định riêng khả năng xe lùi, trước khi khởi động A* tự hành.
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

## 9. Cải tiến leo dốc: AWD, mômen tổng 45 N·m và giới hạn lực 300 N (23/11/2026)

**Lý do:** Độ dốc cực đại của hàm cao độ xấp xỉ 0,5 (≈26,6°). Với xe 20 kg, thành phần trọng lực dọc dốc khoảng `m g sin(atan 0.5) ≈ 87,7 N`. Trước đây tổng mômen cầu sau 22 N·m ⇔ 146,67 N quy đổi; tuy nhiên giới hạn **lực bám** của riêng hai bánh chủ động có thể chỉ khoảng `mu*(m*g*cos(atan(0.5))/2) ≈ 96,5 N` nếu phân bố tải gần 50/50. Khi bánh quay trượt, tăng lực động cơ mà không tăng diện tích/tải bánh chủ động sẽ không khắc phục được.

**Cấu hình nâng cấp, cố ý khác MATLAB:**

- MATLAB v1.0.4 vẫn giữ tham chiếu `22 N·m` (không sửa dữ liệu MATLAB).
- Gazebo v1.0.3 chuyển qua **AWD cả 4 bánh**: tổng mômen động cơ tối đa `45 N·m`, lực quy đổi `300 N`, phân bố đều từng bánh; tại giới hạn `11,25 N·m/bánh`. Lực `300 N` là giới hạn mômen **quy đổi**, không phải lực tiếp tuyến chắc chắn có thể truyền xuống đường.
- Sử dụng ma sát Coulomb giả định `mu=1,1`: lực bám lý tưởng của **cả bốn bánh** trên dốc khoảng `mu*m*g*cos(atan(0.5)) ≈ 193 N`, còn thay đổi theo phân bố tải thực tế. Vì vậy không nên luôn yêu cầu 300 N; trượt bánh có thể tăng nếu đạp lực quá lớn.
- Bộ lập kế hoạch tự hành có **hỗ trợ khởi hành trên dốc** (mặc định lực yêu cầu không thấp hơn `175 N` khi dốc dọc >=0,16 và tốc độ tiến thực <0,15 m/s), nhưng các giới hạn an toàn quỹ đạo vẫn giữ nguyên. Hỗ trợ này chỉ mang tính thử nghiệm, không bảo đảm xe leo dốc thành công nếu sai trục bánh, thiếu ma sát hoặc Gazebo contact solver bị kẹt.

**Phải biên dịch lại plugin C++** sau khi pull bản sửa, và phải đóng Gazebo cũ rồi tạo world mới để tải plugin và URDF mới:

```bash
cd ~/ackermann_humble_ws/Thanh
git pull --ff-only origin main
cd ackermann-force-trajectory-v1.0.3
bash setup_v103.sh
python3 -m unittest discover -s tests -v
```

Terminal 1: `bash run_world_v103.sh`. Terminal 2: `bash spawn_force_v103.sh`. Terminal 3: thử điều khiển `bash manual_drive_v103.sh --force-step 20 --steer-step 0.05 --deadman 5`. Tăng lực **từng bước**, lần lượt 40, 80, 120, 160–180 N, quan sát `/ackermann_vehicle/force_state`. Nếu bánh quay nhanh nhưng vận tốc thân gần 0, **bánh trượt hoặc bị kẹt về cơ khí**; không tiếp tục tăng lực.

`force_state` giữ 6 phần tử đầu: `[F_N, delta_ref, delta_actual, omega_RL, omega_RR, v_mps]`; thêm 4 phần tử: `[omega_FL, omega_FR, tau_per_rear_wheel_Nm, tau_per_front_wheel_Nm]`. Tất cả có dấu, thuận tiện so sánh trục quay/bám đường. Dừng thủ công bằng X hoặc Q, **X gửi lực 0 chứ không tạo phanh giữ dốc**.

Sau khi tiến/lùi thủ công được, tắt manual rồi thử `bash auto_drive_v103.sh --dry-run`; chỉ chạy auto thật sau khi xác nhận chiều quay và bám đường. Không chạy manual và auto cùng lúc. Nếu muốn so sánh với MATLAB 22 Nm, chỉnh cả `matlab_vehicle_params.json` và `mission.json` về `146.6666667 N`, đặt `enhanced_max_total_drive_torque_Nm: 22.0` và `drive_mode: rear`, rồi respawn và kiểm tra thử; đổi mode không cần build lại C++ nếu plugin đã build theo phiên bản mới.

## 10. Sửa lỗi *F=300 N nhưng xe không leo qua x≈3,07 m* (23/11/2026)

**Dấu hiệu thực tế:** xe nhận `F=+300 N`, `delta=0`, nhưng từ `x≈3.38` lùi về `x≈3.01` rồi dao động quanh `x≈3.07`. Vận tốc bánh chỉ khoảng 0,1–3 rad/s, nên đây không phải bằng chứng bánh quay trơn dữ dội.

**Phát hiện lỗi cốt lõi của plugin:** trong `ackermann_force_drive.cpp` trước bản vá, `OnUpdate()` có `if (dt <= 0.0 || dt < 0.01) return;`, tức bỏ qua gần 9/10 bước vật lý trong world có `max_step_size=0.001 s`. Gazebo Classic `Joint::SetForce()` là tác động lên bước mô phỏng hiện tại; muốn mômen liên tục thì phải gọi ở mỗi `WorldUpdateBegin`. Vì thế **mômen yêu cầu 45 N.m không được đưa đều vào mô phỏng**, dù `force_state` in ra giá trị lệnh 300 N.

**Đã sửa:** mọi vòng cập nhật vật lý đều tính mômen 4 bánh và góc lái; chỉ giới hạn xuất `odom`/`force_state` ở 50 Hz bằng biến `last_publish_`, sau các lệnh `SetForce`. Bản vá **không tăng giới hạn lực** và không làm thay đổi địa hình, khối lượng, ma sát hay kiến trúc A*.

**Quan trọng: phải biên dịch plugin và khởi động lại Gazebo**, vì sửa C++ không thể áp dụng vào server Gazebo đang chạy:

```bash
cd ~/ackermann_humble_ws/Thanh
git pull --ff-only origin main
cd ackermann-force-trajectory-v1.0.3
bash setup_v103.sh
python3 -m unittest discover -s tests -v
# Đóng Gazebo cũ, trong Terminal 1:
bash run_world_v103.sh
# Terminal 2, cùng thư mục:
bash spawn_force_v103.sh
# Terminal 3:
bash manual_drive_v103.sh --force-step 20 --steer-step 0.05 --deadman 5
```

Thử ở đường bằng `F=40–60 N` trước để xác nhận tiến/lùi đúng chiều và không đột ngột vọt nhanh. Sau đó leo dốc, tăng có kiểm soát `80 → 120 → 160 N` nếu cần; nhấn `X` để trả lực về 0, nhưng **X không phải phanh đỗ trên dốc**. Chỉ cân nhắc trên 160 N sau khi quan sát bánh và vận tốc thân. Khi đã sửa thời gian áp lực, cùng một mức lệnh giờ có thể tạo tác động **mạnh hơn nhiều** so với bản lỗi.

Kiểm tra `/ackermann_vehicle/force_state`: 10 phần tử đầu là `[F_cmd, steer_ref, steer_actual, omega_RL, omega_RR, body_v, omega_FL, omega_FR, tau_each_rear, tau_each_front]`. Để đánh giá leo dốc cần quan sát **tọa độ X tăng thực tế theo thời gian**, không suy ra thành công từ `body_v` tức thời. Nếu vẫn kẹt dù bánh không quay trượt, kiểm tra va chạm gầm và khoảng sáng bánh bằng `View → Collisions` trong Gazebo. Đừng nâng 300 N thành 500 N khi chưa xác định nguồn cản.

**Kiểm thử có giới hạn:** kiểm thử Python chỉ kiểm tra cấu trúc và mô hình dự báo; **chưa xác nhận** trong Gazebo thật với lốp–đường. Nếu `bash setup_v103.sh` báo lỗi biên dịch, lấy cả phần `error:` của C++ và CMake để sửa tiếp.

## 11. Sửa lỗi autonav chọn lực âm và mắc kẹt ở x≈1,86 m (23/11/2026)

**Log gốc:** A* tìm 5 waypoints `(-0.4,-2)→(4.5,3.8)→(13.2,5.9)→(16,2)→(16,0.6)`. Robot đã chạy tới `P≈(1.86,-0.26)`, `yaw≈63.5°`, rồi dừng với `v≈0.02–0.16 m/s` mặc dù `vref=0.75`. Bộ dự báo thường chọn `F=-4..+30 N` thay vì tiến, sau 12 giây báo `STALL`.

**Hai lỗi thuật toán đã được phát hiện và sửa trong `mission.json` + `tools/planner_core.py`:**

1. Ngưỡng độ dốc toàn cục `max_terrain_grade=0.50` thấp hơn gradient thực của heightmap tại `x≈4.5,y≈-2.1` (`≈0.510`). Do đó hành lang đi dưới các vật cản bị A* loại, dù xe đã được thử chạy lên dốc bằng lực thủ công. Đã đặt `max_terrain_grade=0.55` (**chỉ lập kế hoạch, không sửa vật lý terrain**). Đồng thời thêm `route_clearance_margin_m=0.35` ngoài bán kính vùng cấm đã inflation 1,20 m; cả A* và quá trình rút gọn đường đều phải giữ khoảng cách này. Thử nghiệm A* độc lập với cấu hình mới tìm đường gần như đi thẳng theo X ở phía dưới các vật cản, thay vì tuyến cũ áp sát vùng cấm với dư khoảng 3 mm.
2. Bộ chọn lực cũ chỉ xét `F0±30 N`, có thể chọn lực âm khi vận tốc gần 0 dù lệnh vận tốc tham chiếu vẫn dương. Hàm thưởng `-0.8*pred.v` khuyến khích chạy nhanh quá mức, trong khi dự báo `step()` lại chặn tốc độ ngay ở `0.75 m/s`, che mất hiện tượng robot thực đạt `1.05 m/s`. Đã sửa: mẫu lực rộng hơn, có ràng buộc `min_progress_force_N=15` khi đang đứng chậm và cần tiến; chấm điểm theo sai số tốc độ và phạt vượt tốc; tích phân tốc độ vật lý đến `physical_max_speed_m_s=1.45`. **Mọi ứng viên vẫn phải qua ràng buộc an toàn** (va chạm/độ dốc). Nếu không có ứng viên an toàn, bộ điều khiển trả về lực 0 chứ không ép xe tiến.

**Kiểm thử thuật toán** (mô hình dự báo rút gọn, chưa phải Gazebo): với tuyến gần `(-0.4,-2)→(11.5,-2.15)→(16,0.6)`, vòng lặp mô phỏng điều khiển mới có thể đi tới vùng đích mà không bị kẹt gần `x=1.86`. Dự báo này không chứng minh chắc chắn xe chạy thành công trong Gazebo, vì Gazebo có động lực học tiếp xúc/trượt.

**Cập nhật và chạy lại từ đầu** (không cần build lại plugin C++, vì lần này chỉ sửa Python và JSON):

```bash
cd ~/ackermann_humble_ws/Thanh
git pull --ff-only origin main
cd ackermann-force-trajectory-v1.0.3
python3 -m unittest discover -s tests -v
# Đóng hết world và các controller cũ trước khi thử
# Terminal 1:
bash run_world_v103.sh
# Terminal 2:
bash spawn_force_v103.sh
# Terminal 3: chạy kiểm tra trước:
bash auto_drive_v103.sh --dry-run
# Ctrl+C sau khi đọc dòng A*; sau đó chạy thực:
bash auto_drive_v103.sh
```

Khi mở auto mới, tuyến A* **không nên** còn bắt đầu bằng `(-0.4,-2)→(4.5,3.8)`. Log dự kiến sẽ có đoạn dài đi gần `y=-2` và rẽ tới đích sau khi qua các vật cản đầu tiên. Nếu vẫn còn 5 waypoint tương tự cũ, phải kiểm tra `git pull` và file `mission.json` đang dùng. Không chạy manual cùng auto; nếu xe đi sai thì Ctrl+C, nhưng lực về 0 không phải phanh giữ trên dốc.

## 12. Dry-run A* tìm đường được nhưng sau 12 giây báo STALL (24/11/2026)

Khi chạy `bash auto_drive_v103.sh --dry-run`, robot **được thiết kế để đứng yên**: node vẫn đọc odom và tìm đường nhưng không tạo publisher lệnh `/ackermann_vehicle/force_steer`. Vì vậy thông báo `STOP: STALL: <0.20m progress over 12s` trong các bản trước là lỗi logic của watchdog, **không phải xe bị kẹt**. Đã sửa: khi `dry_run=True`, vô hiệu hóa kiểm tra tiến triển vị trí; còn kiểm tra kẹt vẫn bật trong chế độ điều khiển thật.

Log đã nhận từ người sử dụng cho thấy cấu hình mới hoạt động đúng: `max_grade=0.55, extra_clearance=0.35m` và A* tìm 3 waypoint `[(-0.4,-2.0),(11.5,-2.2),(16.0,0.6)]`. Lệnh dự kiến tại chỗ khoảng `F=+20.8 N, delta=+9.9°`, nhưng **không được gửi đi** ở chế độ `--dry-run`. Để xe chạy thật, ngừng dry-run rồi chạy `bash auto_drive_v103.sh` (không có tham số `--dry-run`).

Bản vá dry-run chỉ thay đổi Python; cập nhật bằng `git pull --ff-only origin main` và **không cần biên dịch lại C++**. Trong log mới, node in `[DRY RUN] Planner preview only: NO force_steer commands are published ... STALL detection disabled.` và sẽ tiếp tục hiển thị tuyến/điều khiển dự kiến tới khi người dùng nhấn Ctrl+C hoặc giới hạn thời gian khác. Không chạy manual_drive cùng lúc với auto_drive thật. Trong Gazebo cần quan sát chuyển động và dừng thử nghiệm nếu điều khiển bất thường; lực về 0 không phải phanh đỗ trên dốc.
