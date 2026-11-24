# HMI Ackermann v1.0.4 — Điều khiển, ghi đồ thị và đối chiếu MATLAB

HMI được viết bằng **PyQt5 + Matplotlib + rclpy**, chạy riêng trên Ubuntu 22.04 với ROS 2 Humble và Gazebo Classic 11. Không thay đổi plugin lực C++ hoặc các world và không phụ thuộc camera.

## 1. Cài đặt

```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-matplotlib python3-numpy
cd ~/ackermann_humble_ws/Thanh
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
```

HMI chạy từ **Python hệ thống** `/usr/bin/python3` để tương thích `rclpy` của ROS 2 Humble, không nên chạy qua Conda.

**Terminal 1 — world:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash run_world_v104.sh
```

**Terminal 2 — robot có LiDAR:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash spawn_force_v104.sh
```

**Terminal 3 — mở HMI:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash hmi_v104.sh
```

Nếu `hmi_v104.sh` báo thiếu thư viện, cài lại gói Python bằng `apt`, kiểm tra `/usr/bin/python3 -c "import PyQt5, matplotlib, rclpy"`. HMI cần môi trường desktop Ubuntu có DISPLAY; dùng SSH không có X11 sẽ không mở được cửa sổ.

## 2. Chức năng và nguyên tắc điều khiển

Cửa sổ HMI có khu vực điều khiển bên trái, bảng trạng thái ROS 2 bên phải, bên dưới là 4 tab.

**Điều khiển:**

- **Chỉ theo dõi:** HMI **không tạo publisher** `/ackermann_vehicle/force_steer`; có thể mở để ghi log khi controller khác chạy trong terminal.
- **Điều khiển thủ công:** HMI tạo đúng 1 publisher `std_msgs/Float64MultiArray`, phát `[F_N,delta_rad]` tại 10 Hz. Hai ô số nhập `F` từ −300 tới +300 N và góc `delta` từ −29,8° tới +29,8°; giao diện tự đổi sang radian trước khi gửi. Khi odom/scan cũ hơn 1,2 s, HMI đặt lệnh lực 0.
- **Chạy tự hành A* + LiDAR:** HMI giải phóng publisher thủ công, kiểm tra không có controller khác, sau đó mở `auto_drive_v104.sh` trong tiến trình riêng. Nhận log tự hành hiển thị ngay trong tab Nhật ký.
- **DỪNG LỆNH:** Dừng tiến trình auto (nếu có), trả F=0, delta=0 và phát các lệnh zero ngắn hạn. **Đây KHÔNG phải phanh cơ khí, không giữ xe trên dốc.**
- Không chạy `manual_drive_v104.sh` hoặc `auto_drive_v104.sh` trong terminal đồng thời với nút thủ công/tự hành trên HMI. Ngoại lệ: HMI ở chế độ **Chỉ theo dõi**, cho phép một controller khác phát lực.

**Giám sát:** vị trí cầu sau `xP, yP`; cao độ mô hình; v; yaw/pitch/roll; lực phát động yêu cầu; góc lái tham chiếu/ước lượng từ plugin; mômen các bánh; tốc độ góc RL/RR/FL/FR; khoảng cách LiDAR trước/sau; độ dốc theo hướng xe từ heightmap; sai số bám quỹ đạo A*; sai số tới đích.

**Các tab:**
1. Đồ thị thời gian thực: `v(t)`, `F_cmd(t)`, `delta_actual(t)`, `omega_wheel(t)`.
2. Quỹ đạo XY + LiDAR: vệt chạy, tuyến A*, bốn trụ cố định, điểm vật cản mới LiDAR, đích và tường bao.
3. Đối chiếu MATLAB: 6 đồ thị `xP,yP,v,yaw,delta,tau_total` vẽ chồng MATLAB/Gazebo, kèm RMSE ở phần dưới.
4. Nhật ký tự hành: các log `[AUTO]`, `[LIDAR]`, `[LIDAR HOLD]`, `[LIDAR REPLAN]` từ tiến trình điều khiển.

Đồ thị được cập nhật định kỳ để giảm tải GUI, trong khi CSV giữ dữ liệu đã ghi với thời gian mô phỏng Gazebo. Tối đa 100.000 mẫu gần nhất để giới hạn bộ nhớ.

## 3. Xuất dữ liệu và biểu đồ từ Gazebo

Chương trình tự ghi từ khi nhận odometry. Nút `Xóa dữ liệu / Ghi mới` tạo phiên ghi mới; `Tạm dừng ghi` ngừng thêm mẫu nhưng không tắt ROS hoặc các lệnh điều khiển.

- **Xuất CSV Gazebo:** file có các cột `t_s`, `sim_time_s`, `x_rear_m`, `y_rear_m`, `z_body_m`, `yaw_deg`, `pitch_deg`, `roll_deg`, `v_m_s`, `F_cmd_N`, `delta_ref_deg`, `delta_actual_deg`, `omega_*_rad_s`, `tau_*_Nm`, `tau_total_Nm`, `road_grade_deg`, `route_error_m`, các phép đo LiDAR.
- **Xuất biểu đồ PNG:** lưu `gazebo_time_history.png` và `gazebo_trajectory.png`. Nếu đã nạp CSV MATLAB, thêm `matlab_gazebo_comparison.png`.
- **Xuất CSV so sánh:** nội suy MATLAB theo timestamp Gazebo, tạo các cột `Gazebo_*`, `MATLAB_*`, `error_*` để dùng trong MATLAB, Excel hoặc báo cáo Word.

Không suy diễn dữ liệu chưa có: khi `/force_state` mất, các cột lực và mômen ghi NaN thay vì 0.

## 4. Xuất bảng CSV từ MATLAB/Simulink hiện có

Trong repo đã có mô hình `ackermann-simulink-v1.0.2`. MATLAB lưu các log:
`StateLog=[xP,yP,theta,v,psi]`, `CommandLog` (mômen điều khiển) và `RearForceLog` (lực tiếp xúc bánh–đường).

Đã thêm script MATLAB mới:
`ackermann-lidar-autonav-v1.0.4/matlab/export_ackermann_hmi_csv.m`.

Khi đã có `out` là kết quả từ `sim(...,'ReturnWorkspaceOutputs','on')`:

```matlab
addpath('duong_dan/Thanh/ackermann-lidar-autonav-v1.0.4/matlab');
T = export_ackermann_hmi_csv(out, 'matlab_reference.csv');
```

Sau đó chọn nút **Nạp CSV MATLAB** trên HMI để hiển thị biểu đồ chồng.

Nếu dùng chương trình MATLAB `.m` tự lập, bạn có thể tự xuất `writetable(...)` với tối thiểu các cột:
`t_s, x_rear_m, y_rear_m, v_m_s, yaw_deg, delta_actual_deg, tau_total_Nm`.
HMI cũng nhận một số tên thay thế như `t, xP, yP, v, theta` (radian), `psi` (radian), `tau`.

### Cách tính sai số

Cùng một trục thời gian `t_s=0` bắt đầu ở mẫu đầu của mỗi mô phỏng. Nội suy tuyến tính MATLAB vào các thời điểm đo Gazebo; không ngoại suy ra ngoài thời gian tham chiếu:

```text
RMSE(q) = sqrt( sum((q_Gazebo(t_i) - q_MATLAB(t_i))^2) / N )
```

Riêng góc yaw lấy sai số vòng tròn (wrap ±180°), tránh nhầm -179° và +179° cách nhau 358°. **Không tự bù điểm xuất phát**; muốn so sánh định lượng phải đặt vị trí và hướng xuất phát tương đương.

### Cảnh báo kỹ thuật về MATLAB và Gazebo

- Mô hình Simulink cũ có `maxTorque=7.5 Nm` cho cầu sau; Gazebo v1.0.4 đang điều khiển **AWD** với tổng mômen tối đa 45 Nm. Chạy nguyên thông số sẽ chỉ phù hợp so sánh định tính.
- Gazebo `F_cmd_N` là **lực phát động quy đổi từ mômen ra lệnh**, `tau_total_Nm` là mômen cấp bánh theo plugin. **KHÔNG phải lực tiếp xúc thật**. MATLAB `RearForceLog(:,1:2)` là lực dọc bánh–đường; không so sánh trực tiếp với `F_cmd_N` hay `tau_total_Nm`.
- `delta_actual_deg` Gazebo là trung bình góc của hai khớp lái từ plugin, trong khi `psi` MATLAB là góc lái tương đương của mô hình; tại góc lái lớn có thể khác nhau.
- Hai mô hình phải dùng cùng khối lượng, bán kính bánh, khối lượng quán tính, ma sát, địa hình, thuật toán điều khiển và trạng thái đầu nếu muốn RMSE có ý nghĩa vật lý.
- Trong tình huống robot đổi hướng/đi lùi, `v_m_s` là vận tốc dọc có dấu.

## 5. Kiểm tra nhanh trên Ubuntu

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
python3 -m compileall -q tools
python3 -m unittest discover -s tests -v
bash -n hmi_v104.sh
```

Nên mở HMI khi Gazebo và robot đã khởi động. Nếu nhấn Auto mà báo `/scan` không có, kiểm tra `scan_v104.sh` trước. Nếu HMI báo trùng publisher, đóng những chương trình manual/auto ở terminal khác rồi mới giành quyền điều khiển.

**Mức xác minh:** các bài kiểm tra offline chạy trên GitHub Actions. Tương tác PyQt5 với ROS 2 Humble, Gazebo, cảm biến LiDAR và các tác động lệnh cần kiểm tra thực tế trên máy Ubuntu.
