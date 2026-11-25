# Gazebo v1.0.4: 2 bánh sau dẫn động + 2 bánh trước đánh lái (RWD Ackermann)

## 1. Sơ đồ cơ khí đúng với yêu cầu

- Hai bánh sau **rear_left_wheel** và **rear_right_wheel** nhận mô-men
  từ bộ điều khiển lực, **không đánh lái**.
- Hai bánh trước **front_left_wheel** và **front_right_wheel** là bánh
  **lăn tự do**, mô-men **dẫn động = 0**. Bánh trước vẫn quay do tiếp xúc
  mặt đường; chúng **không bị khóa cứng**.
- Hai khớp **front_left_steer_joint** và **front_right_steer_joint**
  được điều khiển góc qua PD và hình học Ackermann. Đây là **hai khớp
  đánh lái khác với bốn khớp quay bánh**.
- Giữ nguyên tổng khối lượng ~20 kg, bán kính bánh 0,15 m,
  chiều dài cơ sở 0,8 m, khoảng cách hai bánh cùng cầu 0,5 m,
  mô hình Gazebo Classic 11 và nguồn cảm nhận 4 m **không dùng LiDAR**.

Các topic A*, DWA, TEB, HMI và đầu vào lực–lái vẫn như cũ.

## 2. Phân bố lực và mô-men

ROS 2 vẫn phát hai tín hiệu:

```text
/ackermann_vehicle/force_steer
std_msgs/Float64MultiArray: [F_command_N, delta_command_rad]
```

Trong `ackermann_force_drive.cpp`, plugin C++ đã hỗ trợ cả hai
cấu hình `drive_mode=all` / `drive_mode=rear` từ trước. Phiên bản
**v1.0.4 hiện chọn rear** trong cả
`matlab_vehicle_params.json` và `mission.json`:

```text
total_torque = F_command_N * R
torque_RL = total_torque / 2
torque_RR = total_torque / 2
torque_FL = 0
torque_FR = 0
```

Góc lái hai bên được tính riêng:

```text
delta_left  = atan( tan(delta_ref) / (1 - track_width/(2L)*tan(delta_ref)) )
delta_right = atan( tan(delta_ref) / (1 + track_width/(2L)*tan(delta_ref)) )
```

Ví dụ `F=80 N`, `R=0,15 m`: mô-men kéo RL = RR = **6 Nm**;
FL = FR = **0 Nm**. Hai bánh trước vẫn quay lăn tự do và đánh lái.

**Giới hạn hiện tại được GIỮ NGUYÊN:** `F_max=300 N`, tương ứng
tổng mô-men 45 Nm, **22,5 Nm/bánh sau ở ngưỡng tối đa**. Đây là
mô-men động cơ đặt lên khớp bánh, không phải lực bám chắc chắn
trên địa hình. Không thay đổi hệ ma sát hay bộ điều khiển để tránh
mất khả năng phân tích nguyên nhân trượt.

**Chưa đồng nhất hoàn toàn với MATLAB:** MATLAB dùng tổng mô-men
cầu sau tối đa **22 Nm** (~146,67 N). Để làm thí nghiệm so sánh
định lượng sau này, cần chỉnh đồng bộ hai giá trị
`max_drive_force_N` thành `146.6666667` và
`enhanced_max_total_drive_torque_Nm` thành `22` trong
`matlab_vehicle_params.json`, đồng thời chỉnh
`mission.json.max_drive_force_N`. Ngoài ra cần xem lại các lực
hỗ trợ lên dốc >146,67N (ví dụ 175N) trước khi thử chế độ này.
**Không cần làm bước đồng nhất mô-men này để chuyển sang RWD.**

## 3. Cập nhật mã nguồn

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4

python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
bash -n spawn_force_v104.sh run_world_v104.sh
```

Nếu `git pull --ff-only` báo thay đổi cục bộ, lưu lại thay đổi
trước, không dùng `git reset --hard` để tránh mất mã của bạn.

### A. Nếu Gazebo và HMI đang chạy

1. Bấm **DỪNG LỆNH**; dừng các controller đang chạy trong terminal.
2. Đóng HMI, cập nhật mã GitHub, mở lại `bash hmi_v104.sh`.
3. Chọn tab **Quản lý mô phỏng** và nhấn **XÓA ROBOT**; chờ xóa xong.
4. Chọn vị trí X/Y/yaw và nhấn **THÊM ROBOT TẠI X–Y**. HMI sẽ
   chạy lại `spawn_force_v104.sh` với cấu hình rear mới.
5. Bốn vật cản, tường và địa hình vẫn giữ nguyên trong world.

**Cực kỳ quan trọng:** Thay đổi cấu hình file không tự thay đổi
plugin của **robot đã spawn**; phải xóa và spawn robot mới.
Không cần biên dịch lại C++ vì thư viện cũ đã hỗ trợ `rear`.

### B. Nếu muốn khởi động sạch

Dừng HMI, controller, Gazebo trước, sau đó chạy tại thư mục v1.0.4:

```bash
# Terminal 1
bash run_world_v104.sh
# Terminal 2 — sau khi Gazebo nạp xong
bash spawn_force_v104.sh
# Terminal 3
bash hmi_v104.sh
```

World sạch sẽ đưa lại các vật cản gốc nếu trước đó bạn đã xóa/thêm
chúng trong HMI; nếu muốn giữ kịch bản, dùng cách A.

## 4. Kiểm tra hai bánh sau kéo, hai bánh trước đánh lái

`/ackermann_vehicle/force_state` có thứ tự:

```text
[0] F_command_N
[1] steering_reference_rad
[2] steering_actual_rad
[3] omega_rear_left_rad_s
[4] omega_rear_right_rad_s
[5] body_velocity_m_s
[6] omega_front_left_rad_s
[7] omega_front_right_rad_s
[8] torque_each_REAR_wheel_Nm
[9] torque_each_FRONT_wheel_Nm
```

Trong HMI **Manual**, thử **lực nhỏ 40N trên đường phẳng và trống**;
không chạy Auto song song. Sau đó mở terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ackermann_humble_ws/install/setup.bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
python3 tools/check_rwd_v104.py --timeout 15 --min-force 5
```

Chương trình đọc trạng thái, **không phát lệnh lực**. Cần giữ lệnh
lực khác 0 trong lúc chạy công cụ để thu được bản tin. Mong đợi:

```text
PASS RWD: F=+40.00N | tau_RL=tau_RR=+3.000Nm ... |
tau_FL=tau_FR=+0.000Nm ... | steer_ref=...
```

Đồng thời có thể xem topic bằng:

```bash
ros2 topic echo /ackermann_vehicle/force_state --once
```

Kiểm tra các bánh trước **quay theo mặt đường** (omega_FL/FR có
thể khác 0) dù **mô-men dẫn động front bằng 0**.
Để kiểm tra lái, thử góc lái nhỏ 10° và quan sát `steer_ref`
và `steer_actual` thay đổi; góc lái hai bên khác nhau theo
hình học Ackermann, nhưng force_state chỉ xuất góc trung bình.

## 5. Đánh giá ảnh hưởng

- Đổi AWD → RWD làm giảm số bánh chủ động; lực kéo hữu ích trên dốc
  có thể nhỏ hơn hoặc bánh sau dễ trượt hơn vì phân bố phản lực.
- Kể cả giữ cùng 300N lệnh, xe **có thể không leo được nơi mà AWD
  đã leo được**. Đó là kết quả động lực học vật lý đáng ghi nhận,
  không nhất thiết là lỗi DWA/TEB.
- Khi so sánh MATLAB, ghi rõ **cùng cấu hình RWD nhưng chưa cùng
  giới hạn mô-men**, khối lượng/quán tính xấp xỉ và khác biệt
  mô hình tiếp xúc bánh–đường.
- Mọi thử nghiệm nên bắt đầu ở đoạn bằng phẳng để xác nhận
  hướng lực kéo, góc lái và chuyển động trước khi lên dốc.

Chỉ kết luận hệ dẫn động đúng sau khi kiểm tra runtime `force_state`
và quan sát bánh trong Gazebo. Unit tests kiểm tra file cấu hình và
mã nguồn, chưa tự chạy động lực học ODE.
