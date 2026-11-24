# Quy trình kiểm tra tích hợp ROS 2 – Gazebo – LiDAR – HMI – MATLAB v1.0.4

> Dự án nghiên cứu trong Gazebo Classic 11 / ROS 2 Humble. Chưa được xác minh
> bằng thử nghiệm trực tiếp trên máy Ubuntu của người dùng. Không dùng làm hệ
> thống tránh va chạm an toàn cho robot thật.

## 0. Chuẩn bị

Giữ nguyên Gazebo world và robot đang chạy thành công. Chỉ cần cập nhật mã HMI,
không cài lại ROS, không biên dịch lại plugin C++ nếu nguồn v1.0.3 không đổi.

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
bash -n auto_drive_v104.sh hmi_v104.sh scan_v104.sh
```

Nếu `git pull --ff-only` từ chối vì có thay đổi cục bộ, kiểm tra và lưu các
thay đổi đó trước, không dùng `git reset --hard`.

## 1. Cảm biến LiDAR

```bash
bash scan_v104.sh
# Terminal ROS đã source:
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Mong đợi `sensor_msgs/LaserScan`, `frame_id=lidar_link`, 91 tia,
tần số thiết kế 5 Hz, các giá trị khoảng cách hữu hạn tới vật cản trong 8 m.
Có topic /scan nhưng không có bản tin không được tính là thử nghiệm đạt.

## 2. Bảng điều khiển thủ công mới

```bash
bash hmi_v104.sh
```

Bấm **Điều khiển thủ công**. Khi ODOM và /scan còn mới, nhấn giữ các phím:
W hoặc ↑ tiến, S hoặc ↓ lùi, A hoặc ← sang trái, D hoặc → sang phải.
Kết hợp W+A hoặc W+D để vừa tiến vừa lái. Nhả phím phải trả lệnh tương ứng
về 0 trong chu kỳ phát lệnh 0,1 s. **VỀ 0/SPACE** chỉ gửi lệnh 0,
**không hủy quyền Manual**; **DỪNG LỆNH** trả lệnh về 0 và thoát Manual.

Điều chỉnh hai thanh trượt **Độ mạnh** / **Độ lái**; mặc định 80 N / 15°.
Các thanh trượt chỉnh biên độ của nút nhấn giữ, không tự làm xe chạy.
Đối với thí nghiệm, nhập F và delta rồi bấm **GIỮ LỆNH SỐ** để giữ giá trị
cho đến khi chọn **VỀ 0**, đổi chế độ hoặc HMI mất kết nối.
Mất focus cửa sổ sẽ đưa lệnh về 0. Khi ODOM hoặc LiDAR quá hạn 1,2 s,
nút cũ không được tự tiếp tục chạy khi dữ liệu quay lại.

**Lưu ý vật lý:** F=0 là ngừng mô-men phát động, KHÔNG phải lệnh phanh
đỗ, không thể đảm bảo xe đứng yên trên dốc. Hãy kiểm tra vùng xung quanh
robot trong Gazebo trước khi cấp lực lớn.

## 3. Tự hành và vật cản mới

Chọn **Chỉ theo dõi** nếu khởi động tự hành từ terminal. Không được có hai
controller phát đồng thời tới `/ackermann_vehicle/force_steer`.

Trước khi chạy thật:
```bash
bash auto_drive_v104.sh --dry-run
```

Sau khi quan sát `/scan` và `[LIDAR A*]`, nhấn Ctrl+C.
Nếu muốn theo dõi bằng HMI, mở HMI và chọn **Chạy tự hành A* + LiDAR**.
Không chạy `auto_drive_v104.sh` ở terminal đồng thời.

Khi robot đi theo tuyến:
```bash
BOX_X=8.2 BOX_Y=-2.2 bash spawn_test_box_v104.sh
```

Vị trí thử nghiệm này cần nằm trên hành lang đường đi thực tế. Nếu
đường A* không đi qua đây, đặt một hộp mới bằng Gazebo Insert **lên trước
đường robot** nhưng không chồng lên robot. Quan sát:

1. /scan có điểm đo vật mới.
2. Điểm vật cản mới xuất hiện trên tab Quỹ đạo XY + LiDAR.
3. Controller log `[LIDAR HOLD]` khi khoảng cách không an toàn.
4. Nếu đủ khả năng tạo đường vòng: `[LIDAR REPLAN]` xuất hiện và đường
   A* mới được phát lên `/ackermann_vehicle/planned_path`.
5. Xe chỉ tiến tiếp khi kiểm tra khoảng cách và bộ theo dõi đều cho phép.
   Trong không gian hẹp, xe có thể giữ phanh mà không vòng tránh được.

Nếu bước 4 không xuất hiện, kiểm tra `lidar_dynamic_radius_m`, giới hạn
đường và các điểm LiDAR trên bản đồ. Không tăng lực để ép xe xuyên vật cản.

## 4. Đối chiếu MATLAB – Gazebo

Để RMSE có ý nghĩa vật lý, cần thống nhất: điểm xuất phát và yaw,
m(t), độ dốc, bán kính bánh, chiều dài cơ sở, lực kéo/mô-men theo thời gian,
dẫn động AWD/RWD, giả thiết lốp–đường và tín hiệu góc lái. Gazebo v1.0.4
hiện là AWD tối đa 300 N/45 Nm; MATLAB Simulink cũ dùng cầu sau với mô-men
giới hạn khác, **chưa phù hợp để khẳng định định lượng**.

Gợi ý thí nghiệm dễ đối chiếu: trước tiên chọn đoạn địa hình bằng,
đặt robot và MATLAB cùng pose, cho F=0, delta=0 trong 2 s;
cho F=+100 N, delta=0 trong 5 s; tiếp đó F=0 trong 3 s.
Chỉ khi hai bộ mô phỏng được đặt cùng đầu vào và điều kiện vận hành mới
so sánh sai số x, y, v, yaw, δ, mô-men và lực quy đổi.

Từ HMI, bấm **Xóa dữ liệu / Ghi mới** khi bắt đầu kịch bản,
**Xuất CSV Gazebo** sau khi kết thúc. MATLAB:
```matlab
addpath('duong_dan/Thanh/ackermann-lidar-autonav-v1.0.4/matlab');
T = export_ackermann_hmi_csv(out, 'matlab_reference.csv');
metrics = compare_matlab_gazebo_v104('gazebo_run.csv', ...
                                    'matlab_reference.csv', 'ket_qua_so_sanh');
```

Nếu không sử dụng Simulink, MATLAB có thể dùng `writetable` tạo CSV với
các cột `t_s,x_rear_m,y_rear_m,v_m_s,yaw_deg,delta_actual_deg,tau_total_Nm,F_cmd_N`.
Bấm **Nạp CSV MATLAB** trong HMI để xem 7 đồ thị chồng và cảnh báo
khác biệt điều kiện. Kết quả MATLAB độc lập lưu `rmse_v104.csv` và
`matlab_gazebo_v104.png`.

## 5. Ghi nhận kết quả

Báo cáo tối thiểu: tần số /scan, 3 ảnh HMI (lệnh Manual, LiDAR với hộp mới,
đồ thị chồng), đoạn log `[LIDAR HOLD]`/`[LIDAR REPLAN]`, hai CSV
Gazebo/MATLAB và bảng RMSE. Những dữ liệu này là bằng chứng kiểm thử;
có mã nguồn hoặc có topic chưa đủ để kết luận mô hình hoạt động đúng.
