# Ackermann ROS2 v1.0.4 — sửa DWA kẹt sau khi TEB đã chạy được

## Tái hiện từ nhật ký 24/11/2026 lúc 19:05–19:06

- **TEB** đã chạy được trong Gazebo, cần giữ nguyên.
- **DWA** đến `P=(9.06,4.25)`, yaw≈25°, còn ≈7.84 m tới đích.
- `[LOCAL] requested=DWA effective=DWA DWA_valid=54/54`:
  phần tìm quỹ đạo **vẫn tìm được phương án an toàn**.
- Lực kéo hầu hết **F=15 N**. Vận tốc DWA được chọn nhảy xuống
  `vref=0.04,0.05,0.14` rồi lại `0.23 m/s`.
- `STOP: STALL ... XY_progress_speed=nan ... last_traction_floor=0.0N`.
  Tại thời điểm dừng, `wheel_surface=0.124 m/s`,
  `odom=0.129 m/s`; chưa có chứng cứ đủ để xác nhận trượt bánh lớn.
- `mission.json` đặt ngưỡng giám sát `traction_min_vref_m_s=0.15`.
  **Lỗi phần mềm:** watchdog dùng `mission.last_reference_v`
  là vận tốc DWA đã chọn, và RESET toàn bộ cửa sổ 1.6 giây
  mỗi lần một mẫu <0.15 m/s xuất hiện. Vì vậy, dù robot đứng hàng
  chục giây, hỗ trợ mô-men đã lập trình vẫn không được phép bật.

## Sửa đã triển khai

1. Bộ giám sát chỉ dùng **ý định thực hiện tuyến A***:
   khi Auto DWA đang có đường, còn xa đích, chưa bị giữ bởi
   collision/nearby guards, duy trì `intent_vref = ±0.22 m/s`
   cho bộ theo dõi dịch chuyển X–Y, bất kể DWA nhất thời chọn
   `0.04` hay `0.23 m/s`.
   Lệnh thực phát cho động cơ vẫn là kết quả DWA;
   `intent_vref` không phải lệnh vận tốc áp thẳng lên robot.
2. Sau 1.6 giây không có dịch chuyển X–Y thực (>0.045m)
   và 0.35 giây xác nhận, cơ chế phục hồi yêu cầu lực từ 65 N,
   tăng 22 N/s, tối đa 145 N. Không thay đổi Fmax=300 N.
3. Bộ `LocalPlanningMission._force()` cho phép áp dụng lực
   này với vận tốc DWA dương nhỏ từ 0.035 m/s (trước là 0.14).
   Cả lệnh đầu ra và mô phỏng dự báo 2.5 s dùng **cùng lực**.
4. DWA có phạt chi phí mềm đối với tốc độ gần 0 **chỉ khi
   watchdog đã kích hoạt** để ưu tiên phương án đi tiếp.
   Mọi ứng viên vẫn bị kiểm tra `safe_motion()` và bị loại
   nếu va chạm hoặc vi phạm dốc. Khi không có quỹ đạo tiến
   an toàn, DWA vẫn có thể chọn dừng/phanh.
5. Chỉ thay đổi nhánh lực phục hồi cho **DWA**;
   chế độ TEB đã chạy được không bị thay đổi cơ chế đo.
6. Log `[XY TRACTION RECOVERY]` xuất thêm
   `intent_vref` và `chosen_vref` để phân biệt rõ 2 tín hiệu.

## Chạy và kiểm tra

Tắt tiến trình điều khiển tự hành và đóng HMI cũ; Gazebo và URDF
hiện tại không cần khởi động lại.

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
bash hmi_v104.sh
```

Trong HMI, chọn **DWA**; khởi động từ cùng vị trí đã dùng cho TEB,
cùng vật cản và yaw xuất phát.

Khi tới vùng trước đây bị kẹt, quan sát các dòng:
```text
[LOCAL] requested=DWA effective=DWA DWA_valid=...
[XY TRACTION RECOVERY] F_floor=65.0N ...
intent_vref=+0.22 chosen_vref=+0.04 ...
[AUTO] P=(...) ... F=+65.0N ...
```

Nếu phục hồi có hiệu lực, X–Y phải tăng theo đường đi và sau khi
thoát kẹt, lực bổ sung trở về 0. Nếu lực tăng nhưng không có tiến
thực, cần đọc `/ackermann_vehicle/force_state`, kiểm tra bánh tiếp
xúc địa hình, wheel torque và ma sát, không tăng lực vô điều kiện.

**Giới hạn:** kiểm thử GitHub Actions là các phép thử offline,
không thể khẳng định xe đến đích cho tới khi thử trong Gazebo.
