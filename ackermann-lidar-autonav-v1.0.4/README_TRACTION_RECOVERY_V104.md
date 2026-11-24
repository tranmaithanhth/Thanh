# V1.0.4 — Khắc phục xe đứng trên dốc dù DWA có quỹ đạo an toàn

## Chẩn đoán từ log người dùng 24/11/2026

- Xe chạy từ P=(1.60,0.83) đến P≈(2.34,1.75), yaw≈45°.
- Bộ DWA liên tục ghi 54/54 hoặc 63/63 đường hợp lệ; sai lệch
  tuyến rất nhỏ (~0.05 m); đây KHÔNG phải lỗi `NO_SAFE_DWA`.
- Từ khoảng 18:50:35 tới 18:50:45, P gần như cố định,
  nhưng `odom.twist.linear.x` dao động 0.12–0.18 m/s.
- Bộ điều khiển liên tục gửi `F≈+15 N`, delta≈−13.6° đến −15°;
  bộ tự hành kết thúc do `STALL: <0.20m progress over 12s`.
- Dốc phía trước ở vị trí này ~0.04, **dưới cả**
  `recovery_min_front_grade=0.06` (lệnh phục hồi v1.0.3) và
  `hill_start_min_grade=0.10` (lực khởi động lên dốc).
  Chưa thể kết luận bánh trượt hay vướng va chạm chỉ bằng log vận tốc.

## Bản sửa

1. `tools/traction_recovery_v104.py` đo **dịch chuyển X–Y thực**
   trong cửa sổ 1.6 s, không dùng odom tốc độ làm bằng chứng tiến.
2. Khi DWA/TEB vẫn cần chuyển động, xe chưa tới đích và dịch chuyển
   X–Y <0.045 m trong cửa sổ, bộ điều khiển chờ thêm 0.35s,
   rồi yêu cầu lực bổ sung từ 65 N, tăng 22 N/s, tối đa 145 N
   (không thay đổi max 300 N của plugin). Những giá trị này là
   giới hạn THỬ NGHIỆM, không có nghĩa lực bám thực tăng tương ứng.
3. Trước khi sử dụng lực bổ sung, `Mission.rollout_safe` kiểm tra
   chuyển động ngắn hạn; chính DWA dùng lực tăng trong mô phỏng
   quỹ đạo 2.5 s, tiếp tục dùng `safe_motion` và chướng ngại vật
   đã biết. Không bỏ vùng tránh va chạm hay ép tăng lực vô hạn.
4. Đọc tốc độ quay RL/RR/FL/FR từ plugin `force_state`.
   Nếu bề mặt bánh có vẻ quay >0.65 m/s nhưng dịch chuyển thật
   <0.035 m/s, báo `slip_possible=True`, giảm trần lực hỗ trợ
   xuống 75 N để hạn chế quay trượt. Đây chỉ là nhận biết sơ bộ.
5. Khi xe tiến thực trở lại, ngừng kích hoạt hỗ trợ.
   Nếu xe vẫn đứng và đạt ngưỡng STALL 12 s, chương trình vẫn
   dừng với chẩn đoán thêm: `XY_progress_speed`, `odom`,
   `wheel_surface` và `last_traction_floor`.
   **Không vô hiệu hóa STALL/điều kiện an toàn.**

## Chạy thử sau khi cập nhật GitHub

Không cần khởi động lại world hay spawn robot; chỉ cần dừng
controller cũ và đóng HMI, cập nhật Python rồi mở HMI mới:

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
bash hmi_v104.sh
```

Đặt cùng vị trí xuất phát `P=(1.60,0.83), yaw≈53°`, chọn DWA,
nhấn tự hành. Quan sát sau khi đến gần `(2.34,1.75)`:

```text
[LOCAL] requested=DWA effective=DWA DWA_valid=.../... 
[XY TRACTION RECOVERY] F_floor=65.0N measured_XY_speed=... odom_v=... wheel_surface=... slip_possible=False
[AUTO] P=(...) F=+65.0N ...
```

Nếu XY bắt đầu tăng, lực hỗ trợ sẽ được giải phóng và
DWA trở về phản hồi lực thông thường. Nếu `slip_possible=True`
và xe không tiến: kiểm tra contact/ma sát và lực tác dụng trên
`/ackermann_vehicle/force_state` (không chỉ tăng lực).

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /ackermann_vehicle/force_state --once
ros2 topic echo /ackermann_vehicle/odom --once
```

`force_state` là mảng, với 4 vận tốc góc bánh ở chỉ số 3,4,6,7;
F lệnh ở chỉ số 0; góc lái tham chiếu ở chỉ số 1.
Tại F=15 N và R=0.15m với AWD, mô-men mỗi bánh chỉ khoảng
0.5625 Nm — có thể không thắng được tiếp xúc cục bộ trong Gazebo.
F=65 N tương ứng khoảng 2.44 Nm/bánh, **không phải lực bám đo được**.

Nếu sau bản vá xe vẫn không vượt qua cùng vị trí, gửi:
- Log `[XY TRACTION RECOVERY]` và các dòng `[AUTO]`.
- `force_state` 10 phần tử tại lúc xe đứng.
- Ảnh HMI và kết quả `ros2 topic echo .../odom --once`.

Nếu wheel_surface thấp mà X–Y đứng, tiếp tục kiểm tra cấu hình
motor-force, collision bánh/sườn dốc, chiều cao thân và ma sát.
Nếu wheel_surface cao nhưng X–Y đứng, ưu tiên xử lý trượt bánh.
Trường hợp robot nằm chồng lên collision khác cần sửa vị trí đặt
hoặc hình học, không thể chữa bằng tăng lực.

## Giới hạn kết luận

Các bài unit test/CI chỉ xác minh cơ chế chọn và giới hạn lực,
không thay thế thử nghiệm Gazebo với ma sát thực tế.
Không chỉnh lại khối lượng, bán kính bánh, lực trần plugin, địa hình,
vùng quan sát 4 m, hay A* chỉ vì log này.
