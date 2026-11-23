# Kiểm tra theo 6 cấp robot Ackermann – ROS 2 Humble / Gazebo Classic 11

## Mục tiêu

Phân biệt nguyên nhân `gzserver` chỉ chạy 0,6 giây SimTime trong nhiều phút (CPU cao) sau khi spawn robot. Cùng một world địa hình 3D v1.0.2, tăng dần thành phần robot; KHÔNG sửa trực tiếp mã nguồn XACRO hoặc package đã cài. Script gọi `xacro`, xuất URDF tạm theo cấp, spawn vào Gazebo đang chạy qua `/spawn_entity`.

| Cấp | Thành phần | Dấu hiệu cần kiểm tra |
| --- | --- | --- |
| 0 | base_footprint, chassis, fixed joint | Chassis rơi, va chạm và dừng trên đất; RTF tiếp tục > 0 |
| 1 | Cấp 0 + 2 bánh sau, khớp quay | Hai bánh tiếp xúc mặt đường; không kẹt ODE |
| 2 | Cấp 1 + hai cụm lái trước và 2 bánh trước | Đủ 4 bánh và khớp lái; chưa có plugin hoặc cảm biến |
| 3 | Cấp 2 + plugin `ackermann_drive` | Có subscriber `/ackermann_vehicle/cmd_vel`; thử điều khiển tốc độ nhỏ |
| 4 | Cấp 3 + LiDAR | LiDAR xuất bản topic; RTF không bị tụt quá sâu |
| 5 | Cấp 4 + camera độ sâu | Camera hoạt động; RTF được đo và so với cấp 4 |

## Cách chạy mỗi cấp

**Terminal 1:** Khởi động world, không spawn robot cũ. Đóng phiên Gazebo trước đó nếu còn.

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-ros2-gazebo-terrain-v1.0.2
bash run_world_clean.sh
```

**Terminal 2:** Trong lúc Gazebo đang chạy và nút Play đã bật:

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-ros2-gazebo-terrain-v1.0.2
bash spawn_robot_stage.sh 0
```

Tại **Terminal 3**, theo dõi:

```bash
ps -C gzserver -o pid,stat,%cpu,%mem,etime,args
timeout 10s gz stats
```

Kiểm tra 10–20 giây. Nếu `SimTime` tăng và RTF không về gần 0, cấp đó đạt. **Sau đó đóng Gazebo, mở lại world sạch, spawn cấp 1**, lặp đến cấp 5. Không spawn các cấp liên tiếp vào cùng một phiên Gazebo, vì các robot cũ vẫn tồn tại và kết quả sẽ sai.

## Khi cấp nào lỗi

- **Cấp 0 lỗi:** kiểm tra va chạm chassis/terrain, quán tính thân xe, tham số vật lý.
- **Cấp 1 lỗi:** bánh sau/khớp quay, va chạm và ma sát bánh với heightmap.
- **Cấp 2 lỗi:** hình học khớp lái, joint axes và bánh trước.
- **Cấp 3 lỗi:** plugin Gazebo Ackermann Drive hoặc tương tác plugin – các khớp.
- **Cấp 4 lỗi:** cảm biến LiDAR hoặc cấu hình tần số dữ liệu của nó.
- **Cấp 5 lỗi:** camera chiều sâu / GPU / tốc độ cập nhật cảm biến.

Nếu lỗi xuất hiện ở cấp 1 hoặc 2, thử **sau khi khởi động lại Gazebo**:

```bash
ACKERMANN_SAFE_FRICTION=1 bash spawn_robot_stage.sh 2
```

Tùy chọn trên hạ `mu1`, `mu2` của các bánh xuống 1.0 và loại bỏ `fdir1` của bánh **chỉ trong URDF tạm**. Đây là thử nghiệm cô lập để đánh giá cấu hình tiếp xúc cực lớn (`mu1=mu2=1000`) của mô hình gốc; chưa phải một hiệu chỉnh vật lý cuối cùng.

Ở cấp 3, thử tiến nhẹ:

```bash
ros2 topic pub /ackermann_vehicle/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}' -r 5
```

Nhấn Ctrl+C rồi gửi lệnh dừng bằng `ros2 topic pub --once /ackermann_vehicle/cmd_vel geometry_msgs/msg/Twist '{}'`.

## Lưu ý

- `spawn_robot_stage.sh` đặt root `base_footprint` ở `(x=0, y=-2, z=0.10)` m, gần mặt đường. Nếu thay đổi hình học xe, có thể dùng `ACKERMANN_SPAWN_Z=0.15`.
- Model stage 0 chỉ có chassis; hình dáng và khoảng cách đến mặt đất khác xe đầy đủ là điều bình thường.
- Nếu Gazebo đang Pause, bật ▶ Play. Kiểm tra `gz stats` trả `Paused[F]` và `SimTime` tăng.
- Nếu Gazebo kẹt, không nhất thiết `delete_entity` sẽ phản hồi; đóng rồi khởi động lại để thử cấp kế tiếp.
- Chỉ tiến hành test này trên môi trường mô phỏng, không gửi lệnh cho robot thật.
- Script chưa được chạy thực tế trong ROS 2 Humble ở môi trường tạo mã; đã kiểm tra cú pháp và cấu trúc URDF bằng bộ kiểm tra độc lập.