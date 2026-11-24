# Ackermann v1.0.4 — ROS 2 Humble / Gazebo Classic 11

> **CẬP NHẬT 24/11/2026: CẤU HÌNH MẶC ĐỊNH ĐÃ BỎ LiDAR.** Toàn bộ
> hướng dẫn `/scan`/LiDAR bên dưới là tài liệu lịch sử cho bản cũ.
> Hệ thống mới dùng dữ liệu mô hình Gazebo **chỉ trong bán kính 4 m**
> và không biết trước tọa độ vật cản. Hãy dùng tài liệu chính:
> **[README_NEARBY_V104.md](README_NEARBY_V104.md)**.
> Cần dừng rồi **khởi động lại Gazebo và spawn robot mới**, không chỉ mở
> lại HMI; đường dẫn lệnh vẫn là `run_world_v104.sh`,
> `spawn_force_v104.sh`, `auto_drive_v104.sh`, `hmi_v104.sh`.

## Tài liệu cho chế độ LiDAR cũ (không còn là chế độ mặc định)



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

### Kiểm tra nhầm sườn dốc thành vật cản (bản vá 24/11/2026)

Nếu HMI hiển thị **hàng chấm màu cam gần như thẳng đứng tại X≈4,9 m**
khi robot ở gần điểm xuất phát (0,-2), đây có thể là điểm LaserScan
va vào sườn dốc nhưng bị phép chiếu độ cao phân loại nhầm thành vật cản mới.

Bản vá trong `tools/lidar_obstacles.py` loại các điểm có độ cao suy ra từ
LaserScan **tại hoặc thấp hơn độ cao địa hình dự đoán cộng sai số 0,12 m**.
Điểm cao hơn ngưỡng vẫn đi qua bộ lọc vật cản động và phanh. Không sửa
`/scan` gốc, không bỏ LiDAR, không thay đổi động lực học, plugin C++,
hay tham số khối lượng/ma sát trong MATLAB.

HMI bổ sung **LiDAR: đất / dưới đất / vật mới**; log điều khiển tự hành
trong mỗi vài giây sẽ có `ground=...`, `below_map=...`, `dynamic=...`.
Đường chấm cam do mặt dốc phải biến mất trên tab XY sau khi HMI/controller
đã khởi động lại với file Python mới. Khi thêm một hộp đủ cao so với mặt
dốc vào đường đi, `dynamic` vẫn phải tăng và LiDAR phải phanh/lập đường.

Sau `git pull`, **dừng rồi mở lại HMI và tiến trình auto** để nạp bộ lọc
mới; không cần tắt Gazebo hoặc spawn robot lại nếu chỉ sửa Python. Chú ý:
ngay cả sau khi loại điểm sai, xe vẫn có thể đứng yên vì A* rollout,
ma sát/tiếp xúc hoặc góc khởi đầu. Đối chiếu thêm `/force_steer`,
`/force_state`, `/odom` và log `[AUTO]` trước khi kết luận.

**Giới hạn:** Đây là lọc theo bản đồ địa hình đã biết; vật thể thấp dưới
`lidar_ground_tolerance_m`, bản đồ địa hình sai và vật cản nhỏ có thể
không được nhận diện. Chưa đủ an toàn để sử dụng ngoài mô phỏng.

### Bản sửa tiếp theo: điểm LiDAR màu cam sát robot tại X≈2,8; Y≈−2

Ảnh HMI sau bản vá đầu tiên cho thấy một điểm cam nằm gần như **sát
thân/bánh sau** khi robot đi tới vùng X≈3 m. Đây không đủ dữ liệu để
khẳng định là dốc; rất có thể là tín hiệu LiDAR phản xạ từ **chính
chassis hoặc bánh xe**. Các bản vá đầu tiên chỉ xử lý độ cao địa hình,
chưa loại phản xạ nằm trong hình chiếu thân xe.

Bộ lọc mới trong `tools/lidar_obstacles.py` gồm hai nhánh:
- **Loại tự phản xạ:** đưa mỗi điểm LiDAR về tọa độ tương đối của thân
  xe (dùng hướng yaw và vị trí thân từ odometry); loại điểm nằm bên
  trong footprint gần `x∈[-0.64,0.67]` m, `|y|≤0.43` m. Vùng này
  không phải bán kính tránh va chạm 1,2 m của A*; **vật thật ở trước
  cản/ngoài footprint vẫn phải phát hiện và phanh**.
- **Đối chiếu ray–địa hình:** đối với điểm không bị loại nhưng nằm
  **chỉ hơi cao hơn địa hình dự đoán**, kiểm tra giao điểm của toàn
  bộ tia 3D với mặt độ cao MATLAB; nếu khoảng cách giao nằm cách số
  đo LiDAR không quá 0,20 m thì phân loại là mặt dốc. Tia quét có
  điểm nằm cao hẳn so với địa hình không được loại bởi nhánh này.

`mission.json` có các tham số `lidar_self_x_back_m`,
`lidar_self_x_front_m`, `lidar_self_half_width_m`,
`lidar_ground_raycast_tolerance_m` và
`lidar_ground_raycast_height_gate_m`. Không tăng vùng loại self tùy
tiện: vùng quá lớn có thể làm mất vật cản thật.

HMI hiển thị `LiDAR: đất / tự thân / vật mới`; log tự hành có
`ground=...`, `ray_ground=...`, `self=...`, `dynamic=...`.
Chỉ tính `nearest(front/rear)` sau khi bỏ điểm địa hình và tự
phản xạ, tránh phanh giả.

**Hướng dẫn áp dụng:** dừng tự hành và thoát HMI, `git pull`, chạy
`python3 -m unittest discover -s tests -v`, rồi mở lại HMI và bộ
điều khiển. Không cần spawn lại robot vì các thay đổi chỉ liên quan
đến Python và JSON. Nếu xe vẫn đứng, gửi log `[LIDAR HOLD]`,
`[LIDAR]`, `[AUTO]` và `/ackermann_vehicle/force_state` để
phân biệt phanh giả với lỗi lực kéo/khả năng bám địa hình.

**Hạn chế:** Giải pháp dùng bản đồ địa hình có sẵn và một vùng mask
vật lý ước tính; không phải mô hình tự nhận biết chướng ngại 3D độc
lập, không đảm bảo phát hiện vật thể thấp hay sát thân. Chỉ dùng
trong mô phỏng, không dùng làm cảm biến an toàn cho robot thực tế.

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

## 8. HMI điều khiển, giám sát và đối chiếu MATLAB

Đã bổ sung HMI desktop PyQt5 (không sửa plugin động lực học):

```bash
sudo apt install -y python3-pyqt5 python3-matplotlib python3-numpy
# Terminal 1: bash run_world_v104.sh
# Terminal 2: bash spawn_force_v104.sh
# Terminal 3, tại thư mục ackermann-lidar-autonav-v1.0.4:
bash hmi_v104.sh
```

Trên HMI có điều khiển thủ công F (N), delta (độ), bật tự hành A*+LiDAR, chế độ theo dõi không phát lệnh, nút trả lệnh về 0, đồ thị v/F/góc lái/tốc độ bánh, bản đồ XY và LiDAR, xuất CSV/PNG, nạp CSV MATLAB và tính RMSE.

**Hướng dẫn chi tiết:** [README_HMI_V104.md](README_HMI_V104.md). Script chuyển log MATLAB Simulink: [matlab/export_ackermann_hmi_csv.m](matlab/export_ackermann_hmi_csv.m).
