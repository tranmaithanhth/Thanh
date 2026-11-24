# Ackermann v1.0.4 – Cảm nhận vật cản vùng lân cận không dùng LiDAR

## Mục tiêu và nguyên tắc

- **Không sử dụng /scan, không tạo lidar_link, không có ray sensor trong robot mới.**
- Gazebo Classic 11 cung cấp trạng thái thực các mô hình qua
  `/gazebo/model_states` (`gazebo_msgs/msg/ModelStates`).
- Node tự hành chỉ **đưa vào A*** các đối tượng có mép nằm **trong 4 m quanh
  điểm cầu sau P** và những đối tượng đã nhìn thấy trước đó (nhớ 60 s).
- **Bản đồ A* khởi đầu không biết bất kỳ tọa độ vật cản nào**.
  `mission.json` đặt `"obstacles_xy_radius": []`.
- Bốn trụ và các vật thể sinh thêm vẫn tồn tại và va chạm trong Gazebo, nhưng
  chúng không được sao chép sẵn vào bộ lập đường.
- Phần biết trước: độ cao địa hình MATLAB, giới hạn vùng hoạt động, điểm đích,
  khối lượng và thông số robot, **không phải tọa độ các trụ vật cản**.
- Bộ cảm biến vùng lân cận ảo đọc **trạng thái thực của thế giới mô phỏng**
  (ground truth), không phải cảm biến có thể lắp trực tiếp lên robot thật.
  Đây là cách kiểm thử thuật toán lập đường, KHÔNG kiểm chứng cảm nhận thật.

Phát hiện vật cản dựa trên **tâm mô hình** và bán kính bao tròn ước lượng
0,75 m cho mỗi model; vì `ModelStates` không mô tả kích thước collision,
mô hình mới quá lớn hoặc có hình dạng dài cần kiểm tra/sửa giá trị này trước
khi kết luận an toàn. Không biết hình dạng chính xác, không có occlusion.

## Bộ lập quỹ đạo cục bộ DWA và TEB

Bản cập nhật ROS 2 đã bổ sung **DWA** và **TEB rút gọn** bên cạnh
bộ rollout lực–lái cũ, cùng chung tuyến A* toàn cục và cảm biến vùng
4 m của Gazebo. Trong HMI, chọn thuật toán cục bộ bằng ô
**Lập quỹ đạo cục bộ** trước khi bấm Tự hành. HMI hiển thị quỹ đạo
cục bộ màu tím từ topic `/ackermann_vehicle/local_path` cùng đường
A* toàn cục. Lực và góc lái vẫn được cấp qua `/force_steer`.

**Giải thích thuật toán, giới hạn, lệnh chạy và đối chiếu MATLAB:**
[README_LOCAL_PLANNERS_V104.md](README_LOCAL_PLANNERS_V104.md).

## Quản lý mô phỏng từ HMI

Từ bản cập nhật mới, HMI có thêm tab **Quản lý mô phỏng** để xóa/spawn
robot ở X–Y–yaw tự chọn, và tạo/xóa hình trụ hoặc hình hộp có collision
thật tại bất kỳ vị trí hợp lệ trên terrain. Có thể nhấp trực tiếp lên
bản đồ XY để điền tọa độ. Bốn vật cản gốc có thể xóa, hoặc xóa tất cả
vật cản mà không xóa terrain/tường/đích. Robot giới hạn **một robot
đang chạy**, phải dừng controller trước khi xóa hoặc tạo lại.

**Hướng dẫn thao tác, giới hạn và kiểm thử:**
[README_SCENE_EDITOR_V104.md](README_SCENE_EDITOR_V104.md).

## Thay đổi các file chính

- `tools/prepare_proximity_world.py`: thêm world plugin
  `libgazebo_ros_state.so` trong bản sao world tạm thời; không sửa terrain
  v1.0.2 hay wall/collision gốc.
- `run_world_v104.sh`: khởi động world đã bổ sung publisher model states.
- `spawn_force_v104.sh`: giữ động lực học AWD, bỏ LiDAR trong URDF.
- `tools/nearby_scene.py`: cảm biến vùng lân cận và nhớ vật đã thấy.
- `tools/autonomous_nearby_nav.py`: A* chưa biết trước vật cản;
  cập nhật bản đồ khi có đối tượng mới; kiểm tra an toàn/phanh mô phỏng.
- `auto_drive_v104.sh`: mặc định khởi động controller Nearby, **không**
  gọi `autonomous_lidar_nav.py`.
- `tools/hmi_v104.py`: tab bản đồ chỉ hiển thị các vật cản nhìn thấy
  trong 4 m, vòng tròn vùng cảm nhận, quỹ đạo, đích; không vẽ các vật đã
  khai báo trong world như thông tin biết trước.
- `scan_v104.sh`: tên script cũ được giữ tương thích; nội dung hiện giám sát
  trạng thái vật cản gần từ Gazebo, **không đọc /scan**.

## Khởi động sau khi cập nhật (BẮT BUỘC khởi động lại Gazebo)

Do world plugin và mô hình robot đã thay đổi, lần này **không thể** chỉ mở
lại HMI: cần đóng toàn bộ cửa sổ Gazebo, controller và HMI v1.0.4 cũ.
Dừng các terminal cũ bằng Ctrl+C, xác nhận không còn Gazebo cũ đang chạy.

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
bash setup_v104.sh
python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
```

**Terminal 1 – World:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash run_world_v104.sh
```

**Terminal 2 – Kiểm tra world-state plugin:**

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep /gazebo/model_states
ros2 topic hz /gazebo/model_states
ros2 topic echo /gazebo/model_states --once
```

Nếu chưa xuất hiện topic `/gazebo/model_states`, chưa chạy tự hành:
kiểm tra world plugin `/opt/ros/humble/lib/libgazebo_ros_state.so`,
Terminal Gazebo và bảo đảm KHÔNG vô tình chạy world cũ.

**Terminal 3 – Spawn robot không LiDAR:**

```bash
cd ~/ackermann_humble_ws/Thanh/ackermann-lidar-autonav-v1.0.4
bash spawn_force_v104.sh
```

Dự kiến topic `/ackermann_vehicle/odom` và `force_state` xuất hiện,
nhưng `/scan` **không cần tồn tại** (và không được controller sử dụng).

**Terminal 4 – Quan sát cảm biến 4 m (không cấp lực):**

```bash
bash scan_v104.sh
```

**Terminal 5 – HMI và tự hành:**

```bash
bash hmi_v104.sh
```

Mở tab **Quỹ đạo XY + vật cản gần**, bật
**Tự hành A* + cảm biến vùng**. KHÔNG chạy
`auto_drive_v104.sh` từ Terminal song song với Auto trên HMI.
HMI ở chế độ Chỉ theo dõi thì có thể chạy controller ngoài Terminal.

Trong Terminal khác, kiểm tra dữ liệu đầu ra riêng (khi Auto đã chạy):

```bash
ros2 topic echo /ackermann_vehicle/nearby_obstacles --once
ros2 topic hz /ackermann_vehicle/nearby_obstacles
```

Dữ liệu `std_msgs/Float64MultiArray`: `[x1,y1,r1,x2,y2,r2,...]`.
Mảng `[]` thể hiện **đã nhận được trạng thái Gazebo nhưng không có vật
cản trong tầm**, khác với topic không có bản tin (mất dữ liệu).

## Các kiểm tra cần đạt

1. Khi robot mới spawn, không xuất hiện đồng thời cả bốn trụ trên HMI;
   chỉ các trụ ở trong 4 m (tính từ mép vật tới P) mới hiện.
2. Khi xe đi vào phạm vi một vật mới, xuất hiện `[NEARBY]` và sau đó
   `[NEARBY REPLAN]` nếu tuyến A* cũ bị chặn.
3. Khoảng cách không an toàn: log `[NEARBY HOLD]`, lực kéo được trả 0
   hoặc lực hãm ngược chiều chuyển động. Hãm ngược không phải phanh đỗ.
4. Khi đến gần hộp thêm bằng `spawn_test_box_v104.sh`, hộp xuất hiện
   mà KHÔNG cần khai báo trước tọa độ của nó trong `mission.json`.
5. Xe chạy được tới mục tiêu cần được xác minh bằng quỹ đạo Gazebo, không
   chỉ bằng việc có path A*. Nếu robot mắc kẹt, kiểm tra `[AUTO]`,
   `[NEARBY HOLD]`, `/ackermann_vehicle/force_state` và odom.
6. Kiểm tra `[NEARBY]` luôn báo `radius=4.0m`, tên/đếm đối tượng chỉ
   trong vùng cảm nhận. Khi scene topic mất >1,3 s, auto không cấp lực lái.

### Đo đạc MATLAB/Gazebo

Các tab/CSV so sánh MATLAB giữ nguyên. CSV Gazebo đổi các trường đo cảm nhận
thành `nearby_front_m`, `nearby_rear_m`, `nearby_age_s`,
`nearby_count`. Động lực học (20 kg, 0,15 m bánh, 300 N AWD) không thay đổi.

**Giới hạn:** Cảm biến ảo quan sát trực tiếp pose model từ Gazebo nên
không có lỗi góc quét, không đo độ cao hay hình dạng thật, không
suy ra vật cản nhỏ/vô hình, và không thay thế LiDAR/camera trên phần cứng.
Dù phát hiện được vật cản, A* hình học và Ackermann có thể không tự
chuyển được qua hành lang hẹp vì giới hạn bán kính quay vòng.
