# ĐỊA HÌNH MATLAB TERRAIN 3D → ROS 2 HUMBLE / GAZEBO CLASSIC 11

Gói dữ liệu địa hình 3D để thử mô hình robot Ackermann bốn bánh đang có trong `ackermann_humble_ws`. Hình học mô phỏng được tạo theo công thức của `terrain3d_height.m` v1.0.4: đoạn lên dốc ở X=2,5–6,5 m, đoạn trên cao, xuống dốc ở X=10,6–14,6 m, đỉnh khoảng 1,05 m, có gợn sóng/nghiêng ngang và bốn vật cản.

**Lưu ý:** Repository này cung cấp địa hình và script khởi chạy Gazebo, **không thay thế package URDF, bộ điều khiển hay launch spawn robot hiện tại**. Gazebo Classic 11 chạy trên Ubuntu 22.04 / ROS 2 Humble; không cần cài ROS 2 Jazzy.

## Chạy world

Mở terminal mới, tránh trộn môi trường ROS Noetic và Humble:

```bash
git clone https://github.com/tranmaithanhth/Thanh.git
cd Thanh/ackermann-ros2-gazebo-terrain-v1.0.1
chmod +x run_world.sh
./run_world.sh
```

Hoặc thủ công:

```bash
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH="$(pwd)/models:${GAZEBO_MODEL_PATH:-}"
ros2 launch gazebo_ros gazebo.launch.py world:="$(pwd)/worlds/ackermann_matlab_terrain.world" verbose:=true
```

Khi world hiện lên, sử dụng phần spawn robot Ackermann từ launch cũ. Không mở Gazebo lần thứ hai hoặc spawn cùng robot hai lần. Nếu đã có `/robot_description`, một ví dụ spawn riêng:

```bash
source /opt/ros/humble/setup.bash
source ~/ackermann_humble_ws/install/setup.bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity ackermann_car -x 0 -y 0 -z 0.55
```

`-z 0.55` chỉ là độ cao khởi tạo tham khảo, cần điều chỉnh theo `base_link`, bán kính bánh và hình học URDF.

## Cấu trúc

```text
ackermann-ros2-gazebo-terrain-v1.0.1/
├── README_VI.md
├── terrain_params.json
├── export_report.json
├── run_world.sh
├── tools/generate_terrain.py
├── worlds/ackermann_matlab_terrain.world
└── models/ackermann_matlab_terrain/
    ├── model.config
    ├── model.sdf
    └── terrain_height.png
```

## Thay đổi địa hình

Sửa `terrain_params.json`: `terrainHeight`, `terrainRiseX`, `terrainFallX`, `terrainCamber`, `terrainRipple`, `obstacles`.

Tạo lại heightmap và world:

```bash
python3 -m pip install --user numpy pillow
python3 tools/generate_terrain.py
```

PNG 257×257 và SDF đã được lưu sẵn nên **không cần cài numpy/Pillow để chỉ chạy world mặc định**. Ảnh được lượng tử hóa từ hàm độ cao. Sai số lượng tử tại nút lưới xấp xỉ dưới 2,3 mm; sai số nội suy và thuật toán tiếp xúc Gazebo có thể lớn hơn.

## Các phép thử

1. Mở riêng world, kiểm tra có dốc, không xuất hiện mặt phẳng `ground_plane` chồng lên heightmap.
2. Spawn robot với khớp bánh có collision, quán tính, ma sát và chuyển động khớp thực.
3. Lái chậm (0,2–0,4 m/s), theo dõi `base_link.z`, pitch và roll khi lên dốc và xuống dốc.
4. Kiểm tra lực kéo, lực bám và giới hạn mômen; robot không leo dốc nếu động cơ hoặc tiếp xúc không đủ.
5. Sau khi vật lý vận hành ổn mới tích hợp DWA/TEB hoặc Nav2. Heightmap **không tự động** khiến planner tránh độ dốc quá lớn: cần cập nhật slope cost hoặc bộ lập kế hoạch tương ứng.

## Giới hạn

Mã và các tệp đã được kiểm tra cấu trúc và số học khi tạo gói; **chưa thử trực tiếp trong Gazebo Classic 11 trên máy người dùng**. Giá trị `mu=1.2` trong SDF là tham số giả định. Mô hình Gazebo có thể có độ trượt, khác giả thiết lăn không trượt tuyệt đối trong MATLAB.

Đây là bản terrain độc lập, không yêu cầu sửa hay xóa những file của dự án Ackermann MATLAB v1.0.4.
