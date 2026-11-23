# Gazebo 3D Ackermann v1.0.2 — Sửa lỗi vị trí địa hình

Dành cho **Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11**. Đây là gói **world/heightmap**, không chứa robot. Khi world chạy đúng, spawn robot từ project ROS 2 hiện có.

## Lỗi v1.0.1

Mô hình cũ có `<pose>8.5 1.0 -0.060... 0 0 0</pose>` trong heightmap model, nhưng các vật cản được đặt ở tọa độ thế giới MATLAB (X 4,1; 7,3; 10,15; 12,4 m) và điểm đích (16;0,6). Trên ảnh Gazebo, bề mặt lại nằm quanh gốc thế giới nên vật cản thứ tư và điểm đích có vẻ nằm ngoài biên. Một số phiên bản Gazebo Classic có sai lệch visual/collision khi dịch chuyển heightmap.

## Điểm đã sửa

- Lấy mẫu trực tiếp hàm `terrain3d_height.m` trên lưới **vuông 40 × 40 m**, với tâm tại **gốc thế giới (0,0)**; PNG 257×257 (bạn có thể tăng lên 513×513 trong terrain_params.json).
- Cả SDF **model pose** và **heightmap pos** đều là `0 0 0`; tránh dịch chuyển hình học heightmap.
- Vật cản và điểm đích **giữ tọa độ XY giống MATLAB**, không sửa dữ liệu A*/DWA/TEB.
- Địa hình lên dốc x=2,5–6,5 m; xuống dốc x=10,6–14,6 m; đỉnh ~1,05 m; gợn sóng và nghiêng ngang nhẹ.
- Do ảnh xám 8-bit không biểu diễn được z âm, toàn bộ Gazebo có **độ nâng Z chung ~+0,063 m** so với MATLAB. Tất cả vật cản và mốc đầu/đích được nâng đúng bằng độ lệch này; **độ dốc không thay đổi**.
- Mốc bắt đầu xanh lam và đích xanh lục (chỉ visual, không cản robot).
- Mô hình và world có tên mới `ackermann_matlab_terrain_v102` để không sử dụng nhầm cache của v1.0.1.
- Script chạy không bật `set -u`, tránh lỗi `AMENT_TRACE_SETUP_FILES: unbound variable`.

## Chạy bản sửa từ GitHub

Mở Terminal mới và đóng Gazebo cũ trước khi chạy:

```bash
cd ~/ackermann_humble_ws/Thanh
git pull --ff-only origin main
cd ackermann-ros2-gazebo-terrain-v1.0.2
chmod +x run_world.sh
./run_world.sh
```

Gazebo dự kiến hiển thị đủ **4 vật cản trên địa hình**, cùng mốc đầu và đích **bên trong sa bàn**. World này **không spawn robot**: hãy sử dụng lệnh spawn của dự án Ackermann ROS 2 hiện có, không mở thêm Gazebo.

## Khi muốn sửa bản đồ

Sửa `terrain_params.json` rồi chạy:

```bash
sudo apt install -y python3-numpy python3-pil
python3 tools/generate_terrain.py
python3 tools/check_alignment.py
./run_world.sh
```

Vị trí vật cản được tự cập nhật cả XY và Z. Nếu bạn thay vị trí mục tiêu, phải cập nhật mục tiêu Nav2/ROS 2 riêng; world chỉ lưu vật thể trực quan ở vị trí mục tiêu.

## Kiểm tra robot khi lên dốc

Spawn robot gần (x=0,y=0) với độ cao ban đầu phụ thuộc bán kính bánh và chiều cao `base_link`, thường cao hơn đỉnh địa hình thêm một khoảng. Điều khiển chậm trước để xác nhận bánh chạm mặt đất. Kiểm tra `odom` hoặc trạng thái Gazebo về z, pitch, roll và mômen bánh sau. Lưu ý: **world 3D không tự bổ sung nhận biết độ dốc cho Nav2 / DWA / TEB**.

## Giới hạn kiểm thử

Đã kiểm tra Python, cấu trúc XML, tọa độ SDF, ảnh PNG, vị trí các vật cản và cao độ trên lưới. **Chưa chạy Gazebo Classic 11 trực tiếp ở môi trường tạo bản sửa**; cần chạy trong Ubuntu của bạn để xác nhận phần hiển thị và phần tiếp xúc trùng nhau. Nếu vẫn lệch sau khi chạy bản mới, chụp Gazebo và gửi `gz model -l` cùng phần log ROS/Gazebo.
## Gazebo không mở / lỗi môi trường ROS

**Mới bổ sung 23/11/2026:** Có thể chạy world trong môi trường sạch (không lẫn ROS Noetic, Python/Qt của workspace khác) mà không phải cài lại Ubuntu hoặc ROS:

```bash
cd ~/ackermann_humble_ws/Thanh
git pull --ff-only origin main
cd ackermann-ros2-gazebo-terrain-v1.0.2
bash tools/diagnose_gazebo.sh
bash run_world_clean.sh
```

Nếu vẫn không mở cửa sổ, hãy phân tách server/GUI:

```bash
# Chạy chỉ server địa hình (không GUI, không plugin ROS):
bash run_world_clean.sh --server
# Khi đã xác nhận server hoạt động, Ctrl+C rồi chạy GUI bằng Mesa:
bash run_world_clean.sh --software
```

Để lưu thông báo lỗi:

```bash
bash run_world_clean.sh 2>&1 | tee ~/ackermann_gazebo_error.log
```

Không tự xóa thư mục `~/.gazebo` hoặc `~/.ros`. Nếu `gzserver/gzclient` cũ còn chạy, đóng chúng trước khi mở world mới. Việc các file XML/PNG hợp lệ không bảo đảm plugin, GUI hoặc GPU đang hoạt động; dùng log để xác định chính xác.

## Spawn robot Ackermann vào world đang chạy

Đã bổ sung `spawn_robot_on_terrain.sh`, `drive_robot_on_terrain.sh` và `tools/drive_slope_demo.py`. **Không chạy launch Gazebo của robot gốc**, vì nó sẽ mở world khác. Giữ world v1.0.2 mở, rồi chạy các lệnh sau tại thư mục này trong terminal mới:

```bash
bash spawn_robot_on_terrain.sh
bash drive_robot_on_terrain.sh
```

Robot bắt đầu ở `(x=0,y=-2,z=0.10,yaw=0)` và chạy chậm dọc +X để thử lên/xuống dốc, tránh các vật cản trụ. Xem thêm [README_ROBOT.md](README_ROBOT.md). Đây là demo di chuyển thẳng; **chưa phải DWA/TEB tự động đến đích**.

**Nếu robot trông như đang lơ lửng:** nhìn góc dưới trái Gazebo. Khi có nút tam giác `▶ Play` và **Real Time Factor = 0**, mô phỏng đang Pause: nhấn Play để bật trọng lực và tiếp xúc. Spawn hiện đặt `z=0.10` m (base_footprint) để bánh gần mặt đất ở `(0,-2)`; có thể đổi bằng `ACKERMANN_SPAWN_Z=0.15 bash spawn_robot_on_terrain.sh` trước khi spawn. Robot đã spawn trước đó sẽ không tự đổi cao độ: nếu muốn thả lại, phải xóa entity cũ hoặc khởi động lại world và spawn mới.

## Khoanh vùng nghẽn gzserver bằng 6 cấp robot

Nếu world chạy với RTF≈1 nhưng spawn robot làm RTF≈0, dùng [`README_STAGES_VI.md`](README_STAGES_VI.md) và `spawn_robot_stage.sh`. Hai file mới tách robot thành 6 cấp: thân xe → bánh sau → bánh trước/khớp lái → Ackermann Drive → LiDAR → camera. Ví dụ thử cấp 0: `bash spawn_robot_stage.sh 0`. **Khởi động lại Gazebo sạch trước mỗi cấp**, không gọi launch Gazebo từ package robot gốc.
