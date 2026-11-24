# HMI v1.0.4 — Quản lý robot và vật cản trong Gazebo

Đã bổ sung tab **Quản lý mô phỏng** cho Gazebo Classic 11 + ROS 2 Humble,
không thay đổi động lực học robot hay phép so sánh MATLAB.

## 1. Các thao tác trong HMI

| Hành động | Thao tác |
|---|---|
| Thêm vật cản | Chọn hình trụ/hộp, nhập X–Y (m), bán kính hoặc nửa cạnh (m), chiều cao (m), nhấn **THÊM VẬT CẢN**. |
| Xóa vật cản | Chọn tên vật cản từ danh sách **ĐANG TỒN TẠI trong Gazebo**, nhấn **Xóa vật cản được chọn** và xác nhận. |
| Xóa toàn bộ vật cản | Nhấn **XÓA TẤT CẢ VẬT CẢN** và xác nhận. **Không xóa** terrain, tường, robot hay marker đích. |
| Xóa robot | Bấm **DỪNG LỆNH** nếu đang điều khiển, đợi controller nhả quyền, rồi **XÓA ROBOT**. |
| Thêm/đổi vị trí robot | Sau khi xóa robot cũ, nhập X–Y–yaw (độ) và nhấn **THÊM ROBOT TẠI X–Y**. |
| Chọn bằng bản đồ | Ở tab Quản lý mô phỏng chọn **Vật cản** hoặc **Robot** trong ô “Nhấp bản đồ để đặt vị trí cho”. Chuyển sang **Quỹ đạo XY + vật cản gần**, nhấp chuột trái vào vị trí cần đặt, trở lại Quản lý mô phỏng và bấm Thêm. |

**Đặc biệt:** Đồ thị XY CHỈ hiển thị vật cản trong vùng quan sát 4 m
theo topic `/ackermann_vehicle/nearby_obstacles`, nên robot vẫn **không
biết trước vị trí vật cản**. Danh sách toàn bộ vật cản ở tab Quản lý
mô phỏng dành riêng cho *người vận hành* để chỉnh kịch bản, không phải
dữ liệu đầu vào của A*.

## 2. Robot: một robot tại một thời điểm

Robot sử dụng tên `ackermann_force_nearby_v104` và các ROS topic cố định
`/ackermann_vehicle/...`. Phiên bản này **chỉ hỗ trợ một robot hoạt động
tại một thời điểm**: để đổi vị trí phải xóa robot cũ rồi spawn lại.

Không xóa hoặc spawn robot khi HMI đang **MANUAL/AUTO**, có controller
ngoài phát lệnh `/ackermann_vehicle/force_steer`, hoặc HMI còn giữ
publisher phát xung dừng 0. Kịch bản xóa/spawn robot bắt buộc qua
dịch vụ Gazebo và tiến trình `spawn_force_v104.sh`. Lệnh spawn tái sử
dụng URDF stage 3, khối lượng 20 kg, dẫn động AWD và lực tối đa 300 N.
Cao độ Z được tính theo địa hình MATLAB; góc yaw dùng đơn vị **độ**
trong HMI, đổi sang **radian** khi gửi đến script.

Script spawn đã được sửa để kiểm tra **publisher odom** thay vì sự có
mặt của topic. HMI có subscriber odom nên ngay cả khi robot đã bị xóa,
topic vẫn có thể xuất hiện; điều này không có nghĩa robot chưa xóa.

## 3. Vật cản có collision thật

- Hình trụ: `radius`, `height`, pose `[x,y, terrain_z+height/2]`.
- Hộp: `[2*half_size,2*half_size,height]`, pose tại mặt địa hình.
- Cả hai tạo ra **collision + visual + static model** bằng `SpawnEntity`,
  không phải một dấu tròn trang trí trên HMI.
- Tên tự sinh `hmi_obstacle_001_r092` (ví dụ): `r092` biểu thị bán kính
  bao ngoài **0,92 m** được suy ra từ kích thước vật cản (hộp dùng
  `sqrt(2)*nửa_cạnh`). Bộ cảm biến Gazebo chỉ giải mã bán kính này
  **sau khi model thực sự đi vào vùng quan sát**, không biết trước XY.
- Bốn hình trụ gốc `obstacle_01...04` cũng có thể xóa.
- **Xóa vật cản khi robot tự hành:** được phép để thử khả năng phản ứng
  với thay đổi môi trường; bộ nhớ cảm biến xóa model ngay khi Gazebo
  xác nhận model không còn tồn tại.
- **Thêm vật cản khi robot tự hành:** được phép nhưng HMI từ chối
  đặt vật cản trùng/áp sát robot. Không đặt vật cản trong hành lang
  đang tăng tốc nếu chưa kiểm chứng quãng đường phanh.
- `ModelStates` chỉ cung cấp tâm và vị trí model, các model không có
  hậu tố `_rNNN` sẽ sử dụng bán kính mặc định ~0,75 m. Nếu thêm model
  rất lớn bằng cách khác, phải kiểm tra riêng bán kính xấp xỉ trong planner.

## 4. Cập nhật và sử dụng

Khi Gazebo và robot đang chạy thành công với `/gazebo/model_states`,
**chỉ cần đóng HMI cũ, cập nhật GitHub và mở lại HMI**. Không bắt buộc
restart world hay spawn lại robot vì tính năng mới chủ yếu là UI và
dịch vụ Gazebo (mã `nearby_scene.py` được bộ tự hành nạp khi Auto chạy lại).

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
bash hmi_v104.sh
```

Nếu world chưa chạy, thứ tự: `bash run_world_v104.sh` →
`bash spawn_force_v104.sh` → `bash hmi_v104.sh`.

Nếu các nút không thể hoạt động, kiểm tra dịch vụ và mô hình:

```bash
source /opt/ros/humble/setup.bash
ros2 service list | grep -E 'spawn_entity|delete_entity'
ros2 topic list | grep -E '/gazebo/model_states|/ackermann_vehicle/odom'
```

Đã bổ sung `tests/test_scene_editor.py` cho hình học vật cản, lựa chọn
XY, giới hạn thao tác, ẩn vị trí khi ngoài vùng quan sát, giải mã bán
kính và loại vật cản đã bị xóa. Test offline không thay thế kiểm tra
thực tế PyQt5/Gazebo.

**Lưu ý:** Thay đổi kịch bản qua dịch vụ Gazebo có hiệu lực trong
phiên mô phỏng đang chạy, không ghi đè vào file world gốc. Khởi động
lại world sẽ khôi phục bốn vật cản mặc định. Hãy xuất log/CSV trước khi
xóa robot nếu muốn giữ dữ liệu thí nghiệm.
