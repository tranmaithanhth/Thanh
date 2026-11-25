# Ackermann v1.0.4 — A* toàn cục + DWA / TEB rút gọn cục bộ

## 1. Kết quả kiểm tra mã nguồn trước khi sửa

Bản v1.0.4 đã có A* toàn cục và `Mission.local_force_steer()` của
v1.0.3. Hàm này lấy mẫu **[F,delta]** để tối ưu rollout ngắn và kiểm tra
va chạm, **không phải** phương pháp DWA theo cửa sổ [v,delta] hay
TEB với các nút và thời khoảng.

Đã đối chiếu cách xây dựng bộ lập đường trong tài liệu MATLAB của dự án:
`Huong_dan_chi_tiet_code_Ackermann_Terrain3D_MATLAB_v1.0.4.docx`
(chương 7; hướng dẫn cho 23 file .m).
- MATLAB DWA: 6 vận tốc + 0, 9 góc lái, cửa sổ biến thiên theo
  gia tốc/steer-rate mỗi chu kỳ, rollout ~2.5s, kiểm tra địa hình /
  chướng ngại / bám tuyến / tốc độ / lực/mô-men.
- MATLAB TEB rút gọn: 7 điểm và 6 `dt` dương; tối ưu vị trí và
  `log(dt)` bằng `fminsearch` trong MATLAB, phạt lệch tuyến,
  độ cong, vận tốc, gia tốc, địa hình, vật cản; kiểm tra quỹ đạo rồi
  chuyển về [v_ref,delta_ref], **fallback DWA** nếu không khả thi.
- KHÔNG khẳng định tương đương đầy đủ với ROS `teb_local_planner`.

ROS 2 được bổ sung `tools/local_planners_v104.py` với:
- **DWA:** tập vận tốc có thể đạt tới trong chu kỳ 0,1 s, 6 mức + 0,
  9 mức góc lái; mô phỏng tiếp 2,5 s với bước 0,2 s trên mô hình
  Ackermann có lực kéo/ma sát nhớt/trọng lực trên dốc.
- **TEB rút gọn:** 7 điểm quỹ đạo, 6 thời khoảng, tối ưu qua một
  phiên bản Nelder-Mead giới hạn số lượt đánh giá (không cần SciPy).
  Bản ROS không phải một bản sao nhị phân của file MATLAB hay thuật
  toán TEB chính thức. Sử dụng A* làm đường tham chiếu và các hình
  tròn vật cản chỉ **sau khi được cảm biến 4 m phát hiện**.
  Sau tối ưu kiểm tra đường và rollout động lực học; nếu không an toàn,
  sử dụng DWA thay thế.
- **ROLLOUT:** giữ nguyên bộ lấy mẫu lực và góc lái từng giúp xe
  tới đích để đối chứng và chẩn đoán, không bị xóa.

## 2. Sơ đồ ROS 2

```
     Gazebo /gazebo/model_states
                  |
       [NearbyScene, r <= 4 m]
                  |
       [Object detection / memory]
                  |
   [A* /planned_path]  ----- GLOBAL
                  |
   [LOCAL planner DWA | TEB]
                  |
   [/ackermann_vehicle/local_path]
                  |
    [v_ref,delta_ref] -> F motor using slope and mass/rolling forces
                  |
     /ackermann_vehicle/force_steer [F_N,delta_rad]
                  |
             Gazebo vehicle
```

A* vẫn được cập nhật khi tuyến gặp vật cản đã quan sát. Hai thuật toán
cục bộ chỉ tạo đoạn đường dự đoán phù hợp Ackermann trong khu vực quanh
xe; cả hai dùng `safe_motion` để loại quỹ đạo cắt vật cản được phát
hiện hoặc vi phạm độ dốc. Hệ thống vẫn không dùng LiDAR.

**Cách đổi lệnh:**
- v_ref (m/s): đầu ra bộ planner.
- δ_ref (rad): đầu ra bộ planner, giới hạn bởi `max_steer_rad`
  và tốc độ thay đổi lái.
- F (N) = sat[ m_eq * sat(1.4(v_ref-v), ±a_max)
  + c_roll * v + m*g*grade ], sau đó tăng lực hỗ trợ khi lên dốc
  theo `hill_start_force_floor` và chặn ở ±300 N.
- Mô hình dùng `Mission.step` để dự đoán lực và chuyển động
  `yaw_dot=v*tan(delta)/wheelbase`. Lưu ý lực là lệnh mô-men
  động cơ quy đổi, không phải lực bám thực đã đo.

## 3. Chọn thuật toán trên HMI

Ở phía trên khu vực điều khiển có ô **Lập quỹ đạo cục bộ**:
1. `DWA — cửa sổ động` (mặc định).
2. `TEB — dải đàn hồi thời gian`.
3. `Rollout cũ — đối chứng`.

Chọn chế độ **khi robot chưa chạy Auto**, rồi bấm
**Tự hành A* + cảm biến vùng**. Khi đang chạy không thể chuyển nóng
để tránh thay đổi bộ điều khiển bất ngờ. Muốn so sánh, nhấn
**DỪNG LỆNH**, chọn mode khác, đặt lại vị trí xuất phát ở tab
**Quản lý mô phỏng**, chạy lại.

- A*: đường **nét đứt xanh lá**.
- DWA/TEB: đường **liền tím có các nút** là quỹ đạo cục bộ.
- Gazebo: đường **xanh lam** là quỹ đạo đã đi.
- Vật cản cam: chỉ các vật có trong vùng cảm nhận 4m.

ROS có thêm topic `/ackermann_vehicle/local_path` (`nav_msgs/Path`).
A* toàn cục vẫn ở `/ackermann_vehicle/planned_path`.
Sau khi planner phanh vì một vật cản quá gần, local_path được
xóa để không hiển thị đường cũ là tuyến có thể chạy.

## 4. Chạy từ Terminal (khi HMI chỉ theo dõi)

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
```

Giữ Gazebo v1.0.4, robot và world hiện tại. Chỉ cần tắt HMI/controller
cũ, rồi mở HMI mới. Nếu dùng Terminal điều khiển tự hành, chỉ chạy
một controller tại một thời điểm:

```bash
bash auto_drive_v104.sh --local-planner DWA
# hoặc
bash auto_drive_v104.sh --local-planner TEB
# hoặc đối chứng:
bash auto_drive_v104.sh --local-planner ROLLOUT

# Kiểm tra không cấp lực:
bash auto_drive_v104.sh --local-planner DWA --dry-run
```

Trong Terminal đã source ROS Humble:
```bash
ros2 topic echo /ackermann_vehicle/local_path --once
ros2 topic echo /ackermann_vehicle/planned_path --once
ros2 topic echo /ackermann_vehicle/force_steer --once
```

## Khắc phục sự cố NO SAFE LOCAL ROLLOUT (24/11/2026)

Người dùng báo xe chạy tới P=(1,88; -0,31), yaw=-23°, v~0,30m/s
rồi dừng với `NO SAFE LOCAL ROLLOUT` khi vừa nhìn thấy trụ tại
(4,1;0,3). Trước khi dừng log `vref=0.75m/s`, lực thay đổi từ -3,5
đến 23,1N, góc lái tối đa -28,1°. Không nên tăng Fmax=300 N hoặc
độ lệch lái max=0,52 rad chỉ dựa trên log này.

**Chẩn đoán kỹ thuật:** ở 10Hz, DWA có giới hạn tăng vận tốc ~0,135m/s
mỗi chu kỳ, nên với vận tốc thực 0,16m/s không thể sinh
vref=0,75m/s trực tiếp. Các dòng đó cho thấy rất có thể
chế độ `ROLLOUT` đang chạy hoặc bộ DWA/TEB đã nhảy sang fallback
`local_force_steer` v1.0.3. Fallback cũ chỉ đánh giá ~1,44s,
trong khi DWA xét 2,5s, do đó có thể đánh giá lạc quan về quãng
đường tới vật cản và đưa xe vào thế không thể xoay đủ an toàn.

**Đã sửa:**
1. DWA và TEB khi không có ứng viên dự báo an toàn sẽ **không gọi
   bộ rollout cũ**, trừ khi người vận hành chọn rõ `ROLLOUT`.
   Thay vào đó báo `NO_SAFE_DWA`, phanh và chờ A* lập lại đường.
2. Trước đây base node kết thúc ngay khi nhận lệnh cục bộ rỗng.
   Bộ Nearby mới có **recovery giới hạn 12 giây**: phanh theo vận tốc,
   chỉ lập lại A* khi vận tốc <0,12m/s, không nhảy qua vùng collision.
   Sau 12s không khắc phục sẽ kết thúc với lý do rõ ràng.
3. Khi có vật cản mới và sai lệch A*>0,95m, đánh giá lập lại đường
   sớm hơn ngưỡng cũ 2m, bảo đảm kiểm tra khoảng cách thực.
4. TEB không dùng lại lực kéo từ tư thế cũ khi cached band vẫn hợp lệ;
   lực được tính lại từ vận tốc và độ dốc hiện tại.
5. Thêm topic `/ackermann_vehicle/local_planner_status` kiểu
   `std_msgs/String` có JSON `requested`, `effective`,
   `dwa_valid`, `dwa_tested`, `fallback`, `no_safe`.
   HMI hiển thị **Bộ cục bộ thực tế / ứng viên an toàn**.
   Log có `[LOCAL] requested=... effective=... DWA_valid=.../...`.
6. Thêm bài kiểm thử tái hiện P=(.63,-.15), v=.16 và
   P=(1.88,-.31), yaw=-23°, v=.30 với trụ gần; bảo đảm bộ DWA
   có phương án và không giả làm vref=.75 ở đầu hành trình.

**Thử đúng cách:**

```bash
cd ~/ackermann_humble_ws/Thanh
git status --short
git pull --ff-only origin main
cd ackermann-lidar-autonav-v1.0.4
python3 -m compileall -q tools tests
python3 -m unittest discover -s tests -v
bash hmi_v104.sh
```

Chọn **DWA** trước khi bấm Tự hành; theo dõi ngay trong HMI
`DWA → DWA [xx/yy]`. Các trạng thái chẩn đoán:
- `DWA → DWA`: chính DWA thực thi, không fallback.
- `TEB → DWA_FALLBACK`: TEB tìm quỹ đạo không an toàn, DWA thay thế.
- `DWA/TEB → NO_SAFE_DWA`: không có ứng viên an toàn, lệnh phanh/
  tìm tuyến lại, không chạy mù theo fallback cũ.
- `ROLLOUT → LEGACY_ROLLOUT`: người vận hành chủ động chọn chế độ cũ.

Kiểm tra topic nếu cần:
```bash
ros2 topic echo /ackermann_vehicle/local_planner_status --once
ros2 topic echo /ackermann_vehicle/force_steer --once
```

Nếu vẫn bị phanh lâu và kết thúc ở cùng tọa độ, lưu 10–20 dòng
`[LOCAL]`, `[LOCAL RECOVERY]`, `[NEARBY DEVIATION REPLAN]`, cùng
ảnh A* và quỹ đạo tím. Điều đó cho biết nhánh nào đã từ chối ứng viên.
**Không tăng góc lái tối đa** nếu chưa đối chiếu hành trình bánh và
hình học trục quay, vì max steer hiện tại ±0,52rad (±29,8°)
tương ứng bán kính tối thiểu `0,8/tan(0,52)≈1,40m` trong mô hình
xe đạp.

Hệ thống đã qua kiểm thử offline/GitHub Actions; hành vi trên Gazebo
cụ thể của người dùng vẫn cần chạy lại để xác nhận đến đích.

## 5. Các bài thử so sánh MATLAB — Gazebo

Làm 3 lần mô phỏng có cùng (x,y,yaw), cùng vật cản, thông số
địa hình, bước ghi và thời gian đo, lần lượt với DWA / TEB /
ROLLOUT. Lưu:
- thời gian đến đích;
- quãng đường di chuyển thực tế và chiều dài A*;
- RMSE độ lệch quỹ đạo;
- lực kéo, góc lái, tốc độ, số lần phanh, số lần lập lại A*;
- thời gian tính toán (cần đo thêm khi profiling).

DWA và TEB trong MATLAB cũ dùng **dẫn động cầu sau với lệnh mômen**,
ROS v1.0.4 hiện dùng **RWD (hai bánh sau dẫn động) với lực kéo tối đa 300 N**. Chưa thể kết luận
định lượng rằng hai nền tảng có động lực học giống nhau nếu không hiệu
chỉnh các điều kiện đó.

## 6. Cảnh báo và hạn chế

- Bản TEB này là **TEB rút gọn / TEB-inspired**, dùng Nelder-Mead
  giới hạn lần đánh giá, không phải tối ưu sparse least squares đầy
  đủ của ROS `teb_local_planner`. Có fallback DWA khi phương án TEB
  không khả thi.
- Tối ưu là không lồi; đạt cost hữu hạn không chứng minh tối ưu toàn cục.
- `ModelStates` là thông tin mô phỏng; không phải cảm biến an toàn.
- Bản đồ A* vẫn chỉ biết vật đã được phát hiện. Vật chưa phát hiện
  không xuất hiện trong cost và không được tránh trước.
- Phanh bằng lệnh lực ngược chiều chuyển động không phải phanh đỗ;
  nếu có dốc hoặc mô hình tiếp xúc xấu, robot vẫn có thể trượt.
- Chỉ kết luận thử nghiệm thành công khi kiểm chứng robot chạy trong
  Gazebo, không dựa trên các kiểm thử offline riêng lẻ.
