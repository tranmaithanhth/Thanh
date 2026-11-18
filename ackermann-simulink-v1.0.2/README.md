# Ackermann 4 bánh — MATLAB/Simulink, A*, DWA, TEB, Newton–Euler (v1.0.2)

**Đây là thư mục mã nguồn ĐẦY ĐỦ**, được đưa lên GitHub từ gói `Ackermann_GiaoTrinh_DWA_TEB_NE_v1.0.2_AStar.zip`: 27 file dự án (gồm sơ đồ PNG) và bản ZIP nguyên gốc trong `dist/`.

## Chức năng

- Robot 4 bánh Ackermann, 2 bánh trước lái, 2 bánh sau chủ động; một mômen tổng cầu sau và một lệnh lái.
- Mô hình động lực học Newton–Euler rút gọn, ràng buộc lăn không trượt, lực cản lăn nhớt theo vận tốc.
- Quy hoạch toàn cục A* theo `c.obstacles` (hình tròn), tạo đường dẫn mới trước mỗi lần chạy.
- Hai bộ lập kế hoạch cục bộ: DWA lấy mẫu và TEB *rút gọn* có fallback DWA, chọn bằng Manual Switch.
- Sa bàn động và các đồ thị lực hai bánh sau, góc lái; log dữ liệu, xuất GIF.
- Sơ đồ khối theo bố cục Hình 4.19 trong giáo trình; xem `so_do_khoi_giong_giaotrinh.png`.

## Chạy mô hình

Cần **MATLAB + Simulink**. Clone repository, vào đúng thư mục này trong MATLAB:

```bash
git clone https://github.com/tranmaithanhth/Thanh.git
cd Thanh/ackermann-simulink-v1.0.2
```

Trong MATLAB, chọn **Current Folder** đến `ackermann-simulink-v1.0.2`, rồi:

```matlab
clear functions
rehash
c = ackermann_cfg();
disp(c.route)
build_ackermann_giaotrinh
out = run_ackermann_giaotrinh('DWA');   % hoặc 'TEB'
```

Sau lần đầu, file `Ackermann_GiaoTrinh_DWA_TEB_NE.slx` được tạo bởi builder. Trong Simulink, dùng `CHON_DWA_HOAC_TEB` để chọn thuật toán.

## Thay vật cản và tính lại đường

Sửa ma trận `c.obstacles = [x y radius; ...]` trong `ackermann_cfg.m`, sau đó chạy lại `clear functions; rehash; build_ackermann_giaotrinh`. Đường A* được đưa vào cả bộ bám quỹ đạo và bộ lập kế hoạch; xem `GlobalRoute_ASTAR`, `GlobalRouteLog`. Không hỗ trợ di chuyển vật cản **ngay khi simulation đang chạy**.

```matlab
addpath('tests');
test_global_route
```

## Các file quan trọng

| File/Thư mục | Chức năng |
|---|---|
| `ackermann_cfg.m` | Thông số robot, điểm đầu–đích, bản đồ vật cản |
| `ackermann_global_route.m` | A* 8 hướng có khoảng hở an toàn |
| `build_ackermann_giaotrinh.m` | Tạo và nối các khối Simulink |
| `ackermann_planners_giaotrinh.m` | DWA và TEB rút gọn |
| `msfcn_planner_giaotrinh.m` | Level-2 MATLAB S-Function quản lý chu kỳ planner |
| `ackermann_dynamics.m` | Mô hình động lực học robot |
| `function_scripts/` | Các MATLAB Function được builder nạp vào khối Simulink |
| `run_ackermann_giaotrinh.m` | Chạy mô phỏng và xuất dữ liệu |
| `ackermann_live_view.m` | Hoạt cảnh sa bàn, đồ thị |
| `tests/` | Kiểm tra mã nguồn và A* |
| `dist/` | Bản ZIP phát hành đầy đủ |

## Hạn chế và tình trạng kiểm thử

- Các tham số mẫu là **giả định**, chưa được nhận dạng từ robot thật.
- A* tạo đường ngắn theo lưới với robot được quy về vùng bao an toàn; **không bảo đảm** khả thi Ackermann ở mọi góc cua hoặc tối ưu thời gian.
- TEB ở đây là thuật toán **rút gọn**, không phải triển khai TEB chính thức/đầy đủ.
- Chỉ lập kế hoạch lại khi cập nhật mô hình/trước phiên chạy; không xử lý vật cản di chuyển trực tuyến.
- Bộ mã được kiểm tra cấu trúc và thử nghiệm A* độc lập; **chưa biên dịch/chạy trên MATLAB/Simulink trong môi trường xuất bản**. Nếu gặp lỗi, cung cấp Diagnostic Viewer để sửa theo đúng phiên bản MATLAB.

> Các file `config_and_builder.patch` và hướng dẫn bản vá cũ được giữ lại để tham khảo lịch sử sửa lỗi. Mã nguồn hoàn chỉnh hiện có trực tiếp trong thư mục này.
