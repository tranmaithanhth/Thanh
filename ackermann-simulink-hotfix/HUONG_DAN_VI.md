# Hướng dẫn sửa lỗi Simulink Ackermann — v1.0.1

## Lỗi đã xử lý

**Thông báo:** `Variable 'q' is not fully defined on some execution paths` ở `TKQD_cho_P/Phuong_trinh`.

**Nguyên nhân:** MATLAB Coder không bảo đảm `q` và `th` đã được gán trước khi tạo `pd=[q(1);q(2);th]`. Sửa bằng cách khởi tạo cả hai trước vòng tìm điểm trên tuyến.

Đồng thời sửa lỗi thứ hai trong `msfcn_planner_giaotrinh.m`: trước đây khởi tạo vectơ trạng thái Dwork bằng cách ghép vectơ cột với một ma trận N×2, gây lỗi kích thước. Phiên bản mới chuyển dải quỹ đạo 2×N thành vectơ cột bằng `band0(:)`.

## Cập nhật mô hình đã tải

1. Giải nén gói **Ackermann_GiaoTrinh_DWA_TEB_NE_fix_v1.0.1.zip** trong cuộc trò chuyện, hoặc tải hai file mã nguồn đã sửa từ thư mục GitHub này.
2. Sao lưu `Ackermann_GiaoTrinh_DWA_TEB_NE.slx` nếu đã chỉnh sửa trực tiếp trong Simulink.
3. Thay hai file:
   - `function_scripts/tkqd_p.txt`
   - `msfcn_planner_giaotrinh.m`
4. Trong `build_ackermann_giaotrinh.m`, bỏ `try/catch` bao quanh `set_param(mdl,'SimulationCommand','update')`, theo file `build_compile_check.patch`.
5. Mở MATLAB và đặt **Current Folder** ở thư mục mã nguồn, chạy:

```matlab
clear functions
rehash
build_ackermann_giaotrinh
```

6. Nếu tạo mô hình thành công, chạy:

```matlab
out = run_ackermann_giaotrinh('DWA');
% hoặc:
out = run_ackermann_giaotrinh('TEB');
```

## Lưu ý

- Code bên trong block MATLAB Function được builder nạp từ file `.txt`; chỉ sửa trong block `.slx` sẽ bị ghi đè khi dựng lại.
- Đây là **thư mục bản sửa trên repository Thanh hiện có**, không phải repository Ackermann mới.
- GitHub lưu các file hotfix; gói ZIP đầy đủ do ChatGPT gửi qua cuộc trò chuyện.
- Đã qua kiểm tra cấu trúc mã; **chưa có MATLAB/Simulink trong môi trường kiểm thử** nên chưa xác nhận chạy thực tế và không khẳng định đã sửa mọi lỗi tiếp theo.
