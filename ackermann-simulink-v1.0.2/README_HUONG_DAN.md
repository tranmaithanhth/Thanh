# Ackermann MATLAB/Simulink — v1.0.2: Tự tính đường đi theo vật cản

**Sửa lỗi cốt lõi:** phiên bản cũ dùng đường dẫn cứng tại `c.route` và trong `function_scripts/tkqd_p.txt`, nên di chuyển chướng ngại vật không đổi đường tham chiếu. Bản mới dùng A* trên lưới 8 láng giềng để tính lại `c.route` dựa trên `c.obstacles`, sau đó truyền **cùng tuyến A*** vào khối `TKQD_cho_P` (cổng `Route`) và bộ lập kế hoạch DWA / TEB. Sa bàn vẽ tuyến A* thực tế, không vẽ tuyến cũ.

## Chạy lại hoàn toàn

1. Giải nén gói mới vào thư mục **mới**. Không chồng file `.slx` phiên bản cũ.
2. Mở MATLAB, chọn Current Folder trỏ đến thư mục `Ackermann_GiaoTrinh_v1.0.2`.
3. Chỉnh sửa `ackermann_cfg.m`:

```matlab
c.start = [0 0 0 0 0];
c.goal = [16.0 0.6];
c.obstacles = [4.1 -1.3 0.75; 7.3 -1.2 0.72; 10.1 -1.4 0.76];
c.globalGridResolution = 0.20;
```

4. Lưu file và chạy:

```matlab
clear functions; rehash;
c = ackermann_cfg();
disp(c.route);  % route moi duoc tinh tu c.obstacles
figure; plot(c.route(:,1),c.route(:,2),'b-o'); axis equal; grid on;
build_ackermann_giaotrinh
out = run_ackermann_giaotrinh('DWA');
```

Chuyển `DWA` sang `TEB` để so sánh. Trong sơ đồ Simulink có khối mới `GlobalRoute_ASTAR` nối trực tiếp tới cổng thứ ba của `TKQD_cho_P`. Khối `GlobalRouteLog` lưu đường tham chiếu hiện tại.

## Giải thích giới hạn

- A* tính đường tối ưu theo khoảng cách **trên lưới** đã nới rộng các vật cản theo bán kính bao robot. Không bảo đảm quỹ đạo điều khiển Ackermann tối ưu về thời gian hoặc tuyệt đối không kẹt. Kết quả cần kiểm tra bằng mô phỏng.
- DWA là bộ lấy mẫu điều khiển cục bộ; TEB là bản tối ưu hóa dải quỹ đạo rút gọn, có DWA fallback, không phải TEB chính thức.
- Thay đổi `c.obstacles` trước mỗi lần Run: `ackermann_cfg` sẽ tính tuyến mới; nên chạy lại `build_ackermann_giaotrinh` sau khi thay đổi để đảm bảo các khối và cấu hình đồng bộ. Thay đổi mảng trong lúc **đang chạy** không được hỗ trợ như bản đồ động trong phiên bản này.
- Nếu có vật cản ở quá gần điểm đầu hoặc điểm đích theo kích thước thân xe, A* sẽ báo lỗi `globalRouteStartBlocked` / `globalRouteGoalBlocked` thay vì dùng tuyến nguy hiểm.
- Nếu kết quả sau khi chỉnh vẫn y hệt, kiểm tra `which ackermann_cfg -all` và `which build_ackermann_giaotrinh -all`: rất có thể MATLAB đang mở thư mục bản cũ.

## Kiểm tra toán học

Chạy `tests/test_global_route` (bổ sung thư mục tests vào path trước). Nó kiểm tra: có đường thẳng khi vật cản ở xa, có đường vòng khi vật cản chắn đường, và đoạn đường không cắt qua vùng chướng ngại đã nới rộng.

**CẢNH BÁO:** Môi trường tạo gói không có MATLAB/Simulink; các mã MATLAB chưa được biên dịch và mô phỏng bằng MATLAB thực. Đây là bản sửa theo mã nguồn và kiểm tra tĩnh, không khẳng định đã chạy thành công trên máy của bạn.
