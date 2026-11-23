# Chẩn đoán xe Ackermann cấp 3 không lùi được

Áp dụng cho Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 với **chỉ robot cấp 3, không LiDAR/camera**. Xe đang có `cmd=(-0.10,0)` nhưng odometry chỉ `v=-0.01 m/s` khi gần `x=2.5`. Hạ hệ số ma sát của bánh xe không giúp xe lùi bình thường, nên chưa thể kết luận lỗi do ma sát.

## 1. Thử tiến và lùi trên cùng đoạn đường bằng một chương trình

1. Đóng Gazebo và chương trình điều khiển đang chạy. Khởi động lại Gazebo **không có robot** bằng `bash run_world_clean.sh`.
2. Ở Terminal khác spawn đúng **cấp 3** bằng `bash spawn_robot_stage.sh 3` (hoặc kiểm tra với `ACKERMANN_SAFE_FRICTION=1 bash spawn_robot_stage.sh 3` ở lượt thử riêng). Để robot ổn định, bảo đảm `gz stats` có `Paused[F]`, SimTime tăng.
3. **Không mở** `stage3_control_feedback.sh` hoặc chương trình khác gửi `cmd_vel` trong khi chạy phép thử. Khởi động terminal thứ ba, gọi:

   ```bash
   bash test_stage3_reverse.sh
   ```

4. Script kiểm tra chỉ có **một publisher** trên `/ackermann_vehicle/cmd_vel` và có subscriber Ackermann Drive, rồi tự thử **lùi −0,10 m/s trong 5 s**, dừng 1 s, **lùi −0,25 m/s trong 5 s**, dừng 1 s, **tiến +0,25 m/s trong 5 s**, cuối cùng dừng. Kết quả in `Δlong` = quãng đường theo hướng dọc trục thân xe. Thử nghiệm có vùng an toàn mặc định: `−3 < x < 1,5`, `−4 < y < −1` m; script sẽ tự dừng khi ra ngoài giới hạn này hoặc mất odometry. Lưu ý giới hạn an toàn của **phép thử** không phải ranh giới Gazebo.

5. So sánh các kết quả:

   | Kết quả | Hướng xác minh tiếp |
   | --- | --- |
   | Lùi −0,10 yếu, −0,25 tốt | Kiểm tra lực kéo và tham số PID ở tốc độ thấp; đối chiếu độ dốc và trượt bánh. |
   | Cả hai chế độ lùi yếu nhưng tiến +0,25 tốt | Kiểm tra động lực học ngược chiều: mômen bánh, hướng trục khớp, phản lực tiếp xúc và bộ điều khiển. |
   | Cả tiến và lùi yếu | Kiểm tra plugin Ackermann, va chạm thân xe, trượt bánh, real-time factor và lệnh từ ROS. |
   | Lùi/tiến tốt trên đường phẳng, nhưng không lùi được gần x≈2,5 | Kiểm tra tương tác địa hình và bánh tại chân dốc; không quy lỗi cho plugin chỉ từ một điểm dừng. |

**Chú ý:** không có LiDAR tự tránh vật cản. Chỉ thử ở world mới, nhìn robot liên tục, và dùng `Ctrl+C` nếu có tình trạng nguy hiểm. Script tự gửi lệnh 0 khi thoát, nhưng nếu Gazebo bị treo, không bảo đảm lệnh được xử lý ngay.

## 2. Thử tốc độ lùi bằng bàn phím

`stage3_console.py` đã có tùy chọn `--reverse-speed` để chủ động đặt tốc độ lùi. Ví dụ:

```bash
bash stage3_control_feedback.sh --deadman 10 --speed-step 0.10 --max-speed 0.60
```

Chỉ dùng khi **không có** `test_stage3_reverse.sh` hoặc chương trình điều khiển khác. Lệnh Phiên bản console mới điều khiển theo **bước vận tốc**: mỗi lần nhấn `S` giảm 0,10 m/s, `W` tăng 0,10 m/s. Tham số cũ `--reverse-speed 0.25` hiện chỉ giới hạn tốc độ lùi tối đa ở −0,25 m/s, không đặt ngay giá trị lệnh khi nhấn `S`.

## 3. Phản hồi cần gửi

Gửi ba dòng kết quả `REVERSE_010`, `REVERSE_025`, `FORWARD_025` cùng `gz stats` để khoanh vùng tiếp. Khi chưa có kết quả A/B, không nên điều chỉnh hàng loạt tham số hình học, PID và ma sát cùng lúc.