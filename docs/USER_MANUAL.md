# Hướng Dẫn Vận Hành & Quy Trình An Toàn (User Manual)

Tài liệu hướng dẫn vận hành hệ thống **Smart-Wheelchair** dành cho kỹ sư và người sử dụng.

## 1. Quy Tắc An Toàn (Safety First)
⚠️ **CẢNH BÁO:** Đây là thiết bị cơ điện tử có động cơ công suất lớn. Luôn tuân thủ các quy tắc sau:

1.  **Luôn kê cao bánh xe (Jack up):** Khi code hoặc test PID lần đầu, hãy kê xe lên cao sao cho bánh xe không chạm đất để tránh xe chạy mất kiểm soát.
2.  **Sẵn sàng E-Stop:** Luôn giữ tay gần nút ngắt nguồn khẩn cấp hoặc sẵn sàng rút dây nguồn khi thử nghiệm thực tế.
3.  **Kiểm tra pin:** Đảm bảo điện áp pin nằm trong ngưỡng an toàn (không dưới 10.5V với pin 3S, 14V với pin 4S).

---

## 2. Quy Trình Khởi Động (Startup Sequence)

### Bước 1: Khởi động phần cứng
1.  Bật công tắc nguồn chính (Main Power).
2.  Chờ Raspberry Pi khởi động (khoảng 30-60 giây).
3.  Kết nối máy tính điều khiển (Laptop) vào cùng mạng WiFi với Raspberry Pi.

### Bước 2: Truy cập hệ thống
Sử dụng SSH để truy cập vào xe:
```bash
ssh ubuntu@<IP_ADDRESS_CUA_PI>
# Nhập mật khẩu
```

### Bước 3: Kiểm tra trạng thái
Kiểm tra xem các thiết bị đã kết nối chưa:
```bash
ls /dev/ttyUSB* 
# Phải thấy ít nhất ttyUSB0 (Lidar) và ttyUSB1 (Arduino)
```

---

## 3. Vận Hành Tự Hành (Autonomous Operation)

### Chạy Mapping (Tạo bản đồ mới)
1.  Khởi động driver và SLAM:
    ```bash
    ros2 launch smart_wheelchair_cartographer mapping.launch.py
    ```
2.  Dùng tay cầm hoặc bàn phím điều khiển xe đi khắp phòng để quét.
3.  Lưu bản đồ:
    ```bash
    ros2 run nav2_map_server map_saver_cli -f my_map
    ```

### Chạy Navigation (Dẫn đường tự động)
1.  Khởi động Nav2 với bản đồ đã lưu:
    ```bash
    ros2 launch smart_wheelchair_nav navigation.launch.py map:=my_map.yaml
    ```
2.  Trên máy tính Laptop, mở Rviz.
3.  Sử dụng công cụ **"2D Pose Estimate"** để chỉ vị trí xe hiện tại.
4.  Sử dụng công cụ **"Nav2 Goal"** để bấm vào điểm đích muốn đến. Xe sẽ tự tìm đường đi.

---

## 4. Xử Lý Sự Cố Thường Gặp (Troubleshooting)

| Triệu chứng | Nguyên nhân có thể | Cách khắc phục |
| :--- | :--- | :--- |
| **Xe không di chuyển** | Driver chưa bật, hoặc nút E-Stop đang nhấn. | Kiểm tra nguồn động cơ, kiểm tra log `/cmd_vel` có dữ liệu không. |
| **Xe đi lùi khi bấm tiến** | Đấu dây động cơ bị ngược. | Đảo chiều dây + và - của động cơ tại cầu đấu Driver, hoặc sửa trong code Arduino. |
| **Bản đồ bị trôi (Drift)** | Bánh xe bị trượt hoặc thông số Encoder sai. | Kiểm tra lại độ căng bánh xe, cân chỉnh lại `ticks_per_meter` trong code. |
| **Lidar báo lỗi** | Cổng USB bị sai quyền truy cập. | Chạy lệnh `sudo chmod 666 /dev/ttyUSB0`. |

---

## 5. Bảo Trì (Maintenance)
*   Sạc pin đầy đủ sau mỗi phiên làm việc.
*   Kiểm tra ốc vít bánh xe định kỳ.
*   Vệ sinh mắt đọc Lidar và Camera bằng khăn khô mềm.
