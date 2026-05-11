# HƯỚNG DẪN DEMO DỰ ÁN SMART-WHEELCHAIR (DÀNH CHO NGƯỜI MỚI)

Chào bạn! Đây là kịch bản chi tiết nhất để bạn thực hiện buổi báo cáo chuyên đề thành công rực rỡ. Hãy thực hiện theo từng bước dưới đây.

---

## I. CHUẨN BỊ (LÀM TRƯỚC KHI THẦY ĐẾN)

### 1. Cài đặt thư viện AI (Nếu chưa có)
Mở Terminal và chạy các lệnh:
```bash
pip install ultralytics opencv-python pyserial
sudo apt update
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-tf-transformations
```

### 2. Build lại toàn bộ dự án
```bash
cd ~/ros2_ws  # Chuyển đến workspace của bạn
colcon build --symlink-install
source install/setup.bash
```

---

## II. KỊCH BẢN DEMO 5 GIAI ĐOẠN

### Giai đoạn 1: Khởi động hệ thống (Show mô hình)
**Hành động:** 
Mở **Terminal 1** và chạy:
```bash
ros2 launch smart_wheelchair_description gazebo.launch.py
```
**Lời dẫn:** *"Thưa thầy, đây là mô hình xe lăn thông minh được thiết kế dưới dạng URDF. Em đã đặt xe vào môi trường giả lập bệnh viện với các vật cản như giường bệnh, tủ thuốc và đặc biệt là một nhân vật người đi bộ đang di chuyển để thử nghiệm tính năng AI."*

---

### Giai đoạn 2: Lập bản đồ thời gian thực (SLAM)
**Hành động:**
Mở **Terminal 2**:
```bash
ros2 run slam_toolbox async_slam_toolbox_node
```
Mở **Terminal 3** (Điều khiển xe đi vẽ bản đồ):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Lời dẫn:** *"Xe đang sử dụng thuật toán SLAM Toolbox. Khi em điều khiển xe di chuyển, cảm biến Lidar ảo sẽ quét môi trường và xây dựng bản đồ lưới xác suất (Occupancy Grid Map) ngay lập tức trên màn hình."*

---

### Giai đoạn 3: Điều hướng tự hành (Navigation 2)
**Hành động:**
Mở **Terminal 4**:
```bash
ros2 launch smart_wheelchair_navigation navigation.launch.py
```
Trên giao diện RVIZ, nhấn nút **"Nav2 Goal"** và chọn một điểm xa trên bản đồ.
**Lời dẫn:** *"Bây giờ xe đã có bản đồ. Khi em đặt một điểm đến, thuật toán Nav2 sẽ tính toán quỹ đạo tối ưu. Nếu có vật cản đột xuất, thuật toán DWA (Dynamic Window Approach) sẽ giúp xe tự động lách qua mà không cần can thiệp tay."*

---

### Giai đoạn 4: Đỉnh cao AI - Bám theo người (Follow-me)
**Hành động:**
Mở **Terminal 5**:
```bash
ros2 run smart_wheelchair_vision human_tracker
```
**Lời dẫn (Quan trọng nhất):** *"Đây là tính năng cốt lõi của đề tài. Thầy có thể thấy cửa sổ AI hiện lên:*
1. **Nhận diện:** Xe sử dụng YOLOv8 Nano để nhận diện người bệnh.
2. **Tracking (DeepSORT):** Thầy thấy mỗi người có một ID riêng, xe sẽ khóa mục tiêu vào đúng ID người hỗ trợ để không bị bám nhầm người khác.
3. **Sensor Fusion:** Xe không chỉ nhìn bằng ảnh, mà nó còn dùng Lidar để đo khoảng cách chính xác đến từng milimet. Nó sẽ luôn giữ cự ly an toàn 1.2m, dù người đó đứng hay ngồi xuống (giải quyết lỗi Bounding Box Fallacy)."*

---

### Giai đoạn 5: Chứng minh an toàn (Fail-safe)
**Hành động:** 
Giải thích về code Arduino (Firmware) đã nạp.
**Lời dẫn:** *"Về mặt an toàn phần cứng, em đã cài đặt cơ chế Heartbeat Monitor. Nếu phần mềm trên máy tính bị treo, Arduino sẽ tự ngắt động cơ sau 1 giây. Đồng thời, cảm biến siêu âm sẽ đóng vai trò E-Stop ngắt cứng nếu có vật cản quá gần dưới 50cm."*

---

## III. CÁC CÂU HỎI THƯỜNG GẶP VÀ CÁCH TRẢ LỜI

| Câu hỏi | Câu trả lời "ăn điểm" |
|:---|:---|
| **Tại sao dùng YOLOv8?** | Dạ, YOLOv8 Nano cho tốc độ xử lý trên 30fps trên các thiết bị nhúng như Raspberry Pi, đảm bảo phản ứng thời gian thực. |
| **Lidar giúp gì cho AI?** | Dạ, Lidar giúp xác định khoảng cách vật lý thực, giúp xe không bị tăng tốc lao vào người khi họ đột ngột cúi thấp (Bounding Box to ra). |
| **Nếu mất mạng có chạy được không?** | Dạ, toàn bộ hệ thống chạy Offline 100%, không cần internet sau khi đã tải model lần đầu. |

---

## IV. XỬ LÝ LỖI NHANH (TROUBLESHOOTING)

1. **Lỗi không thấy ảnh Camera:** Kiểm tra xem đã cài `gazebo_ros_pkgs` chưa.
2. **Xe không di chuyển khi bật AI:** Kiểm tra xem có Node nào khác đang chiếm topic `/cmd_vel` không (tắt Teleop đi).
3. **Màn hình AI bị giật:** Giảm độ phân giải trong file URDF xuống 320x240 nếu máy yếu.

---
*Chúc bạn có buổi demo thành công rực rỡ!*
