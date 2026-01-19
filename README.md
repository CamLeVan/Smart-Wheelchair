# Smart-Wheelchair: Hệ Thống Xe Lăn Thông Minh Tự Hành (Assistive Smart Wheelchair)

![ROS 2](https://img.shields.io/badge/ROS_2-Foxy%2FHumbe-green)
![Language](https://img.shields.io/badge/Language-Python%20%7C%20C%2B%2B-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry_Pi_%2B_Arduino-red)

## 1. Giới Thiệu (Introduction)
**Smart-Wheelchair** là một hệ thống nhúng di động (Mobile Embedded System) hỗ trợ người khuyết tật di chuyển độc lập và an toàn. Dự án chuyển đổi xe lăn cơ học truyền thống thành một robot tự hành cấp độ 3-4 (theo chuẩn SAE), tích hợp trí tuệ nhân tạo để nhận diện môi trường và thuật toán điều hướng tiên tiến.

### Tính Năng Chính
*   **Tự hành (Autonomous Navigation):** Tự động lập bản đồ (SLAM) và tìm đường đi ngắn nhất trong môi trường bệnh viện/trong nhà.
*   **Tránh vật cản (Safety & Obstacle Avoidance):** Sử dụng Lidar và Cảm biến siêu âm để phát hiện và né tránh vật cản động thời gian thực.
*   **Bám theo người (Human Follow-me):** Tích hợp AI (CNN/YOLO) và Camera để nhận diện và bám theo người hỗ trợ.
*   **Hệ thống an toàn chủ động:** Cơ chế phanh khẩn cấp (E-Stop) và ổn định vận tốc (PID control).

---

## 2. Yêu Cầu Hệ Thống (Prerequisites)

### Phần Mềm
*   **OS:** Ubuntu 20.04 (ROS Noetic) hoặc Ubuntu 22.04 (ROS 2 Humble/Iron) - *Khuyên dùng ROS 2*.
*   **Simulator:** Gazebo Classic 11 hoặc Gazebo Fortress.
*   **Languages:** Python 3.8+, C++ 14.

### Phần Cứng (Hardware Requirements)
*   **High-Level ECU:** Raspberry Pi 4 Model B (4GB/8GB RAM).
*   **Low-Level ECU:** Arduino Nano / STM32.
*   **Sensors:** Lidar (RPLidar A1/A2), Camera (Pi Cam v2/Intel Realsense), IMU (MPU6050).
*   **Actuators:** Động cơ DC Servo/Stepper có Encoder.

---

## 3. Cài Đặt (Installation)

### Bước 1: Clone dự án
```bash
cd ~/ros2_ws/src
git clone https://github.com/CamLeVan/Smart-Wheelchair.git
cd ..
```

### Bước 2: Cài đặt các thư viện phụ thuộc
Sử dụng `rosdep` để tự động cài đặt các dependencies còn thiếu:
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Bước 3: Build dự án
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4. Hướng Dẫn Sử Dụng (Usage)

### Chế độ Mô Phỏng (Simulation)
Khởi chạy môi trường giả lập Gazebo và Robot:
```bash
ros2 launch smart_wheelchair_description gazebo.launch.py
```

Để điều khiển xe bằng bàn phím (Teleop):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Chế độ Thực Tế (Real Robot)
*(Yêu cầu kết nối SSH vào Raspberry Pi)*
1. Khởi động các Driver phần cứng (Lidar, Motor, Camera):
   ```bash
   ros2 launch smart_wheelchair_base robot.launch.py
   ```
2. Khởi động Navigation Stack:
   ```bash
   ros2 launch smart_wheelchair_nav navigation.launch.py
   ```

---

## 5. Tài Liệu Chi Tiết (Documentation)
Để hiểu rõ hơn về kiến trúc và thuật toán, vui lòng tham khảo bộ tài liệu chi tiết trong thư mục `docs/`:

*   [**Kiến Trúc Hệ Thống (Architecture)**](docs/ARCHITECTURE.md): Giải thích luồng dữ liệu ROS, Nodes và Topics.
*   [**Thiết Kế Phần Cứng (Hardware)**](docs/HARDWARE.md): Sơ đồ đấu nối ECU, Pinout và BOM.
*   [**Thuật Toán & AI (Algorithms)**](docs/ALGORITHMS.md): Chi tiết về PID, SLAM, và mạng CNN 11 lớp.
*   [**Hướng Dẫn Vận Hành (User Manual)**](docs/USER_MANUAL.md): Quy trình khởi động và an toàn.

---

## 6. Lộ Trình Phát Triển (Roadmap)
- [x] Thiết kế URDF và Mô phỏng Gazebo (Phase 1).
- [ ] Tích hợp driver điều khiển động cơ (Motor Control).
- [ ] Triển khai SLAM & Navigation (Phase 2).
- [ ] Tích hợp AI Vision (Follow-me) (Phase 3).

## 7. Liên Hệ
*   **Tác giả:** [Tên Của Bạn]
*   **Đơn vị:** VKU - Vietnam-Korea University of ICT
