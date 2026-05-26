# Hướng Dẫn Chạy Demo Chi Tiết Dự Án Xe Lăn Thông Minh

Tài liệu này hướng dẫn chi tiết từng bước để biên dịch và vận hành thử nghiệm (demo) 4 chế độ mô phỏng trên Gazebo/RViz cùng hướng dẫn nạp chương trình và chạy thực tế với xe lăn thật.

---

## 1. Các Tính Năng Demo Có Sẵn

Dự án hỗ trợ 4 chế độ demo chính thông qua launch file tổng quát:
1. **`world`**: Khởi chạy xe lăn trong môi trường mô phỏng bệnh viện, hiển thị dữ liệu cảm biến Lidar/Camera và điều khiển bằng bàn phím.
2. **`slam`**: Sử dụng thư viện **SLAM Toolbox** để quét và vẽ bản đồ 2D của bệnh viện bằng Lidar.
3. **`nav`**: Định vị và tự hành đến vị trí đích chỉ định trên RViz qua **Nav2**, tự động tránh vật cản tĩnh và động bằng cấu hình `footprint` đa giác chữ nhật.
4. **`follow`**: Xe lăn tự động nhận diện người đi bộ qua camera bằng **YOLOv8** và kết hợp dữ liệu quét Lidar (Sensor Fusion) để tự bám theo giữ khoảng cách an toàn (mặc định 1.2m).

---

## 2. Chuẩn Bị Môi Trường Khuyến Nghị

* **Hệ điều hành**: Ubuntu 22.04 LTS (hoặc cài đặt ROS 2 Humble thông qua Docker/WSL 2).
* **Phiên bản ROS 2**: ROS 2 Humble Hawksbill.
* **Trình mô phỏng**: Gazebo Classic 11.
* **Ngôn ngữ**: Python 3.10+, C++ (dành cho ROS 2 nodes).
* **Phần cứng thực tế (nếu chạy thật)**: Raspberry Pi 4 (hoặc tương đương) cài Ubuntu Server 22.04 + Board mạch Arduino Nano điều khiển động cơ PID.

---

## 3. Cài Đặt Hệ Thống & Dependencies

### Bước 3.1: Cập nhật hệ thống và cài đặt ROS 2 Packages
Chạy lệnh sau trên terminal Ubuntu để cài đặt toàn bộ công cụ biên dịch và các thư viện ROS 2 cần thiết:
```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-rviz-plugins \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  ros-humble-cv-bridge \
  ros-humble-tf-transformations \
  ros-humble-rviz2
```

### Bước 3.2: Khởi tạo và cập nhật `rosdep`
```bash
sudo rosdep init
rosdep update
```
*(Nếu hệ thống báo đã khởi tạo `rosdep init` trước đó, bạn chỉ cần chạy lệnh `rosdep update`)*.

### Bước 3.3: Cài đặt thư viện Python cho AI & Giao tiếp base
```bash
python3 -m pip install --upgrade pip
pip install ultralytics opencv-python pyserial
```

---

## 4. Tải Mã Nguồn Và Biên Dịch

### Bước 4.1: Tạo Workspace và clone dự án
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/CamLeVan/Smart-Wheelchair.git
cd ~/ros2_ws
```

### Bước 4.2: Tự động cài dependencies bổ sung từ repo
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Bước 4.3: Biên dịch Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```
Kiểm tra xem hệ thống đã nhận diện đầy đủ 4 packages chưa bằng lệnh:
```bash
ros2 pkg list | grep smart_wheelchair
```
**Kết quả mong đợi:**
* `smart_wheelchair_base`
* `smart_wheelchair_description`
* `smart_wheelchair_navigation`
* `smart_wheelchair_vision`

---

## 5. Hướng Dẫn Vận Hành Từng Chế Độ Demo

> [!IMPORTANT]
> Chỉ chạy **một nguồn điều khiển phát `/cmd_vel` duy nhất** tại một thời điểm. Ví dụ: khi đang dùng Nav2 hoặc AI Follow-me, bạn phải tắt chương trình điều khiển bàn phím (Teleop).

---

### Chế độ 1: Mô phỏng xe lăn, cảm biến & Điều khiển bàn phím (`world`)

1. **Khởi chạy mô phỏng thế giới bệnh viện và xe lăn**:
   ```bash
   ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=world
   ```
   *Gazebo và RViz sẽ hiển thị mô hình xe lăn cùng dữ liệu Lidar quét môi trường bệnh viện.*

2. **Mở terminal mới để lái xe lăn thủ công**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   *Dùng các phím `u i o j k l m , .` để điều khiển xe tiến, lùi, quay đầu.*

---

### Chế độ 2: Thiết lập bản đồ môi trường bằng Lidar (`slam`)

1. **Khởi chạy mô phỏng tích hợp SLAM Toolbox**:
   ```bash
   ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=slam
   ```
   *RViz sẽ hiển thị bản đồ đang dựng dở dưới dạng lưới màu xám/trắng.*

2. **Lái xe lăn đi vòng quanh bệnh viện để quét bản đồ**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   *Di chuyển xe lăn chậm rãi qua tất cả các hành lang và phòng bệnh để bản đồ được quét sắc nét nhất.*

3. **Lưu lại bản đồ đã quét**:
   Khi bản đồ đã hoàn thiện trên RViz, mở terminal mới và lưu bản đồ vào thư mục maps của dự án:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/smart_wheelchair_navigation/maps/hospital_map
   ```

---

### Chế độ 3: Xe lăn tự hành thông minh tránh vật cản (`nav`)

1. **Khởi chạy mô phỏng tích hợp định vị và Nav2**:
   ```bash
   ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=nav
   ```
   *Bản đồ bệnh viện định sẵn `hospital_map` sẽ được load tự động.*

2. **Định vị điểm xuất phát ban đầu cho xe lăn**:
   * Nhấn nút **`2D Pose Estimate`** trên thanh công cụ của RViz.
   * Click chuột vào vị trí thực tế của xe trên bản đồ, kéo chuột để xoay mũi tên chỉ đúng hướng đầu xe.
   * Quan sát thấy các đám mây hạt màu xanh (AMCL) bao quanh xe lăn để khớp vị trí.

3. **Đặt mục tiêu tự hành**:
   * Nhấn nút **`Nav2 Goal`** trên RViz.
   * Click chuột vào điểm cần tới trên bản đồ và chỉ định hướng đỗ của xe lăn.
   * Xe lăn sẽ tự thiết lập đường đi toàn cục (màu đỏ) và tự lái đi đến đích, tự lách qua các góc hẹp và né tránh chướng ngại vật di động nhờ cấu hình `footprint` chữ nhật thực tế.

---

### Chế độ 4: AI Follow-me bám theo người đi bộ (`follow`)

1. **Tải trước mô hình YOLOv8 Nano (nếu máy không có Internet)**:
   Mở terminal chạy Python để tải file weights của YOLOv8 về cache hệ thống:
   ```bash
   python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
   ```

2. **Khởi chạy mô phỏng kết hợp AI tracking**:
   ```bash
   ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=follow use_rviz:=false
   ```
   * Cửa sổ camera trực quan của OpenCV mang tên `Sensor Fusion & Tracking` sẽ xuất hiện.
   * YOLOv8 sẽ tự động khóa và vẽ khung bao màu xanh lục lên actor người đi bộ (`Target ID: 1`).
   * Xe lăn sẽ kết hợp góc lệch camera và khoảng cách Lidar chuẩn để tự động hướng đầu về phía người và tiến hành bám theo ở khoảng cách an toàn.

3. **Thay đổi thông số bám đuổi (nếu cần)**:
   Bạn có thể chỉnh khoảng cách giữ an toàn hoặc tốc độ bám đuôi trực tiếp từ tham số dòng lệnh:
   ```bash
   ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=follow use_rviz:=false \
     target_distance:=1.5 max_linear_speed:=0.4 max_angular_speed:=0.8
   ```

---

## 6. Hướng Dẫn Cài Đặt Trên Xe Lăn Thực Tế

### Bước 6.1: Nạp chương trình điều khiển Arduino Nano
1. Mở phần mềm **Arduino IDE** trên máy tính.
2. Mở file mã nguồn tại thư mục [firmware/motor_controller.ino](file:///d:/VKU_learning/HK6/Xe%20t%E1%BB%B1%20h%C3%A0nh/project/Smart-Wheelchair/firmware/motor_controller.ino).
3. Đấu nối các chân phần cứng đúng như định nghĩa trong file (Động cơ BTS7960/L298N kết nối với các chân PWM và Encoder).
4. Nhấn **Upload** để nạp chương trình cho Arduino Nano.

### Bước 6.2: Khởi động giao tiếp Base Controller trên Máy tính nhúng (Pi 4)
1. Cắm cáp USB nối Arduino Nano với Pi 4.
2. Kiểm tra cổng serial kết nối (thường là `/dev/ttyUSB0` hoặc `/dev/ttyACM0`).
3. Khởi chạy node giao tiếp serial trong workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run smart_wheelchair_base base_controller
   ```
   *Node này sẽ tự dịch chuyển các lệnh tốc độ `/cmd_vel` từ ROS 2 thành số xung Encoder mục tiêu động học chính xác để truyền xuống cho Arduino PID điều khiển bánh xe lăn thực tế.*
