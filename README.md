# Smart Wheelchair Gazebo Demo

Du an ROS 2 mo phong xe lan thong minh tren Gazebo, tap trung vao 4 chuc nang demo:

1. Mo phong robot, cam bien Lidar/Camera va dieu khien tay.
2. SLAM de ve ban do moi truong benh vien.
3. Navigation 2 de tu hanh den diem dich tren ban do.
4. AI Follow-me bang YOLO + Lidar sensor fusion.

Trang thai hien tai: repo duoc chuan hoa theo muc tieu **clone tren Linux va demo tren Gazebo**. Phan firmware Arduino van duoc giu lai de tham khao/robot that, nhung khong bat buoc cho demo Gazebo.

## Moi Truong Khuyen Nghi

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10+
- Internet o lan dau chay AI de tai `yolov8n.pt`, hoac dat model co san trong may

## Clone Va Build Nhanh

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/CamLeVan/Smart-Wheelchair.git
cd ~/ros2_ws

sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

pip install ultralytics opencv-python pyserial

colcon build --symlink-install
source install/setup.bash
```

Neu `rosdep` bao thieu package, xem huong dan chi tiet trong [GAZEBO_DEMO.md](GAZEBO_DEMO.md).

## Lenh Demo Chinh

Chi chay **mot mode dieu khien `/cmd_vel` tai mot thoi diem**.

Lenh ngan co menu chon demo:

```bash
./demo.sh
```

```bash
# 1. Gazebo + RViz, dung de test robot/sensor/teleop
./demo.sh world

# 2. Gazebo + SLAM Toolbox + RViz
./demo.sh slam

# 3. Gazebo + Nav2 + RViz
./demo.sh nav

# 4. Gazebo + AI follow-me
./demo.sh follow
```

Dieu khien tay khi can:

```bash
./demo.sh teleop
```

## Cau Truc Quan Trong

- `src/smart_wheelchair_description`: robot URDF, Gazebo world, launch mo phong.
- `src/smart_wheelchair_navigation`: launch tong demo, Nav2 config, SLAM config, map, RViz config.
- `src/smart_wheelchair_vision`: node YOLO follow-me.
- `src/smart_wheelchair_base`: node serial/odometry cho robot that.
- `firmware/motor_controller.ino`: firmware Arduino cho motor PID, encoder, heartbeat, ultrasonic e-stop.
- `GAZEBO_DEMO.md`: huong dan demo chi tiet tung buoc.
- `IMPORTANT_FILES.md`: giai thich nhanh cac file can luu y.

## Topic Chinh

- `/cmd_vel`: lenh van toc cho robot.
- `/odom`: odometry tu Gazebo diff-drive hoac base controller.
- `/scan`: LaserScan tu Lidar.
- `/camera/image_raw`: anh camera cho AI follow-me.
- `/map`: ban do tu SLAM/Nav2 map server.
- `/tf`: cay toa do `map -> odom -> base_link -> sensors`.

## Luu Y Khi Demo

- Khong chay Teleop, Nav2 va Follow-me cung luc vi tat ca deu publish `/cmd_vel`.
- Lan dau chay AI co the mat thoi gian tai model YOLO.
- Neu Nav2 khong di, hay dat lai `2D Pose Estimate` trong RViz truoc khi dat `Nav2 Goal`.
- Neu clone tren may Linux moi, khong dung thu muc `build/`, `install/`, `log/` cu. Hay build lai bang `colcon build --symlink-install`.
