# Huong Dan Demo Gazebo Tren Linux

File nay danh cho ban clone repo ve may Linux va chay toan bo demo Gazebo.

## 1. Muc Tieu Demo

Sau khi cai dat xong, ban co the demo 4 phan:

1. `world`: mo Gazebo, spawn xe, xem Lidar/Camera/Odom, dieu khien bang ban phim.
2. `slam`: dung SLAM Toolbox de ve ban do tu `/scan` va `/odom`.
3. `nav`: dung Nav2 de dat diem den trong RViz va xe tu hanh di toi dich.
4. `follow`: dung YOLOv8 de nhan dien nguoi trong camera, lay khoang cach Lidar va bam theo nguoi.

## 2. Cai ROS 2 Humble

Neu may chua co ROS 2 Humble, cai theo tai lieu chinh thuc cua ROS 2. Sau khi cai xong, moi terminal can source ROS:

```bash
source /opt/ros/humble/setup.bash
```

Co the them vao `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Cai Package He Thong

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

Khoi tao `rosdep` neu may chua co:

```bash
sudo rosdep init
rosdep update
```

Neu `sudo rosdep init` bao da ton tai, bo qua va chay `rosdep update`.

## 4. Clone Repo

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/CamLeVan/Smart-Wheelchair.git
cd ~/ros2_ws
```

## 5. Cai Python Dependencies

```bash
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python pyserial
```

Lan dau chay follow-me, `ultralytics` co the tu tai `yolov8n.pt`. Neu may demo khong co internet, hay tai san model va dat trong thu muc workspace, roi sua tham so `model_path`.

## 6. Cai ROS Dependencies Tu Repo

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 7. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Neu build thanh cong, kiem tra package:

```bash
ros2 pkg list | grep smart_wheelchair
```

Ky vong thay:

```text
smart_wheelchair_base
smart_wheelchair_description
smart_wheelchair_navigation
smart_wheelchair_vision
```

## 8. Demo 1: World + Sensor + Teleop

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=world
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Kiem tra topic:

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic hz /camera/image_raw
```

Ket qua mong doi:

- Gazebo hien moi truong benh vien.
- Xe lan xuat hien trong world.
- RViz hien robot, TF, LaserScan.
- Teleop lam xe di chuyen.

## 9. Demo 2: SLAM

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=slam
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Lai xe cham quanh moi truong de SLAM tao map. Neu muon luu map moi:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/smart_wheelchair_navigation/maps/hospital_map
```

Ket qua mong doi:

- RViz co topic `/map`.
- Khi xe di, map duoc cap nhat theo Lidar.
- Cay TF co `map -> odom -> base_link`.

## 10. Demo 3: Navigation 2

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=nav
```

Trong RViz:

1. Chon `2D Pose Estimate`, click gan dung vi tri xe tren map.
2. Xoay mui ten de khop huong xe.
3. Chon `Nav2 Goal`, click diem dich tren map.
4. Quan sat robot lap duong va di toi diem dich.

Neu xe khong chay:

```bash
ros2 topic echo /cmd_vel
ros2 lifecycle nodes
```

Ket qua mong doi:

- RViz hien map co san `hospital_map`.
- Nav2 lifecycle nodes active.
- Khi dat goal, robot publish `/cmd_vel` va di theo duong.
- Lidar/costmap giup robot tranh vat can.

## 11. Demo 4: AI Follow-me

Khong chay Teleop hoac Nav2 trong demo nay, vi se tranh `/cmd_vel`.

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=follow use_rviz:=false
```

Ket qua mong doi:

- Gazebo mo world co actor nguoi di bo.
- Cua so OpenCV `Sensor Fusion & Tracking` hien anh camera.
- Neu YOLO nhan ra nguoi, man hinh co bounding box, target ID va khoang cach.
- Robot quay ve phia nguoi va giu khoang cach mac dinh 1.2m.

Co the tinh chinh tham so:

```bash
ros2 launch smart_wheelchair_navigation gazebo_demo.launch.py mode:=follow use_rviz:=false \
  target_distance:=1.2 max_linear_speed:=0.4 max_angular_speed:=0.8
```

## 12. Cac Loi Thuong Gap

### Khong thay package `smart_wheelchair_*`

Chua source workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

### Gazebo khong mo hoac thieu plugin

Cai lai Gazebo ROS package:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### RViz khong co nut Nav2 Goal

Cai Nav2 RViz plugin:

```bash
sudo apt install ros-humble-nav2-rviz-plugins
```

### Follow-me khong hien cua so camera

Kiem tra topic camera:

```bash
ros2 topic hz /camera/image_raw
```

Neu dang SSH khong co GUI, can bat X forwarding hoac chay truc tiep tren may co desktop.

### YOLO tai model qua cham

Tai model truoc khi demo:

```bash
python3 - <<'PY'
from ultralytics import YOLO
YOLO("yolov8n.pt")
PY
```

### Xe bi giat hoac khong theo dung

- Chi chay mot nguon `/cmd_vel`.
- Tat Teleop khi demo Nav2/Follow-me.
- Giam toc do bang tham so follow-me neu may yeu.

## 13. Lenh Don Dep Khi Can Build Lai

Neu build bi loi do cache cu:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

