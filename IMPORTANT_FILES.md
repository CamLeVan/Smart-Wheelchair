# Cac File Can Luu Y

Tai lieu nay giup nguoi clone repo biet nen doc/sua file nao khi demo Gazebo.

## File Huong Dan

- `README.md`: tong quan nhanh, lenh clone/build/demo chinh.
- `GAZEBO_DEMO.md`: huong dan chi tiet de chay tren Linux/Gazebo.
- `IMPORTANT_FILES.md`: file nay, giai thich vai tro cac file quan trong.

## Launch Files

- `src/smart_wheelchair_navigation/launch/gazebo_demo.launch.py`
  - Launch tong cho demo Gazebo.
  - Tham so `mode` gom `world`, `slam`, `nav`, `follow`.
  - Day la lenh nen dung khi demo.

- `src/smart_wheelchair_description/launch/gazebo.launch.py`
  - Mo Gazebo world.
  - Spawn robot tu URDF.
  - Chay `robot_state_publisher`.

- `src/smart_wheelchair_navigation/launch/navigation.launch.py`
  - Chay Nav2 bringup voi map va params cua du an.

- `src/smart_wheelchair_navigation/launch/slam.launch.py`
  - Chay SLAM Toolbox online async.

## Robot Va World

- `src/smart_wheelchair_description/urdf/wheelchair.urdf`
  - Mo ta robot.
  - Co differential drive plugin nhan `/cmd_vel`.
  - Publish `/odom`, `/scan`, `/camera/image_raw`.

- `src/smart_wheelchair_description/worlds/hospital.world`
  - Moi truong benh vien cho demo.
  - Co tuong, giuong, vat can va actor nguoi di bo.

## Navigation

- `src/smart_wheelchair_navigation/config/nav2_params.yaml`
  - Tham so Nav2: AMCL, controller DWB, planner, costmap, behavior.
  - Sua file nay khi can tune toc do, ban kinh robot, inflation radius.

- `src/smart_wheelchair_navigation/config/mapper_params_online_async.yaml`
  - Tham so SLAM Toolbox.
  - Sua file nay khi Lidar/SLAM map khong on dinh.

- `src/smart_wheelchair_navigation/maps/hospital_map.yaml`
  - Metadata map Nav2.
  - Dang dung `image: hospital_map.pgm` de clone may nao cung chay.

- `src/smart_wheelchair_navigation/config/nav_config.rviz`
  - Layout RViz cho demo.
  - Co san RobotModel, TF, LaserScan, Map va Nav2 Goal.

## AI Follow-me

- `src/smart_wheelchair_vision/smart_wheelchair_vision/human_tracker.py`
  - Node YOLO tracking nguoi.
  - Subscribe `/camera/image_raw` va `/scan`.
  - Publish `/cmd_vel`.
  - Co tham so `target_distance`, `max_linear_speed`, `max_angular_speed`, `model_path`.

## Robot That / Firmware

- `src/smart_wheelchair_base/smart_wheelchair_base/base_controller.py`
  - Node bridge serial cho robot that.
  - Khong can chay khi demo Gazebo.
  - Dang hardcode `/dev/ttyUSB0`.

- `firmware/motor_controller.ino`
  - Arduino firmware cho motor PID, encoder, heartbeat, ultrasonic e-stop.
  - Khong can nap firmware neu chi demo Gazebo.

## Luu Y Ve `/cmd_vel`

Chi nen co mot node publish `/cmd_vel` trong moi demo:

- Teleop publish `/cmd_vel` khi demo dieu khien tay.
- Nav2 publish `/cmd_vel` khi demo tu hanh.
- `human_tracker` publish `/cmd_vel` khi demo follow-me.

Neu chay chung, robot co the giat hoac nhan lenh sai.

