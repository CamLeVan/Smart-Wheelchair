# Huong Dan Demo Du An Xe Lan Thong Minh Tren Gazebo

Tai lieu nay dung de demo nhanh truoc thay/hoi dong. Muc tieu la trinh bay ro 4 chuc nang chinh:

1. Mo phong Gazebo: robot, benh vien, cam bien.
2. SLAM: tao ban do bang Lidar.
3. Navigation: A* planner cua Nav2 de tinh duong di tren map benh vien.
4. Follow-me: camera/Lidar bam muc tieu; YOLO that co san o tuy chon rieng.

---

## 1. Chuan Bi Truoc Khi Demo

Mo terminal tai thu muc du an:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
```

Neu vua clone moi hoac chua build:

```bash
./demo.sh build
```

Kiem tra script demo:

```bash
./demo.sh
```

Script se hien menu:

```text
1) world  - Gazebo + robot + sensors + manual teleop
2) slam   - SLAM Toolbox builds a 2D hospital map
3) nav    - A* path planning
4) follow - Lidar/camera follow-me
```

Luu y quan trong:

- Moi lan chi chay mot demo chinh.
- Truoc khi chuyen demo, bam `Ctrl+C` de tat demo dang chay.
- Khong chay `teleop`, `nav`, va `follow` cung luc vi tat ca deu publish `/cmd_vel`.

---

## 2. Lenh Ngan Can Nho

Chay menu:

```bash
./demo.sh
```

Chay tung phan:

```bash
./demo.sh world
./demo.sh slam
./demo.sh nav
./demo.sh follow
```

Lenh tuy chon neu may ao du manh:

```bash
./demo.sh nav-gui      # A* planning kem Gazebo GUI
./demo.sh nav-full     # Full Nav2 experimental, chi dung neu da test truoc
./demo.sh follow-gui   # Follow-me kem Gazebo GUI
./demo.sh follow-yolo  # Dung backend YOLO that, khoi dong rat cham tren VM 4 CPU
```

Cong cu phu:

```bash
./demo.sh teleop      # dieu khien ban phim
./demo.sh cmd         # xem lenh /cmd_vel
./demo.sh topics      # xem danh sach ROS topics
./demo.sh astar-test  # test A* khi nav dang chay
./demo.sh costmap     # xuat /global_costmap/costmap ra file PGM/YAML
./demo.sh build       # build lai workspace
```

---

## 3. Thu Tu Demo De De Trinh Bay

Nen demo theo thu tu:

1. `world`: gioi thieu moi truong, robot, cam bien.
2. `slam`: cho thay robot co the tao ban do.
3. `nav`: demo map va A* path planning.
4. `follow`: demo xe publish `/cmd_vel` de bam muc tieu.

Neu thoi gian it, uu tien:

1. `nav` + `astar-test`: de thay A* tinh duong ro rang.
2. `follow`: de thay xe bam muc tieu va publish `/cmd_vel`.

### Kich ban 10 phut khuyen nghi

Neu co 10 phut, demo theo nhip nay:

```text
00:00 - 01:30  World: Gazebo, map benh vien, robot, Lidar, camera, odom
01:30 - 03:30  SLAM: RViz hien /map, /scan, map dang duoc tao tu Lidar
03:30 - 06:00  Navigation: ./demo.sh nav + ./demo.sh astar-test de chung minh A*
06:00 - 08:30  Follow-me: ./demo.sh follow-debug de thay bbox va /cmd_vel
08:30 - 10:00  Tong ket: 4 module, topic ROS, noi ve nav-full la ban thu nghiem
```

Khong dua `nav-full` vao kich ban chinh. Neu muon show them, chi mo sau khi da xong 4 phan chinh va con thoi gian du.

---

## 4. Demo 1 - Mo Phong Gazebo, Robot Va Cam Bien

### Chay lenh

```bash
./demo.sh world
```

### Cho thay xem

Trong Gazebo/RViz, chi ra cac thanh phan:

- Moi truong benh vien.
- Xe lan thong minh.
- Lidar quet moi truong.
- Camera phia truoc.
- Robot co odometry.
- Robot nhan lenh van toc qua `/cmd_vel`.

### Dieu khien tay neu can

Mo terminal moi:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
./demo.sh teleop
```

Dung cac phim teleop:

```text
i: tien
,: lui
j: quay trai
l: quay phai
k: dung
```

### Cau noi goi y

> Day la moi truong mo phong xe lan thong minh trong benh vien bang Gazebo. Robot co differential drive, Lidar, camera va odometry. Cac du lieu nay duoc publish qua ROS 2 topics de dung cho SLAM, navigation va AI follow-me.

### Topic co the noi voi thay

```text
/scan             Lidar
/camera/image_raw Camera
/odom             Odometry
/cmd_vel          Lenh van toc dieu khien xe
/tf               Cay toa do robot
```

---

## 5. Demo 2 - SLAM Tao Ban Do Bang Lidar

### Chay lenh

Tat demo truoc bang `Ctrl+C`, sau do chay:

```bash
./demo.sh slam
```

### Dieu khien xe de quet map

Mo terminal moi:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
./demo.sh teleop
```

Lai xe cham qua hanh lang va cac phong. Khong nen quay qua nhanh vi map co the bi nhieu.

### Cho thay xem

Trong RViz:

- Ban do 2D dang duoc tao dan.
- Khi robot di chuyen, vung trang/xam/den tren map duoc cap nhat.
- Lidar `/scan` la nguon du lieu chinh.

### Cau noi goi y

> Phan nay dung SLAM Toolbox. Robot dung du lieu Lidar de vua uoc luong vi tri, vua tao ban do 2D cua moi truong benh vien. Khi xe di chuyen, ban do duoc cap nhat theo thoi gian thuc.

### Neu can luu map

Mo terminal moi:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/smart_wheelchair_navigation/maps/hospital_map
```

Thong thuong khi demo khong can luu lai map, vi du an da co san `hospital_map`.

---

## 6. Demo 3 - Navigation Va A*

### Chay lenh

Tat SLAM bang `Ctrl+C`, sau do chay:

```bash
./demo.sh nav
```

### Cho thay xem ban dau

Trong RViz, chi ra:

- Map benh vien 18.4m x 12.4m.
- Robot tren map.
- LaserScan.
- A* global planner.
- Global Path topic `/plan`.

Lenh `./demo.sh nav` hien la ban A* planning, phu hop demo 10 phut tren VM 4 CPU. Khong dung `nav-full` lam kich ban chinh: khi VM bi tai, RViz van co the hien duong di nhung controller/lifecycle cua Nav2 full timeout nen xe khong publish `/cmd_vel` de chay toi dich.

### Goal mau de demo duong A* ro

```text
Phong tren ben phai:  x = 7.5,  y = 2.5
Phong duoi ben trai:  x = -7.5, y = -2.5
```

### Cho thay xem

- Lenh `astar-test` tra ve duong di hop le tu start den goal.
- So luong waypoint `poses` va chieu dai duong `length_m`.
- Robot tranh tuong, cua phong va vat can tren global path.

### Cau noi goi y

> Phan navigation dung Nav2. Global planner la `NavfnPlanner` va trong config da bat `use_astar: true`, nen duong di toan cuc duoc tinh bang A*. Em co the kiem chung truc tiep bang action `ComputePathToPose` qua lenh `astar-test`.

### Kiem tra A* bang terminal

Khi `./demo.sh nav` dang chay, mo terminal moi:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
./demo.sh astar-test
```

Ket qua mong doi:

```text
A* path accepted
poses: ...
length_m: ...
start: (0.00, 0.00)
goal: (7.50, 2.50)
```

### Lay Costmap Cua Navigation

Costmap chinh cua demo A* nam o topic:

```text
/global_costmap/costmap
```

Cach lay costmap:

Terminal 1:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
./demo.sh nav
```

Doi RViz/map len on dinh, sau do mo Terminal 2:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
./demo.sh costmap
```

File se duoc luu trong:

```text
demo_outputs/costmaps/
```

Moi lan chay se tao 3 file:

```text
global_costmap_costmap_YYYYMMDD_HHMMSS.pgm   Anh costmap
global_costmap_costmap_YYYYMMDD_HHMMSS.yaml  Metadata resolution/origin
global_costmap_costmap_YYYYMMDD_HHMMSS.txt   Thong ke cell free/inflated/occupied/unknown
```

Neu muon lay local costmap cua `nav-full`, khi `nav-full` dang chay va da active:

```bash
./demo.sh costmap /local_costmap/costmap
```

### Neu thay yeu cau xe tu chay den goal

Chi dung phan nay khi may ao du manh hon, tot nhat 6-8 CPU, va da test truoc ngay demo:

```bash
./demo.sh nav-full
```

Sau do moi dung **Nav2 Goal** trong RViz. Neu chi thay duong di ma xe khong chay, do la full Nav2 controller/lifecycle chua on dinh tren VM hien tai; quay lai `./demo.sh nav` + `./demo.sh astar-test` de demo A*.

Neu van muon test `nav-full`, cho den khi terminal co dung dong nay roi moi bam goal:

```text
[lifecycle_manager_navigation]: Managed nodes are active
```

Khong nham voi dong `lifecycle_manager_localization`, vi localization active chua co nghia la controller/planner cua Nav2 full da san sang.

---

## 7. Demo 4 - Follow-Me Bang Camera Va Lidar

### Chay lenh

Tat Nav2 bang `Ctrl+C`, sau do chay:

```bash
./demo.sh follow
```

Lenh mac dinh nay dung backend `sim_scan` de demo on dinh tren VM 4 CPU. Node `human_tracker` bam theo actor nguoi trong Gazebo, tao bbox trong camera frame, tinh khoang cach va publish `/cmd_vel`.

### Cho thay xem

- Actor nguoi trong Gazebo.
- Xe phat hien muc tieu truoc mat bang backend follow-me.
- Xe tu publish `/cmd_vel`.
- Xe tien/cham lai de giu khoang cach voi nguoi.

### Neu muon dung YOLO that

Chay:

```bash
./demo.sh follow-yolo
```

Tren VM hien tai, backend YOLO that khoi dong rat cham: Torch/Ultralytics co the mat khoang 90-120 giay moi nap xong. Chi dung lenh nay khi co thoi gian cho model load hoac khi VM da cap 6-8 CPU.

### Neu muon xem camera debug

Chay:

```bash
./demo.sh follow-debug
```

Khi do co the xem khung hinh camera va bbox muc tieu.

Neu camera debug thay bbox nam vao tuong thay vi actor, dung `Ctrl+C` tat demo roi chay lai `./demo.sh follow-debug`. Backend hien tai da duoc chinh de bam actor mo phong thay vi lay vat can Lidar gan nhat, nen khong nen khoa vao tuong nua.

### Cau noi goi y

> Phan follow-me dung node `human_tracker` de ket hop camera va Lidar. Backend mac dinh duoc toi uu cho Gazebo/VM de demo on dinh; neu chay `follow-yolo` thi node dung YOLOv8 de nhan dien nguoi tu camera. Sau do Lidar duoc dung de uoc luong khoang cach va tinh `/cmd_vel`.

### Topic co the noi voi thay

```text
/camera/image_raw  Anh dau vao cho backend follow-me/YOLO tuy chon
/scan              Lidar de uoc luong khoang cach
/cmd_vel           Lenh dieu khien xe
```

---

## 8. Tom Tat 4 Chuc Nang De Noi Voi Thay

Co the noi ngan gon:

> Du an cua em gom 4 module demo chinh. Thu nhat la mo phong Gazebo voi robot, benh vien va cam bien. Thu hai la SLAM dung Lidar de tao ban do. Thu ba la Navigation dung Nav2 planner, trong do global planner dung A* de tim duong. Thu tu la follow-me ket hop camera/Lidar de tinh lenh `/cmd_vel`; neu bat backend YOLO thi camera dung YOLOv8 de nhan dien nguoi.

Neu thay hoi "thuat toan nao duoc dung?", tra loi:

```text
SLAM: SLAM Toolbox
Global planning: A* trong NavfnPlanner
Local control tuy chon: DWB local planner/controller trong ./demo.sh nav-full
Follow-me: camera/Lidar target tracking
AI detection tuy chon: YOLOv8 qua ./demo.sh follow-yolo
Sensor fusion: Camera bbox + Lidar distance
```

---

## 9. Loi Thuong Gap Khi Demo

### Gazebo mo cham

Doi them 20-60 giay. May ao VirtualBox co the cham.

Neu RViz bi loi OpenGL trong VirtualBox, co the bat software OpenGL thu cong:

```bash
SMART_WHEELCHAIR_SOFTWARE_GL=1 ./demo.sh nav
```

Neu demo Nav2 full bi cham, lifecycle timeout, hoac RViz co path nhung xe khong di, dung lenh mac dinh `./demo.sh nav` de uu tien toc do A*.

### Khong thay robot trong RViz

Kiem tra Fixed Frame la `map` hoac `odom`. Neu dang o mode `world`, RViz co the dung `odom`; neu mode `nav`, dung `map`.

### Khong thay ket qua A* khi demo Nav

Kiem tra:

1. `./demo.sh nav` van dang chay.
2. Da mo terminal moi de chay `./demo.sh astar-test`.
3. Cho them 10-20 giay neu Gazebo/RViz dang khoi dong cham.
4. Goal khong nam trong tuong/vat can.

Chay lai:

```bash
./demo.sh astar-test
```

### Follow-me chay cham

Lenh mac dinh `./demo.sh follow` da toi uu de chay on dinh tren VM. Neu chay `./demo.sh follow-yolo`, YOLO tren may ao co the cham 90-120 giay. Cho model load xong.

Neu can, chay khong debug:

```bash
./demo.sh follow
```

Chi dung debug khi can xem camera:

```bash
./demo.sh follow-debug
```

### Muon dung demo hien tai

Bam:

```text
Ctrl+C
```

Neu mo them terminal teleop/cmd thi cung tat cac terminal phu do.

---

## 10. Checklist Truoc Khi Len Demo

Truoc khi gap thay, chay nhanh:

```bash
cd /home/cambitzero/XeTuHanh/final/Smart-Wheelchair
./demo.sh build
./demo.sh topics
```

Sau do test nhanh:

```bash
./demo.sh nav
```

Neu map/robot len va `./demo.sh astar-test` tra ve `A* path accepted`, bam `Ctrl+C`.

Test follow:

```bash
./demo.sh follow
```

Neu thay log `Da phat hien nguoi, dang publish /cmd_vel.` va xe co `/cmd_vel`, bam `Ctrl+C`.

Den luc demo that, chi can chay theo thu tu:

```bash
./demo.sh world
./demo.sh slam
./demo.sh nav
./demo.sh follow
```
