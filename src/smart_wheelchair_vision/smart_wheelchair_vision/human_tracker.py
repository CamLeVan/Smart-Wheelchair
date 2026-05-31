import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import math
import os

os.environ.setdefault('TORCH_CPP_LOG_LEVEL', 'ERROR')
os.environ.setdefault('TORCHDYNAMO_DISABLE', '1')

class HumanTracker(Node):
    def __init__(self):
        super().__init__('human_tracker')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('camera_fov_rad', 1.085)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('show_debug_view', False)
        self.declare_parameter('angular_deadband_px', 40.0)
        self.declare_parameter('target_bbox_width_ratio', 0.45)
        self.declare_parameter('vision_backend', 'sim_scan')
        self.declare_parameter('sim_target_start_x', 2.2)
        self.declare_parameter('sim_target_end_x', 3.4)
        self.declare_parameter('sim_target_y', 0.0)
        self.declare_parameter('sim_target_period', 20.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.camera_fov_rad = float(self.get_parameter('camera_fov_rad').value)
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.show_debug_view = bool(self.get_parameter('show_debug_view').value)
        self.angular_deadband_px = float(self.get_parameter('angular_deadband_px').value)
        self.target_bbox_width_ratio = float(self.get_parameter('target_bbox_width_ratio').value)
        self.vision_backend = str(self.get_parameter('vision_backend').value).strip().lower()
        self.sim_target_start_x = float(self.get_parameter('sim_target_start_x').value)
        self.sim_target_end_x = float(self.get_parameter('sim_target_end_x').value)
        self.sim_target_y = float(self.get_parameter('sim_target_y').value)
        self.sim_target_period = float(self.get_parameter('sim_target_period').value)

        # Sub vào camera của Gazebo
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
            
        # Sub vào Lidar cho Sensor Fusion
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # Pub vận tốc điều khiển xe
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.bridge = CvBridge()
        
        self.model = None
        self.hog = None

        if self.vision_backend == 'yolo':
            self._load_yolo_model()
        elif self.vision_backend == 'opencv_hog':
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.get_logger().info("Dang dung OpenCV HOG backend cho follow-me.")
        else:
            if self.vision_backend not in ('sim_scan', 'auto'):
                self.get_logger().warning(
                    f"Backend '{self.vision_backend}' khong hop le. Dung sim_scan de demo on dinh."
                )
            self.vision_backend = 'sim_scan'
            self.get_logger().info("Dang dung Gazebo sim_scan backend cho follow-me.")
        
        self.target_class = 0 # 'person'
        self.tracking = False
        self.latest_scan = None
        self.latest_odom = None
        self.estimated_target_distance = -1.0
        
        # Biến để nhớ ID của người đang theo dõi (tránh nhầm người khác)
        self.target_id = None

    def _load_yolo_model(self):
        model_path = self.get_parameter('model_path').value
        try:
            self.get_logger().info(f"Dang tai mo hinh YOLO & Tracking: {model_path}")
            from ultralytics import YOLO

            try:
                import torch
                if hasattr(torch.backends, 'nnpack'):
                    torch.backends.nnpack.enabled = False
            except ImportError:
                pass

            self.model = YOLO(model_path)
        except Exception as exc:
            self.get_logger().warning(
                f"Khong tai duoc YOLO ({exc}). Chuyen sang sim_scan de demo khong bi dung."
            )
            self.vision_backend = 'sim_scan'
            self.model = None

    def scan_callback(self, msg):
        self.latest_scan = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def _angle_in_scan_range(self, angle_rad, scan):
        two_pi = 2.0 * math.pi
        candidates = [angle_rad, angle_rad + two_pi, angle_rad - two_pi]

        # Also try the canonical [-pi, pi] value for lidars configured that way.
        normalized = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
        candidates.extend([normalized, normalized + two_pi, normalized - two_pi])

        for candidate in candidates:
            if scan.angle_min <= candidate <= scan.angle_max:
                return candidate
        return None

    def _distance_at_angle(self, angle_rad):
        scan = self.latest_scan
        if scan is None or not scan.ranges or scan.angle_increment == 0.0:
            return -1.0

        scan_angle = self._angle_in_scan_range(angle_rad, scan)
        if scan_angle is None:
            return -1.0

        idx = int((scan_angle - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(len(scan.ranges) - 1, idx))
        window = 10
        idx_min = max(0, idx - window)
        idx_max = min(len(scan.ranges), idx + window + 1)

        valid_ranges = []
        for distance in scan.ranges[idx_min:idx_max]:
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min:
                continue
            if scan.range_max > scan.range_min and distance > scan.range_max:
                continue
            valid_ranges.append(distance)

        if not valid_ranges:
            return -1.0
        return min(valid_ranges)

    def _detect_person_with_yolo(self, cv_image):
        results = self.model.track(cv_image, persist=True, verbose=False)
        best_box = None
        current_ids_in_frame = []

        if results and results[0].boxes:
            boxes = results[0].boxes
            ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else None
            best_score = -1.0

            for i, box in enumerate(boxes):
                cls = int(box.cls[0])
                if cls == self.target_class:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    if ids is None:
                        width = max(0, x2 - x1)
                        height = max(0, y2 - y1)
                        confidence = float(box.conf[0]) if box.conf is not None else 1.0
                        score = width * height * confidence
                        if score > best_score:
                            best_score = score
                            best_box = [x1, y1, x2, y2]
                        continue

                    person_id = ids[i]
                    current_ids_in_frame.append(person_id)

                    # Nếu chưa có mục tiêu, khóa vào người đầu tiên thấy
                    if self.target_id is None:
                        self.target_id = person_id
                        best_box = [x1, y1, x2, y2]
                        break
                    # Nếu đã có mục tiêu, chỉ bắt box của đúng ID đó
                    elif person_id == self.target_id:
                        best_box = [x1, y1, x2, y2]
                        break

            # Some ultralytics/ByteTrack frames return person boxes before an ID is assigned.
            # Keep the demo responsive by following the strongest person box until IDs appear.
            if ids is None and best_box is not None:
                self.target_id = 0
                current_ids_in_frame = [0]

        return best_box, current_ids_in_frame

    def _detect_person_with_hog(self, cv_image):
        if self.hog is None:
            return None, []

        boxes, _ = self.hog.detectMultiScale(
            cv_image,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05
        )
        if len(boxes) == 0:
            return None, []

        x, y, w, h = max(boxes, key=lambda rect: rect[2] * rect[3])
        self.target_id = 1
        return [int(x), int(y), int(x + w), int(y + h)], [1]

    def _sim_target_xy(self):
        period = max(self.sim_target_period, 0.1)
        half_period = period / 2.0
        now_sec = self.get_clock().now().nanoseconds / 1e9
        phase = now_sec % period

        if phase <= half_period:
            ratio = phase / half_period
        else:
            ratio = 1.0 - ((phase - half_period) / half_period)

        x = self.sim_target_start_x + ratio * (self.sim_target_end_x - self.sim_target_start_x)
        return x, self.sim_target_y

    @staticmethod
    def _yaw_from_odom(odom_msg):
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _detect_person_with_sim_target(self, width, height):
        if self.latest_odom is None:
            return None, []

        robot = self.latest_odom.pose.pose.position
        robot_yaw = self._yaw_from_odom(self.latest_odom)
        target_x, target_y = self._sim_target_xy()
        dx_world = target_x - robot.x
        dy_world = target_y - robot.y

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        dx_robot = cos_yaw * dx_world + sin_yaw * dy_world
        dy_robot = -sin_yaw * dx_world + cos_yaw * dy_world

        if dx_robot <= 0.2:
            return None, []

        angle = math.atan2(dy_robot, dx_robot)
        half_fov = self.camera_fov_rad / 2.0
        if abs(angle) > half_fov:
            return None, []

        distance = math.hypot(dx_robot, dy_robot)
        self.estimated_target_distance = distance

        person_center_x = int(width / 2.0 - angle * width / self.camera_fov_rad)
        person_center_x = max(0, min(width - 1, person_center_x))
        box_height = int(self._clamp(height * 0.9 / max(distance, 0.35), 70, height * 0.8))
        box_width = int(self._clamp(box_height * 0.32, 35, width * 0.35))
        box_center_y = int(height * 0.55)

        x1 = max(0, person_center_x - box_width // 2)
        x2 = min(width - 1, person_center_x + box_width // 2)
        y1 = max(0, box_center_y - box_height // 2)
        y2 = min(height - 1, box_center_y + box_height // 2)

        self.target_id = 1
        return [x1, y1, x2, y2], [1]

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        height, width, _ = cv_image.shape
        image_center_x = width // 2

        if self.model is not None:
            # Chạy YOLOv8 với tính năng TRACKING (Bộ lọc Kalman nội suy chuyển động).
            # Thông số persist=True giúp mô hình nhớ ID mục tiêu qua các frame.
            best_box, current_ids_in_frame = self._detect_person_with_yolo(cv_image)
        elif self.hog is not None:
            best_box, current_ids_in_frame = self._detect_person_with_hog(cv_image)
        else:
            self.estimated_target_distance = -1.0
            best_box, current_ids_in_frame = self._detect_person_with_sim_target(width, height)
                        
        # Reset ID nếu người đang theo dõi đi khuất
        if self.target_id is not None and self.target_id not in current_ids_in_frame:
            self.target_id = None
        
        twist_msg = Twist()
        
        if best_box:
            x1, y1, x2, y2 = best_box
            person_center_x = (x1 + x2) // 2
            
            # --- SENSOR FUSION (Giải quyết Bounding Box Fallacy) ---
            distance_to_person = -1.0 # Giá trị mặc định
            
            if self.estimated_target_distance > 0:
                distance_to_person = self.estimated_target_distance
            elif self.latest_scan is not None:
                # Tính góc của người so với camera và map sang hệ góc LaserScan.
                # Cần thêm dấu âm (-) vì trong OpenCV X tăng về bên phải, nhưng trong ROS 2 góc quét bên phải là góc âm.
                angle_rad = -(person_center_x - image_center_x) * (self.camera_fov_rad / width)
                distance_to_person = self._distance_at_angle(angle_rad)
            
            # --- ĐIỀU KHIỂN P-CONTROLLER ---
            # Quay xe hướng về người
            error_x = image_center_x - person_center_x
            if abs(error_x) > self.angular_deadband_px:
                twist_msg.angular.z = float(error_x * 0.003)
            else:
                twist_msg.angular.z = 0.0
            
            # Tiến lùi bằng LaserScan thay vì chiều cao Box
            if distance_to_person > 0:
                error_d = distance_to_person - self.target_distance
                twist_msg.linear.x = float(error_d * 0.5) # Kp = 0.5 cho khoảng cách thực
                cv2.putText(cv_image, f"Dist: {distance_to_person:.2f}m", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            else:
                # Fallback if the Gazebo actor mesh is visible to YOLO but sparse for Lidar.
                bbox_width_ratio = max(0.0, min(1.0, (x2 - x1) / float(width)))
                error_w = self.target_bbox_width_ratio - bbox_width_ratio
                twist_msg.linear.x = float(error_w * 0.8)
            
            twist_msg.linear.x = self._clamp(twist_msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
            twist_msg.angular.z = self._clamp(twist_msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
            
            # Vẽ Box và ID (DeepSORT)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, f"Target ID: {self.target_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            if not self.tracking:
                self.get_logger().info("Da phat hien nguoi, dang publish /cmd_vel.")
            self.tracking = True
        else:
            if self.tracking:
                self.get_logger().info("Mat muc tieu, dung xe.")
                self.tracking = False
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)
        if self.show_debug_view:
            cv2.imshow("Sensor Fusion & Tracking", cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HumanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        show_debug_view = getattr(node, 'show_debug_view', False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if show_debug_view:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
