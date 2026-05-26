import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math

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

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.camera_fov_rad = float(self.get_parameter('camera_fov_rad').value)
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

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
            
        # Pub vận tốc điều khiển xe
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.bridge = CvBridge()
        
        # Load model YOLOv8 Nano (Có sẵn thuật toán Tracking tương đương DeepSORT)
        model_path = self.get_parameter('model_path').value
        self.get_logger().info(f"Đang tải mô hình YOLO & Tracking: {model_path}")
        self.model = YOLO(model_path)
        
        self.target_class = 0 # 'person'
        self.tracking = False
        self.latest_scan = None
        
        # Biến để nhớ ID của người đang theo dõi (tránh nhầm người khác)
        self.target_id = None

    def scan_callback(self, msg):
        self.latest_scan = msg

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
        idx_min = max(0, idx - 2)
        idx_max = min(len(scan.ranges), idx + 3)

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
        return sum(valid_ranges) / len(valid_ranges)

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

        # Chạy YOLOv8 với tính năng TRACKING (Bộ lọc Kalman nội suy chuyển động)
        # Thông số persist=True giúp mô hình nhớ ID mục tiêu qua các frame
        results = self.model.track(cv_image, persist=True, verbose=False)
        
        best_box = None
        current_ids_in_frame = []
        
        if results and results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes
            ids = boxes.id.cpu().numpy().astype(int)
            
            for i, box in enumerate(boxes):
                cls = int(box.cls[0])
                if cls == self.target_class:
                    person_id = ids[i]
                    current_ids_in_frame.append(person_id)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Nếu chưa có mục tiêu, khóa vào người đầu tiên thấy
                    if self.target_id is None:
                        self.target_id = person_id
                        best_box = [x1, y1, x2, y2]
                        break
                    # Nếu đã có mục tiêu, chỉ bắt box của đúng ID đó
                    elif person_id == self.target_id:
                        best_box = [x1, y1, x2, y2]
                        break
                        
        # Reset ID nếu người đang theo dõi đi khuất
        if self.target_id is not None and self.target_id not in current_ids_in_frame:
            self.target_id = None
        
        twist_msg = Twist()
        
        if best_box:
            x1, y1, x2, y2 = best_box
            person_center_x = (x1 + x2) // 2
            
            # --- SENSOR FUSION (Giải quyết Bounding Box Fallacy) ---
            distance_to_person = -1.0 # Giá trị mặc định
            
            if self.latest_scan is not None:
                # Tính góc của người so với camera và map sang hệ góc LaserScan.
                # Cần thêm dấu âm (-) vì trong OpenCV X tăng về bên phải, nhưng trong ROS 2 góc quét bên phải là góc âm.
                angle_rad = -(person_center_x - image_center_x) * (self.camera_fov_rad / width)
                distance_to_person = self._distance_at_angle(angle_rad)
            
            # --- ĐIỀU KHIỂN P-CONTROLLER ---
            # Quay xe hướng về người
            error_x = image_center_x - person_center_x
            twist_msg.angular.z = float(error_x * 0.005)
            
            # Tiến lùi bằng LaserScan thay vì chiều cao Box
            if distance_to_person > 0:
                error_d = distance_to_person - self.target_distance
                twist_msg.linear.x = float(error_d * 0.5) # Kp = 0.5 cho khoảng cách thực
                cv2.putText(cv_image, f"Dist: {distance_to_person:.2f}m", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            else:
                # Fallback nếu Lidar mất dữ liệu
                twist_msg.linear.x = 0.0
            
            twist_msg.linear.x = self._clamp(twist_msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
            twist_msg.angular.z = self._clamp(twist_msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
            
            # Vẽ Box và ID (DeepSORT)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, f"Target ID: {self.target_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            self.tracking = True
        else:
            if self.tracking:
                self.tracking = False
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)
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
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
