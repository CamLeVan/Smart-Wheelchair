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
        # Sub vào camera của Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Sub vào Lidar cho Sensor Fusion
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        # Pub vận tốc điều khiển xe
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        
        # Load model YOLOv8 Nano (Có sẵn thuật toán Tracking tương đương DeepSORT)
        self.get_logger().info("Đang tải mô hình YOLOv8 Nano & Tracking...")
        self.model = YOLO('yolov8n.pt')
        
        self.target_class = 0 # 'person'
        self.tracking = False
        self.latest_scan = None
        
        # Biến để nhớ ID của người đang theo dõi (tránh nhầm người khác)
        self.target_id = None

    def scan_callback(self, msg):
        self.latest_scan = msg

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
                # Tính góc của người so với camera (FOV giả định 60 độ = 1.047 rad)
                angle_rad = (person_center_x - image_center_x) * (1.047 / width)
                
                # Tìm index tia Laser quét tại góc đó
                scan = self.latest_scan
                # Giới hạn góc nằm trong khoảng của Lidar
                if scan.angle_min <= angle_rad <= scan.angle_max:
                    idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
                    # Lấy trung bình khoảng cách của 5 tia xung quanh để tăng độ chính xác
                    idx_min = max(0, idx - 2)
                    idx_max = min(len(scan.ranges) - 1, idx + 2)
                    valid_ranges = [r for r in scan.ranges[idx_min:idx_max] if not math.isinf(r) and not math.isnan(r)]
                    if valid_ranges:
                        distance_to_person = sum(valid_ranges) / len(valid_ranges)
            
            # --- ĐIỀU KHIỂN P-CONTROLLER ---
            # Quay xe hướng về người
            error_x = image_center_x - person_center_x
            twist_msg.angular.z = float(error_x * 0.005)
            
            # Tiến lùi bằng LaserScan thay vì chiều cao Box
            if distance_to_person > 0:
                target_distance = 1.2 # Giữ khoảng cách an toàn 1.2 mét
                error_d = distance_to_person - target_distance
                twist_msg.linear.x = float(error_d * 0.5) # Kp = 0.5 cho khoảng cách thực
                cv2.putText(cv_image, f"Dist: {distance_to_person:.2f}m", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            else:
                # Fallback nếu Lidar mất dữ liệu
                twist_msg.linear.x = 0.0
            
            twist_msg.linear.x = max(min(twist_msg.linear.x, 0.5), -0.5)
            twist_msg.angular.z = max(min(twist_msg.angular.z, 1.0), -1.0)
            
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
