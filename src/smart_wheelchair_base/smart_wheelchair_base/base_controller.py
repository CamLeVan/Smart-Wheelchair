import rclpy
from rclpy.node import Node
import serial
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = math.cos(ai); si = math.sin(ai)
    cj = math.cos(aj); sj = math.sin(aj)
    ck = math.cos(ak); sk = math.sin(ak)
    cc = ci*ck; cs = ci*sk; sc = si*ck; ss = si*sk
    return [cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss]

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Kết nối Serial với Arduino (Cần đổi port trên mạch thực tế)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.1)
            self.get_logger().info("Đã kết nối Serial /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().warning(f"Không thể mở Serial: {e}. Đang chạy chế độ mô phỏng.")
            self.serial_port = None
        
        # Thông số vật lý của xe lăn
        self.wheel_radius = 0.165  # mét (bán kính bánh)
        self.wheel_base = 0.45     # mét (khoảng cách 2 bánh)
        self.ticks_per_rev = 360.0 # Số tick 1 vòng
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # Trạng thái Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        
        self.last_time = self.get_clock().now()
        
        # Sub lệnh vận tốc từ Nav2 / AI để gửi xuống Arduino
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Vòng lặp đọc Serial liên tục (20Hz)
        self.create_timer(0.05, self.read_serial)

    def cmd_vel_callback(self, msg):
        if self.serial_port is not None:
            # Chuyển V_linear, V_angular sang V_left, V_right
            v_l = msg.linear.x - (msg.angular.z * self.wheel_base / 2.0)
            v_r = msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
            
            # Gửi dạng PWM giả định (Thực tế Arduino có thể cần map sang PWM)
            cmd_str = f"V,{v_l*200:.1f},{v_r*200:.1f}\n" 
            self.serial_port.write(cmd_str.encode('utf-8'))

    def read_serial(self):
        if self.serial_port is None: return
        
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line.startswith("E,"):
                parts = line.split(',')
                if len(parts) >= 3:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])
                    
                    self.calculate_odometry(left_ticks, right_ticks)
        except Exception as e:
            pass

    def calculate_odometry(self, left_ticks, right_ticks):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0: return

        # Tính toán delta
        d_left = (left_ticks - self.prev_left_ticks) * self.m_per_tick
        d_right = (right_ticks - self.prev_right_ticks) * self.m_per_tick
        
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base
        
        # Tích phân vị trí (Euler integration)
        self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
        self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
        self.theta += d_theta
        
        # Vận tốc tức thời
        vx = d_center / dt
        vth = d_theta / dt
        self.last_time = current_time

        # Phát Transform (TF) odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Phát topic /odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
