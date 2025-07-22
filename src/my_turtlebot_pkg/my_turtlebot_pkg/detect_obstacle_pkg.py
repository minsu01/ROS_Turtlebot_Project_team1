import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleCircleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_circle_avoider')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 상태
        self.state = 'move'
        self.start_time = self.get_clock().now()

        # 매개변수
        self.linear_speed = 0.2
        self.angular_speed = 0.2  # 반  지름 1.0m를 만들기 위한 값

        self.closest = float('inf')

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
        self.closest = min(front_ranges)

        if self.state == 'move' and self.closest < 0.5:
            self.get_logger().info("🛑 장애물 감지됨 → 회피 시작")
            self.state = 'turn_right'
            self.start_time = self.get_clock().now()

    def control_loop(self):
        twist = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.state == 'move':
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        elif self.state == 'turn_right':
            twist.linear.x = 0.0
            twist.angular.z = -math.pi / 2  # 오른쪽 회전 (속도 단위 아님)
            if elapsed > 1.0:
                self.state = 'circle'
                self.start_time = self.get_clock().now()

        elif self.state == 'circle':
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            duration = (2 * math.pi) / self.angular_speed  # 원 1바퀴 도는 시간
            if elapsed > duration:
                self.state = 'turn_left'
                self.start_time = self.get_clock().now()

        elif self.state == 'turn_left':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            if elapsed > 1.0:
                self.state = 'move'
                self.start_time = self.get_clock().now()
                self.get_logger().info("✅ 회피 완료, 직진 재개")

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCircleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


