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

        self.state = 'move'
        self.start_time = self.get_clock().now()

        # 회전 반지름이 1m가 되도록 설정 (v = rw → r=1, v=0.2 ⇒ w=0.2)
        self.linear_speed = 0.2
        self.angular_speed = 0.2

        self.closest = float('inf')

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
        self.closest = min(front_ranges)

        if self.state == 'move' and self.closest < 0.5:
            self.get_logger().info("🛑 장애물 감지 → 회피 시작")
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
            twist.angular.z = -math.pi / 2  # 시계방향 회전
            if elapsed > 1.0:
                self.state = 'circle'
                self.start_time = self.get_clock().now()

        elif self.state == 'circle':
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            duration = (2 * math.pi) / self.angular_speed  # 원 한 바퀴 도는 시간
            if elapsed > duration:
                self.state = 'stop_before_final_turn'
                self.start_time = self.get_clock().now()

        elif self.state == 'stop_before_final_turn':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if elapsed > 1.0:
                self.state = 'final_left_turn'
                self.start_time = self.get_clock().now()
                self.get_logger().info("🌀 회피 종료 → 원점에서 왼쪽 90도 회전")

        elif self.state == 'final_left_turn':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            if elapsed > 1.0:
                self.state = 'stop'
                self.get_logger().info("✅ 최종 회전 완료 → 정지")

        elif self.state == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

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
