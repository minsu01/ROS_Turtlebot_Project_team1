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
        self.timer = self.create_timer(0.05, self.control_loop)

        self.state = 'move'
        self.start_time = self.get_clock().now()

        self.linear_speed = 0.2
        self.angular_speed = 0.2

        self.closest = float('inf')
        self.circle_step = 0

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

        quarter_turn_time = (math.pi / 2) / self.angular_speed
        circle_segment_time = quarter_turn_time + 0.15  #각도 미세 조정!!

        if self.state == 'move':
            twist.linear.x = self.linear_speed

        elif self.state == 'turn_right':
            twist.angular.z = -self.angular_speed
            if elapsed > quarter_turn_time:
                self.state = 'circle_step_0'
                self.start_time = self.get_clock().now()

        elif self.state.startswith('circle_step_'):
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            if elapsed > circle_segment_time:
                if self.circle_step == 3:
                    self.state = 'final_pause_left'
                    self.start_time = self.get_clock().now()
                    self.get_logger().info("🌀 마지막 궤적 완료 → 왼쪽 회전 시작")
                else:
                    self.state = 'pause_left'
                    self.start_time = self.get_clock().now()

        elif self.state == 'pause_left':
            twist.angular.z = self.angular_speed
            if elapsed > quarter_turn_time:
                self.state = 'pause_stop1'
                self.start_time = self.get_clock().now()

        elif self.state == 'pause_stop1':
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            if elapsed > 1.5: #물체쪽 바라보는 시간
                self.state = 'pause_right'
                self.start_time = self.get_clock().now()

        elif self.state == 'pause_right':
            twist.angular.z = -self.angular_speed
            if elapsed > quarter_turn_time:
                self.state = 'pause_stop2'
                self.start_time = self.get_clock().now()

        elif self.state == 'pause_stop2':
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            if elapsed > 1.5: #원위치 후 이동 시간
                self.circle_step += 1
                self.state = f'circle_step_{self.circle_step}'
                self.start_time = self.get_clock().now()

        # 🔻 마지막 회전용 상태
        elif self.state == 'final_pause_left':
            twist.angular.z = self.angular_speed
            if elapsed > quarter_turn_time:
                self.state = 'stop'
                self.start_time = self.get_clock().now()
                self.get_logger().info("✅ 마지막 왼쪽 회전 완료 → 정지")

        elif self.state == 'stop':
            twist.angular.z = 0.0
            twist.linear.x = 0.0

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
