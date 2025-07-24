import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_square)
        self.state = 'forward'
        self.edge_count = 0
        self.start_time = self.get_clock().now()

    def move_square(self):
        twist = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.state == 'forward':
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            if elapsed > 10.0:  # 2미터 직진 (0.2m/s * 10초 = 2m)
                self.state = 'turn'
                self.start_time = self.get_clock().now()

        elif self.state == 'turn':
            twist.linear.x = 0.0
            twist.angular.z = -math.pi / 4  # 반대 방향으로 45도/s 회전
            if elapsed > 2.0:  # 90도 회전 → 2초간 회전
                self.edge_count += 1
                self.state = 'forward'
                self.start_time = self.get_clock().now()

                if self.edge_count >= 4:
                    self.edge_count = 0  # 다시 시작 (무한 반복)

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  
