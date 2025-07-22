import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import math

STATE_PATROL = 'PATROL'
STATE_AVOID = 'AVOID'
STATE_STOP = 'STOP'

class PatrolAndAvoid(Node):
    def __init__(self):
        super().__init__('patrol_and_avoid')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/patrol_status', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.control_sub = self.create_subscription(String, '/patrol_control', self.control_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.pose = None
        self.yaw = 0.0
        self.min_distance = float('inf')

        self.state = STATE_STOP
        self.motion_state = 'TURN'
        self.current_target_idx = 0

        self.target_directions = [math.radians(0), math.radians(-90), math.radians(180), math.radians(90)]

        self.forward_speed = 0.3
        self.turn_speed = 0.4
        self.forward_length = 2.0
        self.yaw_tolerance = 0.03
        self.forward_correction_gain = 2.5
        self.forward_time_target = self.forward_length / self.forward_speed
        self.forward_count_limit = int(self.forward_time_target / 0.1)
        self.forward_count = 0

        self.avoid_start_pose = None
        self.has_moved_away = False
        self.obstacle_threshold = 0.35

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if r > 0.05]
        if valid_ranges:
            self.min_distance = min(valid_ranges)

    def control_callback(self, msg):
        if msg.data == 'start':
            if self.state == STATE_STOP:
                self.get_logger().info('🟢 순찰 시작')
                self.status_pub.publish(String(data='🟢 순찰 시작'))
                self.state = STATE_PATROL
                self.motion_state = 'TURN'
                self.current_target_idx = 0
        elif msg.data == 'stop':
            self.get_logger().info('🛑 순찰 정지')
            self.status_pub.publish(String(data='🛑 정지됨'))
            self.state = STATE_STOP
            self.publisher_.publish(Twist())

    def control_loop(self):
        if self.pose is None:
            return

        twist = Twist()

        if self.state == STATE_STOP:
            self.publisher_.publish(twist)
            return

        if self.state == STATE_PATROL and self.min_distance < self.obstacle_threshold:
            self.state = STATE_AVOID
            self.get_logger().warn("🚧 장애물 감지 → 회피 시작")
            self.status_pub.publish(String(data='🚧 장애물 감지 중'))
            self.avoid_start_pose = self.pose
            self.has_moved_away = False
            return

        if self.state == STATE_PATROL:
            target_yaw = self.target_directions[self.current_target_idx]
            yaw_error = self.normalize_angle(target_yaw - self.yaw)

            if self.motion_state == 'TURN':
                if abs(yaw_error) > self.yaw_tolerance:
                    twist.angular.z = self.turn_speed * (yaw_error / abs(yaw_error))
                else:
                    self.motion_state = 'FORWARD'
                    self.forward_count = 0
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

RED = '\\033[91m'
GREEN = '\\033[92m'
YELLOW = '\\033[93m'
RESET = '\\033[0m'

class SquarePatrolWithObstacle(Node):
        def __init__(self):
            super().__init__('square_patrol_with_obstacle_node')
            self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
            self.timer = self.create_timer(0.1, self.control_loop)

            self.pose = None
            self.yaw = 0.0
            self.min_distance = float('inf')

            self.state = 'PATROL'
            self.motion_state = 'TURN'
            self.current_target_idx = 0

            self.target_directions = [
                math.radians(0),
                math.radians(-90),
                math.radians(180),
                math.radians(90)
            ]

            self.forward_speed = 0.5
            self.turn_speed = 0.4
            self.forward_length = 2.0
            self.yaw_tolerance = 0.03
            self.forward_correction_gain = 2.5

            self.forward_time_target = self.forward_length / self.forward_speed
            self.forward_count_limit = int(self.forward_time_target / 0.1)
            self.forward_count = 0

            self.obstacle_threshold = 0.35

            self.active = False
            self.avoid_callback = None
            self.last_status = ""

        def log_once(self, color, msg):
            if self.last_status != msg:
                self.get_logger().info(f"{color}{msg}{RESET}")
                self.last_status = msg

        def odom_callback(self, msg):
            self.pose = msg.pose.pose.position
            orientation_q = msg.pose.pose.orientation
            _, _, self.yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])

        def scan_callback(self, msg):
            valid_ranges = [r for r in msg.ranges if r > 0.05]
            if valid_ranges:
                self.min_distance = min(valid_ranges)

        def control_loop(self):
            if not self.active or self.pose is None:
                return

            twist = Twist()

            if self.state == 'PATROL':
                if self.min_distance < self.obstacle_threshold:
                    self.log_once(YELLOW, "🚧 장애물 감지 → 회피로 전환")
                    self.active = False  # 중단
                    if self.avoid_callback:
                        self.avoid_callback()
                    return

                self.log_once(GREEN, "🚶 순찰 중")
                target_yaw = self.target_directions[self.current_target_idx]
                yaw_error = self.normalize_angle(target_yaw - self.yaw)

                if self.motion_state == 'TURN':
                    if abs(yaw_error) > self.yaw_tolerance:
                        twist.angular.z = self.turn_speed * (yaw_error / abs(yaw_error))
                    else:
                        self.motion_state = 'FORWARD'
                        self.forward_count = 0

                elif self.motion_state == 'FORWARD':
                    if self.forward_count < self.forward_count_limit:
                        twist.linear.x = self.forward_speed
                        yaw_error = self.normalize_angle(target_yaw - self.yaw)
                        twist.angular.z = self.forward_correction_gain * yaw_error
                        self.forward_count += 1
                    else:
                        self.motion_state = 'TURN'
                        self.current_target_idx = (self.current_target_idx + 1) % 4

            self.publisher_.publish(twist)

        def normalize_angle(self, angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle

class ObstacleCircleAvoider(Node):
        def __init__(self, on_done_callback=None):
            super().__init__('obstacle_circle_avoider')
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
            self.timer = self.create_timer(0.05, self.control_loop)

            self.state = 'move'
            self.start_time = self.get_clock().now()

            self.linear_speed = 0.2
            self.angular_speed = 0.2
            self.closest = float('inf')
            self.circle_step = 0
            self.done_callback = on_done_callback
            self.active = False

        def activate(self):
            self.active = True
            self.state = 'turn_right'
            self.start_time = self.get_clock().now()
            self.circle_step = 0
            self.get_logger().info("🛑 장애물 회피 루틴 시작")

        def scan_callback(self, msg):
            front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
            self.closest = min(front_ranges)

        def control_loop(self):
            if not self.active:
                return

            twist = Twist()
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            quarter_turn_time = (math.pi / 2) / self.angular_speed
            circle_segment_time = quarter_turn_time + 0.15
            if self.state.startswith('circle_step_') or self.state in ['pause_left', 'pause_stop1', 'pause_right', 'pause_stop2', 'final_pause_left']:
                print('\a')  # 터미널에서 반복 부저음

            if self.state == 'turn_right':
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
                    else:
                        self.state = 'pause_left'
                        self.start_time = self.get_clock().now()

            elif self.state == 'pause_left':
                twist.angular.z = self.angular_speed
                if elapsed > quarter_turn_time:
                    self.state = 'pause_stop1'
                    self.start_time = self.get_clock().now()

            elif self.state == 'pause_stop1':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if elapsed > 1.5:
                    self.state = 'pause_right'
                    self.start_time = self.get_clock().now()

            elif self.state == 'pause_right':
                twist.angular.z = -self.angular_speed
                if elapsed > quarter_turn_time:
                    self.state = 'pause_stop2'
                    self.start_time = self.get_clock().now()

            elif self.state == 'pause_stop2':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if elapsed > 1.5:
                    self.circle_step += 1
                    self.state = f'circle_step_{self.circle_step}'
                    self.start_time = self.get_clock().now()

            elif self.state == 'final_pause_left':
                twist.angular.z = self.angular_speed
                if elapsed > quarter_turn_time:
                    self.state = 'stop'
                    self.start_time = self.get_clock().now()

            elif self.state == 'stop':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("✅ 회피 완료 → 순찰 재개")
                self.active = False
                if self.done_callback:
                    self.done_callback()

            self.cmd_pub.publish(twist)

def main(args=None):
        rclpy.init(args=args)

        patrol_node = SquarePatrolWithObstacle()
        avoid_node = ObstacleCircleAvoider(on_done_callback=lambda: setattr(patrol_node, 'active', True))

        # 서로 연결
        patrol_node.avoid_callback = lambda: avoid_node.activate()

        # 둘 다 등록
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(patrol_node)
        executor.add_node(avoid_node)

        patrol_node.active = True  # 시작 시 순찰

        try:
            executor.spin()
        except KeyboardInterrupt:
            print("종료됨")
        finally:
            patrol_node.destroy_node()
            avoid_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
        main()
