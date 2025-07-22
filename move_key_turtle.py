import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import math

class StableSquarePatrol(Node):
    def __init__(self):
        super().__init__('stable_square_patrol')

        # 📢 ROS2 통신 관련
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_subscriber_ = self.create_subscription(String, '/toggle_patrol', self.command_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 📍 상태값 초기화
        self.pose = None
        self.yaw = 0.0
        self.state = 'STOP'
        self.motion_state = 'TURN'
        self.current_target_idx = 0

        # ✅ 사각형 경로 방향 (→ ↓ ← ↑)
        self.target_directions = [
            math.radians(0),
            math.radians(-90),
            math.radians(180),
            math.radians(90)
        ]

        # ✅ 사용자 조절 파라미터
        self.forward_speed = 0.7                 # 전진 속도 (m/s) - 너무 빠르면 궤도 벗어남
        self.turn_speed = 0.4                    # 회전 속도 (rad/s) - 너무 빠르면 overshoot
        self.forward_length = 2.0                # 한 변 길이 (m)
        self.yaw_tolerance = 0.02                # 회전 완료 허용 오차 (rad)
        self.forward_yaw_tolerance = 0.03        # 전진 중 yaw drift 허용치
        self.forward_correction_gain = 2.0       # 전진 중 drift 보정 강도

        # ✅ 계산용: 전진 시간(초)을 타이머 tick 수로 변환
        self.forward_time_target = self.forward_length / self.forward_speed
        self.forward_count_limit = int(self.forward_time_target / 0.1)
        self.forward_count = 0

        # 🔒 안정적 전환을 위한 상태 변수
        self.turn_hold_count = 0
        self.turn_hold_threshold = 4             # yaw 오차가 안정적으로 4번 연속 작아야 전진 전환
        self.forward_delay_counter = 0
        self.forward_delay_limit = 2             # 전진 전 0.2초 대기

    def command_callback(self, msg):
        if msg.data.lower() == 'start':
            self.state = 'RUN'
            self.get_logger().info("▶️ 순찰 시작")
        elif msg.data.lower() == 'stop':
            self.state = 'STOP'
            self.get_logger().info("⏸️ 순찰 정지")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def control_loop(self):
        if self.pose is None or self.state == 'STOP':
            self.publisher_.publish(Twist())
            return

        twist = Twist()
        target_yaw = self.target_directions[self.current_target_idx]
        yaw_error = self.normalize_angle(target_yaw - self.yaw)

        if self.motion_state == 'TURN':
            if abs(yaw_error) > self.yaw_tolerance:
                twist.angular.z = self.turn_speed * yaw_error / abs(yaw_error)
                self.turn_hold_count = 0
            else:
                self.turn_hold_count += 1
                twist.angular.z = 0.0
                if self.turn_hold_count >= self.turn_hold_threshold:
                    self.get_logger().info(f"\u2705 방향 정렬 완료 ({math.degrees(target_yaw):.1f}\u00b0)")
                    self.motion_state = 'FORWARD_PREP'
                    self.forward_delay_counter = 0

        elif self.motion_state == 'FORWARD_PREP':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.forward_delay_counter += 1
            if self.forward_delay_counter >= self.forward_delay_limit:
                self.motion_state = 'FORWARD'
                self.forward_count = 0

        elif self.motion_state == 'FORWARD':
            if self.forward_count < self.forward_count_limit:
                twist.linear.x = self.forward_speed
                yaw_error = self.normalize_angle(target_yaw - self.yaw)
                if abs(yaw_error) > self.forward_yaw_tolerance:
                    twist.angular.z = self.forward_correction_gain * yaw_error
                self.forward_count += 1
            else:
                # 전진 완료 후 재정렬
                yaw_error = self.normalize_angle(target_yaw - self.yaw)
                if abs(yaw_error) > self.yaw_tolerance:
                    self.get_logger().warn(f"\ud83d\udd27 Drift 감지! 각도 재정렬 중... ({math.degrees(yaw_error):.2f}\u00b0)")
                    self.motion_state = 'TURN'
                    self.turn_hold_count = 0
                else:
                    self.current_target_idx = (self.current_target_idx + 1) % len(self.target_directions)
                    self.motion_state = 'TURN'
                    self.turn_hold_count = 0

        self.publisher_.publish(twist)

    def normalize_angle(self, angle):
        """-\u03c0 ~ \u03c0로 angle 보정"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = StableSquarePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\ud83d\udeab 사용자 인터럽트로 종료됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()




# 순찰 시작
#ros2 topic pub /toggle_patrol std_msgs/String "data: 'start'"

# 순찰 정지
#ros2 topic pub /toggle_patrol std_msgs/String "data: 'stop'"
