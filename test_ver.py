import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

# 색상 코드 정의
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RESET = '\033[0m'

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
            math.radians(0),    # 0도 (동쪽 또는 초기 방향)
            math.radians(-90),  # -90도 (남쪽)
            math.radians(180),  # 180도 (서쪽)
            math.radians(90)    # 90도 (북쪽)
        ]

        # --- 순찰 동작 관련 수치 조절 ---
        self.forward_speed = 0.5            # 직진 속도 (m/s)
                                            # - 이 값을 **높이면** 로봇이 더 **빠르게 직진**합니다.
                                            # - 이 값을 **낮추면** 로봇이 더 **천천히 직진**합니다.
                                            # - 너무 높으면 제어가 불안정해지거나 충돌 위험이 커질 수 있습니다.

        self.turn_speed = 0.4               # 회전 속도 (rad/s)
                                            # - 이 값을 **높이면** 로봇이 더 **빠르게 회전**합니다.
                                            # - 이 값을 **낮추면** 로봇이 더 **천천히 회전**합니다.
                                            # - 너무 높으면 오버슈트(목표 각도를 지나쳐 회전)할 수 있습니다.

        self.forward_length = 2.0           # 사각형의 한 변 길이 (미터)
                                            # - 이 값을 **변경하여** 순찰하는 **사각형의 크기를 조절**합니다.
                                            # - 예: 3.0으로 변경하면 한 변이 3미터인 사각형을 순찰합니다.

        self.yaw_tolerance = 0.03           # 목표 방향과의 허용 오차 (라디안). 이 값보다 작아지면 직진 시작
                                            # - 이 값을 **줄이면** 로봇이 더 **정확하게 방향을 맞춘 후 직진**합니다 (회전 시간이 길어질 수 있음).
                                            # - 이 값을 **늘리면** 로봇이 덜 정확하게 방향을 맞추고 **빨리 직진**합니다 (회전 시간이 짧아짐).

        self.forward_correction_gain = 2.5  # 직진 중 방향 보정 게인. 이 값이 클수록 목표 방향으로 더 빠르게 보정
                                            # - 이 값이 **크면** 직진 중 경로 이탈 시 더 **강하게 원래 경로로 돌아오려** 합니다 (지그재그 움직임이 심해질 수 있음).
                                            # - 이 값이 **작으면** 보정 반응이 느려져 경로가 **더 느슨하게 유지**됩니다.
                                            # - 너무 크면 진동이 발생할 수 있습니다 (좌우로 흔들림).

        self.obstacle_threshold = 0.6       # 장애물 감지 임계값 (미터). 이 거리보다 가까우면 회피 시작
                                            # - 이 값을 **줄이면** 로봇이 장애물에 더 **가깝게 접근한 후 회피**합니다.
                                            # - 이 값을 **늘리면** 로봇이 장애물로부터 더 **멀리 떨어져서 회피를 시작**합니다.
                                            # - 0.6은 비교적 안전한 값입니다.

        # 사각형 한 변을 이동하기 위한 시간 및 카운터 계산 (이 값들은 위에서 설정한 'forward_speed'와 'forward_length'에 따라 자동 계산됩니다.)
        self.forward_time_target = self.forward_length / self.forward_speed
        self.forward_count_limit = int(self.forward_time_target / 0.1) # 0.1초마다 제어 루프가 돌기 때문에 카운트 제한 계산
        self.forward_count = 0

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
        # Lidar 스캔 범위를 전방 +-30도로 제한
        front_left_ranges = msg.ranges[0:31] # 인덱스 0 포함
        front_right_ranges = msg.ranges[-30:] # 인덱스 -30부터 끝까지

        relevant_ranges = front_left_ranges + front_right_ranges

        valid_ranges = [r for r in relevant_ranges if r > 0.05] # 0.05는 Lidar의 최소 유효 거리입니다.
        if valid_ranges:
            self.min_distance = min(valid_ranges)
        else:
            self.min_distance = float('inf')

    def control_loop(self):
        if not self.active or self.pose is None:
            return

        twist = Twist()

        if self.state == 'PATROL':
            if self.min_distance < self.obstacle_threshold:
                self.log_once(YELLOW, "🚧 장애물 감지 → 회피로 전환")
                self.active = False
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
                    # 장애물 거리에 따른 속도 조절
                    # obstacle_threshold보다 약간 더 먼 거리에서부터 속도 감소 시작
                    safe_distance = self.obstacle_threshold * 1.5 # 예: 0.6 * 1.5 = 0.9m

                    if self.min_distance < safe_distance:
                        # 거리에 비례하여 속도 감소. min_distance가 0.05(Lidar 최소)에 가까워질수록 속도가 0에 가까워짐
                        speed_factor = max(0.0, (self.min_distance - 0.05) / (safe_distance - 0.05))
                        twist.linear.x = self.forward_speed * speed_factor
                    else:
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

        # --- 원형 회피 동작 관련 수치 조절 ---
        self.linear_speed = 0.2             # 회피 중 직진 속도 (m/s)
                                            # - 이 값을 **높이면** 회피 동작이 더 **빠르게 진행**됩니다.
                                            # - 이 값을 **낮추면** 회피 동작이 더 **천천히 진행**됩니다.
        self.angular_speed = 0.2            # 회피 중 회전 속도 (rad/s)
                                            # - 이 값을 **높이면** 로봇이 더 **빠르게 회전하며 장애물을 돕니다**.
                                            # - 이 값을 **낮추면** 로봇이 더 **천천히 회전하며 장애물을 돕니다**.
                                            # - 너무 높으면 회피 경로가 불안정해질 수 있습니다.

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
        valid_ranges = [r for r in msg.ranges if r > 0.05]
        if valid_ranges:
            self.closest = min(valid_ranges)
        else:
            self.closest = float('inf')

    def control_loop(self):
        if not self.active:
            return

        twist = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # --- 회피 동작의 타이밍 조절 변수 ---
        # 이 변수들을 조절하여 회피 동작의 각 단계별 시간과 속도를 변경할 수 있습니다.
        quarter_turn_time = (math.pi / 2) / self.angular_speed # 90도 회전에 걸리는 시간
                                                                # - angular_speed에 따라 자동 계산됩니다.
        circle_segment_time = quarter_turn_time + 0.15          # 원호 이동 시 추가 시간 (직진 요소)
                                                                # - 이 값을 **늘리면** 원형 회피 경로의 **직진 구간이 길어집니다**.
                                                                # - 이 값을 **줄이면** 원형 회피 경로의 **직진 구간이 짧아집니다**.
        pause_duration = 1.5                                    # 회피 중 정지 시간 (초)
                                                                # - 이 시간을 **늘리면** 로봇이 각 회피 단계 사이에서 **더 오래 정지**합니다.
                                                                # - 이 시간을 **줄이면** 정지 시간이 짧아져 **더 빠르게 다음 단계로 진행**합니다.


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
            if elapsed > pause_duration:
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
            if elapsed > pause_duration:
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
