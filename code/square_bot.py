#사각형으로 도는 코드 2m
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan # LaserScan은 더 이상 직접 사용되지 않지만, 만약을 위해 남겨둡니다.
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

# QoS 프로파일 임포트 (센서 데이터용, Lidar 같은 경우 중요)
from rclpy.qos import qos_profile_sensor_data

# 색상 코드 정의
GREEN = '\033[92m'
YELLOW = '\033[93m'
RESET = '\033[0m'

class PatrolRobotController(Node):
    def __init__(self):
        super().__init__('patrol_robot_controller_node')

        # ROS 2 Publisher & Subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # LaserScan 구독은 유지하되, 콜백 함수 내 로직은 제거 (사용하지 않음)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)


        # Control Loop Timer (전체 제어 로직을 0.1초마다 실행)
        self.control_loop_dt = 0.1 # 100ms
        self.timer = self.create_timer(self.control_loop_dt, self.control_loop)

        # Robot State Variables
        self.pose = None # 오도메트리 초기화 전까지 None
        self.yaw = 0.0

        # --- Main State Machine: 'INITIALIZING', 'PATROL' ---
        self.main_state = 'INITIALIZING' # 시작 시 초기화 대기
        self.patrol_motion_state = 'IDLE' # 순찰 내의 서브 상태: 'FORWARD', 'TURN', 'IDLE' (초기)
        self.current_patrol_idx = 0 # 현재 순찰 단계 인덱스

        # ⭐ 순찰을 위한 절대 목표 yaw 각도 설정
        # 초기 yaw를 기준으로 0, -90, -180, 90 (시계방향 90, 180, 270도) 회전 목표
        self.patrol_absolute_target_yaws = [
            0.0,
            0.0,
            0.0,
            0.0
        ]
        self._initial_yaw_offset = None # 로봇의 초기 yaw 값을 저장 (순찰 경로의 기준점)

        # Patrol Parameters
        self.patrol_forward_speed = 0.3            # 직진 속도 (m/s)
        self.patrol_turn_speed = 0.4               # 회전 속도 (rad/s)
        self.patrol_forward_length = 1.5           # 사각형의 한 변 길이 (미터)
        self.patrol_yaw_tolerance = 0.01           # 목표 방향과의 허용 오차 (라디안)
        self.patrol_forward_correction_gain = 3.5  # 직진 중 방향 보정 게인

        self.patrol_forward_time_target = self.patrol_forward_length / self.patrol_forward_speed
        self.patrol_forward_count_limit = int(self.patrol_forward_time_target / self.control_loop_dt)
        self.patrol_forward_count = 0

        # Data Initialization Flags & Logging
        self._odom_initialized = False
        self._scan_received = False # LaserScan은 사용하지 않지만, 기본 데이터 수신 확인을 위해 유지
        self._last_warn_time = self.get_clock().now() # 경고 메시지 쓰로틀링
        self.last_status_msg = "" # 이전에 출력된 로그 메시지 저장

        # For smooth acceleration/deceleration
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.linear_accel_limit = 0.5  # m/s^2
        self.angular_accel_limit = 1.0 # rad/s^2


    def log_once(self, color, msg):
        """이전과 동일한 메시지는 다시 로깅하지 않아 메시지 스팸을 방지합니다."""
        if self.last_status_msg != msg:
            self.get_logger().info(f"{color}{msg}{RESET}")
            self.last_status_msg = msg

    def odom_callback(self, msg):
        """오도메트리 데이터를 수신하여 로봇의 현재 위치와 방향(yaw)을 업데이트합니다."""
        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation

        _, _, current_absolute_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.yaw = current_absolute_yaw

        # Odom 초기화 및 첫 목표 yaw 설정
        if not self._odom_initialized:
            self._initial_yaw_offset = current_absolute_yaw
            # 첫 번째 목표 yaw는 초기 yaw에서 0도 회전한 방향입니다.
            self.patrol_absolute_target_yaws = [
                self.normalize_angle(self._initial_yaw_offset + math.radians(0)),
                self.normalize_angle(self._initial_yaw_offset + math.radians(-90)),  # 90도 우회전 (시계방향)
                self.normalize_angle(self._initial_yaw_offset + math.radians(-180)), # 180도 회전
                self.normalize_angle(self._initial_yaw_offset + math.radians(90))    # 270도 회전 (반시계방향)
            ]
            self.log_once(GREEN, f"🟢 Odom 초기화 완료. 초기 방향: {math.degrees(self._initial_yaw_offset):.2f}도.")
            self.log_once(GREEN, f"🟢 순찰 목표 방향 설정 완료: {[math.degrees(y) for y in self.patrol_absolute_target_yaws]}")
            self._odom_initialized = True
            # 초기화 완료 후 바로 순찰 모드로 전환하여 직진 시작
            self.main_state = 'PATROL'
            self.patrol_motion_state = 'FORWARD' # 첫 동작은 직진
            self.current_patrol_idx = 0 # 첫 번째 목표 방향 (초기 yaw)으로 시작
            self.log_once(GREEN, "🚶 초기 회전 없이 바로 직진 순찰 시작.")


    def scan_callback(self, msg):
        """Lidar 스캔 데이터를 수신하지만, 이 버전에서는 사용하지 않습니다."""
        # 이 버전에서는 장애물 회피 로직이 없으므로, scan 데이터는 사용되지 않습니다.
        # 하지만 _scan_received 플래그를 true로 설정하여 제어 루프가 시작되도록 합니다.
        if not self._scan_received:
            self._scan_received = True
            self.get_logger().info(f"{YELLOW}⚠️ Lidar Scan 데이터 수신 (순찰 전용 모드에서는 사용 안 함).{RESET}")


    def control_loop(self):
        current_time = self.get_clock().now()
        target_linear_x = 0.0
        target_angular_z = 0.0

        # --- 필수 데이터(Odom/Scan) 수신 대기 ---
        if not self._odom_initialized or not self._scan_received:
            if (current_time - self._last_warn_time).nanoseconds / 1e9 >= 5.0:
                self.get_logger().warn(f"{YELLOW}⚠️ 필수 데이터(Odom/Scan) 수신 대기 중... Odom: {self._odom_initialized}, Scan: {self._scan_received}{RESET}")
                self._last_warn_time = current_time
            target_linear_x = 0.0
            target_angular_z = 0.0

        else: # 데이터가 모두 수신된 경우
            # --- 메인 상태 기계: INITIALIZING, PATROL ---
            if self.main_state == 'INITIALIZING':
                pass # odom_callback에서 상태 해제

            elif self.main_state == 'PATROL':
                self.log_once(GREEN, "🚶 사각형 순찰 중")
                target_yaw = self.patrol_absolute_target_yaws[self.current_patrol_idx]
                yaw_error = self.normalize_angle(target_yaw - self.yaw)

                if self.patrol_motion_state == 'TURN':
                    if abs(yaw_error) > self.patrol_yaw_tolerance:
                        target_angular_z = self.patrol_turn_speed * (yaw_error / abs(yaw_error))
                        target_linear_x = 0.0
                    else:
                        self.patrol_motion_state = 'FORWARD'
                        self.patrol_forward_count = 0
                        target_angular_z = 0.0
                        self.log_once(GREEN, "▶️ 직진 시작")

                elif self.patrol_motion_state == 'FORWARD':
                    if self.patrol_forward_count < self.patrol_forward_count_limit:
                        target_linear_x = self.patrol_forward_speed
                        yaw_error = self.normalize_angle(target_yaw - self.yaw)
                        target_angular_z = self.patrol_forward_correction_gain * yaw_error
                        self.patrol_forward_count += 1
                    else:
                        self.patrol_motion_state = 'TURN'
                        self.current_patrol_idx = (self.current_patrol_idx + 1) % len(self.patrol_absolute_target_yaws)
                        self.log_once(GREEN, f"🏁 한 변 이동 완료. 다음 회전 준비 (다음 목표: {math.degrees(self.patrol_absolute_target_yaws[self.current_patrol_idx]):.2f}도)")

                # 추가적인 오류 또는 대기 상태는 순찰 루프에선 필요 없음

        # --- 속도 스무딩 로직 ---
        twist = Twist()

        # 선형 속도 스무딩
        delta_linear_x = target_linear_x - self.current_linear_x
        max_delta_linear = self.linear_accel_limit * self.control_loop_dt

        if abs(delta_linear_x) > max_delta_linear:
            twist.linear.x = self.current_linear_x + (max_delta_linear if delta_linear_x > 0 else -max_delta_linear)
        else:
            twist.linear.x = target_linear_x

        # 각속도 스무딩
        delta_angular_z = target_angular_z - self.current_angular_z
        max_delta_angular = self.angular_accel_limit * self.control_loop_dt

        if abs(delta_angular_z) > max_delta_angular:
            twist.angular.z = self.current_angular_z + (max_delta_angular if delta_angular_z > 0 else -max_delta_angular)
        else:
            twist.angular.z = target_angular_z

        # 다음 제어 주기를 위해 현재 속도 업데이트
        self.current_linear_x = twist.linear.x
        self.current_angular_z = twist.angular.z

        self.publisher_.publish(twist)

    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화합니다."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PatrolRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 종료됨 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
