import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time
import copy # deepcopy를 위한 import

# ROS 서비스 메시지 임포트
from std_srvs.srv import SetBool # 수동 정지/재개 서비스 메시지 타입

# QoS 프로파일 임포트 (센서 데이터용)
from rclpy.qos import qos_profile_sensor_data

# 색상 코드 정의
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m' # 수동 제어 메시지용
CYAN = '\033[96m'
MAGENTA = '\033[95m'
RESET = '\033[0m'

class SquarePatrolWithObstacleStop(Node):
    def __init__(self):
        super().__init__('square_patrol_with_obstacle_stop_node')

        # ROS 2 Publisher & Subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)

        # ROS 2 서비스 서버 생성 (수동 정지/재개용)
        self.manual_stop_service = self.create_service(SetBool, '/manual_stop_control', self.handle_manual_stop_request)
        self.get_logger().info(f"{CYAN}💡 ROS Service '/manual_stop_control' (std_srvs/srv/SetBool) 준비 완료. 사용법: ros2 service call /manual_stop_control std_srvs/srv/SetBool 'data: true/false'{RESET}")


        # Control Loop Timer (전체 제어 로직을 0.1초마다 실행)
        self.control_loop_dt = 0.1 # 10Hz
        self.timer = self.create_timer(self.control_loop_dt, self.control_loop)

        # Robot State Variables
        self.pose = None # 오도메트리 초기화 전까지 None
        self.yaw = 0.0
        self.min_distance = float('inf') # Lidar에서 감지된 최소 거리 (전방)
        self.lidar_range_min = 0.0
        self.lidar_range_max = 0.0

        # --- Main State Machine: 'INITIALIZING', 'PATROL', 'STOPPED' ---
        self.main_state = 'INITIALIZING' # 시작 시 초기화 대기
        self.patrol_motion_state = 'IDLE' # 순찰 내의 서브 상태: 'FORWARD', 'TURN', 'IDLE' (초기)
        self.current_patrol_idx = 0 # 현재 순찰 단계 인덱스 (0: 첫 직진, 1: 90도 회전 후 직진, ...)

        # 순찰을 위한 절대 목표 yaw 각도 설정 (초기 yaw를 기준으로 설정됨)
        # 0도, -90도, -180도, 90도 (사각형 시계 방향)
        self.patrol_absolute_target_yaws = [
            0.0, 0.0, 0.0, 0.0
        ]
        self._initial_yaw_offset = None # 로봇의 초기 yaw 값을 저장 (순찰 경로의 기준점)

        # Patrol Parameters
        self.patrol_forward_speed = 0.3              # 직진 속도 (m/s)
        self.patrol_turn_speed = 0.4                 # 회전 속도 (rad/s)
        self.patrol_forward_length = 2.0             # 사각형의 한 변 길이 (미터) - ⭐이것은 초기 설정값, 재개 시 변경됨
        self.patrol_yaw_tolerance = 0.01             # 목표 방향과의 허용 오차 (라디안)
        self.patrol_forward_correction_gain = 3.0    # 직진 중 방향 보정 게인 (PID P-gain)

        # 직진 거리 제어 변수 (정지 및 재개 시 남은 거리 계산에 사용)
        self.segment_start_pose = None               # 현재 직진 구간의 시작 위치 (x, y)
        self.segment_start_yaw = 0.0                 # 현재 직진 구간의 시작 방향 (yaw)
        self.current_segment_traveled_distance = 0.0 # 현재 직진 구간에서 실제로 이동한 거리
        self.target_segment_length = self.patrol_forward_length # ⭐현재 구간의 목표 길이

        # --- 장애물 감지 및 수동 정지 관련 변수 ---
        self.obstacle_detection_threshold = 0.6 # 장애물 감지 임계값 (미터)
        self.manual_stop_requested = False # 수동 정지 요청 플래그 (True: 수동 정지, False: 수동 재개)
        self.stopped_reason = "" # 정지 이유 ('', 'obstacle', 'manual')

        # 정지 시 순찰 상태를 저장할 변수들
        self.saved_patrol_state = {
            'main_state': 'INITIALIZING',
            'patrol_motion_state': 'IDLE',
            'current_patrol_idx': 0,
            'segment_start_pose': None,
            'segment_start_yaw': 0.0,
            'current_segment_traveled_distance': 0.0,
            'target_segment_length': 0.0, # ⭐추가: 정지 당시의 목표 길이 저장
            'stop_pose': None, # ⭐추가: 정지 당시 위치
            'stop_yaw': 0.0 # ⭐추가: 정지 당시 yaw
        }

        # Data Initialization Flags & Logging
        self._odom_initialized = False
        self._scan_received = False
        self._last_warn_time = self.get_clock().now()
        self.last_status_msg = "" # log_once 함수용 변수

        # For smooth acceleration/deceleration
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.linear_accel_limit = 0.5 # m/s^2
        self.angular_accel_limit = 1.0 # rad/s^2

        # 센서 데이터 수신 타임스탬프 (꺼짐 현상 진단용)
        self._last_odom_time = self.get_clock().now()
        self._last_scan_time = self.get_clock().now()
        self.sensor_timeout_seconds = 10.0 # 10초 이상 센서 데이터가 없으면 경고 후 종료 고려

        # Lidar 데이터 유효성 플래그
        self.is_lidar_data_valid = False


    def log_once(self, color, msg):
        """이전과 동일한 메시지는 다시 로깅하지 않아 메시지 스팸을 방지합니다."""
        if self.last_status_msg != msg:
            self.get_logger().info(f"{color}{msg}{RESET}")
            self.last_status_msg = msg

    def odom_callback(self, msg):
        """오도메트리 데이터를 수신하여 로봇의 현재 위치와 방향(yaw)을 업데이트하고, 이동 거리를 계산합니다."""
        self._last_odom_time = self.get_clock().now()

        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation

        _, _, current_absolute_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.yaw = current_absolute_yaw

        # 현재 직진 구간에서 이동한 거리 계산 (PATROL 상태의 FORWARD일 때만 업데이트)
        # 중요: segment_start_pose가 현재 구간의 시작점이어야 정확한 이동 거리가 나옴
        if self.main_state == 'PATROL' and self.patrol_motion_state == 'FORWARD' and self.segment_start_pose:
            self.current_segment_traveled_distance = math.sqrt(
                (self.pose.x - self.segment_start_pose.x)**2 +
                (self.pose.y - self.segment_start_pose.y)**2
            )

        # Odom 초기화 및 첫 목표 yaw 설정
        if not self._odom_initialized:
            self._initial_yaw_offset = current_absolute_yaw
            # 초기 오프셋을 기준으로 사각형 순찰 방향 설정
            self.patrol_absolute_target_yaws = [
                self.normalize_angle(self._initial_yaw_offset + math.radians(0)),   # 1번 코너 (시작점)
                self.normalize_angle(self._initial_yaw_offset + math.radians(-90)),  # 2번 코너
                self.normalize_angle(self._initial_yaw_offset + math.radians(-180)), # 3번 코너
                self.normalize_angle(self._initial_yaw_offset + math.radians(90))   # 4번 코너
            ]
            self.log_once(GREEN, f"🟢 Odom 초기화 완료. 초기 방향: {math.degrees(self._initial_yaw_offset):.2f}도.")
            self.log_once(GREEN, f"🟢 순찰 목표 방향 설정 완료: {[math.degrees(y) for y in self.patrol_absolute_target_yaws]}도")
            self._odom_initialized = True

            # 초기화가 완료되면 바로 순찰 상태로 진입 (첫 동작은 직진)
            self.main_state = 'PATROL'
            self.patrol_motion_state = 'FORWARD'
            self.current_patrol_idx = 0 # 0번 인덱스 (첫 직진)

            # 첫 직진 구간 시작점 기록 및 목표 길이 설정
            if self.pose:
                self.segment_start_pose = copy.deepcopy(self.pose) # 깊은 복사로 값만 저장
                self.segment_start_yaw = self.yaw
            self.current_segment_traveled_distance = 0.0
            self.target_segment_length = self.patrol_forward_length # ⭐초기 설정값 사용
            self.log_once(GREEN, "🚶 초기 회전 없이 바로 직진 순찰 시작. (1번 코너 방향)")


    def scan_callback(self, msg):
        """Lidar 스캔 데이터를 수신하여 전방 최소 거리를 업데이트합니다."""
        self._last_scan_time = self.get_clock().now()
        self._scan_received = True

        self.lidar_range_min = msg.range_min
        self.lidar_range_max = msg.range_max

        # 전방 60도 범위 (좌우 30도)의 데이터를 확인
        front_ranges = list(msg.ranges[0:31]) + list(msg.ranges[-30:])

        # 유효한 범위의 데이터만 필터링 (inf, nan, 0, min/max range 밖 값 제외)
        valid_ranges = [
            r for r in front_ranges
            if r > self.lidar_range_min and r < self.lidar_range_max
            and not math.isinf(r) and not math.isnan(r) and r > 0.01
        ]

        # ⭐변경: 수동 정지 요청 상태일 때는 Lidar 값을 강제로 오버라이드하지 않습니다.
        # 대신, control_loop에서 manual_stop_requested 상태에 따라 min_distance를 강제 조정합니다.
        if not self.manual_stop_requested: # 수동 정지 중이 아닐 때만 Lidar 값 반영
            if valid_ranges:
                self.min_distance = min(valid_ranges)
                self.is_lidar_data_valid = True
            else:
                self.min_distance = float('inf') # 유효한 데이터가 없으면 무한대 거리로 설정
                self.is_lidar_data_valid = False

                # 경고 메시지 반복 출력 방지
                current_time_sec = self.get_clock().now().nanoseconds / 1e9
                last_warn_time_sec = self._last_warn_time.nanoseconds / 1e9
                if (current_time_sec - last_warn_time_sec) >= 5.0:
                     self.get_logger().warn(
                         f"{YELLOW}⚠️ Lidar 전방에서 유효한 데이터 없음! (모두 inf/nan/0 또는 범위 밖). "
                         f"Lidar 범위: {self.lidar_range_min:.2f}~{self.lidar_range_max:.2f}m. "
                         f"전방 원본 데이터 샘플 (첫 10개): {front_ranges[:10]}{RESET}"
                     )
                     self._last_warn_time = self.get_clock().now()

        # 장애물 감지/해제 시점의 메시지 출력 로직 (1회만 출력) - Lidar 값에 관계없이 메시지 자체는 출력
        if self.main_state == 'PATROL' and self.min_distance < self.obstacle_detection_threshold:
            if self.stopped_reason != "obstacle" and not self.manual_stop_requested: # 수동 정지 중일 때는 이 메시지 안 띄움
                self.log_once(RED, f"🛑 장애물 감지! 로봇 정지. 전방 거리: {self.min_distance:.2f}m")
        elif self.main_state == 'STOPPED' and self.stopped_reason == "obstacle" and self.min_distance >= self.obstacle_detection_threshold:
            self.log_once(GREEN, f"✅ 장애물 사라짐! 로봇 재개 준비. 전방 거리: {self.min_distance:.2f}m")


    def handle_manual_stop_request(self, request, response):
        """
        /manual_stop_control 서비스 요청을 처리하여 로봇의 수동 정지 상태를 설정합니다.
        request.data = True: 수동 정지 요청
        request.data = False: 수동 재개 요청
        """
        if request.data: # 수동 정지 요청 (True)
            if not self.manual_stop_requested:
                self.manual_stop_requested = True
                self.log_once(BLUE, "⏸️ 서비스 요청: 수동 정지 활성화. 로봇이 정지합니다.")
                self.stopped_reason = "manual" # 수동 정지로 명확히 설정

                # ⭐추가: 수동 정지 시 장애물이 있는 것처럼 Lidar 값 강제 설정
                self.min_distance = 0.0 # 강제로 장애물 감지 임계값보다 작게 설정
                self.is_lidar_data_valid = True # Lidar 데이터가 유효한 것으로 간주 (멈추기 위해)
                self.log_once(YELLOW, f"⚠️ 수동 정지 요청에 따라 Lidar 거리 강제 설정 (0.0m).{RESET}")

                # 정지 시 현재 순찰 상태를 저장
                self.saved_patrol_state['main_state'] = self.main_state
                self.saved_patrol_state['patrol_motion_state'] = self.patrol_motion_state
                self.saved_patrol_state['current_patrol_idx'] = self.current_patrol_idx
                # 중요: 정지 당시의 로봇 위치와 방향을 저장 (재개 시 새 출발점으로 사용)
                self.saved_patrol_state['stop_pose'] = copy.deepcopy(self.pose)
                self.saved_patrol_state['stop_yaw'] = self.yaw
                # 이전에 이동한 거리와 현재 구간의 목표 길이도 저장
                self.saved_patrol_state['current_segment_traveled_distance'] = self.current_segment_traveled_distance
                self.saved_patrol_state['target_segment_length'] = self.target_segment_length # ⭐추가: 현재 목표 길이 저장

                # 로그 메시지를 위한 남은 거리 계산
                remaining_dist_for_log = self.target_segment_length - self.current_segment_traveled_distance
                self.get_logger().info(f"{BLUE}📦 정지 전 순찰 상태 저장 완료: "
                                        f"코너 {self.current_patrol_idx+1}, "
                                        f"서브상태 '{self.patrol_motion_state}', "
                                        f"남은거리: {remaining_dist_for_log:.2f}m (원래 길이: {self.patrol_forward_length:.2f}m){RESET}")

            response.message = f"수동 정지 요청 받음. 현재 전방 거리: {self.min_distance:.2f}m"
        else: # 수동 재개 요청 (False)
            if self.manual_stop_requested:
                self.manual_stop_requested = False
                self.log_once(BLUE, "▶️ 서비스 요청: 수동 정지 비활성화. 로봇이 재개 조건을 확인합니다.")

                # ⭐핵심 변경: 수동 재개 시 stopped_reason 초기화 및 min_distance를 무한대로 설정⭐
                self.stopped_reason = "" # 재개 시 멈춘 이유를 초기화하여 재개 조건에 걸리지 않도록 함
                self.min_distance = float('inf') # 장애물이 사라진 것처럼 처리하여 재개 유도
                self.is_lidar_data_valid = True # Lidar 데이터가 유효한 것으로 간주 (강제 재개를 위함)

                self.log_once(YELLOW, f"⚠️ 수동 재개 요청에 따라 '멈춘 이유' 초기화 및 Lidar 거리 강제 설정 (inf).{RESET}")

                # 저장된 순찰 상태를 복원하여 재개 준비
                self.current_patrol_idx = self.saved_patrol_state['current_patrol_idx']
                self.patrol_motion_state = self.saved_patrol_state['patrol_motion_state']

                # ⭐핵심 수정: 재개 시 segment_start_pose와 segment_start_yaw를 정지했던 지점으로 설정
                # 그리고 current_segment_traveled_distance는 0으로 초기화
                # 또한, target_segment_length를 '남은 거리'로 업데이트합니다.
                if self.saved_patrol_state['stop_pose']:
                    self.segment_start_pose = copy.deepcopy(self.saved_patrol_state['stop_pose'])
                    self.segment_start_yaw = self.saved_patrol_state['stop_yaw']
                    self.current_segment_traveled_distance = 0.0 # ⭐ 이 값을 0으로 설정하여 재개 지점부터 다시 거리 계산

                    # ⭐진짜 중요한 변경: 남은 거리를 새로운 목표 길이로 설정
                    remaining_dist = self.saved_patrol_state['target_segment_length'] - self.saved_patrol_state['current_segment_traveled_distance']
                    # 음수 방지 및 최소 거리 설정 (이미 거의 다 왔다면 0으로 처리)
                    self.target_segment_length = max(0.0, remaining_dist)

                    self.log_once(BLUE, f"🔄 순찰 상태 복원 및 재개 지점 설정 완료. "
                                        f"원래 목표 {self.saved_patrol_state['target_segment_length']:.2f}m 중 "
                                        f"{self.saved_patrol_state['current_segment_traveled_distance']:.2f}m 이동, "
                                        f"이제부터 {self.target_segment_length:.2f}m 더 이동합니다.{RESET}")
                else:
                    # 만약 stop_pose가 설정되지 않았다면 (예: 초기화 중 정지), 기존 로직대로
                    self.segment_start_pose = copy.deepcopy(self.saved_patrol_state['segment_start_pose'])
                    self.segment_start_yaw = self.saved_patrol_state['segment_start_yaw']
                    self.current_segment_traveled_distance = self.saved_patrol_state['current_segment_traveled_distance']
                    self.target_segment_length = self.saved_patrol_state['target_segment_length']

                # 복원된 정보를 바탕으로 로그 메시지 출력 (실제로 이동할 남은 거리)
                log_remaining_dist = self.target_segment_length - self.current_segment_traveled_distance
                self.log_once(BLUE, f"🔄 순찰 상태 복원 완료. (코너 {self.current_patrol_idx+1}, 서브상태 '{self.patrol_motion_state}', 최종 남은 이동 거리: {log_remaining_dist:.2f}m){RESET}")

            response.message = f"수동 재개 요청 받음. 현재 전방 거리: {self.min_distance:.2f}m"

        response.success = True
        return response


    def control_loop(self):
        current_time = self.get_clock().now()
        target_linear_x = 0.0
        target_angular_z = 0.0

        # 센서 데이터 타임아웃 검사
        time_since_last_odom = (current_time - self._last_odom_time).nanoseconds / 1e9
        time_since_last_scan = (current_time - self._last_scan_time).nanoseconds / 1e9

        if time_since_last_odom > self.sensor_timeout_seconds or \
           time_since_last_scan > self.sensor_timeout_seconds:
            self.get_logger().error(f"{RED}❌ 치명적 오류: {self.sensor_timeout_seconds}초 이상 센서 데이터 미수신! Odom: {time_since_last_odom:.2f}s, Scan: {time_since_last_scan:.2f}s. 노드를 종료합니다.{RESET}")
            raise SystemExit("Sensor data timeout, exiting node.")

        # --- 필수 데이터(Odom/Scan) 수신 대기 ---
        if not self._odom_initialized or not self._scan_received or self.pose is None:
            if (current_time - self._last_warn_time).nanoseconds / 1e9 >= 5.0:
                self.get_logger().warn(f"{YELLOW}⚠️ 필수 데이터(Odom/Scan/Pose) 수신 대기 중... Odom 초기화: {self._odom_initialized}, Scan 수신: {self._scan_received}, Pose 유효: {self.pose is not None}{RESET}")
                self._last_warn_time = current_time
            # 데이터 미수신 시 속도를 0으로 즉시 설정하고 리턴
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            self.publisher_.publish(Twist())
            return

        # --- 메인 상태 머신 전환 로직 ---
        prev_main_state = self.main_state

        # ⭐변경: 수동 정지 요청이 있을 때 min_distance를 강제로 0으로 설정하여 장애물 정지 효과⭐
        if self.manual_stop_requested:
            # 수동 정지 요청이 들어오면 Lidar 값에 관계없이 강제로 장애물이 있는 것처럼 만듦
            if self.main_state != 'STOPPED': # 처음 수동 정지 상태로 진입할 때만 로그 출력
                self.log_once(BLUE, "⏸️ 수동 정지 명령 활성화! 장애물을 만난 것처럼 로봇을 정지시킵니다.")
            # 이 부분의 min_distance, is_lidar_data_valid 설정은 handle_manual_stop_request에서 이미 처리됨

            # 메인 상태를 STOPPED로 전환
            if self.main_state != 'STOPPED':
                self.main_state = 'STOPPED'
                self.stopped_reason = "manual" # 수동 정지로 명확히 설정
                # 장애물 정지 시에도 현재 순찰 상태를 저장
                self.saved_patrol_state['main_state'] = self.main_state
                self.saved_patrol_state['patrol_motion_state'] = self.patrol_motion_state
                self.saved_patrol_state['current_patrol_idx'] = self.current_patrol_idx
                # 중요: 정지 당시의 로봇 위치를 저장 (재개 시 새 출발점으로 사용)
                self.saved_patrol_state['stop_pose'] = copy.deepcopy(self.pose)
                self.saved_patrol_state['stop_yaw'] = self.yaw
                # 이전에 이동한 거리와 현재 구간의 목표 길이도 저장
                self.saved_patrol_state['current_segment_traveled_distance'] = self.current_segment_traveled_distance
                self.saved_patrol_state['target_segment_length'] = self.target_segment_length # ⭐현재 목표 길이 저장

                # 로그 메시지를 위한 남은 거리 계산
                remaining_dist_for_log = self.target_segment_length - self.current_segment_traveled_distance
                self.get_logger().info(f"{RED}📦 장애물 감지로 정지 전 순찰 상태 저장 완료: "
                                        f"코너 {self.current_patrol_idx+1}, "
                                        f"서브상태 '{self.patrol_motion_state}', "
                                        f"남은거리: {remaining_dist_for_log:.2f}m (원래 길이: {self.patrol_forward_length:.2f}m){RESET}")

        # 2. 장애물 감지 시 정지 (수동 정지 요청이 없을 때만, 유효한 Lidar 데이터가 있을 때만)
        # 이제 이 로직은 오직 '수동 정지'가 아닐 때만 발동합니다.
        elif self.main_state == 'PATROL' and self.is_lidar_data_valid and self.min_distance < self.obstacle_detection_threshold:
            if self.main_state != 'STOPPED':
                self.main_state = 'STOPPED'
                self.stopped_reason = "obstacle" # 장애물 정지로 명확히 설정
                # 장애물 정지 시에도 현재 순찰 상태를 저장
                self.saved_patrol_state['main_state'] = self.main_state
                self.saved_patrol_state['patrol_motion_state'] = self.patrol_motion_state
                self.saved_patrol_state['current_patrol_idx'] = self.current_patrol_idx
                # 중요: 정지 당시의 로봇 위치를 저장 (재개 시 새 출발점으로 사용)
                self.saved_patrol_state['stop_pose'] = copy.deepcopy(self.pose)
                self.saved_patrol_state['stop_yaw'] = self.yaw
                # 이전에 이동한 거리와 현재 구간의 목표 길이도 저장
                self.saved_patrol_state['current_segment_traveled_distance'] = self.current_segment_traveled_distance
                self.saved_patrol_state['target_segment_length'] = self.target_segment_length # ⭐현재 목표 길이 저장

                # 로그 메시지를 위한 남은 거리 계산
                remaining_dist_for_log = self.target_segment_length - self.current_segment_traveled_distance
                self.get_logger().info(f"{RED}📦 장애물 감지로 정지 전 순찰 상태 저장 완료: "
                                        f"코너 {self.current_patrol_idx+1}, "
                                        f"서브상태 '{self.patrol_motion_state}', "
                                        f"남은거리: {remaining_dist_for_log:.2f}m (원래 길이: {self.patrol_forward_length:.2f}m){RESET}")

        # 3. 정지 상태에서 재개 조건 확인
        elif self.main_state == 'STOPPED':
            # is_obstacle_clear는 (수동 재개 시 inf로 강제 설정되므로 항상 True) 또는 실제 장애물이 없을 때 True
            is_obstacle_clear = (self.min_distance >= self.obstacle_detection_threshold)

            # 재개 조건:
            # 1) 수동 정지 요청이 없어야 하고 (not self.manual_stop_requested) AND
            # 2) (현재 멈춘 이유가 "manual"이거나, 또는 ""(수동 재개 시) 또는 "obstacle"인데 장애물이 치워졌을 때)
            if not self.manual_stop_requested and \
               (self.stopped_reason == "manual" or self.stopped_reason == "" or (self.stopped_reason == "obstacle" and is_obstacle_clear)):

                self.main_state = 'PATROL'
                self.stopped_reason = "" # 재개 후에는 정지 이유 초기화
                self.log_once(GREEN, f"▶️ 정지 해제! (수동 재개 혹은 장애물 클리어). 순찰 재개.")

                # 저장된 순찰 상태를 복원하여 재개
                self.current_patrol_idx = self.saved_patrol_state['current_patrol_idx']
                self.patrol_motion_state = self.saved_patrol_state['patrol_motion_state']

                # ⭐핵심 수정: 재개 시 segment_start_pose와 segment_start_yaw를 정지했던 지점으로 설정
                # 그리고 current_segment_traveled_distance는 0으로 초기화
                # 또한, target_segment_length를 '남은 거리'로 업데이트합니다.
                if self.saved_patrol_state['stop_pose']:
                    self.segment_start_pose = copy.deepcopy(self.saved_patrol_state['stop_pose'])
                    self.segment_start_yaw = self.saved_patrol_state['stop_yaw']
                    self.current_segment_traveled_distance = 0.0 # ⭐ 이 값을 0으로 설정하여 재개 지점부터 다시 거리 계산

                    # ⭐진짜 중요한 변경: 남은 거리를 새로운 목표 길이로 설정
                    remaining_dist = self.saved_patrol_state['target_segment_length'] - self.saved_patrol_state['current_segment_traveled_distance']
                    # 음수 방지 및 최소 거리 설정 (이미 거의 다 왔다면 0으로 처리)
                    self.target_segment_length = max(0.0, remaining_dist)

                    self.log_once(BLUE, f"🔄 순찰 상태 복원 및 재개 지점 설정 완료. "
                                        f"원래 목표 {self.saved_patrol_state['target_segment_length']:.2f}m 중 "
                                        f"{self.saved_patrol_state['current_segment_traveled_distance']:.2f}m 이동, "
                                        f"이제부터 {self.target_segment_length:.2f}m 더 이동합니다.{RESET}")
                else:
                    # 만약 stop_pose가 설정되지 않았다면 (예: 초기화 중 정지), 기존 로직대로
                    self.segment_start_pose = copy.deepcopy(self.saved_patrol_state['segment_start_pose'])
                    self.segment_start_yaw = self.saved_patrol_state['segment_start_yaw']
                    self.current_segment_traveled_distance = self.saved_patrol_state['current_segment_traveled_distance']
                    self.target_segment_length = self.saved_patrol_state['target_segment_length']

                # 복원된 정보를 바탕으로 로그 메시지 출력 (실제로 이동할 남은 거리)
                log_remaining_dist = self.target_segment_length - self.current_segment_traveled_distance
                self.log_once(BLUE, f"🔄 순찰 상태 복원 완료. (코너 {self.current_patrol_idx+1}, 서브상태 '{self.patrol_motion_state}', 최종 남은 이동 거리: {log_remaining_dist:.2f}m){RESET}")

            else:
                reason_log = ""
                if self.stopped_reason == "manual":
                    reason_log = "수동 정지 활성화됨"
                elif self.stopped_reason == "obstacle":
                    reason_log = f"장애물 감지됨 ({self.min_distance:.2f}m)"
                    if not self.is_lidar_data_valid:
                        reason_log += f" (Lidar 데이터 유효하지 않음)"
                else:
                    reason_log = "알 수 없는 이유로 정지"

                current_stop_status_msg = f"⏳ 로봇 정지 중... ({reason_log})"
                self.log_once(YELLOW, current_stop_status_msg)


        # --- 메인 상태에 따른 로봇 동작 ---
        if self.main_state == 'INITIALIZING':
            pass

        elif self.main_state == 'PATROL':

            target_yaw_at_corner = self.patrol_absolute_target_yaws[self.current_patrol_idx]

            if self.patrol_motion_state == 'TURN':
                self.log_once(MAGENTA, f"🔄 코너 회전 중... (현재 {self.current_patrol_idx+1}번 코너. 목표 각도: {math.degrees(target_yaw_at_corner):.2f}도)")

                yaw_error = self.normalize_angle(target_yaw_at_corner - self.yaw)

                if abs(yaw_error) > self.patrol_yaw_tolerance:
                    target_angular_z = self.patrol_turn_speed * (yaw_error / abs(yaw_error))
                    target_linear_x = 0.0
                else: # 회전 완료
                    self.patrol_motion_state = 'FORWARD'
                    # 새로운 직진 구간 시작 시에만 시작점과 방향을 업데이트
                    if self.pose:
                        self.segment_start_pose = copy.deepcopy(self.pose)
                        self.segment_start_yaw = self.yaw
                    self.current_segment_traveled_distance = 0.0
                    self.target_segment_length = self.patrol_forward_length # ⭐원래의 한 변 길이로 재설정
                    self.log_once(GREEN, f"▶️ 직진 시작. ({self.current_patrol_idx+1}번 코너 방향)")

            elif self.patrol_motion_state == 'FORWARD':
                # ⭐여기서 target_segment_length를 사용합니다.
                distance_remaining_in_segment = self.target_segment_length - self.current_segment_traveled_distance

                if distance_remaining_in_segment <= 0.01: # 거의 다 이동했으면 다음 단계로 전환
                    target_linear_x = 0.0
                    target_angular_z = 0.0

                    self.patrol_motion_state = 'TURN'
                    self.current_patrol_idx = (self.current_patrol_idx + 1) % len(self.patrol_absolute_target_yaws)
                    self.log_once(GREEN, f"🏁 한 변 이동 완료. 다음 회전 준비 (다음 목표: {math.degrees(self.patrol_absolute_target_yaws[self.current_patrol_idx]):.2f}도, 다음 코너: {self.current_patrol_idx+1}번)")
                    self.current_segment_traveled_distance = 0.0 # 새 구간 시작 시 초기화
                    self.target_segment_length = self.patrol_forward_length # ⭐원래의 한 변 길이로 재설정

                else:
                    target_linear_x = self.patrol_forward_speed

                    # 직진 중 방향 보정 로직 (계속 진행 중인 segment_start_pose를 기준으로 함)
                    # 만약 segment_start_pose가 재개 시 현재 로봇 위치로 정확히 설정되었다면,
                    # 이 오차는 재개 지점부터의 상대적인 오차를 보정하게 됩니다.
                    # 목표 지점 계산도 target_segment_length를 사용해야 함
                    ideal_segment_target_x = self.segment_start_pose.x + self.target_segment_length * math.cos(self.segment_start_yaw)
                    ideal_segment_target_y = self.segment_start_pose.y + self.target_segment_length * math.sin(self.segment_start_yaw)


                    dx_to_ideal_target = ideal_segment_target_x - self.pose.x
                    dy_to_ideal_target = ideal_segment_target_y - self.pose.y

                    target_angle_for_segment = math.atan2(dy_to_ideal_target, dx_to_ideal_target)
                    yaw_error_for_segment = self.normalize_angle(target_angle_for_segment - self.yaw)

                    if abs(yaw_error_for_segment) < self.patrol_yaw_tolerance:
                        target_angular_z = 0.0
                    else:
                        target_angular_z = self.patrol_forward_correction_gain * yaw_error_for_segment

                    current_forward_status_msg = (
                        f"🏃 직진 중... (현재 {self.current_patrol_idx+1}번 코너 방향. "
                        f"이동 거리: {self.current_segment_traveled_distance:.2f}/{self.target_segment_length:.2f}m, " # ⭐target_segment_length 사용
                        f"남은 거리: {self.target_segment_length - self.current_segment_traveled_distance:.2f}m, 경로 보정 각도: {math.degrees(yaw_error_for_segment):.2f}도)"
                    )
                    self.log_once(CYAN, current_forward_status_msg)

        elif self.main_state == 'STOPPED':
            target_linear_x = 0.0
            target_angular_z = 0.0
            if prev_main_state != 'STOPPED':
                self.current_linear_x = 0.0
                self.current_angular_z = 0.0


        # --- 속도 스무딩 로직 ---
        twist = Twist()
        delta_linear_x = target_linear_x - self.current_linear_x
        max_delta_linear = self.linear_accel_limit * self.control_loop_dt
        if abs(delta_linear_x) > max_delta_linear:
            twist.linear.x = self.current_linear_x + (max_delta_linear if delta_linear_x > 0 else -max_delta_linear)
        else:
            twist.linear.x = target_linear_x

        delta_angular_z = target_angular_z - self.current_angular_z
        max_delta_angular = self.angular_accel_limit * self.control_loop_dt
        if abs(delta_angular_z) > max_delta_angular:
            twist.angular.z = self.current_angular_z + (max_delta_angular if delta_angular_z > 0 else -max_delta_angular)
        else:
            twist.angular.z = target_angular_z

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
    node = SquarePatrolWithObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 종료됨 (Ctrl+C)')
    except SystemExit as e:
        node.get_logger().error(f'🚨 노드 비정상 종료: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#ros2 service call /manual_stop_control std_srvs/srv/SetBool "data: true" 수동 정지 코드
#ros2 service call /manual_stop_control std_srvs/srv/SetBool "data: false" 수동 재개 코드



