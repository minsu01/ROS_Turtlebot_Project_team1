import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math

# 로봇의 행동 상태 정의
STATE_FIND_OBSTACLE = 1
STATE_TURN_TO_ALIGN = 2
STATE_FOLLOW_OBSTACLE = 3
STATE_STOP = 4

class ObstacleCircumnavigator(Node):
  def __init__(self):
    super().__init__('obstacle_circumnavigator')

    # 퍼블리셔, 서브스크라이버 설정
    self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    self.scan_sub = self.create_subscription(
      LaserScan, 'scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)
    self.odom_sub = self.create_subscription(
      Odometry, 'odom', self.odom_callback, 10)

    # 0.1초마다 제어 루프 실행
    self.timer = self.create_timer(0.1, self.control_loop)

    # 로봇 상태 및 데이터 초기화
    self.robot_state = STATE_FIND_OBSTACLE
    self.scan_ranges = []
    self.current_pose = None
    self.start_pose = None
    self.has_moved_away_from_start = False

    # --- 동작 튜닝 파라미터 (안정성을 위해 일부 값 조정) ---
    self.forward_speed = 0.25         # 직진 속도 (조금 줄임)
    self.turning_speed = 0.4          # 회전 속도
    self.stop_distance = 0.4          # 장애물 감지 거리 (조금 늘림)
    self.wall_distance = 0.3          # 오른쪽 벽과 유지할 거리
    self.lap_completion_distance = 0.25
    self.initial_move_away_distance = 0.5

    # 라이다 센서 각도 범위
    self.right_angle_range = (-100, -80)
    #  새로운 '위험 구역' 감지 각도 범위 (전방 오른쪽 전체)
    self.danger_zone_angle_range = (-90, 0)

    self.p_gain = 1.8

  def odom_callback(self, msg):
    self.current_pose = msg.pose.pose

  def scan_callback(self, msg):
    self.scan_ranges = msg.ranges

  def get_min_distance_in_angle_range(self, ranges, angle_min_deg, angle_max_deg):
    if not ranges:
      return float('inf')
    num_ranges = len(ranges)
    start_index = (angle_min_deg + 360) % 360
    end_index = (angle_max_deg + 360) % 360
    if start_index > end_index:
      relevant_ranges = ranges[start_index:] + ranges[:end_index+1]
    else:
      relevant_ranges = ranges[start_index:end_index+1]
    filtered_ranges = [r for r in relevant_ranges if r > 0.0]
    return min(filtered_ranges) if filtered_ranges else float('inf')

  def control_loop(self):
    if not self.scan_ranges or self.current_pose is None:
      self.get_logger().info('센서 데이터를 기다리는 중...', throttle_duration_sec=5)
      return

    twist = Twist()

    # --- 상태 머신 ---
    if self.robot_state == STATE_FIND_OBSTACLE:
      self.get_logger().info(f'상태: 장애물 찾기', throttle_duration_sec=1)
      front_dist = self.get_min_distance_in_angle_range(self.scan_ranges, -15, 15)
      if front_dist < self.stop_distance:
        self.start_pose = self.current_pose
        self.has_moved_away_from_start = False
        self.robot_state = STATE_TURN_TO_ALIGN
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info(f'장애물 감지. 현재 위치를 기록하고 회전을 시작합니다.')
      else:
        twist.linear.x = self.forward_speed

    elif self.robot_state == STATE_TURN_TO_ALIGN:
      self.get_logger().info(f'상태: 정렬을 위한 회전', throttle_duration_sec=1)
      front_dist = self.get_min_distance_in_angle_range(self.scan_ranges, -15, 15)
      if front_dist > self.stop_distance * 1.2:
        self.robot_state = STATE_FOLLOW_OBSTACLE
        self.get_logger().info('전방 확보. 장애물 추종을 시작합니다.')
      else:
        twist.angular.z = self.turning_speed

    elif self.robot_state == STATE_FOLLOW_OBSTACLE:
      # 한 바퀴 완주 체크 로직 (이전과 동일)
      dist_from_start = math.sqrt(
          (self.current_pose.position.x - self.start_pose.position.x)**2 +
          (self.current_pose.position.y - self.start_pose.position.y)**2)
      if not self.has_moved_away_from_start and dist_from_start > self.initial_move_away_distance:
          self.has_moved_away_from_start = True
      if self.has_moved_away_from_start and dist_from_start < self.lap_completion_distance:
          self.robot_state = STATE_STOP
          self.get_logger().info('출발 지점으로 복귀 완료! 주행을 종료합니다.')
          self.cmd_vel_pub.publish(Twist())
          return

      # --- 충돌 방지를 위한 개선된 주행 로직 ---
      danger_dist = self.get_min_distance_in_angle_range(self.scan_ranges,
                                                            self.danger_zone_angle_range[0],
                                                            self.danger_zone_angle_range[1])

      if danger_dist < self.stop_distance:
        # 1. 위험 구역에 장애물 감지 (내부 코너 등) -> 직진 멈추고 회전만!
        self.get_logger().info(f'위험 구역 감지 (거리: {danger_dist:.2f}m). 충돌 방지를 위해 우선 회전합니다.', throttle_duration_sec=1)
        twist.linear.x = 0.0
        twist.angular.z = self.turning_speed
      else:
        # 2. 위험 구역이 안전 -> 일반적인 벽 추종 주행
        self.get_logger().info(f' 추종 주행 중...', throttle_duration_sec=1)
        right_dist = self.get_min_distance_in_angle_range(self.scan_ranges,
                                                            self.right_angle_range[0],
                                                            self.right_angle_range[1])
        error = self.wall_distance - right_dist
        twist.angular.z = self.p_gain * error
        twist.linear.x = self.forward_speed
        twist.angular.z = max(min(twist.angular.z, 0.8), -0.8)

    elif self.robot_state == STATE_STOP:
        twist = Twist()

    self.cmd_vel_pub.publish(twist)

def main(args=None):
  rclpy.init(args=args)
  node = ObstacleCircumnavigator()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('키보드 입력으로 종료')
  finally:
    node.cmd_vel_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
