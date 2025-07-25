#사각형으로 만 도는 코드
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) 

        self.timer = self.create_timer(0.1, self.control_loop)
        
        # --- Robot State Variables (Odometry로부터) ---
        self.pose = None
        self.yaw = 0.0
        self._odom_initialized = False

        # --- Main State Machine & Sub-states ---
        self.main_state = 'INITIALIZING'
        self.patrol_sub_state = 'IDLE'

        self.edge_count = 0
        self.segment_start_pose = None
        
        # --- Patrol Parameters ---
        self.patrol_forward_speed = 0.2
        self.patrol_turn_speed = 0.4     # 회전 속도 (rad/s)
        self.patrol_forward_length = 0.5 # 사각형의 한 변 길이 (미터) - ⭐ MODIFIED TO 0.5 ⭐
        
        # ⭐ 90도 회전을 위한 회전 시간 설정 ⭐
        self.turn_duration_90_deg = (math.pi / 2) / self.patrol_turn_speed # 약 3.927초

        self.turn_start_time = None

        # --- 거리 및 시간 관련 변수 ---
        self.distance_traveled_current_segment = 0.0
        self.total_distance_accumulated = 0.0      
        self.segment_distances_log = []              

        self._last_control_time = self.get_clock().now()
        self._last_odom_time = self.get_clock().now()

        # --- 정지 및 대기 관련 변수 ---
        self.stop_wait_start_time = None
        self.wait_duration = 1.0         
        
        self.last_status_msg = ""
        self.get_logger().info("SquareMover 노드가 시작되었습니다. 오도메트리 데이터 수신 대기 중...")

    def log_status(self, msg):
        if self.last_status_msg != msg:
            self.get_logger().info(msg)
            self.last_status_msg = msg

    def odom_callback(self, msg):
        self._last_odom_time = self.get_clock().now()

        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, current_absolute_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.yaw = current_absolute_yaw

        if not self._odom_initialized:
            self._odom_initialized = True
            self.main_state = 'PATROL'
            self.patrol_sub_state = 'FORWARD_MOVING'
            self.edge_count = 0

            self.segment_start_pose = self.pose 
            self.distance_traveled_current_segment = 0.0
            self.log_status("🟢 Odom 초기화 완료. 시간 기반 순찰 시작.")
            self.log_status("🚶 초기 회전 없이 바로 직진 순찰 시작 (1번 코너).")


        if self.main_state == 'PATROL' and self.patrol_sub_state == 'FORWARD_MOVING' and self.segment_start_pose:
            self.distance_traveled_current_segment = math.sqrt(
                (self.pose.x - self.segment_start_pose.x)**2 +
                (self.pose.y - self.segment_start_pose.y)**2
            )
            if hasattr(self, '_last_pose') and self._last_pose:
                self.total_distance_accumulated += math.sqrt(
                    (self.pose.x - self._last_pose.x)**2 +
                    (self.pose.y - self._last_pose.y)**2
                )
            self._last_pose = self.pose


    def control_loop(self):
        twist = Twist()
        current_time_ros = self.get_clock().now()

        if not self._odom_initialized or self.pose is None:
            if (current_time_ros - self._last_control_time).nanoseconds / 1e9 >= 5.0:
                self.get_logger().warn("⚠️ 필수 데이터(Odom/Pose) 수신 대기 중...")
            self.publisher.publish(twist)
            return
        
        if (current_time_ros - self._last_odom_time).nanoseconds / 1e9 > 10.0:
            self.get_logger().error("❌ Odom 데이터 10초 이상 미수신! 노드를 종료합니다.")
            rclpy.shutdown() 
            return


        if self.main_state == 'INITIALIZING':
            self.log_status("⏳ 로봇 초기화 중...")
            pass 

        elif self.main_state == 'PATROL':
            
            if self.patrol_sub_state == 'FORWARD_MOVING':
                self.current_linear_speed_cmd = self.patrol_forward_speed
                self.current_angular_speed_cmd = 0.0
                twist.linear.x = self.current_linear_speed_cmd
                twist.angular.z = self.current_angular_speed_cmd
                
                self.log_status(
                    f"🏃 직진 중 (변 {self.edge_count + 1}/4): "
                    f"속도 {self.current_linear_speed_cmd:.2f} m/s, "
                    f"현재 변 이동 거리 {self.distance_traveled_current_segment:.2f}/{self.patrol_forward_length:.2f} m"
                )

                if self.distance_traveled_current_segment >= self.patrol_forward_length:
                    self.current_linear_speed_cmd = 0.0
                    self.current_angular_speed_cmd = 0.0
                    self.patrol_sub_state = 'STOPPING_BEFORE_TURN'
                    self.stop_wait_start_time = self.get_clock().now()
                    self.log_status(f"🛑 변 {self.edge_count + 1}/4 직진 완료! 모터 정지, 회전 대기.")
                    
                    self.segment_distances_log.append(self.distance_traveled_current_segment)


            elif self.patrol_sub_state == 'STOPPING_BEFORE_TURN':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                elapsed_wait = (current_time_ros - self.stop_wait_start_time).nanoseconds / 1e9
                self.log_status(f"⏳ 회전 전 정지 대기 중... ({elapsed_wait:.2f}/{self.wait_duration:.2f} s)")

                if elapsed_wait >= self.wait_duration:
                    self.patrol_sub_state = 'TURNING'
                    self.turn_start_time = self.get_clock().now() 
                    self.log_status(f"🔄 변 {self.edge_count + 1}/4 회전 시작! (시간 기반, 90도 목표)")

            elif self.patrol_sub_state == 'TURNING':
                self.current_linear_speed_cmd = 0.0
                self.current_angular_speed_cmd = -self.patrol_turn_speed # 시계 방향 회전
                twist.linear.x = self.current_linear_speed_cmd
                twist.angular.z = self.current_angular_speed_cmd

                elapsed_turn = (current_time_ros - self.turn_start_time).nanoseconds / 1e9
                self.log_status(
                    f"🔄 회전 중 (변 {self.edge_count + 1}/4): "
                    f"각속도 {self.current_angular_speed_cmd:.2f} rad/s, "
                    f"경과 시간 {elapsed_turn:.2f}/{self.turn_duration_90_deg:.2f} s"
                )

                # ⭐ 90도 회전을 위한 계산된 시간 동안 회전 ⭐
                if elapsed_turn >= self.turn_duration_90_deg: 
                    self.current_linear_speed_cmd = 0.0
                    self.current_angular_speed_cmd = 0.0
                    self.patrol_sub_state = 'STOPPING_AFTER_TURN'
                    self.stop_wait_start_time = self.get_clock().now()
                    self.log_status(f"✅ 변 {self.edge_count + 1}/4 회전 완료! 모터 정지, 다음 직진 대기.")
                    
            elif self.patrol_sub_state == 'STOPPING_AFTER_TURN':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                elapsed_wait = (current_time_ros - self.stop_wait_start_time).nanoseconds / 1e9
                self.log_status(f"⏳ 다음 직진 전 정지 대기 중... ({elapsed_wait:.2f}/{self.wait_duration:.2f} s)")

                if elapsed_wait >= self.wait_duration:
                    self.edge_count = (self.edge_count + 1) % 4
                    self.patrol_sub_state = 'FORWARD_MOVING'
                    
                    self.segment_start_pose = self.pose 
                    self.distance_traveled_current_segment = 0.0

                    if self.edge_count == 0:
                        total_dist = sum(self.segment_distances_log)
                        self.get_logger().info("\n" + "="*40)
                        self.get_logger().info(f"🎉 사각형 한 바퀴 완료! 총 누적 추정 거리: {total_dist:.2f} m")
                        self.get_logger().info(f"각 변 추정 거리: {[f'{d:.2f}' for d in self.segment_distances_log]} m")
                        self.get_logger().info("="*40 + "\n")
                        self.segment_distances_log = []
                        self.total_distance_accumulated = 0.0
                    
                    self.log_status(f"✅ 정지 대기 완료! 변 {self.edge_count + 1}/4로 직진 시작합니다.")

        self.publisher.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 노드 종료됨 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
