import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import datetime

class ObstacleCircleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_circle_avoider')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.capture_pub = self.create_publisher(Empty, '/stop_signal', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.state = 'move'
        self.start_time = self.get_clock().now()

        self.linear_speed = 0.2
        self.angular_speed = 0.2

        self.closest = float('inf')
        self.circle_step = 0

        # 📸 이미지 수신 및 저장용 설정
        self.camera_topic = '/camera/image_raw/compressed'
        self.sub_image = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()
        self.current_frame = None

        self.base_output_dir = os.path.join(os.path.expanduser('~'), "turtlebot_captured_images")
        if not os.path.exists(self.base_output_dir):
            try:
                os.makedirs(self.base_output_dir)
            except OSError as e:
                self.get_logger().error(f"디렉토리 생성 오류: {self.base_output_dir} - {e}")
                self.base_output_dir = None

        self.stop_subscription = self.create_subscription(
            Empty,
            '/stop_signal',
            self.stop_callback,
            10
        )

        # ⛔ 중복 경고 및 출력 방지 플래그
        self.warned_frame_missing = False
        self.last_capture_time = self.get_clock().now()
        self.stop_log_shown = False

    def image_callback(self, msg):
        try:
            if isinstance(msg, CompressedImage):
                np_arr = np.frombuffer(msg.data, np.uint8)
                self.current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.warned_frame_missing = False  # 새 프레임 수신되면 경고 초기화
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            self.current_frame = None

    def stop_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_capture_time).nanoseconds / 1e9
        if elapsed < 0.5:
            return

        if not self.stop_log_shown:
            self.get_logger().info("STOP 신호 수신! 이미지 저장 시도 중...")
            self.stop_log_shown = True

        self.last_capture_time = now
        saved = self.save_current_frame()
        if saved:
            self.stop_log_shown = False

    def save_current_frame(self):
        if self.current_frame is not None and self.base_output_dir is not None:
            today_date_str = datetime.datetime.now().strftime("%y-%m-%d")
            date_specific_dir = os.path.join(self.base_output_dir, today_date_str)

            if not os.path.exists(date_specific_dir):
                try:
                    os.makedirs(date_specific_dir)
                except OSError as e:
                    self.get_logger().error(f"날짜별 디렉토리 생성 오류: {date_specific_dir} - {e}")
                    return False

            timestamp = datetime.datetime.now().strftime("%H-%M-%S")
            filename = os.path.join(date_specific_dir, f"capture_{timestamp}.jpg")

            try:
                cv2.imwrite(filename, self.current_frame)
                self.get_logger().info(f"이미지 저장됨: {filename}")
                return True
            except Exception as e:
                self.get_logger().error(f"이미지 저장 오류: {e}")
                return False
        elif self.base_output_dir is None:
            self.get_logger().error("이미지 저장 디렉토리가 유효하지 않습니다.")
            return False
        elif not self.warned_frame_missing:
            self.get_logger().warn("현재 프레임 없음. 이미지 저장 건너뜀.")
            self.warned_frame_missing = True
            return False
        return False

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
        self.closest = min(front_ranges)

        if self.state == 'move' and self.closest < 0.5:
            self.get_logger().info("🛑 장애물 감지 → 회피 시작")
            self.capture_pub.publish(Empty())
            self.state = 'turn_right'
            self.start_time = self.get_clock().now()

    def control_loop(self):
        twist = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        quarter_turn_time = (math.pi / 2) / self.angular_speed
        circle_segment_time = quarter_turn_time + 0.15

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
            if elapsed > 0.5:
                self.capture_pub.publish(Empty())
            if elapsed > 1.5:
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
            if elapsed > 0.5:
                self.capture_pub.publish(Empty())
            if elapsed > 1.5:
                self.circle_step += 1
                self.state = f'circle_step_{self.circle_step}'
                self.start_time = self.get_clock().now()

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
    
