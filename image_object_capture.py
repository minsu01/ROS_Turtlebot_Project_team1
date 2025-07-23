import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import datetime
import math

# ================================================================= #
# 코드 1: ObstacleCircleAvoider 클래스 (원본 구조 유지)
# ================================================================= #
class ObstacleCircleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_circle_avoider')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        # 캡처 신호를 보내기 위한 퍼블리셔
        self.capture_pub = self.create_publisher(Empty, '/stop_signal', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.state = 'move'
        self.start_time = self.get_clock().now()
        self.linear_speed = 0.2
        self.angular_speed = 0.2
        self.closest = float('inf')
        self.circle_step = 0
        self.get_logger().info("✅ 장애물 회피 노드 초기화 완료.")

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
        self.closest = min(front_ranges)

        if self.state == 'move' and self.closest < 0.5:
            self.get_logger().info("🛑 ObstacleAvoider: 장애물 감지 → /stop_signal 발행")
            self.capture_pub.publish(Empty()) # 캡처 노드에 신호 전송
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
                else:
                    self.state = 'pause_left'
                    self.start_time = self.get_clock().now()
        elif self.state == 'pause_left':
            twist.angular.z = self.angular_speed
            if elapsed > quarter_turn_time:
                self.state = 'pause_stop1'
                self.start_time = self.get_clock().now()
        elif self.state == 'pause_stop1':
            if elapsed > 0.5 and elapsed < 0.6: # 신호를 한 번만 보내기 위함
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
            if elapsed > 0.5 and elapsed < 0.6: # 신호를 한 번만 보내기 위함
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
        elif self.state == 'stop':
            pass

        self.cmd_pub.publish(twist)

# ================================================================= #
# 코드 2: TurtlebotCameraCapture 클래스 (원본 구조 유지)
# ================================================================= #
class TurtlebotCameraCapture(Node):
    def __init__(self):
        super().__init__('turtlebot_camera_capture')
        self.camera_topic = '/camera/image_raw/compressed'
        self.sub_image = self.create_subscription(
            CompressedImage, self.camera_topic, self.image_callback, 10)
        self.cv_bridge = CvBridge()
        self.current_frame = None
        self.base_output_dir = os.path.join(os.path.expanduser('~'), "turtlebot_captured_images")
        if not os.path.exists(self.base_output_dir):
            os.makedirs(self.base_output_dir, exist_ok=True)
        # /stop_signal 토픽을 구독하여 신호를 받음
        self.stop_subscription = self.create_subscription(
            Empty, '/stop_signal', self.stop_callback, 10)
        self.get_logger().info("✅ 카메라 캡처 노드 초기화 완료. /stop_signal 대기 중...")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")

    def stop_callback(self, msg):
        self.get_logger().info("📸 CameraCapture: /stop_signal 수신! 이미지 저장 시도.")
        self.save_current_frame()

    def save_current_frame(self):
        if self.current_frame is not None:
            today_date_str = datetime.datetime.now().strftime("%y-%m-%d")
            date_specific_dir = os.path.join(self.base_output_dir, today_date_str)
            if not os.path.exists(date_specific_dir):
                os.makedirs(date_specific_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%H-%M-%S")
            filename = os.path.join(date_specific_dir, f"capture_{timestamp}.jpg")
            try:
                cv2.imwrite(filename, self.current_frame)
                self.get_logger().info(f"이미지 저장됨: {filename}")
            except Exception as e:
                self.get_logger().error(f"이미지 저장 오류: {e}")
        else:
            self.get_logger().warn("저장할 프레임이 없습니다.")

# ================================================================= #
# 통합 실행을 위한 main 함수
# ================================================================= #
def main(args=None):
    rclpy.init(args=args)

    # 두 노드의 인스턴스를 생성
    avoider_node = ObstacleCircleAvoider()
    capture_node = TurtlebotCameraCapture()

    # 멀티 스레드 실행기를 사용하여 두 노드를 함께 실행
    executor = MultiThreadedExecutor()
    executor.add_node(avoider_node)
    executor.add_node(capture_node)

    print("--- 통합 노드 실행 ---")
    print("장애물 회피 노드와 카메라 캡처 노드가 동시에 실행됩니다.")

    try:
        # 실행기가 두 노드의 콜백을 모두 처리하도록 스핀
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료
        executor.shutdown()
        avoider_node.destroy_node()
        capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
