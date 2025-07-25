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

# 코드 1: ObstacleCircleAvoider 클래스 (원본 구조 유지)
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

# 코드 2: TurtlebotCameraCapture 클래스

class TurtlebotCameraCapture(Node):
    def __init__(self):
        super().__init__('turtlebot_camera_capture')

        # self.camera_topic = '/camera/image_raw'               # 비압축 이미지 토픽
        self.camera_topic = '/camera/image_raw/compressed'      # 압축 이미지 토픽

        self.sub_image = self.create_subscription(
            #Image,
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'"{self.camera_topic}" 토픽 구독 시작.')

        self.cv_bridge = CvBridge()
        self.current_frame = None # 현재 프레임을 저장할 변수

        self.base_output_dir = os.path.join(os.path.expanduser('~'), "turtlebot_captured_images") #이미지 저장 경로
        if not os.path.exists(self.base_output_dir):
            try:
                os.makedirs(self.base_output_dir)
                self.get_logger().info(f"'{self.base_output_dir}' 디렉토리를 생성했습니다.")
            except OSError as e:
                self.get_logger().error(f"디렉토리 생성 오류: {self.base_output_dir} - {e}. 권한을 확인하십시오!")
                # 디렉토리 생성 실패 시, 프로그램이 계속 실행될 수 있도록 (이미지 저장은 안 됨)
                self.base_output_dir = None # 저장 디렉토리가 없음을 표시

        # --- STOP 토픽 구독자 추가 ---
        self.stop_subscription = self.create_subscription(
            Empty,            # Empty 메시지 타입 구독
            '/stop_signal',   # 구독할 STOP 토픽 이름 (변경 가능)
            self.stop_callback, # STOP 메시지 수신 시 호출될 콜백 함수
            10
        )
        self.get_logger().info(f"'/stop_signal' 토픽 구독 시작. 이 토픽이 발행되면 이미지가 저장됩니다.")


        self.get_logger().info("터틀봇 카메라 캡처 노드가 시작되었습니다.")
        self.get_logger().info("카메라 캡처를 트리거하려면 '/stop_signal' 토픽을 발행하세요 (예: ros2 topic pub /stop_signal std_msgs/msg/Empty '{}').")
        self.get_logger().info("ROS 2 터미널에서 Ctrl+C를 눌러 노드를 종료하십시오.")

    def image_callback(self, msg):
        """
        카메라 이미지 메시지가 수신될 때 호출되는 콜백 함수.
        """
        try:
            # 메시지 타입에 따라 변환
            # if isinstance(msg, Image):
            #   self.current_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            if isinstance(msg, CompressedImage):
                 np_arr = np.frombuffer(msg.data, np.uint8)
                 self.current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # (선택 사항) 이미지 처리 및 화면 표시. ROS 2 환경에서는 rviz2 등으로 확인하는 것이 일반적입니다.
            # cv2.imshow("Turtlebot Camera Feed", self.current_frame)
            # cv2.waitKey(1) # GUI 창을 표시한다면 필요

        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            self.current_frame = None # 오류 발생 시 현재 프레임 초기화

    # --- STOP 토픽을 위한 새로운 콜백 함수 추가 ---
    def stop_callback(self, msg):
        """
        'STOP' 토픽 메시지가 수신될 때 호출되는 콜백 함수.
        이 함수가 호출되면 현재 카메라 프레임을 저장합니다.
        """
        self.get_logger().info("STOP 신호 수신! 현재 카메라 프레임을 저장합니다.")
        self.save_current_frame() # STOP 신호가 오면 이미지 저장 메서드 호출


    def save_current_frame(self):
        """
        현재 프레임을 파일로 저장하는 메서드.
        날짜별 폴더를 생성하고 그 안에 이미지를 저장합니다.
        """
        # 기본 저장 디렉토리가 유효하고 현재 프레임이 있는 경우에만 저장 시도
        if self.current_frame is not None and self.base_output_dir is not None:
            # 현재 날짜를 'YY_MM_DD' 형식으로 가져옵니다.
            today_date_str = datetime.datetime.now().strftime("%y-%m-%d")

            # 날짜별 하위 디렉토리 경로 생성
            date_specific_dir = os.path.join(self.base_output_dir, today_date_str)

            # 날짜별 디렉토리가 없으면 생성
            if not os.path.exists(date_specific_dir):
                try:
                    os.makedirs(date_specific_dir)
                    self.get_logger().info(f"날짜별 디렉토리 '{date_specific_dir}'를 생성했습니다.")
                except OSError as e:
                    self.get_logger().error(f"날짜별 디렉토리 생성 오류: {date_specific_dir} - {e}. 권한을 확인하십시오!")
                    return # 디렉토리 생성 실패 시 저장 중단

            timestamp = datetime.datetime.now().strftime("%H-%M-%S") # 시간만으로 파일명 생성
            filename = os.path.join(date_specific_dir, f"capture_{timestamp}.jpg") # 날짜 폴더 안에 저장

            try:
                cv2.imwrite(filename, self.current_frame)
                self.get_logger().info(f"이미지 저장됨: {filename}")
            except Exception as e:
                self.get_logger().error(f"이미지 저장 오류: {e}. 저장 경로 권한을 확인하십시오!")
        elif self.base_output_dir is None:
            self.get_logger().error("이미지 저장 기본 디렉토리가 유효하지 않습니다. 초기화 오류를 확인하십시오.")
        else:
            self.get_logger().warn("저장할 현재 프레임이 없습니다. 카메라 메시지를 기다리는 중입니다.")

def main(args=None):
    rclpy.init(args=args)

    try:
        avoider_node = ObstacleCircleAvoider()
        capture_node = TurtlebotCameraCapture()

        executor = MultiThreadedExecutor()
        executor.add_node(avoider_node)
        executor.add_node(capture_node)

        print("\n--- 통합 노드 실행 ---")
        print("장애물 회피 노드와 카메라 캡처 노드가 동시에 실행됩니다.")
        print("Ctrl+C를 눌러 노드를 종료하십시오.")

        executor.spin()

    except KeyboardInterrupt:
        print('노드 종료 요청 (Ctrl+C).')
    finally:

        if 'executor' in locals() and executor:
            executor.shutdown()
        if 'avoider_node' in locals() and avoider_node:
            avoider_node.destroy_node()
        if 'capture_node' in locals() and capture_node:
            capture_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
