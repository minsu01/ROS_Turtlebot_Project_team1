import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os
import datetime
import math
import time
import threading

class ObstacleAvoiderAndTurner(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_and_turner')

        # 스레딩 Lock 객체 생성
        self.frame_lock = threading.Lock()

        # ROS 통신 설정
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # 상태 및 변수
        self.state = 'stop'  # 초기 상태를 'stop'으로 설정
        self.rotation_sequence_started = False

        # 이미지 관련 설정
        self.camera_topic = '/camera/image_raw/compressed'
        self.sub_image = self.create_subscription(CompressedImage, self.camera_topic, self.image_callback, 10)
        self.current_frame = None
        self.base_output_dir = os.path.join(os.path.expanduser('~'), "turtlebot_captured_images")
        os.makedirs(self.base_output_dir, exist_ok=True)

        self.get_logger().info("노드가 'stop' 상태로 시작됩니다. 회전 시퀀스를 바로 준비합니다.")


    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            new_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.frame_lock:
                self.current_frame = new_frame
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")

    def save_current_frame(self):
        # --- [수정된 부분] ---
        # 0.5초 간격으로 사진을 두 번 찍습니다.
        for i in range(2):
            self.get_logger().info(f"📸 {i+1}번째 사진 촬영 시도...")
            # 저장을 위해 잠시 대기하여 최신 프레임을 확보
            time.sleep(0.2)
            frame_to_save = None
            with self.frame_lock:
                if self.current_frame is not None:
                    frame_to_save = self.current_frame.copy()

            if frame_to_save is not None:
                today_date_str = datetime.datetime.now().strftime("%y-%m-%d")
                date_specific_dir = os.path.join(self.base_output_dir, today_date_str)
                os.makedirs(date_specific_dir, exist_ok=True)
                timestamp = datetime.datetime.now().strftime("%H-%M-%S-%f") # 밀리초까지 포함
                filename = os.path.join(date_specific_dir, f"capture_{timestamp}.jpg")
                try:
                    cv2.imwrite(filename, frame_to_save)
                    self.get_logger().info(f"이미지 저장됨: {filename}")
                except Exception as e:
                    self.get_logger().error(f"이미지 저장 오류: {e}")
            else:
                 self.get_logger().warn("현재 프레임 없음. 이미지 저장 건너뜀.")

            # 두 번째 촬영 전에 잠시 대기
            if i == 0:
                time.sleep(0.5)


    def scan_callback(self, msg):
        # 이 콜백은 더 이상 사용되지 않습니다.
        pass

    def control_loop(self):
        if self.state == 'stop':
            # 정지 명령 발행
            self.cmd_pub.publish(Twist())

            # 회전 시퀀스가 시작되지 않았다면, 초기 캡처 후 시퀀스 시작
            if not self.rotation_sequence_started:
                self.rotation_sequence_started = True

                time.sleep(2)
                self.get_logger().info("🛑 초기 상태 'stop' 확인. 첫 이미지를 캡처합니다 (2장).")
                self.save_current_frame() # 1. 초기 이미지 캡처

                self.get_logger().info("정지 완료. 이제 회전 시퀀스를 시작합니다.")
                sequence_thread = threading.Thread(target=self.run_rotation_sequence)
                sequence_thread.start() # 2. 회전 시퀀스 시작
                self.state = 'performing_rotation'

        elif self.state in ['move', 'performing_rotation', 'sequence_done']:
            pass

    def rotate(self, angle_deg, speed_deg=45):
        self.get_logger().info(f"동작: {angle_deg}도 회전을 시작합니다.")
        twist_msg = Twist()
        turn_speed_rad = math.radians(speed_deg)
        target_angle_rad = math.radians(angle_deg)
        twist_msg.angular.z = turn_speed_rad if angle_deg > 0 else -turn_speed_rad
        duration = abs(target_angle_rad / twist_msg.angular.z)
        self.cmd_pub.publish(twist_msg)
        time.sleep(duration)
        self.cmd_pub.publish(Twist()) # 정지
        self.get_logger().info("완료: 회전을 마쳤습니다.")
        time.sleep(0.5)

    def run_rotation_sequence(self):
        time.sleep(1) # 초기 캡처 후 잠시 대기
        self.rotate(-90)
        time.sleep(1)

        self.get_logger().info("동작: 원 그리기 시퀀스를 시작합니다...")
        radius = 1.0
        linear_speed = 0.2
        circle_angular_speed = linear_speed / radius
        quadrant_duration = (math.pi * radius / 2) / linear_speed

        for i in range(4):
            self.get_logger().info(f"==> {i+1}/4 구간 주행 시작")
            twist_msg = Twist()
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = circle_angular_speed
            self.cmd_pub.publish(twist_msg)
            time.sleep(quadrant_duration)
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"<== {i+1}/4 구간 주행 완료")
            time.sleep(1)

            if i < 3:
                self.get_logger().info("--- 추가 동작 시작 ---")
                self.rotate(90)
                self.get_logger().info(f"📸 {i+1}/4 구간의 90도 회전 후 이미지 캡처를 시도합니다 (2장).")
                self.save_current_frame()
                self.get_logger().info("동작: 1초간 정지합니다.")
                time.sleep(1)
                self.rotate(-90)
                self.get_logger().info("--- 추가 동작 완료 ---")

        self.get_logger().info("--- 마지막 90도 반시계 방향 회전 ---")
        self.rotate(90)
        time.sleep(1)

        self.get_logger().info("✅ 모든 회전 시퀀스가 완료되었습니다.")
        self.state = 'sequence_done'

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderAndTurner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.get_logger().info("노드 종료 전 로봇 정지.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
