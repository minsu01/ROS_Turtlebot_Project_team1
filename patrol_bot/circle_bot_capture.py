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

        # --- 기존 기능 초기화 ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.capture_pub = self.create_publisher(Empty, '/stop_signal', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # 상태: 'move', 'stop', 'performing_rotation', 'sequence_done'
        self.state = 'move'
        self.linear_speed = 0.2
        self.closest = float('inf')

        # --- 회전 시퀀스 제어 플래그 ---
        self.rotation_sequence_started = False

        # --- 이미지 관련 설정 ---
        self.camera_topic = '/camera/image_raw/compressed'
        self.sub_image = self.create_subscription(CompressedImage, self.camera_topic, self.image_callback, 10)
        self.current_frame = None
        self.base_output_dir = os.path.join(os.path.expanduser('~'), "turtlebot_captured_images")
        if not os.path.exists(self.base_output_dir):
            os.makedirs(self.base_output_dir, exist_ok=True)
        self.stop_subscription = self.create_subscription(Empty, '/stop_signal', self.stop_callback, 10)
        self.last_capture_time = self.get_clock().now()
        self.warned_frame_missing = False

    # =================================================
    # 기존 기능 메서드 (변경 없음)
    # =================================================
    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.warned_frame_missing = False
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")

    def stop_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_capture_time).nanoseconds / 1e9 < 0.5: return
        self.get_logger().info("STOP 신호 수신! 이미지 저장 시도 중...")
        self.last_capture_time = now
        self.save_current_frame()

    def save_current_frame(self):
        if self.current_frame is not None and self.base_output_dir is not None:
            today_date_str = datetime.datetime.now().strftime("%y-%m-%d")
            date_specific_dir = os.path.join(self.base_output_dir, today_date_str)
            os.makedirs(date_specific_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%H-%M-%S")
            filename = os.path.join(date_specific_dir, f"capture_{timestamp}.jpg")
            try:
                cv2.imwrite(filename, self.current_frame)
                self.get_logger().info(f"이미지 저장됨: {filename}")
            except Exception as e:
                self.get_logger().error(f"이미지 저장 오류: {e}")
        elif not self.warned_frame_missing:
            self.get_logger().warn("현재 프레임 없음. 이미지 저장 건너뜀.")
            self.warned_frame_missing = True

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
        self.closest = min(front_ranges)
        if self.state == 'move' and self.closest < 0.5:
            self.get_logger().info("🛑 장애물 감지 → 정지합니다.")
            self.capture_pub.publish(Empty())
            self.state = 'stop'

    # =================================================
    # 제어 및 회전 로직
    # =================================================
    def control_loop(self):
        twist = Twist()
        if self.state == 'move':
            twist.linear.x = self.linear_speed
            self.cmd_pub.publish(twist)
        elif self.state == 'stop':
            # 정지 명령 발행
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            # 회전 시퀀스가 시작되지 않았다면, 새 스레드에서 시작
            if not self.rotation_sequence_started:
                self.rotation_sequence_started = True
                self.get_logger().info("정지 완료. 이제 회전 시퀀스를 시작합니다.")
                # 새 스레드에서 회전 로직 실행 (ROS 콜백 방해 방지)
                sequence_thread = threading.Thread(target=self.run_rotation_sequence)
                sequence_thread.start()
                self.state = 'performing_rotation'
        elif self.state in ['performing_rotation', 'sequence_done']:
            # 회전 시퀀스가 실행 중이거나 완료되면, control_loop에서는 더 이상 아무것도 하지 않음
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
        twist_msg.angular.z = 0.0
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info("완료: 회전을 마쳤습니다.")
        time.sleep(0.5)

    def run_rotation_sequence(self):
        # 1. 초기 90도 시계 방향 회전
        self.rotate(-90)
        time.sleep(1)

        # 2. 사분원 단위로 원 그리기
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
            self.cmd_pub.publish(Twist()) # 정지
            self.get_logger().info(f"<== {i+1}/4 구간 주행 완료")
            time.sleep(1) # 정지 후 잠시 대기

            if i < 3:
                self.get_logger().info("--- 추가 동작 시작 ---")
                self.rotate(90)

                # --- [수정된 부분] 90도 회전 후 이미지 캡처 ---
                self.get_logger().info(f"📸 {i+1}/4 구간의 90도 회전 후 이미지 캡처를 시도합니다.")
                self.capture_pub.publish(Empty())

                self.get_logger().info("동작: 1초간 정지합니다.")
                time.sleep(1)
                self.rotate(-90)
                self.get_logger().info("--- 추가 동작 완료 ---")

        # 3. 마지막 90도 반시계 방향 회전
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
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.get_logger().info("노드 종료 전 로봇 정지.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
