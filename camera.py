import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Empty
import math

class TurtlebotObjectAligner(Node):
    def __init__(self):
        super().__init__('turtlebot_object_aligner_node')
        self.get_logger().info("Turtlebot Object Aligner Node has been started.")

        self.bridge = CvBridge()

        # --- ROS 2 Subscribers ---
        self.camera_topic = 'camera/image_raw/compressed'
        self.subscription = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            10)
        self.get_logger().info(f'"{self.camera_topic}" 토픽 구독 시작.')

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info("'/odom' 토픽 구독 시작.")

        self.stop_subscription = self.create_subscription(
            Empty,
            '/stop_signal',
            self.stop_callback,
            10
        )
        self.get_logger().info(f"'/stop_signal' 토픽 구독 시작. STOP 수신 시 복귀 동작 시작됩니다.")

        # --- ROS 2 Publishers ---
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("'cmd_vel' 토픽 퍼블리셔 생성.")

        # --- Robot State Variables ---
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_yaw = 0.0

        # --- Control & Alignment Parameters ---
        self.total_angular_offset = 0.0
        self.total_linear_offset = 0.0
        self.last_angular_z = 0.0
        self.last_linear_x = 0.0
        self.last_time = self.get_clock().now()

        self.kp_angular = 0.005
        self.kp_linear = 0.00005
        self.target_x = 0
        self.target_object_area = 20000
        self.image_width = 0
        self.image_height = 0
        self.current_frame = None

        self.angular_alignment_threshold = 20
        self.linear_alignment_threshold = 10000
        self.is_angular_aligned = False
        self.is_linear_aligned = False

        # --- Logging & State Management Flags ---
        self.warned_no_object = False
        self.was_angular_aligning = False
        self.was_linear_approaching = False
        self.was_fully_aligned = False # 객체 정렬 및 접근이 완전히 완료된 상태
        self.has_stopped_and_recorded = False # 위치 기록 후 로봇 제어를 멈추기 위한 플래그
        self.object_detected_and_stopped = False # 물체 감지 후 정지 상태 플래그
        self.last_detected_object_roi = None # 마지막으로 감지된 물체의 ROI (x, y, w, h)

        # --- Return Mechanism Parameters ---
        self.angular_return_timer = None
        self.linear_return_timer = None
        self.return_start_time = None
        self.return_target_angle = 0.0
        self.return_target_distance = 0.0
        self.return_angular_speed = 0.3
        self.return_linear_speed = 0.05

        # OpenCV 창 이름을 미리 정의
        self.window_name = "Turtlebot3 Camera Feed with Object Alignment"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

    def odom_callback(self, msg):
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.current_frame = cv_image

            if self.image_width == 0:
                self.image_height, self.image_width, _ = cv_image.shape
                self.target_x = self.image_width // 2
                self.get_logger().info(f"이미지 해상도: {self.image_width}x{self.image_height}, 중앙 X: {self.target_x}")

            processed_image = cv_image.copy() # 원본 이미지를 복사하여 작업

            object_center_x, object_area, object_roi = self.detect_and_get_object_info(processed_image) # ROI 정보도 받음

            # 로봇이 아직 정지하지 않았거나, 복귀 동작 중일 때
            if not self.object_detected_and_stopped or \
               self.angular_return_timer is not None or \
               self.linear_return_timer is not None:
                if object_center_x is not None:
                    # 물체가 감지되었을 때만 경계 상자 그림
                    if object_roi is not None:
                        x, y, w, h = object_roi
                        cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.putText(processed_image, "Black Object", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.circle(processed_image, (object_center_x, y + h // 2), 5, (0, 255, 255), -1)

                self.control_robot(object_center_x, object_area, object_roi) # ROI 정보도 control_robot으로 전달
            else:
                # 물체 감지 후 정지 상태: 마지막으로 감지된 ROI를 검정색으로 덮기
                if self.last_detected_object_roi is not None:
                    x, y, w, h = self.last_detected_object_roi
                    cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 0, 0), -1) # 검정색으로 채우기
                    self.get_logger().debug("감지된 물체 영역을 가림.")

            cv2.imshow(self.window_name, processed_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
            self.current_frame = None
            self.stop_robot()
            if self.image_width > 0 and self.image_height > 0:
                black_image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
                cv2.imshow(self.window_name, black_image)
                cv2.waitKey(1)

    def detect_and_get_object_info(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = 0
        target_object_center_x = None
        target_object_area = 0
        target_object_roi = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                center_x = x + w // 2
                if area > largest_area:
                    largest_area = area
                    target_object_center_x = center_x
                    target_object_area = area
                    target_object_roi = (x, y, w, h)

        return target_object_center_x, target_object_area, target_object_roi

    def control_robot(self, object_center_x, object_area, object_roi):
        twist_msg = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.total_angular_offset += self.last_angular_z * dt
        self.total_linear_offset += self.last_linear_x * dt

        current_was_angular_aligning = False
        current_was_linear_approaching = False
        current_was_fully_aligned = False

        if object_center_x is not None and self.image_width > 0:
            self.warned_no_object = False

            error_x = self.target_x - object_center_x

            if not self.is_angular_aligned:
                twist_msg.angular.z = self.kp_angular * error_x
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = np.clip(twist_msg.angular.z, -0.5, 0.5)

                if abs(error_x) < self.angular_alignment_threshold:
                    self.is_angular_aligned = True
                    if self.was_angular_aligning:
                        self.get_logger().info("앵귤러 정렬 완료!")
                    self.is_linear_aligned = False
                else:
                    if not self.was_angular_aligning or abs(self.last_angular_z - twist_msg.angular.z) > 0.01:
                        self.get_logger().info(f"앵귤러 정렬 중 - Object X: {object_center_x} -> Angular: {twist_msg.angular.z:.2f}")
                    current_was_angular_aligning = True
            else:
                twist_msg.angular.z = 0.0
                area_error = self.target_object_area - object_area
                twist_msg.linear.x = self.kp_linear * area_error
                twist_msg.linear.x = np.clip(twist_msg.linear.x, -0.1, 0.1)

                if abs(area_error) < self.linear_alignment_threshold:
                    self.is_linear_aligned = True
                    if self.was_linear_approaching:
                        self.get_logger().info("리니어 접근 완료!")
                else:
                    if not self.was_linear_approaching or abs(self.last_linear_x - twist_msg.linear.x) > 0.005:
                        self.get_logger().info(f"리니어 접근 중 - Area: {object_area} -> Linear: {twist_msg.linear.x:.2f}")
                    current_was_linear_approaching = True

            # 최종 정렬 완료 상태 확인 및 정지
            if self.is_angular_aligned and self.is_linear_aligned:
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                if not self.was_fully_aligned:
                    self.get_logger().info("정렬 및 접근 완료! 로봇 정지.")
                    self.stop_robot() # 로봇 최종 정지
                    self.record_stop_location() # 정지 위치 정보 기록
                    self.has_stopped_and_recorded = True # 플래그 설정
                    self.object_detected_and_stopped = True # 물체 감지 후 정지 상태 설정
                    self.last_detected_object_roi = object_roi # 마지막 감지된 ROI 저장
                    self.get_logger().info("객체 감지 후 정지. 카메라 피드에서 물체를 가립니다.")
                current_was_fully_aligned = True
            else:
                self.object_detected_and_stopped = False # 정지 상태가 아니면 플래그 초기화
                current_was_fully_aligned = False

        else: # 물체 감지 안됨
            self.object_detected_and_stopped = False # 물체가 안 보이면 정지 상태 아님
            if not self.warned_no_object:
                self.get_logger().warn("물체 감지 안됨. 로봇 정지.")
                self.warned_no_object = True
            self.stop_robot()
            self.is_angular_aligned = False
            self.is_linear_aligned = False
            current_was_angular_aligning = False
            current_was_linear_approaching = False
            current_was_fully_aligned = False

        # has_stopped_and_recorded가 True이고 복귀 동작 중이 아니라면 더 이상 cmd_vel을 publish하지 않음
        if not self.has_stopped_and_recorded or \
           self.angular_return_timer is not None or \
           self.linear_return_timer is not None:
            self.publisher_cmd_vel.publish(twist_msg)
        else:
            pass # 이미 정지하고 기록했으므로, 더 이상 제어 메시지를 보내지 않음

        self.last_angular_z = twist_msg.angular.z
        self.last_linear_x = twist_msg.linear.x

        self.was_angular_aligning = current_was_angular_aligning
        self.was_linear_approaching = current_was_linear_approaching
        self.was_fully_aligned = current_was_fully_aligned

    def stop_robot(self):
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(stop_twist)
        self.last_angular_z = 0.0
        self.last_linear_x = 0.0

    def record_stop_location(self):
        self.get_logger().info(
            f"✅ 객체 정렬 및 접근 완료. 로봇 위치 기록됨:"
            f"\n  X: {self.current_pose_x:.2f} m"
            f"\n  Y: {self.current_pose_y:.2f} m"
            f"\n  Yaw: {math.degrees(self.current_yaw):.2f} degrees"
        )
        self.get_logger().info("이 노드는 더 이상 로봇을 제어하지 않습니다. 다른 노드를 사용하여 원을 그리세요.")

    # --- 복귀 관련 메서드 (STOP 신호 수신 시 사용) ---
    def angular_return_timer_callback(self):
        self.object_detected_and_stopped = False # 복귀 시작 시 물체 가림 해제
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.return_start_time).nanoseconds / 1e9

        total_duration = abs(self.return_target_angle / self.return_angular_speed) if self.return_angular_speed != 0 else 0
        if total_duration == 0:
            rotated_angle = self.return_target_angle
        else:
            rotated_angle = self.return_angular_speed * elapsed_time * np.sign(self.return_target_angle)

        angle_remaining = abs(self.return_target_angle) - abs(rotated_angle)

        twist_msg = Twist()
        if angle_remaining <= np.radians(2):
            twist_msg.angular.z = 0.0
            self.get_logger().info("회전 복귀 완료!")
            self.stop_robot()
            if self.angular_return_timer:
                self.angular_return_timer.destroy()
                self.angular_return_timer = None
            self.total_angular_offset = 0.0
            self.start_linear_return()
        else:
            min_angular_speed = np.radians(10)
            current_vel_magnitude = self.return_angular_speed * (angle_remaining / abs(self.return_target_angle))
            current_vel_magnitude = max(current_vel_magnitude, min_angular_speed)
            twist_msg.angular.z = current_vel_magnitude * np.sign(self.return_target_angle)
            self.publisher_cmd_vel.publish(twist_msg)

    def linear_return_timer_callback(self):
        self.object_detected_and_stopped = False # 복귀 시작 시 물체 가림 해제
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.return_start_time).nanoseconds / 1e9

        total_duration = abs(self.return_target_distance / self.return_linear_speed) if self.return_linear_speed != 0 else 0
        if total_duration == 0:
            traveled = self.return_target_distance
        else:
            traveled = self.return_linear_speed * elapsed_time * np.sign(self.return_target_distance)

        distance_remaining = abs(self.return_target_distance) - abs(traveled)

        twist_msg = Twist()
        if distance_remaining <= 0.01:
            twist_msg.linear.x = 0.0
            self.get_logger().info("선형 복귀 완료!")
            self.stop_robot()
            if self.linear_return_timer:
                self.linear_return_timer.destroy()
                self.linear_return_timer = None
            self.total_linear_offset = 0.0
        else:
            min_linear_speed = 0.02
            current_vel_magnitude = self.return_linear_speed * (distance_remaining / abs(self.return_target_distance))
            current_vel_magnitude = max(current_vel_magnitude, min_linear_speed)
            twist_msg.linear.x = current_vel_magnitude * np.sign(self.return_target_distance)
            self.publisher_cmd_vel.publish(twist_msg)

    def stop_callback(self, msg):
        # 복귀 시작 시 물체 가림 해제
        self.object_detected_and_stopped = False
        if self.has_stopped_and_recorded and \
           self.angular_return_timer is None and \
           self.linear_return_timer is None:
            self.get_logger().warn("이미 객체 정렬 완료 후 정지 상태입니다. STOP 신호 무시.")
            return

        self.get_logger().info("STOP 신호 수신! 로봇 정지 및 복귀 시작.")
        self.stop_robot()

        # 정렬 플래그 초기화 (재개 시 다시 정렬할 수 있도록)
        self.is_angular_aligned = False
        self.is_linear_aligned = False
        self.was_angular_aligning = False
        self.was_linear_approaching = False
        self.was_fully_aligned = False
        self.warned_no_object = False
        self.has_stopped_and_recorded = False # 복귀 시작 시 이 플래그를 False로 리셋

        self.get_logger().info("STOP 신호로 인해 카메라 피드 업데이트 재개됩니다.")

        self.start_angular_return()

    def start_angular_return(self):
        self.object_detected_and_stopped = False # 복귀 시작 시 물체 가림 해제
        self.return_target_angle = -self.total_angular_offset

        if abs(self.return_target_angle) < np.radians(2):
            self.get_logger().info("각도 복귀 생략. 바로 선형 복귀.")
            self.total_angular_offset = 0.0
            self.start_linear_return()
            return

        self.get_logger().info(f"회전 복귀 시작. 각도: {np.degrees(self.total_angular_offset):.2f}도")
        self.return_start_time = self.get_clock().now()
        if self.angular_return_timer:
            self.angular_return_timer.destroy()
        self.angular_return_timer = self.create_timer(0.05, self.angular_return_timer_callback)

    def start_linear_return(self):
        self.object_detected_and_stopped = False # 복귀 시작 시 물체 가림 해제
        self.return_target_distance = -self.total_linear_offset

        if abs(self.return_target_distance) < 0.01:
            self.get_logger().info("선형 복귀 생략.")
            self.total_linear_offset = 0.0
            return

        self.get_logger().info(f"선형 복귀 시작. 거리: {self.total_linear_offset:.2f}m")
        self.return_start_time = self.get_clock().now()
        if self.linear_return_timer:
            self.linear_return_timer.destroy()
        self.linear_return_timer = self.create_timer(0.05, self.linear_return_timer_callback)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotObjectAligner()

    print("\n--- 터틀봇 객체 정렬 및 정지 노드 ---")
    print("카메라 영상에서 검정색 물체를 추적하여 정렬하고 접근합니다.")
    print("정렬 및 접근이 완료되면 로봇은 정지하고, 카메라 피드에서 감지된 물체 영역을 가립니다.")
    print("이후에는 이 노드가 로봇을 제어하지 않습니다. 다른 노드를 사용하여 원을 그리세요.")
    print("'/stop_signal' 신호 수신 시 로봇은 정지하고, 원점 복귀 동작을 시작하며 카메라 피드에서 가려진 물체가 다시 보입니다.")
    print("Ctrl+C로 종료하세요.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료 요청 (Ctrl+C).')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
