import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Empty

class TurtlebotObjectAligner(Node):
    def __init__(self):
        super().__init__('turtlebot_object_aligner_node')
        self.get_logger().info("Turtlebot Object Aligner Node has been started.")

        self.bridge = CvBridge()

        self.camera_topic = 'camera/image_raw/compressed'
        self.subscription = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            10)
        self.get_logger().info(f'"{self.camera_topic}" 토픽 구독 시작.')

        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("'cmd_vel' 토픽 퍼블리셔 생성.")

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
        self.is_angular_aligned = False

        self.angular_return_timer = None
        self.linear_return_timer = None
        self.return_start_time = None
        self.return_target_angle = 0.0
        self.return_target_distance = 0.0
        self.return_angular_speed = 0.3
        self.return_linear_speed = 0.05

        self.stop_subscription = self.create_subscription(
            Empty,
            '/stop_signal',
            self.stop_callback,
            10
        )
        self.get_logger().info(f"'/stop_signal' 토픽 구독 시작. STOP 수신 시 복귀 동작 시작됩니다.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.current_frame = cv_image

            if self.image_width == 0:
                self.image_height, self.image_width, _ = cv_image.shape
                self.target_x = self.image_width // 2
                self.get_logger().info(f"이미지 해상도: {self.image_width}x{self.image_height}, 중앙 X: {self.target_x}")

            processed_image, object_center_x, object_area = self.detect_and_draw_roi_and_get_info(cv_image)

            if self.angular_return_timer is None and self.linear_return_timer is None:
                self.control_robot(object_center_x, object_area)

            cv2.imshow("Turtlebot3 Camera Feed with Object Alignment", processed_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
            self.current_frame = None
            self.stop_robot()

    def detect_and_draw_roi_and_get_info(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #lower_red1 = np.array([0, 100, 100])
        #upper_red1 = np.array([10, 255, 255])
        #lower_red2 = np.array([170, 100, 100])
        #upper_red2 = np.array([180, 255, 255])

        #mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        #mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        #mask = cv2.bitwise_or(mask1, mask2)

        # 검정색 HSV 범위
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        mask = cv2.inRange(hsv, lower_black, upper_black)


        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        output_image = image.copy()

        largest_area = 0
        target_object_center_x = None
        target_object_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                if area > largest_area:
                    largest_area = area
                    x, y, w, h = cv2.boundingRect(cnt)
                    target_object_center_x = x + w // 2
                    target_object_area = area

                    cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(output_image, "Red Object", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.circle(output_image, (target_object_center_x, y + h // 2), 5, (0, 255, 255), -1)

        return output_image, target_object_center_x, target_object_area

    def control_robot(self, object_center_x, object_area):
        twist_msg = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.total_angular_offset += self.last_angular_z * dt
        self.total_linear_offset += self.last_linear_x * dt
        area_error = float('inf')

        if object_center_x is not None and self.image_width > 0:
            error_x = self.target_x - object_center_x

            if not self.is_angular_aligned:
                twist_msg.angular.z = self.kp_angular * error_x
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = np.clip(twist_msg.angular.z, -0.5, 0.5)

                if abs(error_x) < self.angular_alignment_threshold:
                    self.is_angular_aligned = True
                    self.get_logger().info("앵귤러 정렬 완료!")
                else:
                    self.get_logger().info(f"앵귤러 정렬 중 - Object X: {object_center_x} -> Angular: {twist_msg.angular.z:.2f}")
            else:
                twist_msg.angular.z = 0.0
                area_error = self.target_object_area - object_area
                twist_msg.linear.x = self.kp_linear * area_error
                twist_msg.linear.x = np.clip(twist_msg.linear.x, -0.1, 0.1)

                self.get_logger().info(f"리니어 접근 중 - Area: {object_area} -> Linear: {twist_msg.linear.x:.2f}")

                if abs(area_error) < 10000:
                    twist_msg.linear.x = 0.0
                    self.get_logger().info("리니어 접근 완료!")
                    self.is_angular_aligned = False

            if self.is_angular_aligned and abs(area_error) < 10000:
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                self.get_logger().info("정렬 및 접근 완료!")
                self.is_angular_aligned = False
        else:
            self.get_logger().warn("물체 감지 안됨. 로봇 정지.")
            self.stop_robot()
            self.is_angular_aligned = False
            self.last_angular_z = 0.0
            self.last_linear_x = 0.0

        self.publisher_cmd_vel.publish(twist_msg)
        self.last_angular_z = twist_msg.angular.z
        self.last_linear_x = twist_msg.linear.x

    def stop_robot(self):
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(stop_twist)
        self.last_angular_z = 0.0
        self.last_linear_x = 0.0

    def angular_return_timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.return_start_time).nanoseconds / 1e9

        rotated_angle = self.return_angular_speed * elapsed_time * np.sign(self.return_target_angle)
        angle_remaining = abs(self.return_target_angle) - abs(rotated_angle)

        twist_msg = Twist()
        if angle_remaining <= np.radians(2):
            twist_msg.angular.z = 0.0
            self.get_logger().info("회전 복귀 완료!")
            self.stop_robot()
            self.angular_return_timer.destroy()
            self.angular_return_timer = None
            self.total_angular_offset = 0.0
            self.start_linear_return()
        else:
            current_vel = self.return_angular_speed * np.sign(self.return_target_angle)
            current_vel = np.clip(current_vel * (angle_remaining / abs(self.return_target_angle)),
                                  -self.return_angular_speed, self.return_angular_speed)
            twist_msg.angular.z = current_vel
            self.publisher_cmd_vel.publish(twist_msg)

    def linear_return_timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.return_start_time).nanoseconds / 1e9

        traveled = self.return_linear_speed * elapsed_time * np.sign(self.return_target_distance)
        distance_remaining = abs(self.return_target_distance) - abs(traveled)

        twist_msg = Twist()
        if distance_remaining <= 0.01:
            twist_msg.linear.x = 0.0
            self.get_logger().info("선형 복귀 완료!")
            self.stop_robot()
            self.linear_return_timer.destroy()
            self.linear_return_timer = None
            self.total_linear_offset = 0.0
        else:
            current_vel = self.return_linear_speed * np.sign(self.return_target_distance)
            current_vel = np.clip(current_vel * (distance_remaining / abs(self.return_target_distance)),
                                  -self.return_linear_speed, self.return_linear_speed)
            twist_msg.linear.x = current_vel
            self.publisher_cmd_vel.publish(twist_msg)

    def stop_callback(self, msg):
        self.get_logger().info("STOP 신호 수신! 로봇 정지 및 복귀 시작.")
        self.stop_robot()
        self.start_angular_return()

    def start_angular_return(self):
        self.return_target_angle = -self.total_angular_offset

        if abs(self.return_target_angle) < np.radians(2):
            self.get_logger().info("각도 복귀 생략. 바로 선형 복귀.")
            self.total_angular_offset = 0.0
            self.start_linear_return()
            return

        self.get_logger().info(f"회전 복귀 시작. 각도: {np.degrees(self.total_angular_offset):.2f}도")
        self.return_start_time = self.get_clock().now()
        self.angular_return_timer = self.create_timer(0.05, self.angular_return_timer_callback)

    def start_linear_return(self):
        self.return_target_distance = -self.total_linear_offset

        if abs(self.return_target_distance) < 0.01:
            self.get_logger().info("선형 복귀 생략.")
            self.total_linear_offset = 0.0
            return

        self.get_logger().info(f"선형 복귀 시작. 거리: {self.total_linear_offset:.2f}m")
        self.return_start_time = self.get_clock().now()
        self.linear_return_timer = self.create_timer(0.05, self.linear_return_timer_callback)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotObjectAligner()

    print("\n--- 터틀봇 객체 정렬 및 복귀 노드 ---")
    print("카메라 영상에서 빨간 물체를 추적하여 정렬하고 접근합니다.")
    print("'/stop_signal' 신호 수신 시 원래 방향과 위치로 복귀합니다.")
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
