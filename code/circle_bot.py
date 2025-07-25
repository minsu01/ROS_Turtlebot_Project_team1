import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ComplexCircleNode(Node):
    def __init__(self):
        super().__init__('complex_circle_node')
        # cmd_vel 토픽에 Twist 메시지를 발행하는 퍼블리셔
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # 0.5초 뒤에 메인 로직(run_sequence)을 1회 실행
        self.timer = self.create_timer(0.5, self.run_sequence)

    def rotate(self, angle_deg, speed_deg=45):
        """
        주어진 각도만큼 로봇을 회전시키는 헬퍼(도우미) 함수입니다.
        :param angle_deg: 회전할 각도 (도 단위, 양수: 반시계, 음수: 시계)
        :param speed_deg: 회전 속도 (도/초 단위)
        """
        self.get_logger().info(f"동작: {angle_deg}도 회전을 시작합니다.")
        twist_msg = Twist()

        # 각도와 속도를 라디안 단위로 변환
        turn_speed_rad = math.radians(speed_deg)
        target_angle_rad = math.radians(angle_deg)

        # 회전 방향에 따라 각속도(angular.z)의 부호 결정
        twist_msg.angular.z = turn_speed_rad if angle_deg > 0 else -turn_speed_rad

        # 회전에 필요한 시간 계산 (시간 = 각도 / 속도)
        duration = abs(target_angle_rad / twist_msg.angular.z)

        # 계산된 시간만큼 회전 명령 발행
        self.publisher_.publish(twist_msg)
        time.sleep(duration)

        # 회전 후 정지
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info("완료: 회전을 마쳤습니다.")
        time.sleep(0.5) # 다음 동작을 위한 잠시 대기

    def run_sequence(self):
        # 타이머를 취소하여 이 메소드가 한 번만 실행되도록 보장
        self.timer.cancel()

        # --- 1. 초기 90도 시계 방향 회전 ---
        self.rotate(-90)
        time.sleep(1)

        # --- 2. 사분원 단위로 원 그리기 ---
        self.get_logger().info("동작: 원 그리기 시퀀스를 시작합니다...")

        # 파라미터 설정
        radius = 0.5        # 원의 반지름 (m)
        linear_speed = 0.2  # 로봇의 전진 속도 (m/s)

        # 반시계 방향 원운동에 필요한 각속도 계산
        circle_angular_speed = linear_speed / radius

        # 원의 1/4(90도 호)을 주행하는 데 걸리는 시간 계산
        quadrant_duration = (math.pi * radius / 2) / linear_speed

        # 4개의 사분면을 순서대로 주행
        for i in range(4):
            self.get_logger().info(f"==> {i+1}/4 구간 주행 시작")

            # 원 경로 주행 (전진 + 회전)
            twist_msg = Twist()
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = circle_angular_speed
            self.publisher_.publish(twist_msg)
            time.sleep(quadrant_duration)

            # 주행 후 정지
            twist_msg = Twist()
            self.publisher_.publish(twist_msg)
            self.get_logger().info(f"<== {i+1}/4 구간 주행 완료")
            time.sleep(1)

            # 마지막 사분원이 아닐 경우에만 추가 동작 수행
            if i < 3:
                self.get_logger().info("--- 추가 동작 시작 ---")
                # 1. 반시계 방향으로 90도 회전
                self.rotate(90)
                # 2. 1초 정지
                self.get_logger().info("동작: 1초간 정지합니다.")
                time.sleep(1)
                # 3. 다시 시계 방향으로 90도 회전 (원래 경로 방향으로 복귀)
                self.rotate(-90)
                self.get_logger().info("--- 추가 동작 완료 ---")

        # --- 3. 마지막 90도 반시계 방향 회전 ---
        self.get_logger().info("--- 마지막 90도 반시계 방향 회전 ---")
        self.rotate(90)
        time.sleep(1)

        # self.get_logger().info("종료")
        rclpy.shutdown() # 노드 종료

def main(args=None):
    rclpy.init(args=args)
    node = ComplexCircleNode()
    try:
        # 노드가 종료될 때까지(예: rclpy.shutdown() 호출) 대기
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # spin이 끝나면 노드를 확실히 파괴
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
