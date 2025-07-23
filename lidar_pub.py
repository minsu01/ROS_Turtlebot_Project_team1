#라이다 0도 값 데이터 받기

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class Lidar0DegPublisher(Node):
    def __init__(self):
        super().__init__('lidar_0deg_publisher')

        # ✅ BEST_EFFORT로 설정해서 라이다 퍼블리셔와 호환되도록 함
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=scan_qos
        )

        self.publisher_ = self.create_publisher(Float32, '/scan_0deg', 10)
        self.get_logger().info("✅ 0도 거리 퍼블리셔 초기화 완료")

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        index_0deg = int((0.0 - angle_min) / angle_increment)

        if 0 <= index_0deg < len(msg.ranges):
            distance = msg.ranges[index_0deg]

            if distance == float('inf') or distance == 0.0:
                self.get_logger().warn("⚠️ 0도 거리값이 유효하지 않음 (inf 또는 0)")
                return

            msg_out = Float32()
            msg_out.data = distance
            self.publisher_.publish(msg_out)

            # 디버깅 로그
            self.get_logger().info(f"📡 0도 거리: {distance:.3f} m")
        else:
            self.get_logger().warn("⚠️ 0도 인덱스가 유효한 범위 밖입니다.")

def main(args=None):
    rclpy.init(args=args)
    node = Lidar0DegPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
