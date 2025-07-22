import math
import os
import sys
import termios

from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist as CmdVelMsg

class TurtlebotPose(Node):
    def __init__(self):
        super().__init__('move_pose_turtle')
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.init_odom_state = False
        self.count = 0
        self.get_key_state = False
        self.step = 1
        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, 'cmd_vel', qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.update_timer = self.create_timer(0.010, self.update_callback)
        self.get_logger().info('package Start.')

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state = True

    def update_callback(self):
        if self.init_odom_state:
            self.generate_path()

    def generate_path(self):
        twist = CmdVelMsg()
        if not self.init_odom_state:
            return
        if not self.get_key_state:
            input_x, input_y = self.get_key()
            input_x_global = (
                math.cos(self.last_pose_theta) * input_x - math.sin(self.last_pose_theta) * input_y
            )
            input_y_global = (
                math.sin(self.last_pose_theta) * input_x + math.cos(self.last_pose_theta) * input_y
                )

            self.goal_pose_x = self.last_pose_x + input_x_global
            self.goal_pose_y = self.last_pose_y + input_y_global
            self.get_key_state = True
            twist = CmdVelMsg()
        else:
            if self.step == 1:
                path_theta = math.atan2(
                    self.goal_pose_y - self.last_pose_y,
                    self.goal_pose_x - self.last_pose_x)
                angle = path_theta - self.last_pose_theta
                angular_velocity = 0.3
                angle = math.atan2(math.sin(angle), math.cos(angle))
                if abs(angle) > 0.01:
                    twist.angular.z = angular_velocity if angle > 0 else -angular_velocity
                else:
                    twist.angular.z = 0.0
                    self.step += 1
            elif self.step == 2:
                x_distance = math.sqrt((self.goal_pose_x - self.last_pose_x)**2)
                y_distance = math.sqrt((self.goal_pose_y - self.goal_pose_y)**2)
                print(f'x_distance: {x_distance} , y_distance: {y_distance}')
                linear_velocity = 0.1
                if x_distance > 0.01 or y_distance >0.01:
                    twist.linear.x = linear_velocity
                else:
                    twist.linear.x  = 0.0
                    self.get_key_state = False

            self.cmd_vel_pub.publish(twist)

    def get_key(self):
        input_x = 0.0
        input_y = 0.0
        try:
            input_x = float(input('Input x: '))
            input_y = float(input('Input y: '))
        except:
            self.get_logger().info('Invalid input!')
        else:
            self.get_key_state = True

        return input_x, input_y

    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotPose()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
