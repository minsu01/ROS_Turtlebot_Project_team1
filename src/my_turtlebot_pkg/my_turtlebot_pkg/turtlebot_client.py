import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot3_msgs.action import Patrol


class Turtlebot3PatrolClient(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_client')

        print('TurtleBot3 Patrol Client')
        print('----------------------------------------------')
        print('Input below')
        print('mode: s: square, t: triangle')
        print('travel_distance (unit: m)')
        print('patrol_count')
        print('----------------------------------------------')

        self._action_client = ActionClient(self, Patrol, 'turtlebot3')

        self.mode = 1.0
        self.travel_distance = 1.0
        self.patrol_count = 1

        self.mode, self.travel_distance, self.patrol_count = self.get_key()
        self.send_goal()

    def get_key(self):
        mode = str(input('mode(s: square, t: triangle): '))
        travel_distance = float(input('travel_distance: '))
        patrol_count = int(input('patrol_count: '))

        if mode == 's':
            mode = 1
        elif mode == 't':
            mode = 2
        elif mode == 'x':
            rclpy.shutdown()
        else:
            self.get_logger().info('you selected wrong mode')
            rclpy.shutdown()

        return mode, travel_distance, patrol_count

    def send_goal(self):
        goal_msg = Patrol.Goal()
        goal_msg.goal.x = float(self.mode)
        goal_msg.goal.y = float(self.travel_distance)
        goal_msg.goal.z = float(self.patrol_count)

        self._action_client.wait_for_server()

        self._send_goal_future = \
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))


def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_client = Turtlebot3PatrolClient()

    rclpy.spin(turtlebot3_patrol_client)


if __name__ == '__main__':
    main()
