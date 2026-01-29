import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from autonomy_msgs.action import AutonomousActions

class AutonomousActionClient(Node):
    """
    This is a basic client that can send goals to the autonomy_action_server.
    Hopefully, we could eventually make a client that sends goals when certain buttons on the UI are pressed,
    but I don't know if that should be done here or somewhere else.
    """

    def __init__(self):
        super().__init__('autonomous_action_server')
        self._action_client = ActionClient(
            self,
            AutonomousActions,
            'autonomous_actions')

    def send_goal(self, index):
        goal_msg = AutonomousActions.Goal()
        goal_msg.index = index # same index that the server expects and uses

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

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
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

def main(args=None):
    """Currently hard-coded to send a single goal (with the index arg as 0)"""
    rclpy.init(args=args)

    action_client = AutonomousActionClient()

    action_client.send_goal(0)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
