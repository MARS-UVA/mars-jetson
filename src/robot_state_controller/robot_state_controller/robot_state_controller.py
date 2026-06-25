import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from topic_tools_interfaces.srv import MuxSelect
from control_msgs.msg import RobotState
from std_msgs.msg import UInt8
from rclpy.client import Client

TELEOP_MODE = RobotState.TELEOP
DIG_MODE = RobotState.DIG
DUMP_MODE = RobotState.DUMP
ESTOP_MODE = RobotState.ESTOP

class RobotStateControllerNode(Node):
    """A ROS 2 node which controls the mux nodes based on the /robot_state topic"""
    
    def __init__(self, **kwargs):
        super().__init__('robot_state_controller', **kwargs)

        self.cmd_vel_mux_client_ = self.create_client(
            MuxSelect,
            'cmd_vel_mux/select'
        )
        self.arm_drum_mux_client_ = self.create_client(
            MuxSelect,
            'arm_drum_mux/select'
        )
        
        self.robot_state_toggle_subscriber_ = self.create_subscription(
            UInt8,
            'robot_state/toggle',
            self.robot_state_toggle_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        )

        self.robot_state_publisher_ = self.create_publisher(
            RobotState,
            'robot_state',
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.state = ESTOP_MODE
        self.control_robot_state()

    def robot_state_toggle_callback(self, msg: UInt8) -> None:
        """Callback for the /robot_state/toggle topic."""
        if self.state == ESTOP_MODE and msg.data == ESTOP_MODE:
            self.state = TELEOP_MODE
        elif self.state != ESTOP_MODE and msg.data == ESTOP_MODE:
            self.state = ESTOP_MODE
        else:
            self.state = msg.data

        self.control_robot_state()
    
    def control_robot_state(self) -> None:
        """Control the robot's state based on the current state."""
        if self.state == TELEOP_MODE:
            self.send_mux_request('cmd_vel/teleop', self.cmd_vel_mux_client_)
            self.send_mux_request('arm_drum_control/teleop', self.arm_drum_mux_client_)
        elif self.state == DIG_MODE or self.state == DUMP_MODE:
            self.send_mux_request('cmd_vel/autonomy', self.cmd_vel_mux_client_)
            self.send_mux_request('arm_drum_control/autonomy', self.arm_drum_mux_client_)
        elif self.state == ESTOP_MODE:
            self.send_mux_request('cmd_vel/teleop', self.cmd_vel_mux_client_)
            self.send_mux_request('arm_drum_control/teleop', self.arm_drum_mux_client_)
        self.robot_state_publisher_.publish(RobotState(state=self.state))

    def send_mux_request(self, topic: str, mux_client: Client) -> None:
        """Send a request to the mux node to select the given topic."""
        while not mux_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {mux_client.srv_name} not available, waiting...')
        
        select_request = MuxSelect.Request()
        select_request.topic = topic
        future = mux_client.call_async(select_request)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = RobotStateControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
