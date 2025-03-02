import rclpy
from rclpy.node import Node
from serial_ros.send import send
from std_msgs.msg import String
from teleop_msgs.msg import MotorChanges

MOTOR_CURRENT_MSG = 0
SEND_DELAY_SEC = 0.1
MOTOR_STILL = 127
class SerialNode(Node):

    def __init__(self):
        self.data = [MOTOR_STILL]*6 # 0:header, [0:4]:4 wheels, 4:bucket drum, 5:linear actuator
        super().__init__('Reading from Tele-op')
        self.subscription = self.create_subscription(
            MotorChanges,
            'tele-op',
            self.listener_callback,
            1) #1 queued message
        self.subscription  # prevent unused variable warning
        self.timed_publisher = self.create_timer(SEND_DELAY_SEC, self.sendCurrents)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        for change in msg.changes:
            self.data[change.index] = change.velocity
        
    def sendCurrents(self):
        send(MOTOR_CURRENT_MSG, self.data)

def main(args=None):
    rclpy.init(args=args)

    node = SerialNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()