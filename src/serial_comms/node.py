import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SerialNode(Node):

    def __init__(self):
        self.data = [0,0,0,0,0,0,0] # 0:header, [1:5]:4 wheels, 5:bucket drum, 6:linear actuator
        super().__init__('Reading from Tele-op')
        self.subscription = self.create_subscription(
            String,
            'tele-op',
            self.listener_callback,
            1) #1 queued message
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

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