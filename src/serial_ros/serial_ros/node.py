import rclpy
from rclpy.node import Node
from serial_ros.serial_handler import SerialHandler
from std_msgs.msg import String
from teleop_msgs.msg import MotorChanges

MOTOR_CURRENT_MSG = 0
SEND_DELAY_SEC = 0.1
MOTOR_STILL = 127

class SerialNode(Node):

    def __init__(self):
        self.data = [MOTOR_STILL]*6 # 0:header, [0:4]:4 wheels, 4:bucket drum, 5:linear actuator
        super().__init__('read_from_teleop')
        self.subscription = self.create_subscription(
            MotorChanges,
            'teleop',
            self.listener_callback,
            1) #1 queued message
        self.subscription  # prevent unused variable warning
        self.timed_publisher = self.create_timer(SEND_DELAY_SEC, self.sendCurrents)
        self.serial_handler = SerialHandler()

    def listener_callback(self, msg):
        motors = ["FL", "BL", "FR", "BL", "BucketSpeed", "BucketActuator"]
        i = 0
        for change in msg.changes:
            # self.get_logger().info(f"{motors[i]}: {change.velocity}")
            # self.get_logger().info("Hey")
            # self.get_logger().info(type(change.velocity))
            self.data[change.index] = change.velocity
            i+=1


        for field in self.data:
            self.get_logger().info(str(field))
        
    def sendCurrents(self):
        #print(ok)
        # self.get_logger().info("Sending currents")
        self.serial_handler.send(MOTOR_CURRENT_MSG, self.data)
        

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