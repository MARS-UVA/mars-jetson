import rclpy
from rclpy.node import Node
from serial_ros.serial_handler import SerialHandler
from std_msgs.msg import String, Float32
from teleop_msgs.msg import MotorChanges
from nucleo_msgs.msg import MotorFeedback

MOTOR_CURRENT_MSG = 0
SEND_DELAY_SEC = 0.1
RECV_DELAY_SEC = 0.5
MOTOR_STILL = 127

class SerialNode(Node):

    def __init__(self):
        self.data = [MOTOR_STILL]*6 # 0:header, [0:4]:4 wheels, 4:bucket drum, 5:linear actuator
        super().__init__('read_from_teleop')
        self.subscription = self.create_subscription(
            msg_type=MotorChanges,
            topic='teleop',
            callback=self.updateCurrents,
            qos_profile=1 #1 queued message
        ) 
        self.feedback_publisher = self.create_publisher(
            msg_type=MotorFeedback,
            topic='motor-feedback',
            qos_profile=1
        )
        self.potentiometer_publisher = self.create_publisher(
            msg_type=Float32,
            topic='potentiometer',
            qos_profile=1
        )
        self.subscription  # prevent unused variable warning
        self.send_timer = self.create_timer(SEND_DELAY_SEC, self.sendCurrents)
        self.recv_timer = self.create_timer(RECV_DELAY_SEC, self.readFromNucleo)
        self.serial_handler = SerialHandler()

    def updateCurrents(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        for change in msg.changes:
            self.data[change.index] = change.velocity
        
    def sendCurrents(self):
        #print(ok)
        self.serial_handler.send(MOTOR_CURRENT_MSG, self.data)
    
    def readFromNucleo(self): 
        data = self.serial_handler.readMsg()
        if data[0]==1:
            mf = MotorFeedback(front_left = data[1],
                             front_right = data[2],
                             back_left = data[3],
                             back_right = data[4],
                             drum = data[5])
            self.feedback_publisher.publish(mf)

        elif data[0]==2:
            depth = Float32(data[1])
            self.potentiometer_publisher.publish(depth)
            

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