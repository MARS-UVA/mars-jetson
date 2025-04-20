import rclpy
from rclpy.node import Node
from serial_ros.serial_handler import SerialHandler
from std_msgs.msg import String, Float32
from teleop_msgs.msg import MotorChanges
from nucleo_msgs.msg import Feedback

MOTOR_CURRENT_MSG = 0
SEND_DELAY_SEC = 0.1
RECV_DELAY_SEC = 0.5
MOTOR_STILL = 127

class SerialNode(Node):

    def __init__(self):
        self.motorData = [MOTOR_STILL]*6 # 0:header, [0:4]:4 wheels, 4:bucket drum, 5:linear actuator
        super().__init__('read_from_teleop')
        self.subscription = self.create_subscription(
            msg_type=MotorChanges,
            topic='teleop',
            callback=self.updateCurrents,
            qos_profile=1 #1 queued message
        ) 
        self.feedback_publisher = self.create_publisher(
            msg_type=Feedback,
            topic='feedback',
            qos_profile=1
        )
        self.subscription  # prevent unused variable warning
        self.send_timer = self.create_timer(SEND_DELAY_SEC, self.sendCurrents)
        self.recv_timer = self.create_timer(RECV_DELAY_SEC, self.readFromNucleo)
        self.serial_handler = SerialHandler()

    def updateCurrents(self, msg):
        motors = ["FL", "BL", "FR", "BR", "BucketSpeed", "BucketActuator"]
        i = 0
        for change in msg.changes:
            self.get_logger().info(f"{motors[i]}: {change.velocity}")
            # self.get_logger().info("Hey")
            # self.get_logger().info(type(change.velocity))
            self.motorData[change.index] = change.velocity
            i+=1


        # for field in self.motorData:
        #     self.get_logger().info(str(field))
        
    def sendCurrents(self):
        #print(ok)
        # self.get_logger().info("Sending currents")
        self.serial_handler.send(MOTOR_CURRENT_MSG, self.motorData)
    
    def readFromNucleo(self): 
        data = self.serial_handler.readMsg()
        # msgs = [Float32() for _ in range(8)]
        # for i in range(8): msgs[i].data = float(data[i][0])
        mf = Feedback(front_left = data[0][0],
                            front_right = data[1][0],
                            back_left = data[2][0],
                            back_right = data[3][0],
                            drum = data[4][0],
                            l_actuator = data[5][0],
                            r_actuator = data[6][0],
                            actuator_height = data[7][0])
        self.feedback_publisher.publish(mf)
            

def main(args=None):
    rclpy.init(args=args)

    node = SerialNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()