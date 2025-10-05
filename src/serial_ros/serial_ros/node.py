import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, Duration
from serial_ros.serial_handler import SerialHandler
from std_msgs.msg import String, Bool
from teleop_msgs.msg import MotorChanges
from nucleo_msgs.msg import Feedback

MOTOR_CURRENT_MSG = 0
SEND_DELAY_SEC = 0.02
RECV_DELAY_SEC = 0.03
MOTOR_STILL = 127

class SerialNode(Node):

    def __init__(self):
        self.data = [MOTOR_STILL]*6 # 0:header, [0:4]:4 wheels, 4:bucket drum, 5:linear actuator
        super().__init__('read_from_teleop')
        self.subscription = self.create_subscription(
            msg_type=MotorChanges,
            topic='teleop',
            callback=self.listener_callback,
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE)) #1 queued message
        self.feedback_publisher = self.create_publisher(
            msg_type=Feedback,
            topic='feedback',
            qos_profile=1
        )
        self.subscription  # prevent unused variable warning
        self.send_timer = self.create_timer(SEND_DELAY_SEC, self.sendCurrents)
        self.recv_timer = self.create_timer(RECV_DELAY_SEC, self.readFromNucleo)
        self.serial_handler = SerialHandler()

        #Add heartbeat publisher
        self.heartbeat_pub.publisher(
            Bool,
            'health/serialROS',
            10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period,self.send_heartbeat)


    def send_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.heartbeat_pub.publish(msg)

    def listener_callback(self, msg):
        # self.get_logger().warn("Received motor query")
        # motors = ["FL", "BL", "FR", "BR", "BucketSpeed", "BucketActuator"]
        # i = 0
        for change in msg.changes:
            # self.get_logger().info(f"{motors[change.index]}: {change.velocity}")
            # self.get_logger().info("Hey")
            # self.get_logger().info(type(change.velocity))
            self.data[change.index] = change.velocity
            # if change.index==5:
                # self.get_logger().warn(f"Got actuator message {change.velocity}")
            # i+=1

        for add in msg.adds:
            newVel = self.data[add.index] + add.vel_increment
            self.get_logger().warning(f"{add.index}: {'+' if add.vel_increment>0 else ''}{add.vel_increment}")
            newVel = max(0, newVel)
            newVel = min(254, newVel)
            self.data[add.index] = newVel
        # print(f"motors: {self.data}")# logging message
	# for field in self.data:
        #     self.get_logger().info(str(field))
        
        
    def sendCurrents(self):
        #print(ok)
        self.get_logger().warn(f"Sending currents: {self.data}")
        self.serial_handler.send(MOTOR_CURRENT_MSG, self.data, self.get_logger())
        
    def readFromNucleo(self):
        data = self.serial_handler.readMsg(logger=self.get_logger())
        if data:
            mf = Feedback(front_left = data[0],
                                front_right = data[1],
                                back_left = data[2],
                                back_right = data[3],
                                l_drum = data[4],
                                r_drum = data[5],
                                l_actuator = data[6],
                                r_actuator = data[7],
                                actuator_height = data[8])
            self.feedback_publisher.publish(mf)
        else:
            self.get_logger().warn("no data")

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
