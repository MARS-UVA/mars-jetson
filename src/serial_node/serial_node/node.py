import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, Duration
from serial_node.serial_handler import SerialHandler
from std_msgs.msg import String, Int8
from teleop_msgs.msg import MotorChanges
from serial_msgs.msg import Feedback

NUM_MOTORS = 8
MOTOR_CURRENT_MSG = 0
SEND_DELAY_SEC = 0.02
RECV_DELAY_SEC = 0.03
MOTOR_STILL = 127

TELEOP_MODE = 0
DIGDUMP_MODE = 1

INT2MODE = ["TELEOP", "DIG/DUMP AUTONOMY"]

class SerialNode(Node):

    def __init__(self):
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.mode = TELEOP_MODE
        self.teleop_buffer_ = [MOTOR_STILL]*NUM_MOTORS
        self.digdump_buffer_ = [MOTOR_STILL]*NUM_MOTORS

        super().__init__('serial_mux')
        self.teleop_sub_ = self.create_subscription(
            msg_type = MotorChanges,
            topic =' teleop',
            callback = self.update_buffer,
            qos_profile = qos) #1 queued message
        self.digdump_sub_ = self.create_subscription(
            msg_type = MotorChanges, 
            topic = 'digdump_autonomy',
            callback = self.update_buffer,
            qos_profile = qos)
        self.robot_state_sub_ = self.create_subscription(
            msg_type = Int8,
            topic = 'robot_state',
            callback = self.hange_robot_state,
            qos_profile = qos
        )
        
        self.feedback_publisher = self.create_publisher(
            msg_type = Feedback,
            topic = 'feedback',
            qos_profile = qos
        )
        self.teleop_sub_  # prevent unused variable warning
        self.send_timer = self.create_timer(SEND_DELAY_SEC, self.sendCurrents)
        self.recv_timer = self.create_timer(RECV_DELAY_SEC, self.readFromNucleo)
        self.serial_handler = SerialHandler()
    
    def change_robot_state(self, robot_state_msg):
        self.mode = robot_state_msg.data
        self.get_logger().log(f"CHANGING MODE: {INT2MODE[self.mode]}")

    def get_buffer(self):
        if self.mode == TELEOP_MODE:
            return self.teleop_buffer_
        elif self.mode == DIGDUMP_MODE:
            return self.digdump_buffer_
    
    def update_buffer(self, motor_updates):
        buffer = self.get_buffer()
        
        for change in motor_updates.changes:
            buffer[change.index] = change.velocity
        for add in motor_updates.adds:
            newVel = buffer[add.index] + add.vel_increment
            self.get_logger().info(f"{add.index}: {'+' if add.vel_increment>0 else ''}{add.vel_increment}")
            newVel = max(0, newVel)
            newVel = min(254, newVel)
            buffer[add.index] = newVel
        
    def sendCurrents(self):
        buffer = self.get_buffer()
        self.get_logger().warn(f"Sending currents: {self.buffer}")
        self.serial_handler.send(MOTOR_CURRENT_MSG, buffer, self.get_logger())
        
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
