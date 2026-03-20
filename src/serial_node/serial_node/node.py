import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, Duration
from serial_node.serial_handler import SerialHandler
from std_msgs.msg import String, UInt8
from teleop_msgs.msg import MotorChanges
from serial_msgs.msg import CurrentBusVoltage
from serial_msgs.msg import Position
from serial_msgs.msg import Temperature

TESTING = False

NUM_MOTORS = 8
MOTOR_CURRENT_MSG = 0
SEND_HZ = 20
READ_HZ = 20
ROBOT_STATE_HZ = 10
MOTOR_STILL = 127

TELEOP_MODE = 0
DIG_MODE = 1
DUMP_MODE = 2
ESTOP = 3

INT2MODE = ["TELEOP", "DIG AUTONOMY", "DUMP AUTONOMY", "ESTOPPED"]

class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_mux')

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.mode = ESTOP
        self.teleop_buffer_ = [MOTOR_STILL]*NUM_MOTORS
        self.digdump_buffer_ = [MOTOR_STILL]*NUM_MOTORS # digdump shares buffer since both shouldn't run at same time
        self.STOP_MSG = [MOTOR_STILL]*NUM_MOTORS # DO NOT MODIFY

        self.teleop_sub_ = self.create_subscription(
            msg_type = MotorChanges,
            topic = 'teleop',
            callback = self.update_teleop_buffer,
            qos_profile = qos) #1 queued message
        self.digdump_sub_ = self.create_subscription(
            msg_type = MotorChanges, 
            topic = 'digdump_autonomy',
            callback = self.update_digdump_buffer,
            qos_profile = qos)
        self.robot_state_sub_ = self.create_subscription(
            msg_type = UInt8,
            topic = 'robot_state/toggle',
            callback = self.change_robot_state,
            qos_profile = qos
        )

        # feedback
        self.robot_state_pub_ = self.create_publisher(
            msg_type = UInt8,
            topic = 'robot_state',
            qos_profile = qos
        )
        self.current_bus_voltage_publisher = self.create_publisher(
            msg_type=CurrentBusVoltage,
            topic='current_bus_voltage',
            qos_profile=qos
        )
        self.position_publisher = self.create_publisher(
            msg_type=Position,
            topic='position',
            qos_profile=qos
        )
        self.temperature_publisher = self.create_publisher(
            msg_type=Temperature,
            topic='temperature',
            qos_profile=qos
        )
        self.teleop_sub_  # prevent unused variable warning
        self.send_timer = self.create_timer(1/SEND_HZ, self.sendCurrents)
        self.recv_timer = self.create_timer(1/READ_HZ, self.readFeedback)
        self.robot_state_timer = self.create_timer(1/ROBOT_STATE_HZ, self.publish_robot_state)
        self.serial_handler = SerialHandler()

    def publish_robot_state(self):
        msg = UInt8()
        msg.data = self.mode
        self.robot_state_pub_.publish(msg)
    
    def change_robot_state(self, robot_state_msg):
        new_state = robot_state_msg.data
        if self.mode == ESTOP:
            if new_state == ESTOP:
                self.mode = TELEOP_MODE
        else:
            self.mode = new_state
            if new_state == ESTOP:
                self.teleop_buffer_ = [MOTOR_STILL]*NUM_MOTORS # reset all buffers
                self.digdump_buffer_ = [MOTOR_STILL]*NUM_MOTORS
            
        self.get_logger().warn(f"CHANGING MODE: {INT2MODE[self.mode]}")

    def update_buffer(self, buffer, motor_updates):
        for change in motor_updates.changes:
            buffer[change.index] = change.velocity
        for add in motor_updates.adds:
            newVel = buffer[add.index] + add.vel_increment
            self.get_logger().info(f"{add.index}: {'+' if add.vel_increment>0 else ''}{add.vel_increment}")
            newVel = max(0, newVel)
            newVel = min(254, newVel)
            buffer[add.index] = newVel
    
    def update_teleop_buffer(self, motor_updates):
        self.update_buffer(self.teleop_buffer_, motor_updates)
        
    def update_digdump_buffer(self, motor_updates):
        self.update_buffer(self.digdump_buffer_, motor_updates)
        
    def sendCurrents(self):
        if self.mode == TELEOP_MODE:
            buffer = self.teleop_buffer_
        elif self.mode == DIG_MODE or self.mode == DUMP_MODE:
            buffer = self.digdump_buffer_
        elif self.mode == ESTOP:
            buffer = self.STOP_MSG

        self.get_logger().warn(f"Sending currents: {buffer}")
        if TESTING: return
        self.serial_handler.send(MOTOR_CURRENT_MSG, buffer, self.get_logger())
        
    def readFeedback(self):
        if TESTING: return
        header, feedback = self.serial_handler.readMsg(logger=self.get_logger())
        if header:
            if header[1] == 0x00:
                self.get_logger().warn("no data")
            elif header[1] == 0x01:
                mf = CurrentBusVoltage( 
                    front_left_wheel_current = feedback[0],
                    back_left_wheel_current = feedback[1],
                    front_right_wheel_current = feedback[2],
                    back_right_wheel_current = feedback[3],
                    front_drum_current = feedback[4],
                    back_drum_current = feedback[5],
                    front_actuator_current = feedback[6],
                    back_actuator_current = feedback[7],
                    main_battery_voltage = feedback[8],
                    aux_battery_voltage = feedback[9]
                )
                self.current_bus_voltage_publisher.publish(mf) 
            elif header[1] == 0x02:
                mf = Temperature(
                    front_left_wheel_temperature = feedback[0],
                    back_left_wheel_temperature = feedback[1],
                    front_right_wheel_temperature = feedback[2],
                    back_right_wheel_temperature = feedback[3],
                    front_drum_temperature = feedback[4],
                    back_drum_temperature = feedback[5]
                )
                self.temperature_publisher.publish(mf)
            elif header[1] == 0x03:
                mf = Position(
                    front_actuator_position = feedback[0],
                    back_actuator_position = feedback[1]
                )
                self.position_publisher.publish(mf)
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
