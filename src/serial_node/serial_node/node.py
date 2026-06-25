import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from serial_node.serial_handler import SerialHandler
from serial_node.feedback_mappings import FEEDBACK_PACKETS, TELEOP_HEADER
from serial_msgs.msg import MotorCommands

RECV_HZ = 20

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('mock_serial', 0)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        mock_serial = self.get_parameter('mock_serial').get_parameter_value().integer_value != 0
        self.get_logger().info(f"SerialNode parameters: port={port}, baudrate={baudrate}, mock_serial={mock_serial}")

        self.serial_handler = SerialHandler(port=port, baudrate=baudrate, mock_serial=mock_serial)

        self.motor_command_subscriber = self.create_subscription(MotorCommands, '/motor_commands', self.motor_command_callback, QoSProfile(depth=10))
        self.timer = self.create_timer(1.0 / RECV_HZ, self.read_serial_data)

        self.feedback_packet_map = {(packet['header'], packet['length']): packet for packet in FEEDBACK_PACKETS}
        self.feedback_publishers = {
            packet['header']: self.create_publisher(packet['msg_type'], packet['topic'], QoSProfile(depth=10)) for packet in FEEDBACK_PACKETS
        }

        self.get_logger().info("SerialNode has been initialized.")
        
    def read_serial_data(self):
        packet = self.serial_handler.read(logger=self.get_logger())

        if not packet:
            return
        
        self.parse_publish_serial_data(*packet)

    def parse_publish_serial_data(self, header, data):
        key = (header, len(data))
        if key not in self.feedback_packet_map:
            self.get_logger().warning(f"Received unknown packet format: header {hex(header)}, length {len(data)} bytes")
            return None

        packet_info = self.feedback_packet_map.get(key, None)

        parsed_data = packet_info['parser'](data)
        msg = packet_info['msg_type'](**parsed_data)
    
        self.feedback_publishers[packet_info['header']].publish(msg)
        self.get_logger().debug(f"Published Feedback: {packet_info['msg_type']} - {packet_info['topic']}")

    def motor_command_callback(self, msg):
        self.get_logger().debug(f"Received MotorCommands: {msg.motor_commands}")
        payload = bytes(msg.motor_commands)
        self.serial_handler.write(header=TELEOP_HEADER, payload=payload)
        
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
