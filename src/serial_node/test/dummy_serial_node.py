import rclpy
from rclpy.node import Node
from serial_msgs.msg import CurrentBusVoltage
from serial_msgs.msg import Position
from serial_msgs.msg import Temperature

rclpy.init()
node = Node('one_shot_publisher')

pos_pub = node.create_publisher(
    msg_type=Position,
    topic='position',
    qos_profile=1
)
temp_pub = node.create_publisher(
    msg_type=Temperature,
    topic='temperature',
    qos_profile=1
)
current_pub = node.create_publisher(
    msg_type=CurrentBusVoltage,
    topic='current_bus_voltage',
    qos_profile=1
)

current_msg = CurrentBusVoltage(
    front_left_wheel_current = 0x0000,
    back_left_wheel_current = 0x0000,
    front_right_wheel_current = 0x0000,
    back_right_wheel_current = 0x0000,
    front_drum_current = 0x0000,
    back_drum_current = 0x0000,
    front_actuator_current = 0x0000,
    back_actuator_current = 0x0000,
    main_battery_voltage = 0x0000,
    aux_battery_voltage = 0x0000
)
temp_msg = Temperature(
    front_left_wheel_temperature = 0x0000,
    back_left_wheel_temperature = 0x0000,
    front_right_wheel_temperature = 0x0000,
    back_right_wheel_temperature = 0x0000,
    front_drum_temperature = 0x0000,
    back_drum_temperature = 0x0000
)
pos_msg = Position(
    front_actuator_position = 0x0000,
    back_actuator_position = 0x0000
)

current_pub.publish(current_msg)
temp_pub.publish(temp_msg)
pos_pub.publish(pos_msg)
print("Published dummy messages")
rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()