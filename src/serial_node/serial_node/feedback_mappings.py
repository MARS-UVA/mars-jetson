from serial_msgs.msg import CurrentBusVoltage, Temperature, Position
import struct

START_BYTE = 0xFF
TELEOP_HEADER = 0x00
CURRENT_FEEDBACK_HEADER = 0x01
TEMPERATURE_FEEDBACK_HEADER = 0x02
POSITION_FEEDBACK_HEADER = 0x03

FEEDBACK_PACKETS = [
    {
        'msg_type': CurrentBusVoltage,
        'topic': 'current_bus_voltage',
        'header': CURRENT_FEEDBACK_HEADER,
        'length': 40,
        'parser': lambda data: dict(zip(
            ('front_left_wheel_current', 'back_left_wheel_current', 'front_right_wheel_current', 'back_right_wheel_current', 'front_drum_current', 'back_drum_current', 'front_actuator_current', 'back_actuator_current', 'main_battery_voltage', 'aux_battery_voltage'),
            tuple(struct.unpack('<10f', data))
        ))
    },
    {
        'msg_type': Temperature,
        'topic': 'temperature',
        'header': TEMPERATURE_FEEDBACK_HEADER,
        'length': 24,
        'parser': lambda data: dict(zip(
            ('front_left_wheel_temperature', 'back_left_wheel_temperature', 'front_right_wheel_temperature', 'back_right_wheel_temperature', 'front_drum_temperature', 'back_drum_temperature'),
            tuple(struct.unpack('<6f', data))
        ))
    },
    {
        'msg_type': Position,
        'topic': 'position',
        'header': POSITION_FEEDBACK_HEADER,
        'length': 8,
        'parser': lambda data: dict(zip(
            ('front_actuator_position', 'back_actuator_position'),
            tuple(struct.unpack('<2f', data))
        ))
    }
]
