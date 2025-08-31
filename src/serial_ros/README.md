# op_reader (Node)
Interfaces with the microcontroller over a serial connection. Writes motor currents from a global state on a clock and also reads feedback. Every message sent includes a header byte (0 is the only one we are using) and 6 data bytes, one for each motor. For motor current messages (header 0), data bytes are biased motor currents i.e. 127 is neutral, 0 is full force backward, 254 is full force forward. **255 is reserved**, don't use it as a header or data byte.

Code: `serial_ros/serial_ros/node.py`

## Publishers

| Topic | Message Type | Behavior |
| ----- | ------ | ------- |
| `feedback` | `nucleo_msgs/Feedback` | Send feedback received from the microcontroller. |

## Subscriptions
| Topic | Message Type | Behavior |
| ----- | ------ | ------- |
| `teleop` | `teleop_msgs/MotorChanges` | Accept motor queries to modify the globally stored motor current array. |

## Usage
`ros2 run serial_ros op_reader`

# serial_handler (Utility)
Handles low-level serial sending and receiving. For sending, packets are preceded with a start byte (255). This is to prevent packet misalignment while allowing non-blocking IO reads on the receiving end.

Code: `serial_ros/serial_ros/serial_handler.py`
