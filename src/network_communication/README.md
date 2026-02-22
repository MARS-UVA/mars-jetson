# Network Communication Node
This package contains the code for sending and receiving data to and from the UI

## Available Nodes
### `net_node`
How to run:\
`ros2 run network_communication net_node`

#### Topics
| Name | Message Type | Behavior |
| ---- | ------------ | -------- |
| current_bus_voltage | `serial_msgs/CurrentBusVoltage` | The node subscribes to this topic to get current bus feedback |
| temperature | `serial_msgs/Temperature` | The node subscribes to this topic to get temperature feedback |
| position | `serial_msgs/Position` | The node subscribes to this topic to get actuator position feedback |
