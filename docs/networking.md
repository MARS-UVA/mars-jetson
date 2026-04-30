# Network Communication Node
This package contains the code for sending and receiving data to and from the UI

## Available Nodes
### `udp_client`
How to run: <br>
`ros2 run network_communication udp_client`

#### Topics
| Name | Message Type | Behavior |
| ---- | ------------ | -------- |
| current_bus_voltage | `serial_msgs/CurrentBusVoltage` | The node subscribes to this topic to get current bus feedback |
| temperature | `serial_msgs/Temperature` | The node subscribes to this topic to get temperature feedback |
| position | `serial_msgs/Position` | The node subscribes to this topic to get actuator position feedback |
| robot_state | `std_msgs/UInt8` | The node subscribes to robot_state from the serial_node to get mux node state feedback |
| arm_control_state | `teleop_msgs/ArmControl` | The node subscribes to this topic to get the current arm being controlled by teleop to send back as feedback |

#### Variables
| Name | Type | Function |
| ---- | ---- | -------- |
| FeedbackByteIndices | enum | Contains the indices for all the positions inside of the buffer |
| connection_headers | struct | Contains the client file descriptor and the address of the control station |
| buffer | unsigned char[] | Contains the data to be sent over the Socket |
| timer_ | rclcpp::TimerBase::SharedPtr | A wall timer to call the timer_callback function after a set time passes |


#### Functions
**create_connection_headers(int port)**: <br>
Pulls the **CONTROL_STATION_IP** environment variable and fills the **connection_headers** variable with the socket file descriptor and the control_station_addr with a given port.

**client_send**: <br>
Given a **buffer's data**, attempt sending it over the socket stored in **connection_headers**. TODO: put fields into the message header.

**current_bus_callback**: <br>
Given a message to topic `current_bus_voltage`, fill **buffer** with the message data.

**temperature_callback**: <br>
Given a message to topic `temperature`, fill **buffer** with the message data.

**position_callback**: <br>
Given a message to topic `posiiton`, fill **buffer** with the message data.

**robot_state_callback**: <br>
Given a message to topic `robot_state`, fill **buffer** with the message data.

**arm_control_callback**: <br>
Given a message to topic `arm_control_state`, fill **buffer** with the message data.

**timer_callback**: <br>
Sends the current buffer data through the **client_send** function.

### `udp_server`
How to run: <br>
`ros2 run network_communication udp_server`

#### Topics
| Name | Message Type | Behavior |
| ---- | ------------ | -------- |
| robot_state/toggle | `std_msgs/UInt8` | The node publishes to this topic to update the mux node state |
| human_input_state | `teleop_msgs/HumanInputState` | The node publishes to this topic to send controller inputs to the teleop node |

#### Variables
| Name | Type | Function |
| ---- | ---- | -------- |
| server_active | bool | Stores whether the server is active |
| current_action_state | uint8_t | Stores the current action state, 0 = teleop, 1 = dig, 2 = dump, 3 = estop |
| client_ptr_ | rclcpp_action::Client<DigDump>::SharedPtr | Contains a pointer to the `DigDump Action Client` |
| action_timer_ | rclcpp::TimerBase::SharedPtr | A wall timer to call the action_timer_callback |
| server_thread_ | std::thread | A thread that runs the udp server |

#### Functions
**action_timer_callback**: <br>
If there is an action_update, update current_action_state and send the new goal.

**goal_response_callback**: <br>
When the DigDump action_server sends a goal_response, store the goal_handle if the goal was accepted.

**send_goal**: <br>
Given an **int action_type**, send the associated goal to the DigDump action serer

**cancel_goal**: <br>
Assuming a goal is running, cancels the goal asynchronously.

**runServer**: <br>
Starts the server, and listens for new messages. TODO: Better documentation here.

### Test:

#### UDP Client Test
Make sure the **CONTROL_STATION_IP** environment variable is set before running the `udp_client`.
To test `udp_client`, run `udp_client` using the command listed above. Open a second terminal, and try publishing the following message.  
<br>
`ros2 topic pub --once /position serial_msgs/msg/Position "{front_actuator_position: "0x0010", back_actuator_position: "0x0010"}"` 
<br>
<br>
Assuming the Control Station is running with a compatible branch, the Actuator Position graph should update.

#### UDP Server Test
Open two terminals. On one terminal, run the `udp_server` node. On the other, run:
<br>
`ros2 topic echo /human_input_state`
<br>
<br>
Run the control station with a compatible branch, connect a controller. The topic `/human_input_state` should have data being published.