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

### Test:

First, run net\_node. Open a second terminal, and try publishing the following message.  
`ros2 topic pub --once /position serial_msgs/msg/Position "{front_actuator_position: "0x0000", back_actuator_position: "0x0000"}"`

## **client.cpp:**

**ConnectionHeaders**  
{  
    int client\_socket\_fd;  
    struct sockaddr\_in control\_station\_addr;  
}   
Stores client socket file descriptor and the socket\_address for the control station in a struct.

**create\_connection\_headers** function:  
Pass in a port, and return a **ConnectionHeaders** struct.

**DataHeader**  
{  
    uint16\_t packetToSend;  
    uint16\_t totalPacketsToSend;  
    uint16\_t fragmentSize;  
    uint32\_t crc;  
}  
Stores the current packet to send, how many packets to send, and the size of the packet to send. Also stores the crc to check data corruption.

**HEADER\_SIZE sizeof(DataHeader):**  
Stores the size of DataHeader.

**crc32bit** function:  
Creates a crc32bit to check if there was corrupted data.

**client\_send** function:  
Pass in the data (stored as a buffer), the size of the data, and the port to send it through to. Creates a port using **create\_connection\_headers** to send the data to the control-station.  
Currently has mostly deprecated code to account for multiple packets being sent for image data (currently only one packet is sent).

## **frame\_sender.cpp:**

Deprecated. Purpose was to send image frames which is no longer necessary.

## **net\_node.cpp:**

### Initialization:

**create\_publisher (“human\_input\_state”)**:  
Publishes the current human\_input\_state.

**create\_wall\_timer**:  
Runs **timer\_callback** every 10ms and publishes to **human\_input\_state**.

Subscribes to:

- **current\_bus\_voltage**  
  - serial\_msgs/CurrentBusVoltage.msg  
  - Runs **current\_bus\_voltage\_callback** on reception  
- **temperature**  
  - serial\_msgs/Temperature.msg  
  - Runs **temperature\_callback** on reception  
- **position**  
  - serial\_msgs/Position.msg  
  - Runs **position\_callback** on reception

**unsigned char\* buffer**:  
Stores the data received from the subscriptions. Indexes are stored in **main.hpp** within FeedbackByte Indices.

### Functions

**current\_bus\_voltage\_callback**:  
Updates buffer with data received from **current\_bus\_voltage publisher**, then runs **client.cpp/client\_send**.

**temperature\_callback**:  
Updates buffer with data received from **temperature publisher**, then runs **client.cpp/client\_send**.

**position\_callback**:  
Updates buffer with data received from **position publisher**, then runs **client.cpp/client\_send**.

**timer\_callback**:  
Somehow gets data? Then publishes it to **human\_input\_state**.

## **server.cpp:**

**ThreadInfo** {  
char client\_message\[100000\];  
	bool flag;  
}  
client\_message: possibly new data that is extractable  
flag: if data is new, true

**create\_server** function:  
Opens a server, and while the server is running, gets data. If data is set to **ThreadInfo-\>client\_message**, set flag to true for other files to know to receive new data.

