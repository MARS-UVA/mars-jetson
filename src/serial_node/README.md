Ethan Edwards; February 15th, 2026; serial\_node & serial\_msgs

### serial\_msgs 

Now has three different .msg files:

- **CurrentBusVoltage.msg** (header 0x01) ⚡  
* float32 front\_left\_wheel\_current \#amps  
* float32 back\_left\_wheel\_current  
* float32 front\_right\_wheel\_current  
* float32 back\_right\_wheel\_current  
* float32 front\_drum\_current  
* float32 back\_drum\_current  
* float32 front\_actuator\_current  
* float32 back\_actuator\_current  
* float32 main\_battery\_voltage \#volts  
* float32 aux\_battery\_voltage  
- **Temperature.msg** (header 0x02) 🔥  
* float32 front\_left\_wheel\_temperature \#°C  
* float32 back\_left\_wheel\_temperature  
* float32 front\_right\_wheel\_temperature  
* float32 back\_right\_wheel\_temperature  
* float32 front\_drum\_temperature  
* float32 back\_drum\_temperature  
- **Position.msg** (header 0x03) 🌎  
* float32 front\_actuator\_position \# calibrated position  
* float32 back\_actuator\_position
  
The messages are created via the following [documentation](https://github.com/MARS-UVA/ESP32S3_25-26/blob/main/ESP-Projects/main/user_libs/uart/serial_feedback_format.md).

Each line is a **float32** to account for the four bytes of data for each data piece; first four bytes aren’t written in the message files as they are covered by **serial\_node** in order to extract the **header** (documented later). ✍️

### serial\_node/serial\_handler.py:

**send** function:  
Writes bytes to Serial to be read later by **readMsg**.

**readMsg** function:  
**readMsg** function accounts for **three different message types**. The message is read from the **self.SER** (Serial) object. The **header** is stored within the **first four bytes**, and is saved to the variable **self.currentHeader** if self.currentHeader is zero (aka, doesn’t have a header stored right now). The actual header info is stored within the byte at **index 1** (as seen in the above documentation), and **readMsg** checks for a message of a **certain length** (either 8 bytes, 24 bytes, or 40 bytes) depending on the header.  📖  
When a full message is received, it is read and returned with the header as a **tuple** of **(header, feedback)**, where header is **4 bytes**, and feedback is a **list** of **4 byte pairs** (to match the .msg file sizes). 📋

### serial\_node/node.py:

\_\_init\_\_:  
Subscribes to MotorChanges (from teleop)  
Creates three publishers:

- **current\_bus\_voltage\_publisher**  
- **temperature\_publisher**  
- **position\_publisher**

Creates timer to run **sendCurrents** and **readFeedback**.

**listener\_callback** function:  
Receives a message and stores it as data within the node.

**sendCurrent** function:  
Send data to be written via **serial\_node/serial\_handler.py**.

**readFeedback** function:  
The **readFeedback** function takes a tuple pair **(header, feedback)** from the **readMsg** function in **serial\_node/serial\_handler.py**. Using the header (acquired by header\[1\]), it pairs the feedback with one of the msg classes from **serial\_msgs** and publishes it to the publisher. 🐫  
If the header is 0x00 or nonexistent, it sends “no data” to the logger. 😵

### Mux (Multiplexer) System

The serial node acts as a **multiplexer** that controls which input source gets to drive the motors at any given time. It manages this through a **robot state machine** with four modes:

| Mode | Value | Description |
|------|-------|-------------|
| `TELEOP_MODE` | 0 | Manual teleop control — motors driven by `teleop_buffer_` |
| `DIG_MODE` | 1 | Dig autonomy — motors driven by `digdump_buffer_` |
| `DUMP_MODE` | 2 | Dump autonomy — motors driven by `digdump_buffer_` |
| `ESTOP` | 3 | Emergency stop — all motors sent `MOTOR_STILL` (127) |

**How it works:**

The node subscribes to two independent motor command sources:
- **`teleop`** topic — manual controller input, writes to `teleop_buffer_`
- **`digdump_autonomy`** topic — autonomy commands (dig or dump), writes to `digdump_buffer_`

Both buffers are always updated by incoming messages regardless of the current mode. However, `sendCurrents` (running at 20 Hz) **only forwards the buffer that matches the active mode** to the ESP32 over serial. This means:
- In `TELEOP_MODE` → only `teleop_buffer_` is sent
- In `DIG_MODE` or `DUMP_MODE` → only `digdump_buffer_` is sent (shared buffer since dig and dump are mutually exclusive)
- In `ESTOP` → a constant stop message (`[127]*8`) is sent, ignoring both buffers

**State transitions** are controlled via the `robot_state/toggle` topic (UInt8):
- Send `ESTOP` messages to toggle/untoggle the `ESTOP` state
- From any non-ESTOP mode, sending `ESTOP` immediately stops all motors and **resets both buffers**
- From any non-ESTOP mode, sending a different mode value switches to that mode directly

The current mode is published at 10 Hz on the `robot_state` topic so other nodes can react to the active mode.

**Topic summary:**

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `teleop` | `MotorChanges` | Subscribe | Manual motor commands |
| `digdump_autonomy` | `MotorChanges` | Subscribe | Autonomy motor commands |
| `robot_state/toggle` | `UInt8` | Subscribe | Mode change requests |
| `robot_state` | `UInt8` | Publish | Current mode broadcast |
| `current_bus_voltage` | `CurrentBusVoltage` | Publish | Motor current & battery voltage feedback |
| `temperature` | `Temperature` | Publish | Motor temperature feedback |
| `position` | `Position` | Publish | Actuator position feedback |