# teleop (Node)
Accepts raw human input state and converts to motor queries, which are then published. Performs conversions of the type "A is pressed, so add +7 to motor index 3".

Code: `teleop/teleop/teleop.py`

## Publishers

| Topic | Message type | Behavior |
| ---- | ------------ | -------- |
| `/teleop` | `teleop_msgs/MotorChanges` | Publish changes, including setting and incrementing motor currents, to indicate how the motor currents need to change. |

## Subscriptions

| Topic | Message type | Behavior |
| ---- | ------------ | -------- |
| `/human_input_state` | `teleop_msgs/HumanInputState` | Receive human input state, including gamepad input and all UI buttons. |

## Usage

```shell
ros2 run teleop teleop --ros-args linear_axis:=VALUE -p turn_axis:=VALUE -p full_forward_magnitude:=VALUE [-p shape:=VALUE [-p deadband:=VALUE]]
```

| Parameter | Type | Description |
| ---- | ---- | ----------- |
| `linear_axis` | `string` (read-only) | **Required.** The axis of the gamepad's inputs which controls linear velocity. Must be one of left_x, left_y, right_x, right_y, left_x_inverted, left_y_inverted, right_x_inverted, right_y_inverted. |
| `turn_axis` | `string` (read-only) | **Required.** The axis of the gamepad's inputs which controls linear velocity. Must be one of left_x, left_y, right_x, right_y, left_x_inverted, left_y_inverted, right_x_inverted, right_y_inverted |
| `full_forward_magnitude` | `double` | **Required.** The magnitude of both wheel's speeds when the user inputs completely forward. This affects the amount of wheel speed which will be devoted to turning. 0 indicates the robot can only spin in place, and 1 indicates the robot can only move forward and backward. Must be between 0 and 1. |
| `shape` | `double` | A parameter describing the shape of the curve that converts axis inputs into speeds. The axis input is raised to this power (keeping the sign), so 1 is linear. Must be positive (default: 1). |
| `deadband` | `double` | Minimum gamepad axis input below which the input is assumed to be 0 (default: 0.0). |