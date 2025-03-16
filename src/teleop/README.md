# teleop

The `teleop` package provides a ROS node which processes human input, publishing motor queries for a different node to send to the motors themselves.

This package has primarily been tested on ROS 2 Jazzy, but it likely also supports Humble.

## Available nodes

### `teleop`

How to execute:

```shell
ros2 run teleop teleop
```

Executable name: `teleop`

#### Topics

| Name | Message type | Behavior |
| ---- | ------------ | -------- |
| `/teleop` | `teleop_msgs/MotorChanges` | This node publishes data to this topic to indicate how the motor currents need to change. |
| `/human_input_state` | `teleop_msgs/HumanInputState` | This node subscribes to this topic to obtain human inputs. |

#### Parameters

| Name | Type | Description |
| ---- | ---- | ----------- |
| `linear_axis` | `string` (read-only) | The axis of the gamepad's inputs which controls linear velocity. Must be one of left_x, left_y, right_x, right_y, left_x_inverted, left_y_inverted, right_x_inverted, right_y_inverted |
| `turn_axis` | `string` (read-only) | The axis of the gamepad's inputs which controls linear velocity. Must be one of left_x, left_y, right_x, right_y, left_x_inverted, left_y_inverted, right_x_inverted, right_y_inverted |
| `full_forward_magnitude` | `double` | The magnitude of both wheel's speeds when the user inputs completely forward. This affects the amount of wheel speed which will be devoted to turning. 0 indicates the robot can only spin in place, and 1 indicates the robot can only move forward and backward. Must be between 0 and 1. |
| `shape` | `double` | A parameter describing the shape of the curve that converts axis inputs into speeds. The axis input is raised to this power (keeping the sign), so 1 is linear. Must be positive (default: 1). |
| `deadband` | `double` | Minimum gamepad axis input below which the input is assumed to be 0 (default: 0.0). |
| `wheel_speed_ramp_rate` | `double` | Maximum speed at which wheel speeds change (default: infinity). |