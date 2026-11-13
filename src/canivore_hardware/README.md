# canivore_hardware

ros2_control hardware interface (`canivore_hardware/CanivoreHardware`) that drives the robot's
TalonFX motors through a CTRE CANivore using Phoenix 6, instead of the ESP32 serial link. Every
joint is a TalonFX running closed-loop velocity control on the motor.

It is a drop-in replacement for `robot_serial_hardware/RobotSerialHardware`: same joints, same
velocity command interface, and it also publishes `/current_bus_voltage` and `/temperature`.

For testing without the robot, see the separate `canivore_hardware_bench` package.

## Building

Needs CTRE's `phoenix6` APT package, plus `canivore-usb` on the computer the CANivore plugs
into. Both come from CTRE's APT repository (see CTRE's "Installing Phoenix 6 (non-FRC)" docs).

```bash
colcon build --packages-select teleop_msgs serial_msgs canivore_hardware
```

## Using it in a URDF

```xml
<ros2_control name="RobotSystem" type="system">
  <hardware>
    <plugin>canivore_hardware/CanivoreHardware</plugin>
    <param name="canbus">*</param>
  </hardware>
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="can_id">10</param>
    <param name="gear_ratio">12.0</param>
  </joint>
  <!-- ... -->
</ros2_control>
```

Every joint needs a `velocity` command interface. `position`, `velocity` and `effort` state
interfaces are filled if present. Effort is reported as motor current in amps.

### Hardware parameters (`<hardware>`)

| Param | Default | Meaning |
|---|---|---|
| `canbus` | `*` | CANivore name or serial number; `*` = any CANivore |
| `feed_timeout_ms` | `100` | Motors disable themselves if `write()` stops for this long |
| `enable_foc` | `false` | Field-oriented control on the Krakens; needs Phoenix Pro licenses |
| `status_frequency_hz` | `100` | How often each Kraken reports position/velocity/current |
| `feedback_publish_hz` | `10` | Rate of `/current_bus_voltage` and `/temperature` |
| `activation_timeout_s` | `0.5` | How long to wait for devices to answer when activating |

### Joint parameters (`<joint>`)

| Param | Default | Meaning |
|---|---|---|
| `can_id` | `-1` | TalonFX CAN ID. `-1` = not connected: the joint echoes its command (for testing) |
| `gear_ratio` | `1.0` | Motor rotations per joint rotation |
| `inverted` | `false` | Flip direction so a positive command matches the URDF convention |
| `neutral_mode` | `brake` | `brake` or `coast` when idle |
| `max_velocity` | `0` | Clamp on commands in rad/s (0 = no clamp) |
| `stator_current_limit` | none | Amps |
| `supply_current_limit` | none | Amps |
| `kP`, `kS`, `kV` | `0` | Slot 0 velocity gains (joint units). With all three at 0 the motor won't move |
| `min_position`, `max_position` | none | Travel limits in rad, applied as TalonFX soft limits |
| `initial_position` | none | Joint angle at power-on; needed before soft limits make sense |

Settings are applied to the devices every time the plugin configures, so anything set by hand
in Phoenix Tuner X (gains, limits, inversion, brake mode) is overwritten.

## Behavior notes

- **Activation fails** if any connected device doesn't answer, with the joint name and CAN ID in
  the error. Nothing moves in that case.
- **Lost contact** while running makes `read()` return an error; the controller manager then
  deactivates the hardware and every motor is set to neutral.
- **Soft limits without `initial_position`** log a warning: the encoder reads 0 wherever the
  joint is at power-on, so the limits would be in the wrong place.
- `velocity_controllers/JointGroupVelocityController` (`arm_drum_controller`) has no command
  timeout in Jazzy, so the arms and drums keep their last command until something sends zeros.
