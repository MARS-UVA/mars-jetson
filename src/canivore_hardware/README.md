# canivore_hardware

ros2_control hardware interface (`canivore_hardware/CanivoreHardware`) that drives the robot's
motors through a CTRE CANivore using Phoenix 6, instead of the ESP32 serial link:

- **Wheels and drums**: Kraken X60s (`device_type: talonfx`), closed-loop velocity in rad/s.
- **Arms**: brushed linear actuators on Talon FXS controllers (`device_type: talonfxs`),
  open-loop duty cycle like the old ESP32 (command / `max_velocity`, so 1.0 = full power).

It is a drop-in replacement for `robot_serial_hardware/RobotSerialHardware`: same joints, same
velocity command interface, and it publishes the same feedback topics (`/position`,
`/current_bus_voltage`, `/temperature`).

## Running the robot with it

```bash
./deploy.sh robot_backend:=canivore
```

Settings (CAN IDs, gear ratios, gains, limits) are in `src/startup/config/canivore.yaml`; the
robot's xacro turns them into the URDF parameters below. To try a different file:
`robot_backend:=canivore canivore_config:=/path/to/file.yaml`.

Joints with `can_id: -1` echo their commands, so the full stack runs with no motors connected.
Bring motors up one at a time by giving just that joint its real ID.

## Building

Needs CTRE's `phoenix6` APT package, plus `canivore-usb` on the computer the CANivore plugs
into. Both come from CTRE's APT repository (see CTRE's "Installing Phoenix 6 (non-FRC)" docs).
On machines without Phoenix 6 (e.g. laptops that only run Gazebo) this package skips itself
with a warning instead of failing the workspace build.

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
| `position_publish_hz` | `50` | Rate of `/position` (arm actuator positions) |
| `activation_timeout_s` | `0.5` | How long to wait for devices to answer when activating |

### Joint parameters (`<joint>`)

| Param | Default | Applies to | Meaning |
|---|---|---|---|
| `device_type` | `talonfx` | all | `talonfx` (Kraken) or `talonfxs` (Talon FXS driving a brushed actuator) |
| `can_id` | `-1` | all | Device CAN ID. `-1` = not connected: the joint echoes its command (for testing) |
| `inverted` | `false` | all | Flip direction so a positive command matches the URDF convention |
| `neutral_mode` | `brake` | all | `brake` or `coast` when idle |
| `max_velocity` | talonfx `0`, talonfxs `1.0` | all | talonfx: clamp in rad/s (0 = none). talonfxs: the command that means 100% output |
| `stator_current_limit`, `supply_current_limit` | none | all | Amps |
| `min_position`, `max_position` | none | all | Travel limits in rad. talonfx: device soft limits. Also stop estimated positions |
| `gear_ratio` | `1.0` | talonfx | Motor rotations per joint rotation |
| `kP`, `kS`, `kV` | `0` | talonfx | Slot 0 velocity gains (joint units). With all three at 0 the motor won't move |
| `initial_position` | none | talonfx | Joint angle at power-on; needed before soft limits make sense |
| `brushed_wiring` | `AB` | talonfxs | Which FXS output leads the actuator uses: `AB`, `AC` or `BC` |
| `feedback_sensor` | `none` | talonfxs | Actuator sensor: `none`, `analog` (potentiometer), `quadrature` or `pulsewidth` |
| `quadrature_edges_per_rotation` | `4096` | talonfxs | For a quadrature sensor |
| `sensor_retracted`, `sensor_extended` | none | talonfxs | Raw sensor reading fully retracted / fully extended (volts for analog, rotations for encoders). With these set, `/position` is the measured extension and the plugin stops the actuator at the ends |
| `feedback_angle_start`, `feedback_angle_end` | `-1.2`, `0.2` | arm joints | Arm angle range mapped to 0..1 extension |
| `feedback_output_start`, `feedback_output_end` | `1.0`, `0.0` | arm joints | Extension at those angles (same as the sim's `actuator_feedback.yaml`) |

Settings are applied to the devices every time the plugin configures, so anything set by hand
in Phoenix Tuner X (gains, limits, inversion, brake mode) is overwritten.

## Behavior notes

- **Activation fails** if any connected device doesn't answer, with the joint name and CAN ID in
  the error. Nothing moves in that case.
- **Lost contact** while running makes `read()` return an error; the controller manager then
  deactivates the hardware and every motor is set to neutral.
- **Soft limits without `initial_position`** (talonfx) log a warning: the encoder reads 0
  wherever the joint is at power-on, so the limits would be in the wrong place.
- **Actuators without a calibrated sensor** (talonfxs) log a warning. Their position is
  estimated from commands, and only the actuator's own limit switches stop it at the ends.
- **`/position`** is the measured actuator extension when the sensor is calibrated (like the
  old ESP32), otherwise calculated from the arm angle.
- `velocity_controllers/JointGroupVelocityController` (`arm_drum_controller`) has no command
  timeout in Jazzy, so the arms and drums keep their last command until something sends zeros.
