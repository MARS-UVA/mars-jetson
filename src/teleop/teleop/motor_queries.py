from teleop_msgs.msg import MotorChanges, SetMotor, AddMotor, HumanInputState
from .control import WheelSpeeds
#function converting wheel speeds to motor queries
def wheel_speed_to_motor_queries(wheel_speeds: WheelSpeeds) -> MotorChanges:
    left_wheel_speeds = round(wheel_speeds.left*127) +127
    right_wheel_speeds = round(wheel_speeds.right*127) +127
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds)],
                        adds = [])

def bucket_actuator_speed(human_input: HumanInputState) -> SetMotor:
    actuator_value = 127
    ACTUATOR_BIG_SPEED = 127
    ACTUATOR_SMALL_SPEED_MAX = 64
    if human_input.gamepad_state.du_pressed and not human_input.gamepad_state.dd_pressed:
        actuator_value = 127 + ACTUATOR_BIG_SPEED
    elif human_input.gamepad_state.dd_pressed and not human_input.gamepad_state.du_pressed:
        actuator_value = 127 - ACTUATOR_BIG_SPEED
    else:
        actuator_value = 127 + int(human_input.gamepad_state.right_stick.y * ACTUATOR_SMALL_SPEED_MAX)
    return SetMotor(index=SetMotor.BUCKET_DRUM_ACTUATOR, velocity = actuator_value)

# Moves the arms of the robot
# 127 stops arms, 255 raises arms at full speed, and 0 lowers arms at full speed
def move_arms(velocity, front_arm: bool, back_arm: bool, up: bool, msg: MotorChanges) -> None:
    arm_velocity = velocity
    if up:
        arm_velocity += 127
    if front_arm:
        msg.changes.append(SetMotor(index=SetMotor.ARM_FRONT_ACTUATOR, velocity=arm_velocity))
    if back_arm:
        msg.changes.append(SetMotor(index=SetMotor.ARM_BACK_ACTUATOR, velocity=arm_velocity))

# Stops spinning the bucket drum(s) selected
def stop_drum_spin(front_arm: bool, back_arm: bool, msg: MotorChanges) -> None:
    if front_arm:
        msg.changes.append(SetMotor(index=AddMotor.SPIN_FRONT_DRUM, velocity=127))
    if back_arm:
        msg.changes.append(SetMotor(index=AddMotor.SPIN_BACK_DRUM, velocity=127))

# Increments speed of the bucket drum(s) selected
def increment_drum_spin(velocity_increment: int, front_arm: bool, back_arm: bool, msg: MotorChanges) -> None:
    if front_arm:
        msg.adds.append(AddMotor(index=AddMotor.SPIN_FRONT_DRUM, vel_increment=velocity_increment))
    if back_arm:
        msg.adds.append(AddMotor(index=AddMotor.SPIN_BACK_DRUM, vel_increment=velocity_increment))

def stop_motors() -> MotorChanges:
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.SPIN_FRONT_DRUM, velocity=127),
                                 SetMotor(index=SetMotor.ARM_FRONT_ACTUATOR, velocity=127),
                                 SetMotor(index=SetMotor.SPIN_BACK_DRUM, velocity=127),
                                 SetMotor(index=SetMotor.ARM_BACK_ACTUATOR, velocity=127)])