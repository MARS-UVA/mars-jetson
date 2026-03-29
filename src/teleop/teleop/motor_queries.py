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

# Raises the arms of the robot
def raise_arms(velocity, front_arm: bool, back_arm: bool, msg: MotorChanges) -> None:
    arm_velocity = 127 + velocity
    if front_arm:
        msg.changes.append(SetMotor(index=SetMotor.ARM_FRONT_ACTUATOR, velocity=arm_velocity))
    if back_arm:
        msg.changes.append(SetMotor(index=SetMotor.ARM_BACK_ACTUATOR, velocity=arm_velocity))

def move_actuator(direction: int, msg: MotorChanges) -> None:
    msg.changes.append(SetMotor(index=SetMotor.ACTUATOR, velocity=127 + direction * 127))

def set_vibrator(on: bool, msg: MotorChanges) -> None:
    msg.changes.append(SetMotor(index=SetMotor.VIBRATOR, velocity=127 + (127 if on else -127)))

def stop_motors() -> MotorChanges:
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.ACTUATOR, velocity=127),
                                 SetMotor(index=SetMotor.VIBRATOR, velocity=127)])
