from teleop_msgs.msg import MotorChanges, SetMotor, HumanInputState
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
def stop_motors() -> MotorChanges:
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_SPIN_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_ACTUATOR, velocity=127)])