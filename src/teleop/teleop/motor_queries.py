from teleop_msgs.msg import MotorChanges, SetMotor, HumanInputState
from .control import WheelSpeeds
#function converting wheel speeds to motor queries
def wheel_speed_to_motor_queries(wheel_speeds: WheelSpeeds) -> MotorChanges:
    left_wheel_speeds = round(wheel_speeds.left*127) +127
    right_wheel_speeds = round(wheel_speeds.right*127) +127
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds)])
def bucket_actuator_speed(human_input: HumanInputState) -> SetMotor:
    bucket_actuator_velocity = 127
    if human_input.gamepad_state.du_pressed and not human_input.gamepad_state.dd_pressed:
        bucket_actuator_velocity = 191
    elif human_input.gamepad_state.dd_pressed and not human_input.gamepad_state.du_pressed:
        bucket_actuator_velocity = 63
    return SetMotor(index=SetMotor.BUCKET_DRUM_ACTUATOR, velocity = bucket_actuator_velocity)
def stop_motors() -> MotorChanges:
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_SPIN_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_ACTUATOR, velocity=127)])
