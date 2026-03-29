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
