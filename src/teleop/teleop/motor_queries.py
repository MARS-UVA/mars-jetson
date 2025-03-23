from teleop_msgs.msg import MotorChanges, SetMotor
from .control import WheelSpeeds
#function converting wheel speeds to motor queries
def wheel_speed_to_motor_queries(wheel_speeds: WheelSpeeds) -> MotorChanges:
    left_wheel_speeds = round(wheel_speeds.left*127) +127
    right_wheel_speeds = round(wheel_speeds.right*127) +127
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds)])

