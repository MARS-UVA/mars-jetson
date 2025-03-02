from teleop_msgs.msg import MotorChanges, SetMotorVelocity
from .control import WheelSpeeds
#function converting wheel speeds to motor queries
def wheel_speed_to_motor_queries(wheel_speeds: WheelSpeeds) -> MotorChanges:
    left_wheel_speeds = round(1+wheel_speeds.left*127)
    right_wheel_speeds = round(1+wheel_speeds.right*127)
    return MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds)])
    