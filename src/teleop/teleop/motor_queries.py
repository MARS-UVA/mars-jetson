from teleop_msgs.msg import MotorChanges, SetMotorVelocity

from .control import WheelSpeeds


def wheel_speed_to_motor_queries(wheel_speeds: WheelSpeeds) -> MotorChanges:
    """Return a ``MotorChanges`` message which contains motor queries which will cause the robot's motors to match the
    given wheel speed."""
    left_speed_uint = 127 + int(127 * wheel_speeds.left)
    right_speed_uint = 127 + int(127 * wheel_speeds.right)
    return MotorChanges([
        SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR,
                         velocity=left_speed_uint),
        SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR,
                         velocity=left_speed_uint),
        SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR,
                         velocity=right_speed_uint),
        SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR,
                         velocity=right_speed_uint),
    ])
