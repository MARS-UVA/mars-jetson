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

def increment_drum_spin(increment : int, left_spin: bool, right_spin: bool, start_msg: MotorChanges):
    if left_spin:
        start_msg.adds.append(AddMotor(index = SetMotor.SPIN_LEFT_MOTOR, vel_increment = increment))
    if right_spin:
        start_msg.adds.append(AddMotor(index = SetMotor.SPIN_RIGHT_MOTOR, vel_increment = increment))

    

def raise_arms(actuator_vel_signed, left_control : bool, right_control : bool, start_msg: MotorChanges) -> SetMotor:
    actuator_vel = actuator_vel_signed + 127
    if left_control:
        start_msg.changes.append(SetMotor(index = SetMotor.ARM_LEFT_ACTUATOR, velocity = actuator_vel))
    if right_control:
        start_msg.changes.append(SetMotor(index = SetMotor.ARM_RIGHT_ACTUATOR, velocity = actuator_vel))
    

    
def stop_spin(start_msg: MotorChanges) -> None:
    start_msg.changes += [SetMotor(index = SetMotor.SPIN_LEFT_MOTOR, velocity = 127),
                                   SetMotor(index = SetMotor.SPIN_RIGHT_MOTOR, velocity = 127)]

def stop_motors() -> MotorChanges:
    return MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_SPIN_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_ACTUATOR, velocity=127)])