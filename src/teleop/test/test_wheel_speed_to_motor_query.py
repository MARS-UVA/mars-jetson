from teleop.control import WheelSpeeds
from teleop.motor_queries import wheel_speed_to_motor_queries
from teleop_msgs.msg import MotorChanges, SetMotorVelocity
def check_speeds(observed: MotorChanges, expected: MotorChanges) -> None:
    indices_in_observed={}
    indices_in_expected={}    
    for change in observed.changes:
        indices_in_observed[change.index]=change.velocity
    for change in expected.changes:
        indices_in_expected[change.index]=change.velocity
    assert indices_in_expected==indices_in_observed
def test_no_wheel_speed() -> None:
    speed=WheelSpeeds(0.0,0.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=127)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)

def test_full_wheel_speed() -> None:
    speed=WheelSpeeds(1.0,1.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=254),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=254)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_left_wheel_speed() -> None:
    speed=WheelSpeeds(1.0,0.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=127)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_right_wheel_speed() -> None:
    speed=WheelSpeeds(0.0,1.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=254),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=254)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_negative_wheel_speed_one() -> None:
    speed=WheelSpeeds(-1.0,0.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=0),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=0),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=127)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_negative_wheel_speed_two() -> None:
    speed=WheelSpeeds(0.0,-1.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=0),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=0)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)