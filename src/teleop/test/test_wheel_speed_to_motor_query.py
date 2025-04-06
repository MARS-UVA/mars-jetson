from teleop.control import WheelSpeeds
from teleop.motor_queries import wheel_speed_to_motor_queries, stop_motors
from teleop_msgs.msg import MotorChanges, SetMotor
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
    expected_speed = MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)

def test_full_wheel_speed() -> None:
    speed=WheelSpeeds(1.0,1.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=254),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=254)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_left_wheel_speed() -> None:
    speed=WheelSpeeds(1.0,0.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=254),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_right_wheel_speed() -> None:
    speed=WheelSpeeds(0.0,1.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=254),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=254)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_negative_wheel_speed_one() -> None:
    speed=WheelSpeeds(-1.0,0.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=0),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=0),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_negative_wheel_speed_two() -> None:
    speed=WheelSpeeds(0.0,-1.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=0),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=0)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)
def test_stop_motors() -> None:
    speed = WheelSpeeds(1.0,1.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    observed_speed.changes.append(SetMotor(index=SetMotor.BUCKET_DRUM_SPIN_MOTOR, velocity = 255))
    observed_speed.changes.append(SetMotor(index=SetMotor.BUCKET_DRUM_ACTUATOR, velocity = 191))
    expected_speed = MotorChanges(changes=[SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_SPIN_MOTOR, velocity=127),
                                 SetMotor(index=SetMotor.BUCKET_DRUM_ACTUATOR, velocity = 127)])
    assert not check_speeds(observed_speed,expected_speed)
    observed_speed = stop_motors()
    check_speeds(observed_speed, expected_speed)
