from teleop.control import WheelSpeeds
from teleop import wheel_speed_to_motor_queries
from teleop_msgs.msg import MotorChanges, SetMotorVelocity
def check_speeds(observed: MotorChanges, expected: MotorChanges) -> None:
    observed_copy=observed.copy()
    expected_copy=expected.copy()
    indices_in_observed={}
    indices_not_in_observed=[SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR,SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR,SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR,SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR]
    
    for i in indices_not_in_observed:
        if observed_copy.count(i) > 0:
            observed_index=observed.index(i)
            indices_not_in_observed.remove(i)
            indices_in_observed[i]=observed[observed_index].velocity
            observed_copy.remove(i)
    for j in indices_in_observed:
        if expected_copy.count(j) > 0:
            assert j==expected_copy[expected_copy.index(j)].velocity
            expected_copy.remove(j)
def test_no_wheel_speed() -> None:
    speed=WheelSpeeds(0.0,0.0)
    observed_speed = wheel_speed_to_motor_queries(speed)
    expected_speed = MotorChanges(changes=[SetMotorVelocity(index=SetMotorVelocity.FRONT_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_LEFT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.FRONT_RIGHT_DRIVE_MOTOR, velocity=127),
                                 SetMotorVelocity(index=SetMotorVelocity.BACK_RIGHT_DRIVE_MOTOR, velocity=127)])
    #maintain dict with expected values for indices and maintain set of checked indicies; loop through and if index is in set, index dictionary and remove index from set: check if index is in dictionary
    check_speeds(observed_speed, expected_speed)

# def test_full_wheel_speed() -> None:
#     speed=WheelSpeeds(1.0,1.0)
#     observed_speed = wheel_speed_to_motor_queries(speed)
#     check_speeds(observed_speed, expected_speed)
# def test_left_wheel_speed() -> None:
#     speed=WheelSpeeds(1.0,0.0)
#     observed_speed = wheel_speed_to_motor_queries(speed)
#     check_speeds(observed_speed, expected_speed)
# def test_right_wheel_speed() -> None:
#     speed=WheelSpeeds(0.0,1.0)
#     observed_speed = wheel_speed_to_motor_queries(speed)
#     check_speeds(observed_speed, expected_speed)
# def test_negative_wheel_speed_one() -> None:
#     speed=WheelSpeeds(-1.0,0.0)
#     observed_speed = wheel_speed_to_motor_queries(speed)
#     check_speeds(observed_speed, expected_speed)
# def test_negative_wheel_speed_two() -> None:
#     speed=WheelSpeeds(0.0,-1.0)
#     observed_speed = wheel_speed_to_motor_queries(speed)
#     check_speeds(observed_speed, expected_speed)