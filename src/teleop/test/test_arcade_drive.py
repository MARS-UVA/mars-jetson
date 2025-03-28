import math
from teleop_msgs.msg import GamepadState, StickPosition

from teleop.control import ArcadeDrive, WheelSpeeds, GamepadAxis


def test_full_forward() -> None:
    state = GamepadState(left_stick=StickPosition(y=StickPosition.MAX_Y))
    drive = ArcadeDrive(linear_axis=GamepadAxis.LEFT_Y, turn_axis=GamepadAxis.LEFT_X_INVERTED, full_forward_magnitude=0.7)
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=+0.7, right=+0.7)), \
        f'Wheel speeds {wheel_speeds} are not full forward'


def test_full_reverse() -> None:
    state = GamepadState(left_stick=StickPosition(y=-StickPosition.MAX_Y))
    drive = ArcadeDrive(linear_axis=GamepadAxis.LEFT_Y, turn_axis=GamepadAxis.LEFT_X_INVERTED, full_forward_magnitude=0.7)
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=-0.7, right=-0.7)), \
        f'Wheel speeds {wheel_speeds} are not full reverse'


def test_full_turn_ccw() -> None:
    state = GamepadState(left_stick=StickPosition(x=-StickPosition.MAX_X))
    drive = ArcadeDrive(linear_axis=GamepadAxis.LEFT_Y, turn_axis=GamepadAxis.LEFT_X_INVERTED, full_forward_magnitude=0.7)
    # 0.7 = 1 / (1 + half_wheel_distance)
    # 1 + half_wheel_distance = 10 / 7
    # half_wheel_distance = 3 / 7
    # angular_velocity = (right - left) / wheel_distance
    # Since left = -right, angular_velocity = left / half_wheel_distance
    # angular_velocity = left * 7 / 3
    # 1 = left * 7 / 3
    # left = 3 / 7
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=-0.3, right=+0.3)), \
        f'Wheel speeds {wheel_speeds} are incorrect for CCW turning'


def test_full_turn_cw() -> None:
    state = GamepadState(left_stick=StickPosition(x=StickPosition.MAX_X))
    drive = ArcadeDrive(linear_axis=GamepadAxis.LEFT_Y, turn_axis=GamepadAxis.LEFT_X_INVERTED, full_forward_magnitude=0.7)
    # 0.7 = 1 / (1 + half_wheel_distance)
    # 1 + half_wheel_distance = 10 / 7  |  1 + half_wheel_distance = 1 / full_forward_magnitude
    # half_wheel_distance = 3 / 7
    # angular_velocity = (right - left) / wheel_distance
    # Since left = -right, angular_velocity = left / half_wheel_distance
    # angular_velocity = left * 7 / 3
    # 1 = left * 7 / 3
    # left = 3 / 7
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=+0.3, right=-0.3)), \
        f'Wheel speeds {wheel_speeds} are incorrect for CW turning'


def test_invert_linear() -> None:
    state = GamepadState(left_stick=StickPosition(y=abs(StickPosition.MAX_Y)))
    drive = ArcadeDrive(linear_axis=GamepadAxis.LEFT_Y_INVERTED, turn_axis=GamepadAxis.LEFT_X_INVERTED, full_forward_magnitude=0.7)
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=-0.7, right=-0.7)), \
        f'Wheel speeds {wheel_speeds} are not full reverse'


def test_invert_turn() -> None:
    state = GamepadState(left_stick=StickPosition(x=-abs(StickPosition.MAX_X)))
    drive = ArcadeDrive(linear_axis=GamepadAxis.LEFT_Y, turn_axis=GamepadAxis.LEFT_X, full_forward_magnitude=0.7)
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=+0.3, right=-0.3)), \
        f'Wheel speeds {wheel_speeds} are incorrect for CW turning'


def verify_speeds(linear_velocity: float,
                  angular_velocity: float,
                  full_forward_magnitude: float) -> None:
    state = GamepadState(left_stick=StickPosition(x=-angular_velocity * StickPosition.MAX_X,
                                                  y=linear_velocity * StickPosition.MAX_Y))
    drive = ArcadeDrive(linear_axis=GamepadAxis.LEFT_Y, turn_axis=GamepadAxis.LEFT_X_INVERTED, full_forward_magnitude=full_forward_magnitude)
    half_wheel_distance = (1 / full_forward_magnitude) - 1
    wheel_speeds = drive.get_wheel_speeds(state)
    actual_linear_velocity = (wheel_speeds.left + wheel_speeds.right) / 2
    actual_angular_velocity = (wheel_speeds.right - wheel_speeds.left) / (2 * half_wheel_distance)
    assert math.isclose(linear_velocity * full_forward_magnitude, actual_linear_velocity), \
        (f'Linear velocities were not equal (expected {linear_velocity * full_forward_magnitude}, '
         f'got {actual_linear_velocity})')
    assert math.isclose(angular_velocity * full_forward_magnitude, actual_angular_velocity), \
        (f'Angular velocities were not equal (expected {angular_velocity * full_forward_magnitude}, '
         f'got {actual_angular_velocity})')


def test_random1() -> None:
    verify_speeds(linear_velocity=0.5, angular_velocity=-0.5, full_forward_magnitude=0.7)


def test_random2() -> None:
    verify_speeds(linear_velocity=-0.6, angular_velocity=-0.7, full_forward_magnitude=0.7)


def test_random3() -> None:
    verify_speeds(linear_velocity=0.8, angular_velocity=0.1, full_forward_magnitude=0.7)


def test_random4() -> None:
    verify_speeds(linear_velocity=0.8, angular_velocity=0.1, full_forward_magnitude=0.5)


def test_random5() -> None:
    verify_speeds(linear_velocity=1.0, angular_velocity=1.0, full_forward_magnitude=0.5)