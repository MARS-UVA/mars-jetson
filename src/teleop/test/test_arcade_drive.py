from teleop_msgs.msg import GamepadState, StickPosition

from teleop.control import ArcadeDrive, WheelSpeeds


def test_full_forward() -> None:
    state = GamepadState(left_stick=StickPosition(y=abs(StickPosition.MAX_Y)))
    drive = ArcadeDrive()
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=+1.0, right=+1.0)), f'Wheel speeds {wheel_speeds} are not full forward'


def test_full_reverse() -> None:
    state = GamepadState(left_stick=StickPosition(y=-abs(StickPosition.MAX_Y)))
    drive = ArcadeDrive()
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=-1.0, right=-1.0)), f'Wheel speeds {wheel_speeds} are not full reverse'


def test_full_turn_ccw() -> None:
    state = GamepadState(left_stick=StickPosition(x=-abs(StickPosition.MAX_X)))
    drive = ArcadeDrive()
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=-1.0, right=+1.0)), f'Wheel speeds {wheel_speeds} are not full CCW turn'


def test_full_turn_cw() -> None:
    state = GamepadState(left_stick=StickPosition(x=abs(StickPosition.MAX_X)))
    drive = ArcadeDrive()
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=+1.0, right=-1.0)), f'Wheel speeds {wheel_speeds} are not full CW turn'


def test_invert_linear() -> None:
    state = GamepadState(left_stick=StickPosition(y=abs(StickPosition.MAX_Y)))
    drive = ArcadeDrive(invert_linear=True)
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=-1.0, right=-1.0)), f'Wheel speeds {wheel_speeds} are not full reverse'


def test_invert_turn() -> None:
    state = GamepadState(left_stick=StickPosition(x=-abs(StickPosition.MAX_X)))
    drive = ArcadeDrive(invert_turn=True)
    wheel_speeds = drive.get_wheel_speeds(state)
    assert wheel_speeds.is_close(WheelSpeeds(left=+1.0, right=-1.0)), f'Wheel speeds {wheel_speeds} are not full CW turn'
