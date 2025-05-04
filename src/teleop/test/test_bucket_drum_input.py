from teleop.motor_queries import bucket_actuator_speed
from teleop_msgs.msg import HumanInputState


def test_dpad_up_causes_actuator_up() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_NORMAL
    state.gamepad_state.du_pressed = True
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity > 127


def test_dpad_down_causes_actuator_down() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_NORMAL
    state.gamepad_state.dd_pressed = True
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity < 127


def test_no_dpad_causes_actuator_zero() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_NORMAL
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity == 127


def test_both_dpad_causes_actuator_zero() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_NORMAL
    state.gamepad_state.du_pressed = True
    state.gamepad_state.dd_pressed = True
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity == 127
