from teleop.motor_queries import bucket_actuator_speed, bucket_drum_speed_cruise_control
from teleop_msgs.msg import HumanInputState, SetMotor


def test_dpad_up_causes_actuator_up() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.du_pressed = True
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity > 127


def test_dpad_down_causes_actuator_down() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.dd_pressed = True
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity < 127


def test_no_dpad_causes_actuator_zero() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity == 127


def test_both_dpad_causes_actuator_zero() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.du_pressed = True
    state.gamepad_state.dd_pressed = True
    bucket_actuator_set = bucket_actuator_speed(state)
    assert bucket_actuator_set.velocity == 127

def test_cruise_control_rt_causes_drum_speed_increase() -> None:
    state = HumanInputState
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.lt_pressed = 1
    state.gamepad_state.rt_pressed = 0
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 127)
    assert bucket_drum_set > 127

def test_cruise_control_lt_causes_drum_speed_decrease() -> None:
    state = HumanInputState
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.rt_pressed = 1
    state.gamepad_state.lt_pressed = 0
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 127)
    assert bucket_drum_set < 127

def test_cruise_control_lt_and_rt_causes_nothing() -> None:
    state = HumanInputState
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.lt_pressed = 1
    state.gamepad_state.rt_pressed = 1
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 127)
    assert bucket_drum_set == 127