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

def test_cruise_control_lt_causes_drum_speed_decrease() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.lb_pressed = True
    state.gamepad_state.rb_pressed = False
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 127)
    assert bucket_drum_set < 127

def test_cruise_control_rt_causes_drum_speed_increase() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.rb_pressed = True
    state.gamepad_state.lb_pressed = False
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 127)
    assert bucket_drum_set > 127

def test_cruise_control_lt_and_rt_causes_nothing() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.lb_pressed = True
    state.gamepad_state.rb_pressed = True
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 127)
    assert bucket_drum_set == 127

def test_cruise_control_overflow() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.lb_pressed = False
    state.gamepad_state.rb_pressed = True
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 255)
    assert bucket_drum_set == 255

def test_cruise_control_underflow() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.lb_pressed = True
    state.gamepad_state.rb_pressed = False
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 0)
    assert bucket_drum_set == 0

def test_cruis_control_reset() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.lb_pressed = True
    state.gamepad_state.rb_pressed = False
    bucket_drum_set = bucket_drum_speed_cruise_control(state, 127)
    assert bucket_drum_set != 127
    state.gamepad_state.y_pressed = True
    assert bucket_drum_set == 127