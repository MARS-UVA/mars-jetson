from teleop.motor_queries import bucket_actuator_speed
from teleop_msgs.msg import HumanInputState, SetMotor


def test_dpad_up_causes_actuator_up() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.du_pressed = True
    motor_changes = bucket_actuator_speed(state)
    actuator_value = next((command.velocity for command in motor_changes.changes if command.index == SetMotor.BUCKET_DRUM_ACTUATOR), None)
    assert actuator_value is not None, 'Actuator value was not changed'
    assert actuator_value > 127


def test_dpad_down_causes_actuator_down() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.dd_pressed = True
    motor_changes = bucket_actuator_speed(state)
    actuator_value = next((command.velocity for command in motor_changes.changes if command.index == SetMotor.BUCKET_DRUM_ACTUATOR), None)
    assert actuator_value is not None, 'Actuator value was not changed'
    assert actuator_value < 127


def test_no_dpad_causes_actuator_zero() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    motor_changes = bucket_actuator_speed(state)
    actuator_value = next((command.velocity for command in motor_changes.changes if command.index == SetMotor.BUCKET_DRUM_ACTUATOR), None)
    assert actuator_value is not None, 'Actuator value was not changed'
    assert actuator_value == 127


def test_both_dpad_causes_actuator_zero() -> None:
    state = HumanInputState()
    state.drive_mode = HumanInputState.DRIVEMODE_TELEOP
    state.gamepad_state.du_pressed = True
    state.gamepad_state.dd_pressed = True
    motor_changes = bucket_actuator_speed(state)
    actuator_value = next((command.velocity for command in motor_changes.changes if command.index == SetMotor.BUCKET_DRUM_ACTUATOR), None)
    assert actuator_value is not None, 'Actuator value was not changed'
    assert actuator_value == 127