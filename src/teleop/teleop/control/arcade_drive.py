import math

from teleop_msgs.msg import GamepadState, StickPosition

from .base import DriveControlStrategy, WheelSpeeds


class ArcadeDrive(DriveControlStrategy):
    def __init__(self, invert_linear: bool = False, invert_turn: bool = False, square_inputs: bool = True):
        self.__invert_linear = invert_linear
        self.__invert_turn = invert_turn
        self.__square_inputs = square_inputs

    @property
    def invert_linear(self) -> bool:
        return self.__invert_linear

    @invert_linear.setter
    def invert_linear(self, value: bool) -> bool:
        self.__invert_linear = bool(value)

    @property
    def invert_turn(self) -> bool:
        return self.__invert_turn

    @invert_turn.setter
    def invert_turn(self, value: bool) -> bool:
        self.__invert_turn = bool(value)

    @property
    def square_inputs(self) -> bool:
        return self.__square_inputs

    @square_inputs.setter
    def square_inputs(self, value: bool) -> bool:
        self.__square_inputs = bool(value)

    def get_wheel_speeds(self, gamepad_state: GamepadState) -> WheelSpeeds:
        linear_rate = self.__linear_factor() * (gamepad_state.left_stick.y / abs(StickPosition.MAX_Y))
        turn_rate = -self.__turn_factor() * (gamepad_state.left_stick.x / abs(StickPosition.MAX_X))

        if self.__square_inputs:
            linear_rate = math.copysign(linear_rate * linear_rate, linear_rate)
            turn_rate = math.copysign(turn_rate * turn_rate, turn_rate)

        left_speed = linear_rate - turn_rate
        right_speed = linear_rate + turn_rate

        greater_input = max(abs(linear_rate), abs(turn_rate))
        lesser_input = min(abs(linear_rate), abs(turn_rate))

        if greater_input == 0.0:
            return WheelSpeeds(left=0, right=0)

        saturated_input = (greater_input + lesser_input) / greater_input
        left_speed /= saturated_input
        right_speed /= saturated_input

        return WheelSpeeds(left=left_speed, right=right_speed)

    def __linear_factor(self) -> int:
        return -1 if self.__invert_linear else 1

    def __turn_factor(self) -> int:
        return -1 if self.__invert_turn else 1
