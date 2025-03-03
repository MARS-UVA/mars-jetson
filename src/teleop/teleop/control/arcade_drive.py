import math

from teleop_msgs.msg import GamepadState, StickPosition

from .base import DriveControlStrategy, WheelSpeeds


class ArcadeDrive(DriveControlStrategy):
    """Drive control strategy where linear and angular velocity are controlled separately."""

    def __init__(self, full_forward_magnitude: float, invert_linear: bool = False, invert_turn: bool = False, shape: float = 1):
        if not (0 < full_forward_magnitude <= 1):
            raise ValueError(f'full_forward_magnitude must be between 0 (exclusive) and 1 (inclusive) (got {full_forward_magnitude})')
        self.__full_forward_magnitude = float(full_forward_magnitude)
        self.__invert_linear = invert_linear
        self.__invert_turn = invert_turn
        self.__shape = shape

    @property
    def full_forward_magnitude(self) -> float:
        return self.full_forward_magnitude

    @full_forward_magnitude.setter
    def full_forward_magnitude(self, value: float) -> float:
        if not (0 < value <= 1):
            raise ValueError(f'full_forward_magnitude must be between 0 (exclusive) and 1 (inclusive) (got {value})')
        self.__full_forward_magnitude = float(value)

    @property
    def __virtual_half_wheel_distance(self) -> float:
        # (1 / full_forward_magnitude) - 1
        return (1 / self.__full_forward_magnitude) - 1

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
    def shape(self) -> float:
        return self.__shape

    @shape.setter
    def shape(self, value: float):
        self.__shape = float(value)

    def get_wheel_speeds(self, gamepad_state: GamepadState) -> WheelSpeeds:
        linear_rate = self.__linear_factor() * (gamepad_state.left_stick.y / abs(StickPosition.MAX_Y))
        turn_rate = -self.__turn_factor() * (gamepad_state.left_stick.x / abs(StickPosition.MAX_X))

        if self.__shape != 1:
            linear_rate = math.copysign(linear_rate ** self.__shape, linear_rate)
            turn_rate = math.copysign(turn_rate ** self.__shape, turn_rate)

        angular_factor = self.__virtual_half_wheel_distance * turn_rate
        left_speed = linear_rate - angular_factor
        right_speed = linear_rate + angular_factor
        left_speed *= self.__full_forward_magnitude
        right_speed *= self.__full_forward_magnitude

        return WheelSpeeds(left=left_speed, right=right_speed)

    def __linear_factor(self) -> int:
        return -1 if self.__invert_linear else 1

    def __turn_factor(self) -> int:
        return -1 if self.__invert_turn else 1
