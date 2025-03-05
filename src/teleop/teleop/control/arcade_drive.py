import math

from teleop_msgs.msg import GamepadState

from .base import DriveControlStrategy, WheelSpeeds, GamepadAxis


class ArcadeDrive(DriveControlStrategy):
    """Drive control strategy where linear and angular velocity are controlled separately."""

    def __init__(self, linear_axis: GamepadAxis, turn_axis: GamepadAxis, full_forward_magnitude: float, shape: float = 1):
        if not (0 < full_forward_magnitude <= 1):
            raise ValueError(f'full_forward_magnitude must be between 0 (exclusive) and 1 (inclusive) (got {full_forward_magnitude})')
        if not isinstance(linear_axis, GamepadAxis):
            raise TypeError(f'linear_axis must be of type {GamepadAxis.__name__}')
        if not isinstance(turn_axis, GamepadAxis):
            raise TypeError(f'turn_axis must be of type {GamepadAxis.__name__}')
        self.__linear_axis = linear_axis
        self.__turn_axis = turn_axis
        self.__full_forward_magnitude = float(full_forward_magnitude)
        self.__shape = shape

    @property
    def linear_axis(self) -> GamepadAxis:
        return self.__linear_axis

    @property
    def turn_axis(self) -> GamepadAxis:
        return self.__turn_axis

    @property
    def full_forward_magnitude(self) -> float:
        return self.__full_forward_magnitude

    @full_forward_magnitude.setter
    def full_forward_magnitude(self, value: float) -> float:
        if not (0 < value <= 1):
            raise ValueError(f'full_forward_magnitude must be between 0 (exclusive) and 1 (inclusive) (got {value})')
        self.__full_forward_magnitude = float(value)

    @property
    def shape(self) -> float:
        return self.__shape

    @shape.setter
    def shape(self, value: float):
        self.__shape = float(value)

    def get_wheel_speeds(self, gamepad_state: GamepadState) -> WheelSpeeds:
        linear_rate = self.__linear_axis.of(gamepad_state)
        turn_rate = self.__turn_axis.of(gamepad_state)

        if self.__shape != 1:
            linear_rate = math.copysign(abs(linear_rate) ** self.__shape, linear_rate)
            turn_rate = math.copysign(abs(turn_rate) ** self.__shape, turn_rate)

        angular_factor = self.__virtual_half_wheel_distance * turn_rate
        left_speed = linear_rate - angular_factor
        right_speed = linear_rate + angular_factor
        left_speed *= self.__full_forward_magnitude
        right_speed *= self.__full_forward_magnitude

        return WheelSpeeds(left=left_speed, right=right_speed)

    @property
    def __virtual_half_wheel_distance(self) -> float:
        return (1 / self.__full_forward_magnitude) - 1
