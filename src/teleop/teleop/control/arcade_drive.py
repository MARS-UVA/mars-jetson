import math
from typing import override

from teleop_msgs.msg import GamepadState

from .base import DriveControlStrategy, WheelSpeeds, GamepadAxis
from ..signal_processing import Deadband


class ArcadeDrive(DriveControlStrategy):
    """Drive control strategy where linear and angular velocity are controlled separately.

    The client specifies the gamepad axes corresponding to linear and angular movement, magni
    """

    def __init__(self,
                 linear_axis: GamepadAxis,
                 turn_axis: GamepadAxis,
                 full_forward_magnitude: float,
                 shape: float = 1,
                 deadband: Deadband = Deadband()):
        """
        :param linear_axis: The gamepad axis which corresponds to linear velocity. Positive values indicate
                            forward movement.
        :param turn_axis: The gamepad axis which corresponds to angular velocity. Positive values indicate
                          counterclockwise turning.
        :param full_forward_magnitude: The magnitude of both wheel's speeds when the user inputs
                                       completely forward. This affects the amount of wheel speed which will
                                       be devoted to turning. 0 indicates the robot can only spin in place,
                                       and 1 indicates the robot can only move forward and backward.
        :param shape: A parameter describing the shape of the curve that converts axis inputs into speeds.
                      The axis input is raised to this power (keeping the sign), so 1 is linear (default: 1).
        :param deadband: A ``Deadband`` transformation which will be applied to the gamepad inputs
                        (default: ``Deadband(min_magnitude=0)``).
        """
        if not (0 <= full_forward_magnitude <= 1):
            raise ValueError(f'full_forward_magnitude must be between 0 and 1 (got {full_forward_magnitude})')
        if not isinstance(linear_axis, GamepadAxis):
            raise TypeError(f'linear_axis must be of type {GamepadAxis.__name__}')
        if not isinstance(turn_axis, GamepadAxis):
            raise TypeError(f'turn_axis must be of type {GamepadAxis.__name__}')
        if shape <= 0:
            raise ValueError('shape must be positive')
        self.__linear_axis = linear_axis
        self.__turn_axis = turn_axis
        self.__full_forward_magnitude = float(full_forward_magnitude)
        self.__shape = float(shape)
        self.__deadband = deadband

    @property
    def linear_axis(self) -> GamepadAxis:
        """Gamepad axis which corresponds to linear velocity.

        Positive values indicate forward movement.
        """
        return self.__linear_axis

    @property
    def turn_axis(self) -> GamepadAxis:
        """Gamepad axis which corresponds to angular velocity.

        Positive values indicate counterclockwise movement.
        """
        return self.__turn_axis

    @property
    def full_forward_magnitude(self) -> float:
        """The magnitude of both wheel's speeds when the user inputs completely forward.

        This affects the amount of wheel speed which will be devoted to turning. 0 indicates the robot can only spin
        in place, and 1 indicates the robot can only move forward and backward.
        """
        return self.__full_forward_magnitude

    @full_forward_magnitude.setter
    def full_forward_magnitude(self, value: float) -> float:
        if not (0 <= value <= 1):
            raise ValueError(f'full_forward_magnitude must be between 0 and 1 (got {value})')
        self.__full_forward_magnitude = float(value)

    @property
    def shape(self) -> float:
        """A parameter describing the shape of the curve that converts axis inputs into speeds.

        The axis input is raised to this power (keeping the sign), so 1 is linear. In general, larger inputs make it
        easier for the user to make inputs for slow movement, whereas smaller inputs make it easier for the user to
        make inputs for fast movement.
        """
        return self.__shape

    @shape.setter
    def shape(self, value: float):
        self.__shape = float(value)

    @property
    def deadband(self) -> Deadband:
        """A ``Deadband`` signal transform applied to gamepad axis values."""
        return self.__deadband

    @override
    def get_wheel_speeds(self, gamepad_state: GamepadState) -> WheelSpeeds:
        linear_rate = self.__deadband(self.__linear_axis.of(gamepad_state))
        turn_rate = self.__deadband(self.__turn_axis.of(gamepad_state))

        if self.__shape != 1:
            linear_rate = math.copysign(abs(linear_rate) ** self.__shape, linear_rate)
            turn_rate = math.copysign(abs(turn_rate) ** self.__shape, turn_rate)
        linear_component = self.__full_forward_magnitude * linear_rate
        angular_component = (1 - self.__full_forward_magnitude) * turn_rate
        return WheelSpeeds(left=linear_component - angular_component,
                           right=linear_component + angular_component)
