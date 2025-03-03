import abc
import math
from typing import Any
from ..signal_processing import Clamp
from teleop_msgs.msg import GamepadState


class WheelSpeeds:
    __slots__ = '__left', '__right'
    __clamp = Clamp(-1.0, 1.0)

    def __init__(self, left: float, right: float):
        self.__left = self.__clamp(float(left))
        self.__right = self.__clamp(float(right))

    @property
    def left(self) -> float:
        return self.__left

    @property
    def right(self) -> float:
        return self.__right

    def __eq__(self, value: Any) -> bool:
        if not isinstance(value, WheelSpeeds):
            return False
        return (self.__left == value.__left
                and self.__right == value.__right)

    def is_close(self, other: 'WheelSpeeds', rel_tol: float = 1e-9, abs_tol: float = 0) -> bool:
        if not isinstance(other, WheelSpeeds):
            raise TypeError('other must be of type WheelSpeeds')
        return (math.isclose(self.__left, other.__left, rel_tol=rel_tol, abs_tol=abs_tol)
               and math.isclose(self.__right, other.__right, rel_tol=rel_tol, abs_tol=abs_tol))

    def __str__(self) -> str:
        return f'{type(self).__name__}(left={self.__left}, right={self.__right})'

    def __repr__(self) -> str:
        return f'{type(self).__name__}(left={self.__left!r}, right={self.__right!r})'


class DriveControlStrategy(abc.ABC):
    @abc.abstractmethod
    def get_wheel_speeds(self, gamepad_state: GamepadState) -> WheelSpeeds:
        pass
