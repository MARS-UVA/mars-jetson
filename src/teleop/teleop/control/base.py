import abc
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Any, Self
from ..signal_processing import Clamp
from teleop_msgs.msg import GamepadState, StickPosition


class WheelSpeeds:
    """A class which represents speeds of the robot's left set and right set of wheels."""
    __slots__ = '__left', '__right'
    __clamp = Clamp(-1.0, 1.0)

    def __init__(self, left: float, right: float):
        """
        :param left: Speed of the left set of wheels as a proportion (-1.0 to 1.0, inclusive). Values outside this
                     range are clamped to be between -1 and 1.
        :param right: Speed of the right set of wheels as a proportion (-1.0 to 1.0, inclusive). Values outside this
                      range are clamped to be between -1 and 1.
        """
        self.__left = self.__clamp(float(left))
        self.__right = self.__clamp(float(right))
    
    def left(self) -> float:
        return self.__left
    
    def right(self) -> float:
        return self.__right

    @property
    def left(self) -> float:
        """Speed of the left set of wheels as a proportion (-1.0 to 1.0, inclusive)."""
        return self.__left

    @property
    def right(self) -> float:
        """Speed of the right set of wheels as a proportion (-1.0 to 1.0, inclusive)."""
        return self.__right

    def scaled_by(self, scale_factor: float) -> Self:
        """Return a new `WheelSpeeds` where the wheel speeds are scaled by the given factor.

        If the scaling causes a wheel speed to exceed 1.0 in magnitude, the value is clamped to be between -1 and 1.
        """
        return type(self)(left=self.__left * scale_factor,
                          right=self.__right * scale_factor)

    def apply(self, func: Callable[[float], float]) -> Self:
        """Return a new `WheelSpeeds` where the wheel speeds are the image of the originals under the given function.

        If the new wheel speed exceeds 1.0 in magnitude, the value is clamped to be between -1 and 1.
        """
        return type(self)(left=func(self.__left),
                          right=func(self.__right))

    def __eq__(self, value: Any) -> bool:
        if not isinstance(value, WheelSpeeds):
            return False
        return (self.__left == value.__left
                and self.__right == value.__right)

    def is_close(self, other: 'WheelSpeeds', rel_tol: float = 1e-9, abs_tol: float = 0) -> bool:
        """Returns ``True`` if the wheel speeds are close within a given tolerance.

        See :func:`math.isclose` for more information.

        :param rel_tol: Maximum difference for being considered "close", relative to the magnitude of the input values.
        :param abs_tol: Maximum difference for being considered "close", regardless of the magnitude of the
                        input values.
        :return: ``True`` if the wheel speeds are close within the given tolerance, and ``False`` otherwise.
        """
        if not isinstance(other, WheelSpeeds):
            raise TypeError('other must be of type WheelSpeeds')
        return (math.isclose(self.__left, other.__left, rel_tol=rel_tol, abs_tol=abs_tol)
               and math.isclose(self.__right, other.__right, rel_tol=rel_tol, abs_tol=abs_tol))

    def __str__(self) -> str:
        return f'{type(self).__name__}(left={self.__left}, right={self.__right})'

    def __repr__(self) -> str:
        return f'{type(self).__name__}(left={self.__left!r}, right={self.__right!r})'
    

class _GamepadAxisId(Enum):
    """ID for an axis without any inversion data."""
    LEFT_X = auto()
    LEFT_Y = auto()
    RIGHT_X = auto()
    RIGHT_Y = auto()


@dataclass(frozen=True, kw_only=True)
class AxisProperties:
    """Properties of a ``GamepadAxis``."""
    id_: _GamepadAxisId
    inverted: bool


class GamepadAxis(Enum):
    """An axis of the gamepad, which is possibly inverted."""
    LEFT_X = AxisProperties(id_=_GamepadAxisId.LEFT_X, inverted=False)
    LEFT_Y = AxisProperties(id_=_GamepadAxisId.LEFT_Y, inverted=False)
    RIGHT_X = AxisProperties(id_=_GamepadAxisId.RIGHT_X, inverted=False)
    RIGHT_Y = AxisProperties(id_=_GamepadAxisId.RIGHT_Y, inverted=False)

    LEFT_X_INVERTED = AxisProperties(id_=_GamepadAxisId.LEFT_X, inverted=True)
    LEFT_Y_INVERTED = AxisProperties(id_=_GamepadAxisId.LEFT_Y, inverted=True)
    RIGHT_X_INVERTED = AxisProperties(id_=_GamepadAxisId.RIGHT_X, inverted=True)
    RIGHT_Y_INVERTED = AxisProperties(id_=_GamepadAxisId.RIGHT_Y, inverted=True)

    def of(self, gamepad_state: GamepadState) -> float:
        """Obtain the value of the given axis in the given gamepad state."""
        if self.value.id_ == _GamepadAxisId.LEFT_X:
            value = gamepad_state.left_stick.x / StickPosition.MAX_X
        elif self.value.id_ == _GamepadAxisId.LEFT_Y:
            value = gamepad_state.left_stick.y / StickPosition.MAX_Y
        elif self.value.id_ == _GamepadAxisId.RIGHT_X:
            value = gamepad_state.right_stick.x / StickPosition.MAX_X
        elif self.value.id_ == _GamepadAxisId.RIGHT_Y:
            value = gamepad_state.right_stick.y / StickPosition.MAX_Y
        else:
            raise RuntimeError('axis is invalid; implementation error likely')
        return -value if self.value.inverted else value


class DriveControlStrategy(abc.ABC):
    @abc.abstractmethod
    def get_wheel_speeds(self, gamepad_state: GamepadState) -> WheelSpeeds:
        """Gets speeds for the robot's wheels for the given gamepad state."""
        pass
