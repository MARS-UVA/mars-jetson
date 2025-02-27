import abc
from ..signal_processing import Clamp
from teleop_msgs.msg import GamepadState


class WheelSpeeds:
    __slots__ = '__left', '__right'
    __clamp = Clamp(-1.0, 1.0)

    def __init__(self, left: float, right: float):
        self.__left = self.__clamp(float(left))
        self.__right = self.__clamp(float(right))

    def __str__(self) -> str:
        return f'{type(self).__name__}(left={self.__left}, right={self.__right})'

    def __repr__(self) -> str:
        return f'{type(self).__name__}(left={self.__left!r}, right={self.__right!r})'


class DriveControlStrategy(abc.ABC):
    @abc.abstractmethod
    def get_wheel_speeds(self, gamepad_state: GamepadState) -> WheelSpeeds:
        pass
