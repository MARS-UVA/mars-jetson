from dataclasses import dataclass, field
from functools import cached_property
import math
from typing import Generic, Protocol, TypeVar, runtime_checkable
import warnings

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header


@runtime_checkable
class StampedMessage(Protocol):
    """A message that has a Header."""
    @property
    def header(self) -> Header:
        pass

    @header.setter
    def header(self, value: Header) -> None:
        pass


MessageType = TypeVar('MessageType', bound=StampedMessage)


@dataclass(frozen=True, order=True, kw_only=True)
class Event(Generic[MessageType]):
    """A single event at a given time in the simulation.

    A simulation event has a timestamp as a floating point in seconds and a message which will be
    sent at that time.

    The message provided to the initializer will have its header attribute (if it exists) modified
    to match the current time.
    """
    timestamp: float
    message: StampedMessage = field(compare=False)

    def __post_init__(self) -> None:
        if hasattr(self.message, 'header'):
            self.message.header.stamp = self._time
        else:
            warnings.warn(f'Message type "{type(self.message).__qualname__}" does not have a header')

    @cached_property
    def _time(self) -> Time:
        """An object representing the time at which the message is sent."""
        nanoseconds, seconds = math.modf(self.timestamp)
        return Time(sec=round(seconds), nanosec=round(nanoseconds * (10 ** 10)))

    def get_clock(self) -> Clock:
        """Get a clock message that corresponds to the event's time."""
        return Clock(time=self._time)
