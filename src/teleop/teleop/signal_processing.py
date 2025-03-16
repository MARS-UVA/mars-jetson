from typing import Protocol, Self


class SignalTransform(Protocol):
    """Protocol for a callable which transforms a signal."""
    def __call__(self, signal: float) -> float:
        ...

    def copy(self) -> Self:
        ...


class Deadband:
    """A signal transform which outputs 0 when the input signal has a magnitude less than the given magnitude.

    The deadband transform helps prevent noisy signals from causing spurious behavior when the intended input is 0.
    """
    def __init__(self, min_magnitude: float = 0.0):
        """
        :param min_magnitude: Minimum magnitude below which the signal is output as 0 (default: 0).
        """
        self.__min_magnitude = max(float(min_magnitude), 0)

    @property
    def min_magnitude(self) -> float:
        """Minimum magnitude below which the signal is output as 0."""
        return self.__min_magnitude

    @min_magnitude.setter
    def min_magnitude(self, value: float) -> None:
        self.__min_magnitude = max(float(value), 0)

    def __call__(self, signal: float) -> float:
        if signal < 0:
            return signal if signal <= -self.__min_magnitude else 0
        else:
            return signal if signal >= self.__min_magnitude else 0

    def copy(self) -> Self:
        return type(self)(min_magnitude=self.__min_magnitude)


class Clamp:
    """A signal transform which clamps the input signal within the given range."""
    def __init__(self, min_value: float = -float('inf'), max_value: float = float('inf')):
        """
        :param min_value: Minimum value of the output signal (default: -inf).
        :param max_value: Maximum value of the output signal (default: +inf).
        """
        self.__min_value = float(min_value)
        self.__max_value = float(max_value)

    @property
    def min_value(self) -> float:
        """ Minimum value of the output signal."""
        return self.__min_value

    @min_value.setter
    def min_value(self, value: float) -> None:
        self.__min_value = float(value)

    @property
    def max_value(self) -> float:
        """Maximum value of the output signal."""
        return self.__max_value

    @max_value.setter
    def max_value(self, value: float) -> None:
        self.__max_value = float(value)

    def __call__(self, signal: float) -> float:
        return max(self.__min_value, min(signal, self.__max_value))

    def copy(self) -> Self:
        return type(self)(min_value=self.__min_value, max_value=self.__max_value)
