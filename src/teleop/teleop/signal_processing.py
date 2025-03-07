from numbers import Real
from typing import Protocol
from rclpy.clock import Clock, Time


class SignalTransform(Protocol):
    """Protocol for a callable which transforms a signal."""
    def __call__(self, signal: float) -> float:
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


class Ramp:
    """A signal transform which limits how fast the output signal can change.

    Instances of ``Ramp`` have a falling and rising ramp rate and a ROS clock.
    """
    def __init__(self, ramp_rate: float | tuple[float, float], clock: Clock):
        """
        :param ramp_rate: Either a single value representing the maximum rate of change of the output signal in units
                          per second or a pair of values representing the maximum rate of decrease and increase,
                          respectively.
        :param clock: A ROS clock which indicates to the ramp how much time passes between signals.
        """
        self.__clamp = Clamp()
        if isinstance(ramp_rate, Real):
            self.__clamp.max_value = abs(float(ramp_rate))
            self.__clamp.min_value = -self.__rising_ramp_rate
        else:
            self.__clamp.min_value, self.__clamp.max_value = ramp_rate

        if self.falling_ramp_rate >= 0:
            raise ValueError('falling ramp rate must be negative')
        if self.rising_ramp_rate <= 0:
            raise ValueError('rising ramp rate must be positive')

        self.__clock = clock

        self.__last_time: Time | None = None
        self.__last_output: float | None = None

    @property
    def falling_ramp_rate(self) -> float:
        """Maximum rate at which the output signal can decrease, in units per second."""
        return self.__clamp.min_value

    @falling_ramp_rate.setter
    def falling_ramp_rate(self, value: float) -> None:
        if value >= 0:
            raise ValueError('falling ramp rate must be negative')
        self.__clamp.min_value = -abs(float(value))

    @property
    def rising_ramp_rate(self) -> float:
        """Maximum rate at which the output signal can increase, in units per second."""
        return self.__clamp.max_value

    @rising_ramp_rate.setter
    def rising_ramp_rate(self, value: float) -> None:
        if value <= 0:
            raise ValueError('rising ramp rate must be positive')
        self.__clamp.max_value = abs(float(value))

    def __call__(self, signal: float) -> float:
        this_time = self.__clock.now()
        if self.__last_time is None or self.__last_output is None:
            output = signal
        else:
            dt = (this_time - self.__last_time).nanoseconds / (10 ** 9)
            input_slope = (signal - self.__last_output) / dt
            output_slope = self.__clamp(input_slope)
            output = self.__last_output + (output_slope * dt)
        self.__last_output = output
        self.__last_time = this_time
        return output

    def reset(self) -> None:
        """Resets the ramp so that the next input signal is not transformed."""
        self.__last_time = None
        self.__last_output = None
