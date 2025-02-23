from rclpy.clock import Clock, Time


class Ramp:
    def __init__(self, ramp_rate: float | tuple[float, float], clock: Clock):
        if isinstance(ramp_rate, tuple):
            self.falling_ramp_rate, self.rising_ramp_rate = ramp_rate
        else:
            self.rising_ramp_rate = abs(ramp_rate)
            self.falling_ramp_rate = -self.rising_ramp_rate
        if self.falling_ramp_rate >= 0:
            raise ValueError('falling ramp rate must be negative')
        if self.rising_ramp_rate <= 0:
            raise ValueError('rising ramp rate must be positive')
        self.__clock = clock

        self.__last_input: float | None = None
        self.__last_input_time: Time | None = None
        self.__last_output: float | None = None

    def __call__(self, signal: float) -> float:
        output: float
        if self.__last_input is None or self.__last_input_time is None or self.__last_input is None:
            output = signal
            self.__last_input = signal
            self.__last_input_time = self.__clock.now()
        else:
            self.__last_input, output = signal, self.__process(self.__last_input, self.__last_input_time, signal)
        return output

    def __process(self, last_input: float, last_input_time: float, this_input: float) -> float:
        this_time = self.__clock.now()
        input_slope = (this_input - last_input) / (this_time - last_input_time)
        output_slope: float
        if input_slope < self.falling_ramp_rate:
            output_slope = self.falling_ramp_rate
        elif output_slope > self.rising_ramp_rate:
            output_slope = self.rising_ramp_rate
        else:
            output_slope = input_slope
        return output_slope * (this_time - last_input_time)
