import math
import time


# 周期发出 e^(-t) 的信号生成器
class SignalGenerator:
    def __init__(self, interval_s : float = 6.0):
        self.time_begin_s = time.time()
        self._interval_s = interval_s

    def calc_signal(self, current_time: float) -> float:
        while self.time_begin_s + self._interval_s < current_time:
            self.time_begin_s += self._interval_s

        delta_t = current_time - self.time_begin_s
        signal = math.exp(-delta_t)
        return signal

# region Getters and Setters

    def get_interval(self):
        return self._interval_s

    def set_interval(self, interval):
        self._interval_s = interval

    def get_time_begin(self):
        return self.time_begin_s

    def set_time_begin(self, time_begin):
        self.time_begin_s = time_begin
        
# endregion Getters and Setters


# 返回常量的信号生成器
class ConstSignalGenerator:
    def __init__(self, const: float = 1.0):
        self.const = const

    # leave the empty argument for compatibility to SignalGenerator
    def calc_signal(self, current_time: float) -> float:
        return self.const

    def get_const(self):
        return self.const

    def set_const(self, const):
        self.const = const

