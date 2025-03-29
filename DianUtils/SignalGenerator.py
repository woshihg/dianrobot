import numpy as np
import time
import math


# 周期发出 e^(-t) 的信号生成器
class SignalGenerator:
    def __init__(self):
        self.time_begin_s = time.time()
        self.interval_s = 6.0

    def calc_signal(self, current_time: float) -> float:
        while self.time_begin_s + self.interval_s < current_time:
            self.time_begin_s += self.interval_s

        delta_t = current_time - self.time_begin_s
        signal = math.exp(-delta_t)
        return signal


