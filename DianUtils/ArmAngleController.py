import numpy as np


# 角度控制器，发送插值角度控制信号
class ArmAngleController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self._target_angle = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._current_angle = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.tolerance = 0.001  # dot product tolerance
        self.ratio = 0.1

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出角度
    def calc_current_target(self) -> np.array:
        delta = self._target_angle - self._current_angle
        square = np.dot(delta, delta)

        if square < self.tolerance:
            return self._target_angle
        return self._current_angle + delta * self.ratio

    # region Getters and Setters

    def get_target(self):
        return self._target_angle

    def get_current(self):
        return self._current_angle

    def get_ratio(self):
        return self.ratio

    def get_tolerance(self):
        return self.tolerance

    def set_target(self, target):
        self._target_angle = target
        return self

    def set_current(self, current):
        self._current_angle = current
        return self

    def set_ratio(self, ratio):
        self.ratio = ratio
        return self

    def set_tolerance(self, tolerance):
        self.tolerance = tolerance
        return self

    # endregion Getters and Setters
