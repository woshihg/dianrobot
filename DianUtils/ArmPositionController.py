import numpy as np


# 角度控制器，发送插值角度控制信号
class ArmPositionController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.tolerance = 0.1  # dot product tolerance
        self.ratio = 0.5

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出角度
    def calc_current_target(self) -> np.array:
        delta = self.target_pos - self.current_pos
        square = np.dot(delta, delta)

        if square < self.tolerance:
            return self.current_pos
        return self.current_pos + delta * self.ratio

    # region Getters and Setters

    def get_target(self):
        return self.target_pos

    def get_current(self):
        return self.current_pos

    def get_ratio(self):
        return self.ratio

    def get_tolerance(self):
        return self.tolerance

    def set_target(self, target):
        self.target_pos = target

    def set_current(self, current):
        self.current_pos = current

    def set_ratio(self, ratio):
        self.ratio = ratio

    def set_tolerance(self, tolerance):
        self.tolerance = tolerance

    # endregion Getters and Setters
