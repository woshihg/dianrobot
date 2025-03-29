
# 高度控制
class HeightController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self.target_height = 0.0
        self.current_height = 0.0
        self.tolerance = 0.1
        self.ratio = 1.0

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出角度
    def calc_current_target(self) -> float:
        delta = self.target_height - self.current_height
        abs_delta = abs(delta)

        if abs_delta < self.tolerance:
            return self.current_height
        return self.current_height + delta * self.ratio

    # region Getters and Setters

    def get_target(self):
        return self.target_height

    def get_current(self):
        return self.current_height

    def get_ratio(self):
        return self.ratio

    def get_tolerance(self):
        return self.tolerance

    def set_target(self, target):
        self.target_height = target

    def set_current(self, current):
        self.current_height = current

    def set_ratio(self, ratio):
        self.ratio = ratio

    def set_tolerance(self, tolerance):
        self.tolerance = tolerance

    # endregion Getters and Setters