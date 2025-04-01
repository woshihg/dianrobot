
# 高度控制
class HeightController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self._target_height = 0.0
        self._current_height = 0.0
        self._tolerance = 0.001
        self._ratio = 0.03

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出角度
    def calc_current_target(self) -> float:
        delta = self._target_height - self._current_height
        if abs(delta) < 2 * self._ratio:
            return self._target_height

        # normalize the diff, to keep increment in fixed length
        direction = delta / abs(delta)
        return self._current_height + direction * self._ratio

    def check_is_done(self) -> bool:
        delta = self._target_height - self._current_height
        return abs(delta) < self._tolerance

    # region Getters and Setters

    def get_target(self):
        return self._target_height

    def get_current(self):
        return self._current_height

    def get_ratio(self):
        return self._ratio

    def get_tolerance(self):
        return self._tolerance

    def set_target(self, target):
        self._target_height = float(target)
        return self

    def set_current(self, current):
        self._current_height = current
        return self

    def set_ratio(self, ratio):
        self._ratio = ratio

    def set_tolerance(self, tolerance):
        self._tolerance = tolerance

    # endregion Getters and Setters

class HeightControlSender:
    def __init__(self):
        pass

    def send(self, height: float) -> None:
        pass

class HeightMotorReceiver:
    def __init__(self):
        pass

    def receive(self) -> float:
        pass

