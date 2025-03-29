def _cut_angle(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


# 角度控制器，发送插值角度控制信号
class YawController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self._target_yaw = 0.0
        self._current_yaw = 0.0
        self._tolerance = 0.1
        self._ratio = 0.5

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出电机值
    def calc_current_target(self) -> float:
        delta = self._target_yaw - self._current_yaw
        abs_delta = abs(delta)
        if abs_delta > 180.0:
            delta = 360.0 - abs_delta
            abs_delta = delta

        if abs_delta < self._tolerance:
            return 0
        return delta * 0.005

    def check_is_done(self) -> bool:
        delta = self._target_yaw - self._current_yaw
        abs_delta = abs(delta)
        if abs_delta > 180.0:
            delta = 360.0 - abs_delta
            abs_delta = delta

        return abs_delta < self._tolerance

    # region Getters and Setters

    def get_target(self):
        return self._target_yaw

    def get_current(self):
        return self._current_yaw

    def get_ratio(self):
        return self._ratio

    def get_tolerance(self):
        return self._tolerance

    def set_target(self, target):
        self._target_yaw = _cut_angle(target)
        return self

    def set_current(self, current):
        self._current_yaw = _cut_angle(current)
        return self

    def set_ratio(self, ratio):
        self._ratio = ratio
        return self

    def set_tolerance(self, tolerance):
        self._tolerance = tolerance
        return self

    # endregion Getters and Setters


class YawControlSender:
    def __init__(self):
        pass

    def send(self, yaw: float) -> None:
        pass


class YawMotorReceiver:
    def __init__(self):
        pass

    def receive(self) -> float:
        pass

