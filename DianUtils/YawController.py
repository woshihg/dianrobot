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
        self.target_yaw = 0.0
        self.__current_yaw__ = 0.0
        self.tolerance = 0.1
        self.ratio = 0.5

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出电机值
    def calc_current_target(self) -> float:
        delta = self.target_yaw - self.__current_yaw__
        abs_delta = abs(delta)
        if abs_delta > 180.0:
            delta = 360.0 - abs_delta
            abs_delta = delta

        if abs_delta < self.tolerance:
            return 0
        return delta * 0.005

    def check_is_done(self) -> bool:
        delta = self.target_yaw - self.__current_yaw__
        abs_delta = abs(delta)
        if abs_delta > 180.0:
            delta = 360.0 - abs_delta
            abs_delta = delta

        return abs_delta < self.tolerance

    # region Getters and Setters

    def get_target(self):
        return self.target_yaw

    def get_current(self):
        return self.__current_yaw__

    def get_ratio(self):
        return self.ratio

    def get_tolerance(self):
        return self.tolerance

    def set_target(self, target):
        self.target_yaw = _cut_angle(target)

    def set_current(self, current):
        self.__current_yaw__ = _cut_angle(current)

    def set_ratio(self, ratio):
        self.ratio = ratio

    def set_tolerance(self, tolerance):
        self.tolerance = tolerance

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

