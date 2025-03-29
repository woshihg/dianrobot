import numpy as np

# 角度控制器，发送插值角度控制信号
class ForwardController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self._target_pos = np.array([0.0, 0.0])
        self._current_pos = np.array([0.0, 0.0])
        self._origin_pos = np.array([0.0, 0.0])
        self._tolerance = 0.01
        self._ratio = 0.5

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出电机值
    def calc_current_target(self) -> float:
        delta = self._target_pos - self._current_pos
        abs_delta = np.dot(delta, delta)
        origin_delta = self._target_pos - self._origin_pos
        direction = np.dot(delta, origin_delta)
        if direction < 0:
            abs_delta = -abs_delta
        return abs_delta * 0.005

    def check_is_done(self) -> bool:
        delta = self._target_pos - self._current_pos
        abs_delta = np.dot(delta, delta)

        return abs_delta < self._tolerance

    # region Getters and Setters

    def get_target(self):
        return self._target_pos

    def get_current(self):
        return self._current_pos

    def get_ratio(self):
        return self._ratio

    def get_tolerance(self):
        return self._tolerance

    def get_origin_pos(self):
        return self._origin_pos

    def set_target(self, target):
        self._target_pos = target

    def set_current(self, current):
        self._current_pos = current

    def set_ratio(self, ratio):
        self._ratio = ratio

    def set_tolerance(self, tolerance):
        self._tolerance = tolerance

    def set_origin_pos(self, origin_pos):
        self._origin_pos = origin_pos

    # endregion Getters and Setters


class ForwardControlSender:
    def __init__(self):
        pass

    def send(self, yaw: float) -> None:
        pass


class ForwardMotorReceiver:
    def __init__(self):
        pass

    def receive(self) -> float:
        pass

