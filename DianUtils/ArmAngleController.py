import numpy as np
# 角度控制器，发送插值角度控制信号
class ArmAngleController:
    def __init__(self):
        # ranged form -pi to pi
        self._target_angle = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._current_angle = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._tolerance = 0.1  # dot product tolerance
        self._ratio = 0.1

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出角度
    def calc_current_target(self) -> np.array:
        delta = self._target_angle - self._current_angle
        if np.dot(delta, delta) < 2 * (self._ratio ** 2):
            return self._target_angle

        # normalize the diff, to keep increment in fixed length
        direction = delta / np.linalg.norm(delta)
        return self._current_angle + direction * self._ratio

    def check_is_done(self) -> bool:
        diff_pos = self._target_angle - self._current_angle
        return np.dot(diff_pos, diff_pos) < (self._tolerance ** 2)

    # region Getters and Setters

    def get_target(self):
        return self._target_angle

    def get_current(self):
        return self._current_angle

    def get_ratio(self):
        return self._ratio

    def get_tolerance(self):
        return self._tolerance

    def set_target(self, target: np.array):
        if type(target) != np.ndarray:
            target = np.array(target)
        self._target_angle = target
        return self

    def set_current(self, current: np.array):
        if type(current) != np.ndarray:
            current = np.array(current)
        self._current_angle = current
        return self

    def set_ratio(self, ratio):
        self._ratio = ratio
        return self

    def set_tolerance(self, tolerance):
        self._tolerance = tolerance
        return self

    # endregion Getters and Setters