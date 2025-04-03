import numpy as np

# 前后移动控制，给定当前方向，当前位置和目标位置，计算电机值
class ForwardController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self._target_pos = np.array([0.0, 0.0])
        self._current_pos = np.array([0.0, 0.0])
        self._direction_vec = np.array([1.0, 0.0])
        self._tolerance = 0.01
        self._ratio = 0.5

    def calc_current_target(self) -> float:
        delta = self._target_pos - self._current_pos
        abs_delta = np.dot(delta, delta)
        direction = np.dot(delta, self._direction_vec)
        if direction < 0:
            abs_delta = -abs_delta
        return abs_delta * 5

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

    def get_direction_vec(self):
        return self._direction_vec

    def set_target(self, target: np.array):
        if not isinstance(target, np.ndarray):
            target = np.array(target)
        self._target_pos = target.copy()
        return self

    def set_current(self, current: np.array):
        if not isinstance(current, np.ndarray):
            current = np.array(current)
        self._current_pos = current.copy()
        return self

    def set_ratio(self, ratio):
        self._ratio = ratio
        return self

    def set_tolerance(self, tolerance):
        self._tolerance = tolerance
        return self

    def set_direction_vec(self, origin: np.array):
        if not isinstance(origin, np.ndarray):
            origin = np.array(origin)
        self._direction_vec = origin.copy()
        return self

    # endregion Getters and Setters


class ForwardControlSender:
    def __init__(self):
        pass

    def send(self, yaw: float) -> None:
        pass


class ForwardMotorReceiver:
    def __init__(self):
        pass

    def receive(self) -> np.array:
        pass

