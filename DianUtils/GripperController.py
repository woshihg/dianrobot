# 夹爪控制
class GripperController:
    def __init__(self):
        # the open size of the gripper ranged form 0 to 1
        self._target_open_size = 0.0
        self._current_open_size = 0.0
        self._tolerance = 0.01
        self._ratio = 0.01

    # @brief 计算当前目标角度
    # @return float 当前最佳的输出角度
    def calc_current_target(self) -> float:
        delta = self._target_open_size - self._current_open_size
        if abs(delta) < 2 * self._ratio:
            return self._target_open_size

        # normalize the diff, to keep increment in fixed length
        direction = delta / abs(delta)
        return self._current_open_size + direction * self._ratio

    def check_is_done(self) -> bool:
        delta = self._target_open_size - self._current_open_size
        return abs(delta) < self._tolerance

    # region Getters and Setters

    def get_target(self):
        return self._target_open_size

    def get_current(self):
        return self._current_open_size

    def get_ratio(self):
        return self._ratio

    def get_tolerance(self):
        return self._tolerance

    def set_target(self, target):
        self._target_open_size = target
        return self

    def set_current(self, current):
        self._current_open_size = current
        return self

    def set_ratio(self, ratio):
        self._ratio = ratio

    def set_tolerance(self, tolerance):
        self._tolerance = tolerance

    # endregion Getters and Setters

class GripperControlSender:
    def __init__(self):
        pass

    def send(self, size) -> None:
        pass

class GripperMotorReceiver:
    def __init__(self):
        pass

    def receive(self) -> float:
        pass

