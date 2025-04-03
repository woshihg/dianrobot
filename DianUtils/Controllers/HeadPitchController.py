
# 相机头部俯仰控制器
class HeadPitchController:
    def __init__(self):
        self._target_pitch = 0.0
        self._current_pitch = 0.0
        self._tolerance = 0.1
        self._ratio = 1

    def calc_current_target(self) -> float:
        delta = self._target_pitch - self._current_pitch
        if abs(delta) < 2 * self._ratio:
            return self._target_pitch

        # normalize the diff, to keep increment in fixed length
        direction = delta / abs(delta)
        return self._current_pitch + direction * self._ratio

    def check_is_done(self) -> bool:
        delta = self._target_pitch - self._current_pitch
        return abs(delta) < self._tolerance

    # region Getters and Setters

    def get_target(self):
        return self._target_pitch

    def get_current(self):
        return self._current_pitch

    def get_ratio(self):
        return self._ratio

    def get_tolerance(self):
        return self._tolerance

    def set_target(self, target):
        self._target_pitch = target
        return self

    def set_current(self, current):
        self._current_pitch = current
        return self

    def set_ratio(self, ratio):
        self._ratio = ratio

    def set_tolerance(self, tolerance):
        self._tolerance = tolerance

    # endregion Getters and Setters

class HeadPitchControlSender:
    def __init__(self):
        pass

    def send(self, pitch: float) -> None:
        pass

class HeadPitchMotorReceiver:
    def __init__(self):
        pass

    def receive(self) -> float:
        pass
