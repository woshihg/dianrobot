class HeadPitchController:
    def __init__(self):
        self._target_pitch = 0.0
        self._current_pitch = 0.0
        self._tolerance = 0.1
        self._ratio = 1

