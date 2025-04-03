import numpy as np
from hgext.fsmonitor import poststatus


# 包含三维物体位置与旋转
class Transform3:
    def __init__(self, pos=np.array([0.0, 0.0, 0.0]), rot=np.array([0.0, 0.0, 0.0])):
        self._pos = pos  # position
        self._rot = rot  # rotation in euler angles

    def __str__(self):
        return f"pos: {self._pos}, rot: {self._rot}"

    def copy(self):
        return Transform3(self._pos.copy(), self._rot.copy())

    # region Getters and Setters
    def get_pos(self) -> np.array:
        return self._pos

    def get_rot(self) -> np.array:
        return self._rot

    def set_pos(self, pos : np.array):
        # 类型检查
        if type(pos) != np.ndarray:
            pos = np.array(pos)
        self._pos = pos.copy()

    def set_rot(self, rot : np.array):
        # 类型检查
        if type(rot) != np.ndarray:
            rot = np.array(rot)
        self._rot = rot.copy()
    # endregion Getters and Setters

