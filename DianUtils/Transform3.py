import numpy as np


# 3D transform class
class Transform3:
    def __init__(self, pos=np.array([0.0, 0.0, 0.0]), rot=np.array([0.0, 0.0, 0.0])):
        self.pos = pos  # position
        self.rot = rot  # rotation in euler angles

    def __str__(self):
        return f"pos: {self.pos}, rot: {self.rot}"

    def copy(self):
        return Transform3(self.pos.copy(), self.rot.copy())

    # region Getters and Setters
    def get_pos(self):
        return self.pos

    def get_rot(self):
        return self.rot

    def set_pos(self, pos):
        self.pos = pos

    def set_rot(self, rot):
        self.rot = rot
    # endregion Getters and Setters

