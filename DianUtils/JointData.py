import numpy as np
from DianUtils.Transform3 import Transform3
from discoverse.mmk2 import MMK2FIK

def _pick_motor_angles(target : Transform3, is_right : bool, my_height: float, my_angles: np.array) -> np.array :
    angles = MMK2FIK().get_armjoint_pose_wrt_footprint(
        target.pos, 'pick', 'r' if is_right else 'l', my_height, my_angles, target.rot
    )
    return np.array(angles)

class JointData:
    def __init__(self, is_right: bool = False, my_transform: Transform3 = Transform3(), motor_angles: np.array = np.zeros(6)):
        self.is_right = is_right
        self.my_transform = my_transform
        self.motor_angles = motor_angles

    def calc_target_motor_angles(self, target):
        return _pick_motor_angles(target, True, 0.0, self.motor_angles)

    # region Getters and Setters

    def set_my_transform(self, transform: Transform3):
        self.my_transform = transform

    def get_my_transform(self):
        return self.my_transform
    def set_motor_angles(self, angles: np.array):
        self.motor_angles = angles

    def get_motor_angles(self):
        return self.motor_angles

    # endregion Getters and Setters

