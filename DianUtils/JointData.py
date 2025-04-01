import numpy as np
from DianUtils.Transform3 import Transform3
from discoverse.mmk2 import MMK2FIK
from scipy.spatial.transform import Rotation
# 机械臂逆解算
def _pick_motor_angles(target : Transform3, is_right : bool, my_height: float, cur_angles: np.array) -> np.array :
    rot = target.get_rot().copy()
    for _ in range(10):
        ori_matrix = Rotation.from_euler('zyx', rot).as_matrix()
        angles = MMK2FIK().get_armjoint_pose_wrt_footprint(
            target.get_pos(), 'pick', 'r' if is_right else 'l', my_height, cur_angles, ori_matrix
        )
        if angles is not None:
            return np.array(angles)
        print("Failed to calculate motor angles, retrying...")
        rot = rot + np.random.uniform(-0.01, 0.01, size=3)
    raise Exception("Failed to calculate motor angles, after 10 retries")

class JointData:
    def __init__(self, is_right: bool = False, robot_height: float = 0.0, motor_angles: np.array = np.zeros(6)):
        self._is_right = is_right
        self._robot_height = robot_height
        self._motor_angles = motor_angles

    def calc_target_motor_angles(self, target):
        return _pick_motor_angles(target, self._is_right, self._robot_height, self._motor_angles)

    # region Getters and Setters

    def set_robot_height(self, height: float):
        self._robot_height = height

    def get_robot_height(self):
        return self._robot_height

    def set_motor_angles(self, angles: np.array):
        # 类型检查
        if type(angles) != np.ndarray:
            angles = np.array(angles)
        self._motor_angles = angles.copy()

    def get_motor_angles(self):
        return self._motor_angles

    def set_is_right(self, is_right: bool):
        self._is_right = is_right

    def get_is_right(self):
        return self._is_right

    # endregion Getters and Setters

