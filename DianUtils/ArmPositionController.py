import numpy as np
from DianUtils.JointData import *
from DianUtils.Transform3 import Transform3
from discoverse.mmk2 import MMK2FIK


# 角度控制器，给定目标的transform，发送电机角度控制信号
class ArmPositionController:
    def __init__(self):
        # ranged form -180.0 to 180.0
        self._target = Transform3()
        self._current = Transform3()
        self._current_joint_data = JointData()
        self._tolerance = 0.0001 # dot product tolerance
        self._ratio = 0.0001

    def calc_current_target_pos(self) -> np.array:
        cur_pos = self._current.get_pos()
        tar_pos = self._target.get_pos()
        diff_pos = tar_pos - cur_pos

        if np.dot(diff_pos, diff_pos) < 2 * self._tolerance:
            return tar_pos

        # normalize the diff, to keep increment in fixed length
        direction = diff_pos / np.linalg.norm(diff_pos)

        return cur_pos + direction * self._ratio


    def calc_motor_angles_by_target_pos(self, target_pos: np.array) -> np.array:
        tar_rot = self._target.get_rot()
        new_trans = Transform3(target_pos, tar_rot)
        return self._current_joint_data.calc_target_motor_angles(new_trans)

    def check_is_done(self) -> bool:
        cur_pos = self._current.get_pos()
        tar_pos = self._target.get_pos()
        diff_pos = tar_pos - cur_pos
        print("cur_pos: ", cur_pos)
        print("tar_pos: ", tar_pos)
        return np.dot(diff_pos, diff_pos) < self._tolerance
    # region Getters and Setters

    def get_target_pos(self) -> np.array:
        return self._target.get_pos()

    def get_current_pos(self) -> np.array:
        return self._current.get_pos()

    def get_target_rot(self) -> np.array:
        return self._target.get_rot()

    def get_current_rot(self) -> np.array:
        return self._current.get_rot()

    def get_ratio(self) -> float:
        return self._ratio

    def get_tolerance(self) -> float:
        return self._tolerance

    def set_target_pos(self, target_pos: np.array):
        self._target.set_pos(target_pos)
        return self

    def set_current_pos(self, current_pos: np.array):
        self._current.set_pos(current_pos)
        return self

    def set_target_rot(self, target_rot: np.array):
        self._target.set_rot(target_rot)
        return self

    def set_current_rot(self, current_rot: np.array):
        self._current.set_rot(current_rot)
        return self

    def set_ratio(self, ratio: float):
        self._ratio = ratio
        return self

    def set_tolerance(self, tolerance: float):
        self._tolerance = tolerance
        return self

    # joint data, is right
    def set_is_right(self, is_right: bool):
        self._current_joint_data.set_is_right(is_right)
        return self

    def set_robot_height(self, height: float):
        self._current_joint_data.set_robot_height(height)
        return self

    def set_motor_angles(self, angles: np.array):
        self._current_joint_data.set_motor_angles(angles)
        return self

    def get_is_right(self) -> bool:
        return self._current_joint_data.get_is_right()

    def get_robot_height(self) -> float:
        return self._current_joint_data.get_robot_height()

    def get_motor_angles(self) -> np.array:
        return self._current_joint_data.get_motor_angles()

    # endregion Getters and Setters

class ArmControlSender:
    def __init__(self):
        pass

    def send(self, motor_angles: np.array) -> None:
        pass


class ArmMotorReceiver:
    def __init__(self):
        pass

    def receive(self) -> np.array:
        pass