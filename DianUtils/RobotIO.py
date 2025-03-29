import numpy as np
from scipy.spatial.transform import Rotation as R

import RosNode


# region RobotYawMotor
class RobotYawMotorSender:
    def __init__(self, node : RosNode.DianRobotNode):
        self.node = node

    def send(self, motor_value: float):
        n = self.node
        n.set_robot_chassis_omega_magic_value(motor_value)
        n.publish_messages()


class RobotYawMotorReceiver:
    def __init__(self, node: RosNode.DianRobotNode):
        self.node = node

    def receive(self) -> float:
        bo = self.node.obs["base_orientation"]
        corrected_obs = [bo[1], bo[2], bo[3], bo[0]]
        euler = R.from_quat(corrected_obs).as_euler('zyx', degrees=True)
        return euler[0]

 # endregion RobotYawMotor
# region RobotForwardMotor
class RobotForwardMotorSender:
    def __init__(self, node: RosNode.DianRobotNode):
        self.node = node

    def send(self, motor_value: float):
        n = self.node
        n.set_robot_chassis_speed_magic_value(motor_value)
        n.publish_messages()

class RobotForwardMotorReceiver:
    def __init__(self, node: RosNode.DianRobotNode):
        self.node = node
        self._last_not_none = np.array([0.0, 0.0])

    def receive(self) -> np.array:
        bp = self.node.obs["base_position"]

        if bp is None:
            return self._last_not_none
        self._last_not_none = np.array([bp[0], bp[1]])
        return self._last_not_none

     # endregion RobotForwardMotor