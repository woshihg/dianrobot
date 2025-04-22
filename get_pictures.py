from gpg.gpgme import gpgme_conf_arg

from DianRobot import *
from RosNode import DianRobotNode
import numpy as np
import threading
import rclpy
import cv2



def get_rotation_angle(start, end):
    # 根据二维起止位置计算旋转角度
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    angle = np.arctan2(dy, dx) * 180 / np.pi
    return angle
# 根据机器人朝向计算机器人二维平面方向向量
def calc_robot_direction(base_orientation):
    bo = base_orientation
    corrected_obs = [bo[1], bo[2], bo[3], bo[0]]
    euler = R.from_quat(corrected_obs).as_euler('zyx', degrees=False)
    return np.array([np.cos(euler[0]), np.sin(euler[0])])

def move_to_anywhere(policy: DianRobot, exec_robot: DianRobotNode, dst_pos, dst_angle):
    # 计算运行角度
    angle = get_rotation_angle(exec_robot.obs["base_position"][:2], dst_pos)

    policy.cmd_turn.yaw_controller \
        .set_current(exec_robot.obs["base_orientation"]) \
        .set_target(angle) \
        .set_tolerance(0.01)
    robot_do(policy.cmd_turn)

    policy.cmd_forward.forward_controller \
        .set_current(exec_robot.obs["base_position"][:2]) \
        .set_target(dst_pos) \
        .set_tolerance(0.001) \
        .set_direction_vec(calc_robot_direction(exec_robot.obs["base_orientation"]))
    robot_do(policy.cmd_forward)

    policy.cmd_turn.yaw_controller \
        .set_current(exec_robot.obs["base_orientation"]) \
        .set_target(dst_angle) \
        .set_tolerance(0.001)
    robot_do(policy.cmd_turn)







def get_class(policy: DianRobot, exec_robot: DianRobotNode):

    policy.cmd_head_pitch.head_pitch_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(3.14 * (20 / 180)) \
        .set_tolerance(0.1)
    robot_do(policy.cmd_head_pitch)

    move_to_anywhere(policy, exec_robot, [0.0, 0.1], 270)

    cls_coordinates = policy.get_world_coordinates(exec_robot.obs)

    print(cls_coordinates)

    move_to_anywhere(policy, exec_robot, [0.0, 0.1], 200)








# 示例用法


def main(args=None):
    rclpy.init(args=args)
    exec_robot = DianRobotNode()
    spin_thead = threading.Thread(target=lambda: rclpy.spin(exec_robot))
    spin_thead.start()
    time.sleep(1)
    policy = DianRobot()
    policy.init_io(exec_robot)
    while exec_robot.task_info is None:
        pass
    policy.solve_rule(exec_robot.task_info)
    get_class(policy, exec_robot)
    time.sleep(1)

if __name__ == '__main__':
    main()
