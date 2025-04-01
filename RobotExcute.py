from gpg.gpgme import gpgme_conf_arg

from DianRobot import *
from RosNode import DianRobotNode
import numpy as np
import time
import threading
import rclpy
def get_rotation_angle(start, end):
    # 根据二维起止位置计算旋转角度
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    angle = np.arctan2(dy, dx) * 180 / np.pi
    return angle

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
        .set_tolerance(0.01) \
        .set_origin(exec_robot.obs["base_position"][:2])
    robot_do(policy.cmd_forward)

    policy.cmd_turn.yaw_controller \
        .set_current(exec_robot.obs["base_orientation"]) \
        .set_target(dst_angle) \
        .set_tolerance(0.01)
    robot_do(policy.cmd_turn)
def catch_box(policy: DianRobot, exec_robot: DianRobotNode):
    # 运行到柜子前边
    if policy.R1info_cabinet_dir == "right":
        move_to_anywhere(policy, exec_robot, [policy.R1dst[0], policy.R1dst[1]], 0)
    else:
        move_to_anywhere(policy, exec_robot, [policy.R1dst[0], policy.R1dst[1]], 90)

    l, r = policy.get_catch_pos_lr(exec_robot.obs)
    policy.cmd_pos_arm_lr.l_arm_controller \
        .set_robot_height(exec_robot.obs["jq"][2]) \
        .set_motor_angles(np.array(exec_robot.obs["jq"][5:11])) \
        .set_target_rot(np.array([0, 0.93584134, 1.6])) \
        .set_current_pos(policy.lft_arm_ori_pose) \
        .set_target_pos(l)

    policy.cmd_pos_arm_lr.r_arm_controller \
        .set_robot_height(exec_robot.obs["jq"][2]) \
        .set_motor_angles(np.array(exec_robot.obs["jq"][12:18])) \
        .set_target_rot(np.array([0, 0.93584134, -1.6])) \
        .set_current_pos(policy.rgt_arm_ori_pose) \
        .set_target_pos(r)
    robot_do(policy.cmd_pos_arm_lr)

def go_back(policy: DianRobot, exec_robot: DianRobotNode):
    # 后退避免撞墙
    if policy.R1info_cabinet_dir == "right":
        target = [policy.R1dst[0] - 0.3, policy.R1dst[1], policy.R1dst[2]]
    else:
        target = [policy.R1dst[0], policy.R1dst[1] - 0.3, policy.R1dst[2]]
    policy.cmd_forward.forward_controller \
        .set_current(exec_robot.obs["base_position"][:2]) \
        .set_target(target) \
        .set_tolerance(0.01) \
        .set_origin(exec_robot.obs["base_position"][:2])
    robot_do(policy.cmd_forward)
    # 把盒子放在中间位置
    print("准备置位")
    policy.middle_reset(exec_robot)
    # 回到放置位置
    print("准备回位置")
    move_to_anywhere(policy, exec_robot, [-0.09, 0], -140)
    print("准备放置")
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0.26)
    robot_do(policy.cmd_height)
    print("松开夹爪")
    obs = policy.gripper_control(obs, exec_robot, "both", "open", open_size=0.4)
    print("抬起机械臂")
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0)
    robot_do(policy.cmd_height)

def main(args=None):
    rclpy.init(args=args)
    exec_robot = DianRobotNode()
    spin_thead = threading.Thread(target=lambda: rclpy.spin(exec_robot))
    spin_thead.start()
    # pub_thread = threading.Thread(target=exec_robot.pub_thread)
    # pub_thread.start()
    policy = DianRobot()
    policy.init_cmd(exec_robot)
    # 指令解析
    while exec_robot.task_info is None:
        pass
    dst = policy.solve_rule(exec_robot.task_info)
    # region my commands
    node = exec_robot


    # endregion my commands
    policy.cmd_turn.yaw_controller \
        .set_current(-90.0) \
        .set_target(90.0) \
        .set_tolerance(0.01)
    robot_do(policy.cmd_turn)

    policy.cmd_forward.forward_controller \
        .set_current([0.0, 0.0]) \
        .set_target([0.0, 0.4]) \
        .set_tolerance(0.01) \
        .set_origin([0.0, 0.0])
    robot_do(policy.cmd_forward)

    # 前往观测位置
    obs = policy.gripper_control(exec_robot.obs, exec_robot, "both", "open")
    policy.cmd_turn.yaw_controller \
        .set_current(90.0) \
        .set_target(0.0) \
        .set_tolerance(0.01)
    robot_do(policy.cmd_turn)

    # 通过图片解析箱子位置
    obs = policy.get_box_pos(exec_robot.obs, exec_robot, is_first_call=True)
    if policy.R1info_cabinet_dir == "right":
        # 观测目标位置并根据箱子左右初始化夹爪位置
        obs = policy.set_origin_pos(exec_robot)
        catch_box(policy, exec_robot)
        print("status done: catch box")
        go_back(policy, exec_robot)
    else:
        move_to_anywhere(policy, exec_robot, [policy.R1dst[0], 0], 90)
        # 通过图片解析箱子位置
        obs = policy.get_box_pos(obs, exec_robot, is_first_call=False)
        # 观测目标位置并根据箱子左右初始化夹爪位置
        obs = policy.set_origin_pos(exec_robot)
        catch_box(policy, exec_robot)
        print("status done: catch box")
        go_back(policy, exec_robot)

    print("机械臂回到初始位置")
    obs = policy.reset_arm(obs, exec_robot)
    print("观察视角")
    obs = policy.head_control(obs, exec_robot, 0.2)
    obs = policy.base_forward(obs, 2, exec_robot, [-0.2, 0, policy.R1dst[2]])
    prop_pos = policy.get_grasp_pos(obs)
    if prop_pos is not None:
        obs = policy.height_control(obs, exec_robot, 0.1, step_num=200)
        obs = policy.catch_prop(prop_pos, obs, exec_robot)
        obs = policy.put_prop(obs, exec_robot)
    spin_thead.join()
    # pub_thread.join()


if __name__ == '__main__':
    main()