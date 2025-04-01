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
def catch_box(policy: DianRobot, exec_robot: DianRobotNode):
    # 运行到柜子前边
    if policy.R1info_cabinet_dir == "right":
        move_to_anywhere(policy, exec_robot, [policy.R1dst[0], policy.R1dst[1]], 0)
    else:
        move_to_anywhere(policy, exec_robot, [policy.R1dst[0], policy.R1dst[1]], 90)

    l, r = policy.get_catch_pos_lr(exec_robot.obs)
    print("l", l)
    print("r", r)
    print("lft_arm_ori_pose", policy.lft_arm_ori_pose)
    print("rgt_arm_ori_pose", policy.rgt_arm_ori_pose)
    print("height", exec_robot.obs["jq"][2])
    print("motor_angles_l", exec_robot.obs["jq"][5:11])
    print("motor_angles_r", exec_robot.obs["jq"][12:18])
    policy.cmd_pos_arm_lr.l_arm_controller \
        .set_robot_height(exec_robot.obs["jq"][2]) \
        .set_motor_angles(np.array(exec_robot.obs["jq"][5:11])) \
        .set_target_rot(np.array([0, 0.93584134, 1.6])) \
        .set_current_pos(policy.lft_arm_ori_pose) \
        .set_ratio(0.001) \
        .set_target_pos(l)

    policy.cmd_pos_arm_lr.r_arm_controller \
        .set_robot_height(exec_robot.obs["jq"][2]) \
        .set_motor_angles(np.array(exec_robot.obs["jq"][12:18])) \
        .set_target_rot(np.array([0, 0.93584134, -1.6])) \
        .set_current_pos(policy.rgt_arm_ori_pose) \
        .set_ratio(0.001) \
        .set_target_pos(r)
    robot_do(policy.cmd_pos_arm_lr)

    policy.cmd_grippers.l_gripper_controller \
        .set_current(exec_robot.obs["jq"][11]) \
        .set_target(0.0) \
        .set_ratio(0.01)
    policy.cmd_grippers.r_gripper_controller \
        .set_current(exec_robot.obs["jq"][18]) \
        .set_target(0.0) \
        .set_ratio(0.01)
    robot_do(policy.cmd_grippers)


def go_back(policy: DianRobot, exec_robot: DianRobotNode):
    if policy.R1info_cabinet_dir == "right":
        target = [policy.R1dst[0] - 0.3, policy.R1dst[1]]
    else:
        target = [policy.R1dst[0], policy.R1dst[1] - 0.3]
    print("base_position", exec_robot.obs["base_position"])
    print("target", target)
    policy.cmd_forward.forward_controller \
        .set_current(exec_robot.obs["base_position"][:2]) \
        .set_target(target) \
        .set_tolerance(0.01) \
        .set_direction_vec(calc_robot_direction(exec_robot.obs["base_orientation"]))
    robot_do(policy.cmd_forward)
    print("准备置位")
    policy.middle_reset(exec_robot)
    print("准备回位置")
    move_to_anywhere(policy, exec_robot, [0, 0], -135)
    print("准备放置")
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0.26)
    robot_do(policy.cmd_height)
    print("松开夹爪")
    policy.cmd_grippers.l_gripper_controller \
        .set_current(exec_robot.obs["jq"][11]) \
        .set_target(0.4)
    policy.cmd_grippers.r_gripper_controller \
        .set_current(exec_robot.obs["jq"][18]) \
        .set_target(0.4)
    robot_do(policy.cmd_grippers)
    print("抬起机械臂")
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0)
    robot_do(policy.cmd_height)

def move_and_get_box_pos(policy: DianRobot, exec_robot: DianRobotNode, is_first_call: bool):
    # 设置一个观测高度和角度
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0.5) \
        .set_ratio(0.03)
    policy.cmd_head_pitch.head_pitch_controller \
        .set_current(exec_robot.obs["jq"][4]) \
        .set_target(-0.2) \
        .set_ratio(0.03)
    robot_do_muti_cmd([policy.cmd_height, policy.cmd_head_pitch])
    # 等待机器人到位
    time.sleep(1)
    # 获取观测结果
    cv2.imwrite("test2.jpg", exec_robot.obs["img"][0])
    policy.get_box_pos(exec_robot.obs["img"][0], is_first_call=is_first_call)

    # 恢复机器人原始位置
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0.0) \
        .set_ratio(0.03)
    policy.cmd_head_pitch.head_pitch_controller \
        .set_current(exec_robot.obs["jq"][4]) \
        .set_target(0.0) \
        .set_ratio(0.03)
    robot_do_muti_cmd([policy.cmd_height, policy.cmd_head_pitch])
    # 等待机器人到位
    time.sleep(1)

def go_to_observe_pos(policy: DianRobot, exec_robot: DianRobotNode):
    policy.cmd_turn.yaw_controller \
        .set_current(-90.0) \
        .set_target(90.0) \
        .set_tolerance(0.01)
    robot_do(policy.cmd_turn)

    policy.cmd_forward.forward_controller \
        .set_current([0.0, 0.0]) \
        .set_target([0.0, 0.4]) \
        .set_tolerance(0.01) \
        .set_direction_vec(calc_robot_direction(exec_robot.obs["base_orientation"]))
    robot_do(policy.cmd_forward)

    policy.cmd_grippers.l_gripper_controller \
        .set_current(exec_robot.obs["jq"][11]) \
        .set_target(0.5)
    policy.cmd_grippers.r_gripper_controller \
        .set_current(exec_robot.obs["jq"][18]) \
        .set_target(0.5)
    policy.cmd_turn.yaw_controller \
        .set_current(90.0) \
        .set_target(0.0) \
        .set_tolerance(0.01)
    robot_do_muti_cmd([policy.cmd_grippers, policy.cmd_turn])

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
    policy.solve_rule(exec_robot.task_info)
    # 前往观察箱子的位置
    go_to_observe_pos(policy, exec_robot)
    # 通过图片解析箱子位置
    move_and_get_box_pos(policy, exec_robot, is_first_call=True)
    if policy.R1info_cabinet_dir == "left":
        move_to_anywhere(policy, exec_robot, [policy.R1dst[0], 0], 90)
        # 通过图片解析箱子位置
        move_and_get_box_pos(policy, exec_robot, is_first_call=False)

    # 观测目标位置并根据箱子左右初始化夹爪位置
    policy.set_origin_pos(exec_robot)
    catch_box(policy, exec_robot)
    print("status done: catch box")
    go_back(policy, exec_robot)

    print("机械臂回到初始位置")
    policy.reset_arm(exec_robot)
    print("观察视角")
    policy.cmd_head_pitch.head_pitch_controller \
        .set_current(exec_robot.obs["jq"][4]) \
        .set_target(0.2) \
        .set_ratio(0.03)
    policy.cmd_forward.forward_controller \
        .set_current(exec_robot.obs["base_position"][:2]) \
        .set_target([-0.2, -0.2]) \
        .set_tolerance(0.01) \
        .set_direction_vec(calc_robot_direction(exec_robot.obs["base_orientation"]))
    robot_do_muti_cmd([policy.cmd_head_pitch, policy.cmd_forward])
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