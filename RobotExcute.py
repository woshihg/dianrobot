from DianRobot import *
from RosNode import DianRobotNode
import numpy as np
import time
import threading
import rclpy
def robot_do(command: Command):
    frequency = 50.0
    name = command.description
    start = time.time()
    print("start command({}) at {}s".format(name, start))
    while not command.is_done():
        command.execute(time.time())
        command.feedback()
        time.sleep(1.0 / frequency)

    end = time.time()
    print("end command({}) at {}s, cost {}s".format(name, end, end - start))

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
        obs = policy.set_origin_pos(exec_robot.obs, exec_robot)
        policy.cmd_turn.yaw_controller \
            .set_current(exec_robot.obs["base_orientation"]) \
            .set_target(0.0) \
            .set_tolerance(0.01)
        robot_do(policy.cmd_turn)

        policy.cmd_forward.forward_controller \
            .set_current(exec_robot.obs["base_position"][:2]) \
            .set_target(policy.R1dst) \
            .set_tolerance(0.01) \
            .set_origin(obs["base_position"][:2])
        robot_do(policy.cmd_forward)

        policy.cmd_turn.yaw_controller \
            .set_current(exec_robot.obs["base_orientation"]) \
            .set_target(0.0) \
            .set_tolerance(0.005)
        robot_do(policy.cmd_turn)

        l,r = policy.phase_test_lr(exec_robot.obs)
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

        # obs = policy.catch_box(obs, exec_robot)
        print("status done: catch box")
        obs = policy.base_forward(obs, 0, exec_robot, [policy.R1dst[0] - 0.3, policy.R1dst[1], policy.R1dst[2]])
        print("准备置位")
        obs = policy.middle_reset(obs, exec_robot)
        print("status done: middle reset")
        print("准备回位置")
        obs = policy.base_rotate(obs, -90, exec_robot)
        obs = policy.base_rotate(obs, -140, exec_robot)
        obs = policy.base_forward(obs, 2, exec_robot, [-0.10, 0, policy.R1dst[2]])
        obs = policy.robot_pause(exec_robot)
        print("准备放置")
        obs = policy.height_control(obs, exec_robot, 0.26)
        obs = policy.robot_pause(exec_robot)
        print("松开夹爪")
        obs = policy.gripper_control(obs, exec_robot, "both", "open", open_size=0.4)
        obs = policy.robot_pause(exec_robot)
        print("抬起机械臂")
        obs = policy.height_control(obs, exec_robot, 0)
        obs = policy.robot_pause(exec_robot)

    else:
        obs = policy.base_forward(obs, 0, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 90, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_forward(obs, 1, exec_robot, [policy.R1dst[0], 0, policy.R1dst[2]])
        obs = policy.robot_pause(exec_robot)
        # 通过图片解析箱子位置
        obs = policy.get_box_pos(obs, exec_robot, is_first_call=False)
        # 观测目标位置并根据箱子左右初始化夹爪位置
        # obs = policy.get_box_pos(obs, exec_robot)
        obs = policy.set_origin_pos(obs, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 90, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_forward(obs, 1, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 90, exec_robot)
        obs = policy.robot_pause(exec_robot)
        l,r = policy.phase_test_lr(exec_robot.obs)
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

        # obs = policy.catch_box(obs, exec_robot)
        obs = policy.base_forward(obs, 1, exec_robot, [policy.R1dst[0], policy.R1dst[1] - 0.4, policy.R1dst[2]])
        obs = policy.middle_reset(obs, exec_robot)
        print("准备回位置")
        obs = policy.base_rotate(obs, 10, exec_robot)
        obs = policy.base_rotate(obs, -35, exec_robot)
        obs = policy.base_forward(obs, 2, exec_robot, [-0.09, 0, policy.R1dst[2]])
        obs = policy.robot_pause(exec_robot)
        print("准备放置")
        obs = policy.height_control(obs, exec_robot, 0.26)
        obs = policy.robot_pause(exec_robot)
        print("松开夹爪")
        obs = policy.gripper_control(obs, exec_robot, "both", "open", open_size=0.4)
        print("抬起机械臂")
        obs = policy.height_control(obs, exec_robot, 0)
        obs = policy.robot_pause(exec_robot)
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