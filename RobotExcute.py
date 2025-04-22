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
        .set_ratio(0.002) \
        .set_tolerance(0.002) \
        .set_target_pos(l)

    policy.cmd_pos_arm_lr.r_arm_controller \
        .set_robot_height(exec_robot.obs["jq"][2]) \
        .set_motor_angles(np.array(exec_robot.obs["jq"][12:18])) \
        .set_target_rot(np.array([0, 0.93584134, -1.6])) \
        .set_current_pos(policy.rgt_arm_ori_pose) \
        .set_ratio(0.002) \
        .set_tolerance(0.002) \
        .set_target_pos(r)
    robot_do(policy.cmd_pos_arm_lr)

    policy.cmd_grippers.l_gripper_controller \
        .set_current(exec_robot.obs["jq"][11]) \
        .set_target(0.0) \
        .set_ratio(0.03) \
        .set_tolerance(0.03)
    policy.cmd_grippers.r_gripper_controller \
        .set_current(exec_robot.obs["jq"][18]) \
        .set_target(0.0) \
        .set_ratio(0.03) \
        .set_tolerance(0.03)
    robot_do(policy.cmd_grippers)


def go_back(policy: DianRobot, exec_robot: DianRobotNode):
    if policy.R1info_cabinet_dir == "right":
        target = [policy.R1dst[0] - 0.4, policy.R1dst[1]]
    else:
        target = [policy.R1dst[0], policy.R1dst[1] - 0.4]
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
    move_to_anywhere(policy, exec_robot, [-0.05, -0.05], -135)

    print("准备放置")
    print(exec_robot.obs["jq"][11], exec_robot.obs["jq"][18])
    if exec_robot.obs["jq"][11] < 0.05:
        print("松开夹爪1")
        policy.cmd_gripper_l.gripper_controller \
            .set_current(exec_robot.obs["jq"][11]) \
            .set_target(0.4)
        robot_do(policy.cmd_gripper_l)
    if exec_robot.obs["jq"][18] < 0.05:
        print("松开夹爪2")
        policy.cmd_gripper_r.gripper_controller \
            .set_current(exec_robot.obs["jq"][18]) \
            .set_target(0.4)
        robot_do(policy.cmd_gripper_r)
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0.26) \
        .set_ratio(0.005) \
        .set_tolerance(0.005)
    robot_do(policy.cmd_height)
    time.sleep(2)
    print("松开夹爪")
    policy.cmd_grippers.l_gripper_controller \
        .set_current(exec_robot.obs["jq"][11]) \
        .set_target(0.4)
    policy.cmd_grippers.r_gripper_controller \
        .set_current(exec_robot.obs["jq"][18]) \
        .set_target(0.4)
    robot_do(policy.cmd_grippers)
    time.sleep(1)
    print("抬起机械臂")
    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0)
    robot_do(policy.cmd_height)
    time.sleep(1)

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
    robot_do_multi_cmd([policy.cmd_height, policy.cmd_head_pitch])
    # 等待机器人到位
    time.sleep(1)
    # 获取观测结果
    cv2.imwrite("test2.jpg", exec_robot.obs["img"][0])
    policy.get_box_pos(exec_robot.obs["img"][0], is_first_call=is_first_call)

    policy.cmd_height.height_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(0.0) \
        .set_ratio(0.03)
    policy.cmd_head_pitch.head_pitch_controller \
        .set_current(exec_robot.obs["jq"][4]) \
        .set_target(0.0) \
        .set_ratio(0.03)
    robot_do_multi_cmd([policy.cmd_height, policy.cmd_head_pitch])
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
    robot_do_multi_cmd([policy.cmd_grippers, policy.cmd_turn])

def put_down_prop_at_place(policy: DianRobot, exec_robot: DianRobotNode):
    # 设置一个观测高度和角度
    policy.cmd_head_pitch.head_pitch_controller \
        .set_current(exec_robot.obs["jq"][2]) \
        .set_target(3.14 * (20 / 180)) \
        .set_tolerance(0.1)
    robot_do(policy.cmd_head_pitch)
    obj = policy.R1info_put_down_obj
    pose_to_obj = policy.R1info_put_down_pose

    move_to_anywhere(policy, exec_robot, [0.3, 0.3], -135)

    cls_coordinates = policy.get_world_coordinates(exec_robot.obs)
    if obj in cls_coordinates:
        obj_place = cls_coordinates[obj]
        move_to_anywhere(policy, exec_robot, [0, obj_place[1]], 270)
        cls_coordinates = policy.get_world_coordinates(exec_robot.obs)
        obj_place = cls_coordinates[obj]
        des_place = obj_place.copy()
        if pose_to_obj == "left":
            des_place[1] += 0.1
        if pose_to_obj == "right":
            des_place[1] -= 0.1
        if pose_to_obj == "front":
            des_place[0] -= 0.1
        if pose_to_obj == "back":
            des_place[0] += 0.1
        print(obj_place, pose_to_obj)
        if policy.R1info_table_dir == "left":
            prop_bias = np.array([0.0, 0.0, 0.1])
            left_arm_ori_pose = des_place + prop_bias
            left_arm_target_euler = [0., 0.5, -np.pi]
            left_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(left_arm_ori_pose, "pick", "l",
                                                                       exec_robot.obs["jq"][2], np.array(exec_robot.obs["jq"][5:11]),
                                                                       Rotation.from_euler('zyx',left_arm_target_euler).as_matrix())

            for i in range(6):
                exec_robot.tctr_left_arm[i] = float(left_arm_joint[i])
            exec_robot.publish_messages()

            time.sleep(2)

            policy.cmd_grippers.l_gripper_controller \
                .set_target(1) \
                .set_current(exec_robot.obs["jq"][11])
            robot_do(policy.cmd_grippers)
        else:
            prop_bias = np.array([0.0, 0.0, 0.1])
            right_arm_ori_pose = des_place + prop_bias
            right_arm_target_euler = [0., 0.5, np.pi]  # Adjusted for the right arm
            right_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(right_arm_ori_pose, "pick", "r",
                                                                        exec_robot.obs["jq"][2],
                                                                        np.array(exec_robot.obs["jq"][12:18]),
                                                                        Rotation.from_euler('zyx',
                                                                                            right_arm_target_euler).as_matrix())

            for i in range(6):
                exec_robot.tctr_right_arm[i] = float(right_arm_joint[i])  # Adjusted for the right arm
            exec_robot.publish_messages()

            time.sleep(2)

            policy.cmd_grippers.r_gripper_controller \
                .set_target(1) \
                .set_current(exec_robot.obs["jq"][18])  # Adjusted for the right gripper
            robot_do(policy.cmd_grippers)

    else:
        policy.cmd_turn.yaw_controller \
            .set_current(90.0) \
            .set_target(220.0) \
            .set_tolerance(0.01)
        robot_do(policy.cmd_turn)
        cls_coordinates = policy.get_world_coordinates(exec_robot.obs)
        if obj in cls_coordinates:
            obj_place = cls_coordinates[obj]
        else:
            print("没有找到物体")
            return







def main(args=None):
    rclpy.init(args=args)
    exec_robot = DianRobotNode()
    spin_thead = threading.Thread(target=lambda: rclpy.spin(exec_robot))
    spin_thead.start()
    policy = DianRobot()
    policy.init_io(exec_robot)
    while exec_robot.task_info is None:
        pass
    policy.solve_rule(exec_robot.task_info)
    go_to_observe_pos(policy, exec_robot)
    move_and_get_box_pos(policy, exec_robot, is_first_call=True)
    if policy.R1info_cabinet_dir == "left":
        move_to_anywhere(policy, exec_robot, [policy.R1dst[0], 0], 90)
        move_and_get_box_pos(policy, exec_robot, is_first_call=False)

    policy.set_origin_pos(exec_robot)
    catch_box(policy, exec_robot)
    print("status done: catch box")
    go_back(policy, exec_robot)

    print("policy.reset_arm(exec_robot)")
    policy.reset_arm(exec_robot)
    print("head_pitch_controller")
    policy.cmd_head_pitch.head_pitch_controller \
        .set_current(exec_robot.obs["jq"][4]) \
        .set_target(0.2) \
        .set_ratio(0.03)
    policy.cmd_forward.forward_controller \
        .set_current(exec_robot.obs["base_position"][:2]) \
        .set_target([-0.18, -0.18]) \
        .set_tolerance(0.01) \
        .set_direction_vec(calc_robot_direction(exec_robot.obs["base_orientation"]))
    robot_do_multi_cmd([policy.cmd_head_pitch, policy.cmd_forward])
    time.sleep(1)
    prop_pos = policy.get_grasp_pos(exec_robot.obs)
    if prop_pos is not None:
        policy.cmd_height.height_controller \
            .set_current(exec_robot.obs["jq"][2]) \
            .set_target(0.1) \
            .set_ratio(0.03) \
            .set_tolerance(0.03)
        robot_do(policy.cmd_height)
        time.sleep(2)
        policy.catch_prop(prop_pos, exec_robot.obs, exec_robot)
        #policy.put_prop(exec_robot)
        put_down_prop_at_place(policy, exec_robot)

    spin_thead.join()


if __name__ == '__main__':
    main()