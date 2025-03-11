import time

from RosNode import DianRobotNode
from scipy.spatial.transform import Rotation
from discoverse.mmk2 import MMK2FIK
from ultralytics import YOLO
import numpy as np
import re
import threading
import rclpy
import cv2
class DianRobot:
    def __init__(self):
        self.z_forward = None
        self.x_forward = None
        self.y_forward = None
        self.right_arm_ori_pose = None
        self.lft_arm_ori_pose = None
        self.R1info_prop_name = ""
        self.R1info_floor_idx = ""
        self.R1info_cabinet_dir = ""
        self.R1info_table_dir = ""
        # 第一轮游戏箱子的目标位置(底盘位置+物体高度)
        self.R1dst = [0., 0., 0.]
        # 第一轮游戏箱子在橱柜的左右
        self.R1dst_dirc = ""
        self.model_class_mapping = {"carton": "box",
                                    "disk": "disk",
                                    "sheet": "sheet"}
        # 定义楼层高度和柜子位置的映射关系
        self.floor_height = {
            "second": 0.422,
            "third": 0.742,
            "fourth": 1.062
        }

        self.cabinet_position = {
            "left": (0.4, 0.62),
            "right": (0.62, 0.4)
        }
        model_path = "/home/workspace/best.pt"
        self.cabinet_top = 3  # 柜体顶部边距（像素距离）
        self.cabinet_bottom = 5  # 柜体底部边距（像素距离）
        self.model = YOLO(model_path)


    def get_box_pos(self, observe, robot, is_first_call=True):
        # TODO：根据图像获得小物体的位置
        """机械臂移动的部分"""
        # 获取底盘当前位置与朝向
        base_pos = observe["base_position"]
        base_quat = Rotation.from_quat(observe["base_orientation"]).as_euler('zyx')[0]

        # 设置一个观测高度
        robot.target_control[2] = 0.5
        robot.target_control[4] = -0.2
        self.robot_pause(robot, 100)

        """获取图片并进行处理的部分"""
        conf_threshold = 0.25  # 置信度阈值 (0-1之间)
        iou_threshold = 0.45  # IoU阈值 (非极大值抑制用)
        hide_labels = False  # 是否隐藏标签
        hide_conf = False  # 是否隐藏置信度
        line_width = 2

        img = observe["img"][0]  # 保存在img里面
        # cv2.imwrite("position2.jpg", img)
        # 对图像进行预处理
        processed_img = self._preprocess_image(img)

        # 对observe["img"][0]进行yolo识别
        results = self.model.predict(
            source=img,
            conf=conf_threshold,
            iou=iou_threshold,
            show_labels=not hide_labels,
            show_conf=not hide_conf,
            line_width=line_width
        )
        if is_first_call:
            self._handle_first_detection(results)
            if self.R1info_cabinet_dir == "right":
                self._process_position(processed_img, results)
            else:
                print("目标物体在左侧橱柜")
        else:
            self._process_position(processed_img, results)
        cv2.imwrite('Detection.jpg', processed_img)
        # 获取目标位置信息
        self.R1dst[0] = self.cabinet_position.get(self.R1info_cabinet_dir)[0]
        self.R1dst[1] = self.cabinet_position.get(self.R1info_cabinet_dir)[1]
        print(f"Target position: {self.R1dst}")
        # cv2.waitKey(0)
        # 恢复机器人原始位置
        robot.target_control[2] = 0.
        robot.target_control[4] = 0.
        self.robot_pause(robot, 100)
        return observe

    def _handle_first_detection(self, results):
        """处理首次检测逻辑"""
        target_found = any(
            self._map_class(result.names[int(box.cls.item())]) == self.model_class_mapping[self.R1info_prop_name]
            for result in results
            for box in result.boxes
        )
        self.R1info_cabinet_dir = "right" if target_found else "left"
        print(f"首次检测结果：目标物体在 {self.R1info_cabinet_dir} 侧橱柜")

    def _preprocess_image(self, img):
        """图像增强处理"""
        # CLAHE对比度增强
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l_enhanced = clahe.apply(l)
        return cv2.cvtColor(cv2.merge([l_enhanced, a, b]), cv2.COLOR_LAB2BGR)

    def _process_position(self, img, results):
        """处理目标物体位置判断"""
        h, w = img.shape[:2]
        effective_height = h - self.cabinet_top - self.cabinet_bottom
        layer_height = effective_height // 5

        for result in results:
            for box in result.boxes:
                # 类别映射处理
                if result.names[int(box.cls.item())] != self.model_class_mapping[self.R1info_prop_name]:
                    continue

                # 获取坐标信息
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # 计算所在层（从下往上）
                bottom_start = h - self.cabinet_bottom
                relative_y = bottom_start - center_y
                layer_idx = min(4, max(0, int(relative_y // layer_height)))
                self.R1info_floor_idx = ["第1层", "second", "third", "fourth", "第5层"][layer_idx]
                self.R1dst[2] = self.floor_height.get(self.R1info_floor_idx)
                # 判断左右位置
                self.R1dst_dirc = "right" if center_x > (w // 2) else "left"

                # 绘制可视化标记
                self._draw_detection_markers(img, x1, y1, x2, y2, center_x, center_y)
                print(f"目标位置：{self.R1info_floor_idx} {self.R1dst_dirc}")
                return

    def _map_class(self, detected_class):
        """类别名称映射"""
        return self.model_class_mapping.get(detected_class.lower(), detected_class)

    def _draw_detection_markers(self, img, x1, y1, x2, y2, cx, cy):
        """绘制检测标记（优化版）"""
        # 检测框
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 180, 0), 2)
        # 中心点
        cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
        # 信息标签
        label = f"{self.R1info_prop_name} {self.R1info_floor_idx} {self.R1dst_dirc}"
        cv2.putText(img, label, (x1 + 5, y1 - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # 方向指示
        arrow_len = 40
        end_x = cx + (arrow_len if self.R1dst_dirc == "right" else -arrow_len)
        cv2.arrowedLine(img, (cx, cy), (end_x, cy), (255, 0, 0), 2, tipLength=0.3)


    def get_template_from_baseposition(self, base_pos, base_quat):
        # 根据底盘位置和朝向，获取模板

        pass
    def set_origin_pos(self, observe, robot):
        if self.R1dst_dirc == "left":
            self.lft_arm_ori_pose = np.array([0.520, 0.238 , 1.2])
            self.right_arm_ori_pose = np.array([0.520, 0.070 , 1.2])
        else:
            self.lft_arm_ori_pose = np.array([0.520, -0.065, 1.2])
            self.right_arm_ori_pose = np.array([0.520, -0.233, 1.2])
            # left_arm_target_euler = np.array([0, -0.93584134 ,-1.6])

        left_arm_target_euler = np.array([0, 0.93584134, 1.6])
        left_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(self.lft_arm_ori_pose, "pick", "l",
                                                                   observe["jq"][2], np.array(observe["jq"][5:11]),
                                                                   Rotation.from_euler('zyx',
                                                                                       left_arm_target_euler).as_matrix())
        right_arm_target_euler = np.array([0, 0.93584134, -1.6])
        right_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(self.right_arm_ori_pose, "pick", "r",
                                                                    observe["jq"][2], np.array(observe["jq"][12:18]),
                                                                    Rotation.from_euler('zyx',
                                                                                        right_arm_target_euler).as_matrix())
        print("left_arm_joint", left_arm_joint)
        print("right_arm_joint", right_arm_joint)
        # 调整机械臂的预备位置
        height = 1.015 - self.R1dst[2]
        step_num = 100
        i = 0
        while i < step_num:
            if robot.data_renew is False:
                time.sleep(0.002)
                continue
            robot.data_renew = False
            i += 1
            robot.target_control[5:11] = np.array(left_arm_joint) * i/step_num
            robot.target_control[12:18] = np.array(right_arm_joint) * i/step_num
            robot.target_control[2] = height *i/step_num
            observe, pri_observe, rew, ter, info = robot.step(robot.target_control)
        return observe

    def catch_box(self, observe, robot):
        # TODO：根据小物体的位置，调整机械臂的位置，抓取小物体
        print("开始抓取物体")
        # x ,y ,z的增量
        if self.R1info_cabinet_dir == "right":
            print("base pos:", observe["base_position"])
            base_pos=[0.614, 0.393]
            self.x_forward = 0.15 + base_pos[0] - observe["base_position"][0]
            self.y_forward = 0.0 + base_pos[1] - observe["base_position"][1]
            self.z_forward = -0.035
        else:
            base_pos=[0.370, 0.613]
            self.x_forward = 0.15 + base_pos[0] - observe["base_position"][0]
            self.y_forward = -0.01 + base_pos[1] - observe["base_position"][1]
            self.z_forward = -0.05
        # 增量
        forward = np.array([self.x_forward, self.y_forward, self.z_forward])
        step = 0
        step_num = 100
        while step < step_num:
            if robot.data_renew is False:
                time.sleep(0.002)
                continue
            robot.data_renew = False
            step += 1
            left_arm_target_pose = self.lft_arm_ori_pose + forward * step / step_num
            right_arm_target_pose = self.right_arm_ori_pose + forward * step / step_num
            # 保持相对高度不变
            left_arm_target_pose[2] = left_arm_target_pose[2] - observe["jq"][2]
            right_arm_target_pose[2] = right_arm_target_pose[2] - observe["jq"][2]
            left_arm_target_euler = np.array([0, 0.93584134, 1.6])
            left_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(left_arm_target_pose, "pick", "l",
                                                                       observe["jq"][2], np.array(observe["jq"][5:11]),
                                                                       Rotation.from_euler('zyx', left_arm_target_euler).as_matrix())
            robot.target_control[5:11] = left_arm_joint

            right_arm_target_euler = np.array([0, 0.93584134, -1.6])
            right_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(right_arm_target_pose, "pick", "r",
                                                                        observe["jq"][2], np.array(observe["jq"][12:18]),
                                                                        Rotation.from_euler('zyx', right_arm_target_euler).as_matrix())
            robot.target_control[12:18] = right_arm_joint
            observe, pri_observe, rew, ter, info = robot.step(robot.target_control)


        observe = self.gripper_control(observe, robot, "both", "close")
        return observe

    def gripper_control(self, observe, robot, arm, direction):
        if arm == "left":
            if direction == "open":
                robot.target_control[11] = 0.5
            elif direction == "close":
                robot.target_control[11] = 0.
        elif arm == "right":
            if direction == "open":
                robot.target_control[18] = 0.5
            elif direction == "close":
                robot.target_control[18] = 0.
        else:
            if direction == "open":
                robot.target_control[11] = 0.5
                robot.target_control[18] = 0.5
            elif direction == "close":
                robot.target_control[11] = 0.
                robot.target_control[18] = 0.

        self.robot_pause(robot, 100)
        return observe

    def robot_pause(self, robot, num=20):
        _ = 0
        while _ < num:
            if robot.data_renew is False:
                time.sleep(0.002)
                continue
            _ += 1
            robot.data_renew = False
            observe, pri_observe, rew, ter, info = robot.step(robot.target_control)
        return observe

    def base_forward(self, observe, direction, robot, dst=None):
        if dst is None:
            destination = self.R1dst
        else:
            destination = dst
        cool_down = False
        while True:
            if robot.data_renew is False:
                time.sleep(0.002)
                continue
            robot.data_renew = False
            # 计算目的地距离
            dis = destination[direction] - observe["base_position"][direction]
            if abs(dis) > 0.05:
                cool_down = False
                robot.target_control[0] = 5 * dis / abs(dis) * min(max(dis ** 2, 0.04), 0.5)
                observe, pri_observe, rew, ter, info = robot.step(robot.target_control)
                print("forward", observe["base_position"])
            elif abs(dis) > 0.01:
                cool_down = False
                robot.target_control[0] = 0.02 * dis / abs(dis)
                observe, pri_observe, rew, ter, info = robot.step(robot.target_control)
            else:
                if cool_down:
                    print("前进结束")
                    break
                else:
                    print("进入缓冲")
                    cool_down = True
                    self.robot_pause(robot)

        robot.target_control[0] = 0.
        observe, pri_observe, rew, ter, info = robot.step(robot.target_control)
        return observe

    def base_rotate(self, observe, target_angle, robot):
        dis = 0
        cool_down = False
        while True:
            if robot.data_renew is False:
                time.sleep(0.002)
                continue
            robot.data_renew = False
            euler = Rotation.from_quat(observe["base_orientation"]).as_euler('zyx', degrees=True)
            # print("rotate", euler)
            if target_angle - euler[2] > 0:
                if target_angle - euler[2] > 180:
                    dis = 360 - target_angle + euler[2]
                else:
                    dis = target_angle - euler[2]
                    dis = -dis
            if target_angle - euler[2] <= 0:
                if target_angle - euler[2] < -180:
                    dis = 360 + target_angle - euler[2]
                else:
                    dis = target_angle - euler[2]
                    dis = -dis
            if abs(dis) > 5:
                cool_down = False
                # 速度衰减二次函数
                robot.target_control[1] = 5 * dis / abs(dis) * max((dis / 180) ** 2, 0.01)
                observe, pri_observe, rew, ter, info = robot.step(robot.target_control)
                # print("执行旋转")
            elif abs(dis) > 0.3:
                cool_down = False
                robot.target_control[1] = dis / abs(dis) * 0.01
                observe, pri_observe, rew, ter, info = robot.step(robot.target_control)
            else:
                if cool_down:
                    print("旋转结束")
                    return observe
                else:
                    print("进入缓冲")
                    cool_down = True
                robot.target_control[1] = 0.
                self.robot_pause(robot)


    def solve_rule(self, describe):
        round_num = re.search(r"round(\w+): (\w+)", describe)
        if round_num and round_num.group(1) == "1":
            match = re.search(r'Take the (\w+) from the (\w+) floor of the (\w+) cabinet, '
                              r'and put it on the (\w+) table.',
                              describe)
            if match:
                self.R1info_prop_name = match.group(1)
                self.R1info_floor_idx = match.group(2)
                self.R1info_cabinet_dir = match.group(3)
                self.R1info_table_dir = match.group(4)

                # 获取目标位置信息
                self.R1dst[2] = self.floor_height.get(self.R1info_floor_idx)
                self.R1dst[0] = self.cabinet_position.get(self.R1info_cabinet_dir)[0]
                self.R1dst[1] = self.cabinet_position.get(self.R1info_cabinet_dir)[1]
                print(f"Target position: {self.R1dst}")
        elif round_num and round_num.group(1) == "2":
            match = re.search(r'Find the (\w+) with a surface featuring robot textures, '
                              r'and put it to the (\w+) of the (\w+).',
                              describe)
            if match:
                self.R1info_prop_name = match.group(1)

        elif round_num and round_num.group(1) == "3":
            match = re.search(r'Find another prop as same as the one in the drawer top layer, '
                              r'and put it to the front of the scissors.',
                              describe)
            self.R1info_prop_name = "sheet"

        return self.R1dst

def main(args=None):
    rclpy.init(args=args)
    exec_robot = DianRobotNode()
    spin_thead = threading.Thread(target=lambda: rclpy.spin(exec_robot))
    spin_thead.start()
    # pub_thread = threading.Thread(target=exec_robot.pub_thread)
    # pub_thread.start()

    policy = DianRobot()
    #指令解析
    while exec_robot.task_info is None:
        pass
    dst = policy.solve_rule(exec_robot.task_info)


    # 前往观测位置
    obs = policy.gripper_control(exec_robot.obs, exec_robot, "both", "open")
    obs = policy.base_rotate(exec_robot.obs, 90, exec_robot)
    obs = policy.robot_pause(exec_robot)
    obs = policy.base_forward(exec_robot.obs, 1, exec_robot, [0, 0.4, policy.R1dst[2]])
    obs = policy.robot_pause(exec_robot)
    obs = policy.base_rotate(exec_robot.obs, 180, exec_robot)
    obs = policy.robot_pause(exec_robot)
    obs = policy.base_rotate(exec_robot.obs, 180, exec_robot)

    # 通过图片解析箱子位置
    obs = policy.get_box_pos(obs, exec_robot, is_first_call=True)
    if policy.R1info_cabinet_dir == "right":
        # print("obs1", obs["base_position"])
        # 观测目标位置并根据箱子左右初始化夹爪位置
        obs = policy.set_origin_pos(obs, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 180, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_forward(obs, 0, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 180, exec_robot)
        obs = policy.catch_box(obs, exec_robot)
        obs = policy.base_forward(obs, 0, exec_robot, [policy.R1dst[0] - 0.3, policy.R1dst[1], policy.R1dst[2]])
    else:
        obs = policy.base_forward(obs, 0, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 90, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_forward(obs, 1, exec_robot, [policy.R1dst[0], 0, policy.R1dst[2]])
        obs = policy.robot_pause(exec_robot)
        # 通过图片解析箱子位置
        obs = policy.get_box_pos(obs, exec_robot, is_first_call=False)
        # 根据箱子左右初始化夹爪位置
        obs = policy.set_origin_pos(obs, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 90, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_forward(obs, 1, exec_robot)
        obs = policy.robot_pause(exec_robot)
        obs = policy.base_rotate(obs, 90, exec_robot)
        obs = policy.catch_box(obs, exec_robot)
        obs = policy.base_forward(obs, 1, exec_robot, [policy.R1dst[0], policy.R1dst[1] - 0.3, policy.R1dst[2]])
    spin_thead.join()
    # pub_thread.join()

if __name__ == '__main__':
    main()
