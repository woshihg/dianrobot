from discoverse.mmk2 import MMK2FIK
from ultralytics import YOLO
import re
import cv2
from scipy.spatial.transform import Rotation as R
from DianUtils.RobotIO import *
from DianUtils.Command import *
from RosNode import DianRobotNode

def calc_robot_direction(base_orientation):
    bo = base_orientation
    corrected_obs = [bo[1], bo[2], bo[3], bo[0]]
    euler = R.from_quat(corrected_obs).as_euler('zyx', degrees=False)
    return np.array([np.cos(euler[0]), np.sin(euler[0])])
# region robot utils

def create_transform_matrix(pos, quat=None, euler=None):
    # 通过欧拉角或四元数来计算旋转矩阵
    rot_matrix = np.eye(3)
    if euler is not None:
        rot_matrix = R.from_euler('xyz', euler, degrees=False).as_matrix()
    elif quat is not None:
        rot_matrix = R.from_quat(quat).as_matrix()

    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rot_matrix
    transform_matrix[:3, 3] = pos
    return transform_matrix


def get_cam_t(obs):
    # Define the local transformations
    # 使用wxyz四元数计算欧拉角
    quat_world = obs["base_orientation"]
    quat_world = [quat_world[1], quat_world[2], quat_world[3], quat_world[0]]  # 重新排列四元数元素
    euler_wolrd = Rotation.from_quat(quat_world).as_euler("xyz")
    print("euler_wolrd", euler_wolrd)
    trans_agv_to_slide = create_transform_matrix([0.02371, 0, 0])
    trans_slide_to_head_yaw = create_transform_matrix([0, 0, 1.311 - obs["jq"][2]])
    trans_head_yaw_to_head_pitch = create_transform_matrix([0.18375, 0, 0.023], euler=[0, 0, 1.5708])
    trans_head_pitch_to_link1 = create_transform_matrix([0.00099952, 3.1059e-05, 0.058],
                                                             quat=[-0.5, 0.5, -0.5, 0.5])
    trans_link1_to_cam = create_transform_matrix([0.0755, -0.1855, 0], quat=[0.70711, 0., -0.70711, 0])
    trans_cam_to_head_cam = create_transform_matrix([-0.035, 0, 0], euler=[-0.33 - obs["jq"][4], 0, 0])
    trans_head_cam_to_site = create_transform_matrix([0, 0, 0], euler=[-3.1416, 0, 0])
    # Compute the total transformation from 'agv_link' to 'head_cam'
    trans_agv_to_head_cam = (
            trans_agv_to_slide @ trans_slide_to_head_yaw
            @ trans_head_yaw_to_head_pitch @ trans_head_pitch_to_link1
            @ trans_link1_to_cam @ trans_cam_to_head_cam @ trans_head_cam_to_site)
    print("trans_agv_to_cam", trans_agv_to_head_cam)
    # 截取旋转矩阵计算四元数
    trans_agv_to_head_cam_ = trans_agv_to_head_cam[:3, :3]
    quat = Rotation.from_matrix(trans_agv_to_head_cam_).as_quat()
    print("quat", quat)
    # 计算欧拉角
    euler = Rotation.from_matrix(trans_agv_to_head_cam_).as_euler("ZYX")
    print("euler", euler)
    return trans_agv_to_head_cam

def _preprocess_image(img):
    """图像增强处理"""
    # CLAHE对比度增强
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l_enhanced = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l_enhanced, a, b]), cv2.COLOR_LAB2BGR)

# endregion robot utils


class DianRobot:

    #region initialize

    def __init__(self):
        self.right_arm_target_pose = None
        self.left_arm_target_pose = None
        self.z_forward = None
        self.x_forward = None
        self.y_forward = None
        self.rgt_arm_ori_pose = None
        self.lft_arm_ori_pose = None
        self.R1info_prop_name = ""
        self.R1info_floor_idx = ""
        self.R1info_cabinet_dir = ""
        self.R1info_table_dir = ""
        self.camera_intrinsic = [[327.09, 0, 320], [0, 327.09, 240], [0, 0, 1]]
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
        prize_model_path = "/home/workspace/prize_m.pt"
        self.grasp_model = YOLO(prize_model_path)

        # region init cmd
        self.cmd_turn = TurnRobotCommand()
        self.cmd_forward = ForwardRobotCommand()
        self.cmd_pos_arm_l = MoveArmCommand()
        self.cmd_pos_arm_l.arm_controller.set_is_right(False)
        self.cmd_pos_arm_r = MoveArmCommand()
        self.cmd_pos_arm_r.arm_controller.set_is_right(True)
        self.cmd_pos_arm_lr = MoveArmsCommand()
        self.cmd_rot_arm_l = RotArmCommand()
        self.cmd_rot_arm_r = RotArmCommand()
        self.cmd_rot_arm_lr = RotArmsCommand()
        self.cmd_height = RobotHeightCommand()
        self.cmd_grippers = RobotGrippersCommand()
        self.cmd_gripper_l = RobotGripperCommand()
        self.cmd_gripper_r = RobotGripperCommand()
        self.cmd_head_pitch = RobotHeadPitchCommand()
        # endregion

    def init_io(self, node):
        self.cmd_forward.control_sender = RobotForwardMotorSender(node)
        self.cmd_forward.motor_receiver = RobotForwardMotorReceiver(node)
        self.cmd_turn.control_sender = RobotYawMotorSender(node)
        self.cmd_turn.motor_receiver = RobotYawMotorReceiver(node)

        self.cmd_pos_arm_l.control_sender = RobotLArmMotorSender(node)
        self.cmd_pos_arm_l.motor_receiver = RobotLArmMotorReceiver(node)
        self.cmd_pos_arm_r.control_sender = RobotRArmMotorSender(node)
        self.cmd_pos_arm_r.motor_receiver = RobotRArmMotorReceiver(node)
        self.cmd_rot_arm_l.control_sender = RobotLArmMotorSender(node)
        self.cmd_rot_arm_l.motor_receiver = RobotLArmMotorReceiver(node)
        self.cmd_rot_arm_r.control_sender = RobotRArmMotorSender(node)
        self.cmd_rot_arm_r.motor_receiver = RobotRArmMotorReceiver(node)
        self.cmd_pos_arm_lr.control_sender = RobotLRArmMotorSender(node)
        self.cmd_pos_arm_lr.motor_receiver = RobotLRArmMotorReceiver(node)
        self.cmd_rot_arm_lr.control_sender = RobotLRArmMotorSender(node)
        self.cmd_rot_arm_lr.motor_receiver = RobotLRArmMotorReceiver(node)

        self.cmd_height.control_sender = RobotHeightMotorSender(node)
        self.cmd_height.motor_receiver = RobotHeightMotorReceiver(node)
        self.cmd_gripper_l.control_sender = RobotLGripperMotorSender(node)
        self.cmd_gripper_l.motor_receiver = RobotLGripperMotorReceiver(node)
        self.cmd_gripper_r.control_sender = RobotRGripperMotorSender(node)
        self.cmd_gripper_r.motor_receiver = RobotRGripperMotorReceiver(node)
        self.cmd_grippers.control_sender = RobotLRGripperMotorSender(node)
        self.cmd_grippers.motor_receiver = RobotLRGripperMotorReceiver(node)
        self.cmd_head_pitch.control_sender = RobotHeadPitchMotorSender(node)
        self.cmd_head_pitch.motor_receiver = RobotHeadPitchMotorReceiver(node)

    #endregion initialize

    # region robot control
    def get_grasp_pos(self, obs):
        detected_objects = self.detect_grasp_objects(obs["img"][0], obs["depth"])
        if len(detected_objects) == 0:
            print("No object detected!")
            return None
        obj = detected_objects[0]
        print(f"Detected object: {obj['class']} at ({obj['X']},{obj['Y']},{obj['Z']})m")
        trans_agv_to_head_cam = get_cam_t(obs)
        obj_cam_pos = [obj['X'], obj['Y'], obj['Z']]
        obj_cam_roll = Rotation.from_euler("ZYX", [0, 0, 0]).as_matrix()
        obj_euler = Rotation.from_matrix(obj_cam_roll).as_euler("ZYX")
        print("obj_euler", obj_euler)
        # 构建物体坐标系在相机坐标系下的变换矩阵
        obj_cam_t = np.eye(4)
        obj_cam_t[:3, :3] = obj_cam_roll
        obj_cam_t[:3, 3] = obj_cam_pos
        # 计算物体在世界坐标系下的变换矩阵
        obj_world_t = trans_agv_to_head_cam @ obj_cam_t
        print("obj_world_T", obj_world_t)

        # 物体坐标系在世界坐标系下的欧拉角
        obj_world_euler = Rotation.from_matrix(obj_world_t[:3, :3]).as_euler("ZYX")
        print("obj_world_euler", obj_world_euler)

        # 物体坐标系在世界坐标系下的位置
        obj_world_pos = obj_world_t[:3, 3]
        print("obj_world_pos", obj_world_pos)
        return obj_world_pos

    def postprocess(self, results, orig_img):
        image = orig_img.copy()
        detected_objects = []
        class_names = ["box", "carton", "disk", "sheet", "airbot", "blue_circle", "-1"]
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.conf.item() < 0.7:
                    continue

                x0, y0, x1, y1 = map(int, box.xyxy[0].cpu().numpy())
                conf = box.conf.item()
                cls_id = int(box.cls.item())
                if class_names[cls_id] != self.R1info_prop_name:
                    continue
                obj_info = {
                    'class': class_names[cls_id],
                    'confidence': conf,
                    'x': int((x0 + x1) / 2),
                    'y': int((y0 + y1) / 2),
                    'w': int(x1 - x0),
                    'h': int(y1 - y0)
                }
                detected_objects.append(obj_info)
                color = (0, 255, 0)
                cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)

        return image, detected_objects

    def detect_grasp_objects(self, rgb_image, img_depth):
        results = self.grasp_model(rgb_image, verbose=False)
        image, detected_objects = self.postprocess(results, rgb_image)
        # print(f"inference time: {inference_time:.2f}ms")

        fx = self.camera_intrinsic[0][0]
        fy = self.camera_intrinsic[1][1]
        cx = self.camera_intrinsic[0][2]
        cy = self.camera_intrinsic[1][2]
        for obj in detected_objects:
            center_x = obj['x']
            center_y = obj['y']
            depth_m = img_depth[center_y, center_x]  # mm -> m
            # X->right，Y->down Z->forward
            z = depth_m / 1000.0
            x = (center_x - cx) * z / fx
            y = (center_y - cy) * z / fy
            obj.update({
                'X': x,
                'Y': y,
                'Z': z
            })
        cv2.imwrite("grasp_detect.jpg", image)
        return detected_objects

    def catch_prop(self, pos, observe, robot):
        prop_bias = [0.03, 0, 0.1]
        prop_height = -0.1
        target_open = 1.0
        # 夹物体时的开合度
        target_close = 0.0
        if self.R1info_prop_name == "carton":
            # height 0.9781288889181835
            print("抓取 carton")
            prop_bias = [0.04, 0, 0.1]
            if self.R1info_table_dir == "left":
                prop_height = -0.08
                print("left arm")
            else:
                prop_height = -0.08
                print("right arm")
            target_open = 1.0
            target_close = 0.15
        elif self.R1info_prop_name == "disk":
            print("抓取 disk")
            prop_bias = [0.03, 0, 0.1]
            prop_height = -0.1
            target_open = 0.4
            target_close = 0.01
        elif self.R1info_prop_name == "sheet":
            print("抓取 sheet")
            prop_bias = [0.03, 0, 0.1]
            prop_height = -0.1
            target_open = 0.4
            target_close = 0.0
        self.cmd_grippers.l_gripper_controller \
            .set_target(target_open) \
            .set_current(observe["jq"][11])
        self.cmd_grippers.r_gripper_controller \
            .set_target(target_open) \
            .set_current(observe["jq"][18])
        robot_do(self.cmd_grippers)
        if self.R1info_table_dir == "left":
            # 预制机械臂的位置
            left_arm_ori_pose = pos + prop_bias
            left_arm_target_euler = [0., 0.5, -np.pi]
            left_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(left_arm_ori_pose, "pick", "l",
                                                                       observe["jq"][2], np.array(observe["jq"][5:11]),
                                                                       Rotation.from_euler('zyx',
                                                                                           left_arm_target_euler).as_matrix())
            if left_arm_joint is None:
                right_arm_ori_pose = pos + prop_bias
                right_arm_target_euler = [0., 0.3, np.pi]
                right_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(right_arm_ori_pose, "pick", "r",
                                                                            observe["jq"][2],
                                                                            np.array(observe["jq"][12:18]),
                                                                            Rotation.from_euler('zyx',
                                                                                                right_arm_target_euler).as_matrix())
            # self.cmd_rot_arm_l.arm_controller \
            #     .set_target(left_arm_joint) \
            #     .set_current(observe["jq"][5:11])
            # robot_do(self.cmd_rot_arm_l)
            for i in range(6):
                robot.tctr_left_arm[i] = float(left_arm_joint[i])
            robot.publish_messages()
            #
            time.sleep(2)
            left_arm_target_pose = left_arm_ori_pose.copy()
            if self.R1info_prop_name == "carton":
                left_arm_target_pose[2] = 0.9691288889181835
            else:
                left_arm_target_pose[2] = left_arm_target_pose[2] + prop_height
            #print("left_arm_target_pose[2]", left_arm_target_pose[2])
            robot.target_control[5:11] = left_arm_joint
            self.cmd_pos_arm_l.arm_controller \
                .set_robot_height(robot.obs["jq"][2]) \
                .set_motor_angles(np.array(observe["jq"][5:11])) \
                .set_target_rot(np.array(left_arm_target_euler)) \
                .set_current_pos(left_arm_ori_pose) \
                .set_ratio(0.001) \
                .set_target_pos(left_arm_target_pose)
            robot_do(self.cmd_pos_arm_l)
            time.sleep(2)
            self.cmd_grippers.l_gripper_controller \
                .set_target(target_close) \
                .set_current(observe["jq"][11])
            robot_do(self.cmd_grippers)
            time.sleep(2)
            left_arm_target_pose1 = left_arm_target_pose.copy()
            left_arm_target_pose1[2] = left_arm_target_pose[2] + 0.2
            self.cmd_pos_arm_l.arm_controller \
                .set_robot_height(robot.obs["jq"][2]) \
                .set_motor_angles(np.array(observe["jq"][5:11])) \
                .set_target_rot(np.array(left_arm_target_euler)) \
                .set_current_pos(left_arm_target_pose) \
                .set_ratio(0.001) \
                .set_target_pos(left_arm_target_pose1)
            robot_do(self.cmd_pos_arm_l)
            time.sleep(2)
        else:
            right_arm_ori_pose = pos + prop_bias
            right_arm_target_euler = [0., 0.5, np.pi]
            right_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(right_arm_ori_pose, "pick", "r",
                                                                        observe["jq"][2], np.array(observe["jq"][12:18]),
                                                                        Rotation.from_euler('zyx',
                                                                                            right_arm_target_euler).as_matrix())
            # self.cmd_rot_arm_r.arm_controller \
            #     .set_target(right_arm_joint) \
            #     .set_current(observe["jq"][12:18])
            # robot_do(self.cmd_rot_arm_r)
            for i in range(6):
                robot.tctr_right_arm[i] = float(right_arm_joint[i])
            robot.publish_messages()
            time.sleep(2)
            right_arm_target_pose = right_arm_ori_pose.copy()
            if self.R1info_prop_name == "carton":
                right_arm_target_pose[2] = 0.9691288889181835
            else:
                right_arm_target_pose[2] = right_arm_target_pose[2] + prop_height
            #print("right_arm_target_pose[2]", right_arm_target_pose[2])
            self.cmd_pos_arm_r.arm_controller \
                .set_robot_height(robot.obs["jq"][2]) \
                .set_motor_angles(np.array(observe["jq"][12:18])) \
                .set_target_rot(np.array(right_arm_target_euler)) \
                .set_current_pos(right_arm_ori_pose) \
                .set_ratio(0.001) \
                .set_target_pos(right_arm_target_pose)
            robot_do(self.cmd_pos_arm_r)
            time.sleep(2)
            self.cmd_grippers.r_gripper_controller \
                .set_target(target_close) \
                .set_current(observe["jq"][11])
            robot_do(self.cmd_grippers)
            time.sleep(2)
            right_arm_target_pose1 = right_arm_target_pose.copy()
            right_arm_target_pose1[2] = right_arm_target_pose1[2] + 0.2
            self.cmd_pos_arm_r.arm_controller \
                .set_robot_height(robot.obs["jq"][2]) \
                .set_motor_angles(np.array(observe["jq"][12:18])) \
                .set_target_rot(np.array(right_arm_target_euler)) \
                .set_current_pos(right_arm_target_pose) \
                .set_ratio(0.001) \
                .set_target_pos(right_arm_target_pose1)
            robot_do(self.cmd_pos_arm_r)
            time.sleep(2)

        return
    def put_prop(self, robot):
        # self.cmd_forward.forward_controller \
        #     .set_current(robot.obs["base_position"][:2]) \
        #     .set_target([-0.15, -0.15]) \
        #     .set_ratio(0.01) \
        #     .set_tolerance(0.01) \
        #     .set_direction_vec(calc_robot_direction(robot.obs["base_orientation"]))
        # robot_do(self.cmd_forward)
        if self.R1info_table_dir == "left":
            self.cmd_turn.yaw_controller \
                .set_current(robot.obs["base_orientation"]) \
                .set_target(-120.0) \
                .set_tolerance(0.01)
            robot_do(self.cmd_turn)
        else:
            self.cmd_turn.yaw_controller \
                .set_current(robot.obs["base_orientation"]) \
                .set_target(-150.0) \
                .set_tolerance(0.01)
            robot_do(self.cmd_turn)

        self.cmd_height.height_controller \
            .set_current(robot.obs["jq"][2]) \
            .set_target(0.23) \
            .set_ratio(0.01) \
            .set_tolerance(0.01)
        robot_do(self.cmd_height)
        time.sleep(2)
        # 松开夹爪
        self.cmd_grippers.l_gripper_controller \
            .set_target(0.5) \
            .set_current(robot.obs["jq"][11])
        self.cmd_grippers.r_gripper_controller \
            .set_target(0.5) \
            .set_current(robot.obs["jq"][18])
        robot_do(self.cmd_grippers)
        time.sleep(2)
        return

    def get_box_pos(self, head_img, is_first_call=True):
        """获取图片并进行处理的部分"""
        conf_threshold = 0.25  # 置信度阈值 (0-1之间)
        iou_threshold = 0.45  # IoU阈值 (非极大值抑制用)
        hide_labels = False  # 是否隐藏标签
        hide_conf = False  # 是否隐藏置信度
        line_width = 2

        img = head_img.copy()
        # 对图像进行预处理
        processed_img = _preprocess_image(img)

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
            self._process_position(processed_img, results)
        # 获取目标位置信息
        self.R1dst[0] = self.cabinet_position.get(self.R1info_cabinet_dir)[0]
        self.R1dst[1] = self.cabinet_position.get(self.R1info_cabinet_dir)[1]
        return

    def _handle_first_detection(self, results):
        """处理首次检测逻辑"""
        target_found = any(
            self._map_class(result.names[int(box.cls.item())]) == self.model_class_mapping[self.R1info_prop_name]
            for result in results
            for box in result.boxes
        )
        self.R1info_cabinet_dir = "right" if target_found else "left"

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
                print(f"Detected object at {self.R1info_floor_idx}")

                # 判断左右位置
                self.R1dst_dirc = "right" if center_x > (w // 2) else "left"

                # 绘制可视化标记
                self._draw_detection_markers(img, x1, y1, x2, y2, center_x, center_y)
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

    def get_catch_pos_lr(self, observe):
        # x ,y ,z的增量
        if self.R1info_cabinet_dir == "right":
            base_pos = [0.605, 0.40]
            self.x_forward = 0.15 + base_pos[0] - observe["base_position"][0]
            self.y_forward = 0.01 + base_pos[1] - observe["base_position"][1]
            self.z_forward = -0.05
        else:
            base_pos = [0.370, 0.613]
            self.x_forward = 0.15 + base_pos[0] - observe["base_position"][0]
            self.y_forward = -0.01 + base_pos[1] - observe["base_position"][1]
            self.z_forward = -0.05
        # 增量
        forward = np.array([self.x_forward, self.y_forward, self.z_forward])

        left_arm_target_pose = self.lft_arm_ori_pose + forward
        right_arm_target_pose = self.rgt_arm_ori_pose + forward
        self.left_arm_target_pose = left_arm_target_pose.copy()
        self.right_arm_target_pose = right_arm_target_pose.copy()
        return left_arm_target_pose, right_arm_target_pose


    def set_origin_pos(self, robot):
        # self.R1dst_dirc = "left"
        if self.R1dst_dirc == "left":
            if self.R1info_cabinet_dir == "right":
                self.lft_arm_ori_pose = np.array([0.520, 0.237, 1.2])
                self.rgt_arm_ori_pose = np.array([0.520, 0.070, 1.2])
            else:
                self.lft_arm_ori_pose = np.array([0.521, 0.236, 1.2])
                self.rgt_arm_ori_pose = np.array([0.521, 0.070, 1.2])
        else:
            if self.R1info_cabinet_dir == "right":
                self.lft_arm_ori_pose = np.array([0.520, -0.065, 1.2])
                self.rgt_arm_ori_pose = np.array([0.520, -0.233, 1.2])
            else:
                self.lft_arm_ori_pose = np.array([0.520, -0.055, 1.2])
                self.rgt_arm_ori_pose = np.array([0.520, -0.222, 1.2])

        left_arm_target_euler = np.array([0, 0.93584134, 1.6])
        left_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(self.lft_arm_ori_pose, "pick", "l",
                                                                   robot.obs["jq"][2], np.array(robot.obs["jq"][5:11]),
                                                                   Rotation.from_euler('zyx',
                                                                                       left_arm_target_euler).as_matrix())
        right_arm_target_euler = np.array([0, 0.93584134, -1.6])
        right_arm_joint = MMK2FIK().get_armjoint_pose_wrt_footprint(self.rgt_arm_ori_pose, "pick", "r",
                                                                    robot.obs["jq"][2], np.array(robot.obs["jq"][12:18]),
                                                                    Rotation.from_euler('zyx',
                                                                                        right_arm_target_euler).as_matrix())
        self.cmd_rot_arm_lr.l_arm_controller \
            .set_target(left_arm_joint) \
            .set_current(np.array(robot.obs["jq"][5:11]))

        self.cmd_rot_arm_lr.r_arm_controller \
            .set_target(right_arm_joint) \
            .set_current(np.array(robot.obs["jq"][12:18]))

        robot_do(self.cmd_rot_arm_lr)

        # 调整机械臂的预备位置
        height = 1.015 - self.R1dst[2]
        self.cmd_height.height_controller \
            .set_target(height) \
            .set_current(robot.obs["jq"][2])

        robot_do(self.cmd_height)
        time.sleep(1)
        # 对齐机械臂的高度
        self.lft_arm_ori_pose[2] -= robot.obs["jq"][2]
        self.rgt_arm_ori_pose[2] -= robot.obs["jq"][2]
        return

    # 移动夹爪至中间位置
    def middle_reset(self, robot):
        left_ori_pose = self.left_arm_target_pose.copy()
        right_ori_pose = self.right_arm_target_pose.copy()
        left_target_pose = self.left_arm_target_pose.copy()
        right_target_pose = self.right_arm_target_pose.copy()
        left_target_pose[1] = (self.left_arm_target_pose[1] - self.right_arm_target_pose[1] + 0.0005) / 2
        right_target_pose[1] = -(self.left_arm_target_pose[1] - self.right_arm_target_pose[1] + 0.0005) / 2
        self.cmd_pos_arm_lr.l_arm_controller \
            .set_robot_height(robot.obs["jq"][2]) \
            .set_motor_angles(np.array(robot.obs["jq"][5:11])) \
            .set_target_rot(np.array([0, 0.93584134, 1.6])) \
            .set_current_pos(left_ori_pose) \
            .set_target_pos(left_target_pose) \
            .set_ratio(0.005)

        self.cmd_pos_arm_lr.r_arm_controller \
            .set_robot_height(robot.obs["jq"][2]) \
            .set_motor_angles(np.array(robot.obs["jq"][12:18])) \
            .set_target_rot(np.array([0, 0.93584134, -1.6])) \
            .set_current_pos(right_ori_pose) \
            .set_target_pos(right_target_pose) \
            .set_ratio(0.005)
        robot_do(self.cmd_pos_arm_lr)

        self.cmd_height.height_controller \
            .set_target(0.) \
            .set_current(robot.obs["jq"][2]) \
            .set_ratio(0.01)
        robot_do(self.cmd_height)
        return

    def reset_arm(self, robot):
        self.cmd_height.height_controller \
            .set_target(0.) \
            .set_current(robot.obs["jq"][2]) \
            .set_ratio(0.05) \
            .set_tolerance(0.05)
        robot_do(self.cmd_height)
        time.sleep(1)
        target_control_l = np.array([-0.0, -0.166, 0.032, 0.0, 1.5708, 2.223])
        target_control_r = np.array([-0.0, -0.166, 0.032, 0.0, -1.5708, -2.223])
        self.cmd_rot_arm_lr.l_arm_controller \
            .set_target(target_control_l) \
            .set_current(np.array(robot.obs["jq"][5:11]))
        self.cmd_rot_arm_lr.r_arm_controller \
            .set_target(target_control_r) \
            .set_current(np.array(robot.obs["jq"][12:18]))
        self.cmd_height.height_controller \
            .set_target(0.28) \
            .set_current(robot.obs["jq"][2]) \
            .set_ratio(0.05) \
            .set_tolerance(0.05)
        robot_do(self.cmd_rot_arm_lr)
        robot_do(self.cmd_height)
        time.sleep(2)
        return

    def solve_rule(self, describe):
        #print("describe", describe)
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
        elif round_num and round_num.group(1) == "2":
            match = re.search(r'Find the (\w+) with a surface featuring robot textures, '
                              r'and put it to the (\w+) of the (\w+).',
                              describe)
            if match:
                self.R1info_prop_name = match.group(1)
                self.R1info_table_dir = "left"

        elif round_num and round_num.group(1) == "3":
            # match = re.search(r'Find another prop as same as the one in the drawer top layer, '
            #                   r'and put it to the front of the scissors.',
            #                   describe)
            self.R1info_prop_name = "sheet"
            self.R1info_table_dir = "left"

        return

    # endregion robot control

def robot_do(command: Command, time_s = 60.0):
    frequency = 24.0
    name = command.description
    start = time.time()
    timeout = start + time_s
    print("start command({}) at {}s".format(name, start))
    while not time.time() > timeout and not command.is_done():
        command.feedback()
        command.execute(time.time())
        time.sleep(1.0 / frequency)

    end = time.time()
    if command.is_done():
        print("end command({}) at {}s, cost {}s".format(name, end, end - start))
    else:
        print("timeout command({}) at {}s, cost {}s".format(name, end, end - start))

def robot_do_multi_cmd(commands: list):
    frequency = 24.0
    start = time.time()
    print("start command at {}s".format(start))
    while not all([command.is_done() for command in commands]):
        for command in commands:
            command.feedback()
            command.execute(time.time())
        time.sleep(1.0 / frequency)

    end = time.time()
    print("end command at {}s, cost {}s".format(end, end - start))