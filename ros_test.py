import numpy as np
import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

class DianRobotNode(Node):
    def __init__(self):
        super().__init__('dianrobot_node')
        self.imshow = False
        self.init_subscription()
        self.init_publisher()
    def init_subscription(self):
        self.subscription_clock = self.create_subscription(
            Clock, '/clock', self.clock_callback, 10)
        self.subscription_head_depth_camera_info = self.create_subscription(
            CameraInfo, '/mmk2/head_camera/aligned_depth_to_color/camera_info', self.head_depth_camera_info_callback, 10)
        self.subscription_head_depth_image = self.create_subscription(
            Image, '/mmk2/head_camera/aligned_depth_to_color/image_raw', self.head_depth_image_callback, 10)
        self.subscription_head_rgb_camera_info = self.create_subscription(
            CameraInfo, '/mmk2/head_camera/color/camera_info', self.head_rgb_camera_info_callback, 10)
        self.subscription_head_rgb_image = self.create_subscription(
            Image, '/mmk2/head_camera/color/image_raw', self.head_rgb_image_callback, 10)
        self.subscription_left_camera_info = self.create_subscription(
            CameraInfo, '/mmk2/left_camera/color/camera_info', self.left_camera_info_callback, 10)
        self.subscription_left_image = self.create_subscription(
            Image, '/mmk2/left_camera/color/image_raw', self.left_image_callback, 10)
        self.subscription_right_camera_info = self.create_subscription(
            CameraInfo, '/mmk2/right_camera/color/camera_info', self.right_camera_info_callback, 10)
        self.subscription_right_image = self.create_subscription(
            Image, '/mmk2/right_camera/color/image_raw', self.right_image_callback, 10)
        self.subscription_odom = self.create_subscription(
            Odometry, '/mmk2/odom', self.odom_callback, 10)
        self.subscription_joint_states = self.create_subscription(
            JointState, '/mmk2/joint_states', self.joint_states_callback, 10)
        # 1Hz信息：
        self.subscription_taskinfo = self.create_subscription(
            String, '/s2r2025/taskinfo', self.taskinfo_callback, 10)
        self.subscription_gameinfo = self.create_subscription(
            String, '/s2r2025/gameinfo', self.gameinfo_callback, 10)

    def init_publisher(self):
        self.publisher_cmd_vel = self.create_publisher(Twist, '/mmk2/cmd_vel', 10)
        self.publisher_head = self.create_publisher(Float64MultiArray, '/mmk2/head_forward_position_controller/commands', 10)
        self.publisher_left_arm = self.create_publisher(Float64MultiArray, '/mmk2/left_arm_forward_position_controller/commands', 10)
        self.publisher_right_arm = self.create_publisher(Float64MultiArray, '/mmk2/right_arm_forward_position_controller/commands', 10)
        self.publisher_spine = self.create_publisher(Float64MultiArray, '/mmk2/spine_forward_position_controller/commands', 10)

        self.timer = self.create_timer(1.0, self.publish_messages)

    def publish_messages(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(twist_msg)

        float_array_msg_head = Float64MultiArray()
        float_array_msg_head.data = [1.0, 2.0]  # Adjust with desired values
        self.publisher_head.publish(float_array_msg_head)

        float_array_msg_left_arm = Float64MultiArray()
        float_array_msg_left_arm.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]  # Adjust with desired values
        self.publisher_left_arm.publish(float_array_msg_left_arm)

        float_array_msg_right_arm = Float64MultiArray()
        float_array_msg_right_arm.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]  # Adjust with desired values
        self.publisher_right_arm.publish(float_array_msg_right_arm)

        float_array_msg_spine = Float64MultiArray()
        float_array_msg_spine.data = [1.0]  # Adjust with desired values
        self.publisher_spine.publish(float_array_msg_spine)

    def pub_thread(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            self.publish_messages()
            rate.sleep()

    def clock_callback(self, msg):
            # 仿真时钟
            self.get_logger().info('Received clock: %s' % msg)

    def head_depth_camera_info_callback(self, msg):
        # mmk2机器人头部深度相机 内参
        self.get_logger().info('Received head depth camera info: %s' % msg)

    def head_depth_image_callback(self, msg):
        # mmk2机器人头部深度相机 图像和rgb图像对齐，编码格式为mono16，单位毫米
        # 恢复图像
        depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        if self.imshow:
            # 将图像范围缩放到0-255
            depth_image_show = depth_image.copy()
            depth_image_show = cv2.normalize(depth_image_show, None, 0, 255, cv2.NORM_MINMAX)
            # 图像转化为整数
            depth_image_show = depth_image_show.astype(np.uint8)
            cv2.imshow('head_depth_image', depth_image_show)
            cv2.waitKey(1)
        self.get_logger().info('Received head depth image, height: %d, width: %d, ' % (msg.height, msg.width))


    def head_rgb_camera_info_callback(self, msg):
        # mmk2机器人头部RGB相机 内参
        self.get_logger().info('Received head RGB camera info: %s' % msg)

    def head_rgb_image_callback(self, msg):
        # mmk2机器人头部RGB相机 图像，编码格式rgb8
        # 恢复图像
        rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self.imshow:
            cv2.imshow('head_rgb_image', rgb_image)
            cv2.waitKey(1)
        self.get_logger().info('Received head RGB image, height: %d, width: %d' % (msg.height, msg.width))

    def left_camera_info_callback(self, msg):
        # mmk2机器人左臂RGB相机 内参
        self.get_logger().info('Received left camera info: %s' % msg)

    def left_image_callback(self, msg):
        # mmk2机器人左臂RGB相机 图像，编码格式rgb8
        rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self.imshow:
            cv2.imshow('left_image', rgb_image)
            cv2.waitKey(1)
        self.get_logger().info('Received left image, height: %d, width: %d' % (msg.height, msg.width))

    def right_camera_info_callback(self, msg):
        # mmk2机器人右臂RGB相机 内参
        self.get_logger().info('Received right camera info: %s' % msg)

    def right_image_callback(self, msg):
        # mmk2机器人右臂RGB相机 图像，编码格式rgb8
        rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self.imshow:
            cv2.imshow('right_image', rgb_image)
            cv2.waitKey(1)
        self.get_logger().info('Received right image, height: %d, width: %d' % (msg.height, msg.width))

    def odom_callback(self, msg):
        # 获取时间戳
        timestamp = msg.header.stamp
        seconds = timestamp.sec
        nanoseconds = timestamp.nanosec

        # 获取坐标系信息
        frame_id = msg.header.frame_id

        # 获取child_frame_id
        child_frame_id = msg.child_frame_id

        # 获取位置信息
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        position_z = msg.pose.pose.position.z

        # 获取姿态信息
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        # 获取线速度信息
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        linear_z = msg.twist.twist.linear.z

        # 获取角速度信息
        angular_x = msg.twist.twist.angular.x
        angular_y = msg.twist.twist.angular.y
        angular_z = msg.twist.twist.angular.z

        # 打印相关信息
        self.get_logger().info('Received odometry:')
        self.get_logger().info('Timestamp: %d seconds, %d nanoseconds' % (seconds, nanoseconds))
        self.get_logger().info('Frame ID: %s' % frame_id)
        self.get_logger().info('Child Frame ID: %s' % child_frame_id)
        self.get_logger().info('Position: x=%f, y=%f, z=%f' % (position_x, position_y, position_z))
        self.get_logger().info(
            'Orientation: x=%f, y=%f, z=%f, w=%f' % (orientation_x, orientation_y, orientation_z, orientation_w))
        self.get_logger().info('Linear Velocity: x=%f, y=%f, z=%f' % (linear_x, linear_y, linear_z))
        self.get_logger().info('Angular Velocity: x=%f, y=%f, z=%f' % (angular_x, angular_y, angular_z))

    def joint_states_callback(self, msg):
        # mmk2机器人全身关节状态量，顺序为 joint_names: [
        # - slide_joint
        # - head_yaw_joint
        # - head_pitch_joint
        # - left_arm_joint1
        # - left_arm_joint2
        # - left_arm_joint3
        # - left_arm_joint4
        # - left_arm_joint5
        # - left_arm_joint6
        # - left_arm_eef_gripper_joint
        # - right_arm_joint1
        # - right_arm_joint2
        # - right_arm_joint3
        # - right_arm_joint4
        # - right_arm_joint5
        # - right_arm_joint6
        # - right_arm_eef_gripper_joint ]
        self.get_logger().info('Received joint states: %s' % msg)

    def taskinfo_callback(self, msg):
        # 任务信息
        self.get_logger().info('Received task info: %s' % msg)

    def gameinfo_callback(self, msg):
        # 当前比赛的任务完成情况
        self.get_logger().info('Received game info: %s' % msg)


def main(args=None):
    rclpy.init(args=args)
    my_node = DianRobotNode()
    spin_thead = threading.Thread(target=lambda: rclpy.spin(my_node))
    spin_thead.start()
    pub_thread = threading.Thread(target=my_node.pub_thread)
    pub_thread.start()

    spin_thead.join()
    pub_thread.join()
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
