# -- coding: UTF-8
import os
import time
import numpy as np
import h5py
import argparse
import dm_env

import collections
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, CameraInfo
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2


class RobotKinematics:
    """
    机器人运动学类，用于计算正运动学（FK）
    使用Modified DH参数（Craig convention）
    """
    def __init__(self, arm_type='piper'):
        """
        初始化机器人运动学参数
        Args:
            arm_type: 机械臂类型，默认为'piper'
        """
        self.arm_type = arm_type
        # Modified DH参数：[alpha(i-1), a(i-1), d(i), theta_offset(i)]
        # alpha: 连杆扭角，绕x(i-1)轴旋转的角度
        # a: 连杆长度，沿x(i-1)轴的距离
        # d: 连杆偏距，沿z(i)轴的距离
        # theta_offset: 关节角偏移量
        
        if arm_type == 'piper':
            # Piper机械臂的Modified DH参数（从URDF文件提取）
            # 每个关节：[alpha(i-1), a(i-1), d(i), theta_offset(i)]
            self.dh_params = [
                [0.0,        0.0,         0.123,    0.0],                    # 关节1: 基座到joint1
                [np.pi/2,    0.0,         0.0,      -np.pi + 0.1359],        # 关节2: joint1到joint2
                [0.0,        0.28503,     0.0,      -1.7939],                # 关节3: joint2到joint3
                [np.pi/2,    -0.021984,   0.25075,  0.0],                    # 关节4: joint3到joint4
                [-np.pi/2,   0.0,         0.0,      0.0],                    # 关节5: joint4到joint5
                [np.pi/2,    0.000088259, 0.091,    0.0],                    # 关节6: joint5到joint6
                [0.0,        0.0,         0.1358,   0.0]                     # 末端: joint6到gripper_base
            ]
            
            # 从URDF文件中提取的双臂基座位置
            # 世界坐标系 = 左臂基座坐标系（base_link）
            # 左臂基座偏移: (0, 0, 0)
            # 右臂基座偏移: (0, -0.6, 0) 相对于base_link
            self.left_base_offset = np.array([0.0, 0.0, 0.0])    # 左臂基座在世界坐标系中的位置
            self.right_base_offset = np.array([0.0, -0.6, 0.0])  # 右臂基座在世界坐标系中的位置
            
            print(f'\033[32m已加载Piper机械臂的DH参数（7个关节+末端）\033[0m')
            print(f'\033[32m世界坐标系: 左臂基座坐标系 (base_link)\033[0m')
            print(f'\033[32m左臂基座偏移: {self.left_base_offset}\033[0m')
            print(f'\033[32m右臂基座偏移: {self.right_base_offset}\033[0m')
        else:
            raise ValueError(f"不支持的机械臂类型: {arm_type}")
    
    def modified_dh_matrix(self, alpha, a, d, theta):
        """
        根据Modified DH参数计算单个关节的变换矩阵（Craig convention）
        Modified DH的变换顺序：
        1. 绕x(i-1)旋转alpha(i-1)
        2. 沿x(i-1)平移a(i-1)
        3. 沿z(i)平移d(i)
        4. 绕z(i)旋转theta(i)
        
        Args:
            alpha: 连杆扭角（绕x(i-1)轴）
            a: 连杆长度（沿x(i-1)轴）
            d: 连杆偏距（沿z(i)轴）
            theta: 关节角度（绕z(i)轴）
        Returns:
            4x4变换矩阵
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        # Modified DH变换矩阵
        T = np.array([
            [ct,    -st,     0,      a],
            [st*ca,  ct*ca, -sa,  -sa*d],
            [st*sa,  ct*sa,  ca,   ca*d],
            [0,      0,      0,      1]
        ])
        return T
    
    def forward_kinematics(self, joint_positions):
        """
        计算正运动学，从基坐标系到末端执行器的变换矩阵
        Args:
            joint_positions: 关节角度数组，numpy array，shape=(6,) 或 (7,)
                           注意：这里只包含6个旋转关节，不包括夹爪关节
        Returns:
            4x4变换矩阵，从基坐标系到末端坐标系
        """
        # 初始化为单位矩阵
        T = np.eye(4)
        
        # 确定关节数量（6个旋转关节 + 1个固定末端变换）
        num_joints = min(len(joint_positions), 6)
        
        # 逐个关节累乘变换矩阵
        for i in range(num_joints):
            alpha, a, d, theta_offset = self.dh_params[i]
            # 当前关节角度 = 关节读数 + 偏移量
            theta = joint_positions[i] + theta_offset
            # 计算当前关节的Modified DH变换矩阵
            T_i = self.modified_dh_matrix(alpha, a, d, theta)
            # 累乘到总变换矩阵
            T = np.dot(T, T_i)
        
        # 添加末端固定变换（gripper_base）
        if len(self.dh_params) > num_joints:
            alpha, a, d, theta_offset = self.dh_params[num_joints]
            T_end = self.modified_dh_matrix(alpha, a, d, theta_offset)
            T = np.dot(T, T_end)
        
        return T
    
    def compute_dual_arm_fk(self, joint_positions_left, joint_positions_right):
        """
        计算双臂的正运动学（相对于世界坐标系）
        世界坐标系 = 左臂基座坐标系 (base_link)
        
        Args:
            joint_positions_left: 左臂关节角度，numpy array，shape=(6,) 或 (7,)
            joint_positions_right: 右臂关节角度，numpy array，shape=(6,) 或 (7,)
        Returns:
            T_world_to_left_ee: 从世界坐标系到左臂末端的变换矩阵 (4x4)
            T_world_to_right_ee: 从世界坐标系到右臂末端的变换矩阵 (4x4)
        """
        # 1. 计算左臂：从左臂基座到左臂末端的变换
        T_left_base_to_ee = self.forward_kinematics(joint_positions_left[:6])
        
        # 2. 左臂基座相对于世界坐标系的变换（世界坐标系就是左臂基座）
        T_world_to_left_base = np.eye(4)
        T_world_to_left_base[:3, 3] = self.left_base_offset
        
        # 3. 从世界坐标系到左臂末端的变换
        T_world_to_left_ee = np.dot(T_world_to_left_base, T_left_base_to_ee)
        
        # 4. 计算右臂：从右臂基座到右臂末端的变换
        T_right_base_to_ee = self.forward_kinematics(joint_positions_right[:6])
        
        # 5. 右臂基座相对于世界坐标系的变换
        # URDF中：<origin xyz="0 -0.6 0" rpy="0 0 0"/>
        T_world_to_right_base = np.eye(4)
        T_world_to_right_base[:3, 3] = self.right_base_offset
        
        # 6. 从世界坐标系到右臂末端的变换
        T_world_to_right_ee = np.dot(T_world_to_right_base, T_right_base_to_ee)
        
        return T_world_to_left_ee, T_world_to_right_ee
    
    def extract_position_orientation(self, T):
        """
        从变换矩阵中提取位置和姿态信息
        Args:
            T: 4x4变换矩阵
        Returns:
            position: [x, y, z] 位置向量
            rotation_matrix: 3x3旋转矩阵
            euler_angles: [roll, pitch, yaw] 欧拉角（ZYX顺序）
        """
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        
        # 计算ZYX欧拉角
        sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            pitch = np.arctan2(-rotation_matrix[2, 0], sy)
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            pitch = np.arctan2(-rotation_matrix[2, 0], sy)
            yaw = 0
        
        euler_angles = np.array([roll, pitch, yaw])
        return position, rotation_matrix, euler_angles

    def quaternion_to_rotation_matrix(self, quat):
        """
        将四元数转换为3x3旋转矩阵
        Args:
            quat: 四元数数组 [x, y, z, w] (scalar last)
        Returns:
            3x3旋转矩阵
        """
        x, y, z, w = quat
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])

    def get_ee_to_camera_transform(self, side):
        """
        获取从末端执行器到相机光学帧的固定变换矩阵
        Args:
            side: 'left' 或 'right'
        Returns:
            4x4变换矩阵 T_ee_to_camera
        """
        if side == 'left':
            xyz = np.array([-0.07354390146283836, 0.007804615886680326, 0.038433103882865485])
            quat = np.array([0.12827667735969864, -0.14997843923484816, 0.7053150782153692, -0.6808687114652002])  # xyzw
        elif side == 'right':
            xyz = np.array([-0.07249626512541643, 0.006136740643933003, 0.038600128562769884])
            quat = np.array([0.1256876224765708, -0.1305963178064823, 0.6996428946145881, -0.6911201366961431])  # xyzw
        else:
            raise ValueError(f"不支持的侧别: {side}")

        R = self.quaternion_to_rotation_matrix(quat)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T

    def compute_dual_camera_fk(self, joint_positions_left, joint_positions_right):
        """
        计算双臂的相机正运动学（相对于世界坐标系）
        世界坐标系 = 左臂基座坐标系 (base_link)
        
        Args:
            joint_positions_left: 左臂关节角度，numpy array，shape=(6,) 或 (7,)
            joint_positions_right: 右臂关节角度，numpy array，shape=(6,) 或 (7,)
        Returns:
            T_world_to_left_camera: 从世界坐标系到左臂相机光学帧的变换矩阵 (4x4)
            T_world_to_right_camera: 从世界坐标系到右臂相机光学帧的变换矩阵 (4x4)
        """
        T_world_to_left_ee, T_world_to_right_ee = self.compute_dual_arm_fk(joint_positions_left, joint_positions_right)
        
        T_left_ee_to_camera = self.get_ee_to_camera_transform('left')
        T_right_ee_to_camera = self.get_ee_to_camera_transform('right')
        
        T_world_to_left_camera = np.dot(T_world_to_left_ee, T_left_ee_to_camera)
        T_world_to_right_camera = np.dot(T_world_to_right_ee, T_right_ee_to_camera)
        
        return T_world_to_left_camera, T_world_to_right_camera


# 保存数据函数
def save_data(args, timesteps, timestamps, actions, dataset_path, cam_infos=None ):
    data_size = len(actions)
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/observations/end_pose_left': [],
        '/observations/end_pose_right': [],
    }

    # 相机图像字典
    for cam_name in args.camera_names:
        data_dict[f'/observation/{cam_name}/rgb'] = []
        data_dict[f'/observation/{cam_name}/extrinsic'] = []
        if args.use_depth_image:
            data_dict[f'/observation/{cam_name}/depth'] = []

    # 提取每一帧数据
    for i in range(len(actions)):
        ts = timesteps[i + 1]  

        data_dict['/observations/end_pose_left'].append(ts.observation['end_pose_left'])
        data_dict['/observations/end_pose_right'].append(ts.observation['end_pose_right'])
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])

        for cam_name in args.camera_names:
            data_dict[f'/observation/{cam_name}/rgb'].append(ts.observation['rgb'][cam_name])

            if args.use_depth_image and 'depth' in ts.observation:
                data_dict[f'/observation/{cam_name}/depth'].append(ts.observation['depth'][cam_name])

        if 'extrinsic' in ts.observation:
            if '/observations/extrinsic' not in data_dict:
                data_dict['/observations/extrinsic'] = {
                    'camera_left': [],
                    'camera_right': []
                }

        data_dict['/observations/extrinsic']['camera_left'].append(
                ts.observation['extrinsic']['camera_left']
            )
        data_dict['/observations/extrinsic']['camera_right'].append(
                ts.observation['extrinsic']['camera_right']
            )

    # 自动识别每个相机的深度图尺寸
    depth_shapes = {}
    if args.use_depth_image:
        for cam_name in args.camera_names:
            if len(data_dict[f'/observation/{cam_name}/depth']) > 0:
                depth_shapes[cam_name] = np.array(data_dict[f'/observation/{cam_name}/depth'][0]).shape
                print(f"[INFO] Detected depth shape for {cam_name}: {depth_shapes[cam_name]}")

    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        root.attrs['compress'] = False

        obs = root.create_group('observations')
        for cam_name in args.camera_names:
            cam_group = obs.create_group(cam_name)

            # === RGB ===
            rgb_shape = np.array(data_dict[f'/observation/{cam_name}/rgb'][0]).shape
            cam_group.create_dataset(
                "rgb", (data_size, *rgb_shape), dtype='uint8', chunks=(1, *rgb_shape)
            )
            cam_group["rgb"][...] = np.array(data_dict[f'/observation/{cam_name}/rgb'])

            # === Depth ===
            if args.use_depth_image and cam_name in depth_shapes:
                depth_shape = depth_shapes[cam_name]
                cam_group.create_dataset(
                    "depth", (data_size, *depth_shape), dtype='uint16', chunks=(1, *depth_shape)
                )
                cam_group["depth"][...] = np.array(data_dict[f'/observation/{cam_name}/depth'])

            # === Intrinsics ===
            if cam_infos and cam_name in cam_infos:
                cam_info = cam_infos[cam_name]
                P = np.array(cam_info.p, dtype=np.float32).reshape(3, 4)

                cam_group.create_dataset("intrinsic_cv", data=P, dtype="float32")

            # === Extrinsics ===
            if cam_name == 'camera_left' and '/observations/extrinsic' in data_dict:
                cam_group.create_dataset(
                    "extrinsic",
                    data=np.array(data_dict['/observations/extrinsic']['camera_left'], dtype=np.float32)
                )
            elif cam_name == 'camera_right' and '/observations/extrinsic' in data_dict:
                cam_group.create_dataset(
                "extrinsic",
                data=np.array(data_dict['/observations/extrinsic']['camera_right'], dtype=np.float32)
                )
            elif cam_name == 'camera_middle':
                # 固定矩阵（你的外参）
                middle_extrinsic = np.array([
                    [-0.015359485464868823, -0.9983525471687079, 0.05528361211070677, -0.28614708353400986],
                    [-0.6831232291348326,  -0.02989678191732818, -0.7296909182985538,  0.3628194342945678],
                    [ 0.7301415890241107,  -0.04897319667675693, -0.6815386166495675,  0.3402189061035694],
                    [0.0, 0.0, 0.0, 1.0]
                ], dtype=np.float32)
                cam_group.create_dataset(
                    "extrinsic",
                    data=np.repeat(middle_extrinsic[None, :, :], data_size, axis=0)  # 每帧相同
                )


        # === 机械臂与状态 ===
        obs.create_dataset('end_pose_left', data=np.array(data_dict['/observations/end_pose_left']))
        obs.create_dataset('end_pose_right', data=np.array(data_dict['/observations/end_pose_right']))
        obs.create_dataset('qpos', data=np.array(data_dict['/observations/qpos']))
        obs.create_dataset('qvel', data=np.array(data_dict['/observations/qvel']))
        obs.create_dataset('effort', data=np.array(data_dict['/observations/effort']))
        root.create_dataset('time_stamps', data=np.array(timestamps, dtype=np.float64))

        # === Meta 信息 ===
        meta_data = {
            "user_name": args.user_name,
            "task_name": args.task_name,
            "instruction": args.instruction,
            "uuid": f"{args.task_name}/{args.user_name}/episode_{args.episode_idx:02d}",
            "user": args.user_name,
            "num_steps": len(actions),
            "simulation": False
        }
        root.create_dataset("meta_data", data=json.dumps(meta_data))

    print(f'\033[32m\nSaving completed in {time.time() - t0:.1f}s → {dataset_path}.hdf5 \033[0m\n')


class RosOperator(Node):
    def __init__(self, args):
        super().__init__('ros_operator')
        # self.robot_base_deque = None
        self.puppet_arm_right_deque = None
        self.puppet_arm_left_deque = None
        self.end_pose_left_deque = None
        self.end_pose_right_deque = None
        self.img_front_deque = None
        self.img_right_deque = None
        self.img_left_deque = None
        self.img_front_depth_deque = None
        self.img_right_depth_deque = None
        self.img_left_depth_deque = None
        self.bridge = None
        self.cam_info_left_deque = None
        self.cam_info_middle_deque = None
        self.cam_info_right_deque = None
        self.args = args
        self.cam_infos = {}

        # 如果启用正运动学计算，初始化运动学对象
        if args.use_forward_kinematics:
            self.kinematics = RobotKinematics(arm_type=args.arm_type)
            args.kinematics = self.kinematics  # 保存到args中以便后续使用
            print(f'\033[32m已启用正运动学计算，机械臂类型: {args.arm_type}\033[0m')
            print(f'\033[32mDH约定: Modified DH (Craig convention)\033[0m')
        else:
            self.kinematics = None

        self.init()
        self.init_ros()

    def init(self):
        self.bridge = CvBridge()
        self.img_left_deque = deque()
        self.img_right_deque = deque()
        self.img_front_deque = deque()
        self.img_left_depth_deque = deque()
        self.img_right_depth_deque = deque()
        self.img_front_depth_deque = deque()
        self.puppet_arm_left_deque = deque()
        self.puppet_arm_right_deque = deque()
        self.cam_info_left_deque = deque()
        self.cam_info_middle_deque = deque()
        self.cam_info_right_deque = deque()
        self.end_pose_left_deque = deque() 
        self.end_pose_right_deque = deque() 

    def get_frame(self):

        if len(self.img_left_deque) == 0 or len(self.img_right_deque) == 0 or len(self.img_front_deque) == 0 or \
                (self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or len(self.img_right_depth_deque) == 0 or len(self.img_front_depth_deque) == 0)):
            return False
        
        # ROS2时间戳转换
        def stamp_to_sec(stamp):
            return stamp.sec + stamp.nanosec * 1e-9

        if self.args.use_depth_image:
            frame_time = min([stamp_to_sec(self.img_left_deque[-1].header.stamp),
                                stamp_to_sec(self.img_right_deque[-1].header.stamp),
                                stamp_to_sec(self.img_front_deque[-1].header.stamp),
                                stamp_to_sec(self.img_left_depth_deque[-1].header.stamp),
                                stamp_to_sec(self.img_right_depth_deque[-1].header.stamp),
                                stamp_to_sec(self.img_front_depth_deque[-1].header.stamp)])
        else:
            frame_time = min([stamp_to_sec(self.img_left_deque[-1].header.stamp),
                                stamp_to_sec(self.img_right_deque[-1].header.stamp),
                                stamp_to_sec(self.img_front_deque[-1].header.stamp)])

        if len(self.img_left_deque) == 0 or stamp_to_sec(self.img_left_deque[-1].header.stamp) < frame_time:
            return False
        if len(self.img_right_deque) == 0 or stamp_to_sec(self.img_right_deque[-1].header.stamp) < frame_time:
            return False
        if len(self.img_front_deque) == 0 or stamp_to_sec(self.img_front_deque[-1].header.stamp) < frame_time:
            return False
        if len(self.puppet_arm_left_deque) == 0 or stamp_to_sec(self.puppet_arm_left_deque[-1].header.stamp) < frame_time:
            return False
        if len(self.puppet_arm_right_deque) == 0 or stamp_to_sec(self.puppet_arm_right_deque[-1].header.stamp) < frame_time:
            return False
        if self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or stamp_to_sec(self.img_left_depth_deque[-1].header.stamp) < frame_time):
            return False
        if self.args.use_depth_image and (len(self.img_right_depth_deque) == 0 or stamp_to_sec(self.img_right_depth_deque[-1].header.stamp) < frame_time):
            return False
        if self.args.use_depth_image and (len(self.img_front_depth_deque) == 0 or stamp_to_sec(self.img_front_depth_deque[-1].header.stamp) < frame_time):
            return False
        if len(self.end_pose_left_deque) == 0 or stamp_to_sec(self.end_pose_left_deque[-1].header.stamp) < frame_time:
            return False
        if len(self.end_pose_right_deque) == 0 or stamp_to_sec(self.end_pose_right_deque[-1].header.stamp) < frame_time:
            return False

        while stamp_to_sec(self.end_pose_left_deque[0].header.stamp) < frame_time:
            self.end_pose_left_deque.popleft()
        end_pose_left_msg = self.end_pose_left_deque.popleft()

        while stamp_to_sec(self.end_pose_right_deque[0].header.stamp) < frame_time:
            self.end_pose_right_deque.popleft()
        end_pose_right_msg = self.end_pose_right_deque.popleft()

        end_pose_left = np.array([
        end_pose_left_msg.pose.position.x,
        end_pose_left_msg.pose.position.y,
        end_pose_left_msg.pose.position.z,
        end_pose_left_msg.pose.orientation.x,
        end_pose_left_msg.pose.orientation.y,
        end_pose_left_msg.pose.orientation.z,
        end_pose_left_msg.pose.orientation.w
        ], dtype=np.float32)

        end_pose_right = np.array([
        end_pose_right_msg.pose.position.x,
        end_pose_right_msg.pose.position.y,
        end_pose_right_msg.pose.position.z,
        end_pose_right_msg.pose.orientation.x,
        end_pose_right_msg.pose.orientation.y,
        end_pose_right_msg.pose.orientation.z,
        end_pose_right_msg.pose.orientation.w
        ], dtype=np.float32)
    
        while stamp_to_sec(self.img_left_deque[0].header.stamp) < frame_time:
            self.img_left_deque.popleft()
        img_left = self.bridge.imgmsg_to_cv2(self.img_left_deque.popleft(), 'passthrough')

        while stamp_to_sec(self.img_right_deque[0].header.stamp) < frame_time:
            self.img_right_deque.popleft()
        img_right = self.bridge.imgmsg_to_cv2(self.img_right_deque.popleft(), 'passthrough')

        while stamp_to_sec(self.img_front_deque[0].header.stamp) < frame_time:
            self.img_front_deque.popleft()
        img_front = self.bridge.imgmsg_to_cv2(self.img_front_deque.popleft(), 'passthrough')

        while stamp_to_sec(self.puppet_arm_left_deque[0].header.stamp) < frame_time:
            self.puppet_arm_left_deque.popleft()
        puppet_arm_left = self.puppet_arm_left_deque.popleft()

        while stamp_to_sec(self.puppet_arm_right_deque[0].header.stamp) < frame_time:
            self.puppet_arm_right_deque.popleft()
        puppet_arm_right = self.puppet_arm_right_deque.popleft()

        img_left_depth = None
        if self.args.use_depth_image:
            while stamp_to_sec(self.img_left_depth_deque[0].header.stamp) < frame_time:
                self.img_left_depth_deque.popleft()
            img_left_depth = self.bridge.imgmsg_to_cv2(self.img_left_depth_deque.popleft(), 'passthrough')
            top, bottom, left, right = 40, 40, 0, 0
            img_left_depth = cv2.copyMakeBorder(img_left_depth, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

        img_right_depth = None
        if self.args.use_depth_image:
            while stamp_to_sec(self.img_right_depth_deque[0].header.stamp) < frame_time:
                self.img_right_depth_deque.popleft()
            img_right_depth = self.bridge.imgmsg_to_cv2(self.img_right_depth_deque.popleft(), 'passthrough')
            top, bottom, left, right = 40, 40, 0, 0
            img_right_depth = cv2.copyMakeBorder(img_right_depth, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

        img_front_depth = None
        if self.args.use_depth_image:
            while stamp_to_sec(self.img_front_depth_deque[0].header.stamp) < frame_time:
                self.img_front_depth_deque.popleft()
            img_front_depth = self.bridge.imgmsg_to_cv2(self.img_front_depth_deque.popleft(), 'passthrough')
            top, bottom, left, right = 40, 40, 0, 0
            img_front_depth = cv2.copyMakeBorder(img_front_depth, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

    
        return (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
                puppet_arm_left, puppet_arm_right,end_pose_left, end_pose_right)

    def img_left_callback(self, msg):
        if len(self.img_left_deque) >= 2000:
            self.img_left_deque.popleft()
        self.img_left_deque.append(msg)

    def img_right_callback(self, msg):
        if len(self.img_right_deque) >= 2000:
            self.img_right_deque.popleft()
        self.img_right_deque.append(msg)

    def img_front_callback(self, msg):
        if len(self.img_front_deque) >= 2000:
            self.img_front_deque.popleft()
        self.img_front_deque.append(msg)

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= 2000:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= 2000:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= 2000:
            self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)

    def puppet_arm_left_callback(self, msg):
        if len(self.puppet_arm_left_deque) >= 2000:
            self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)

    def puppet_arm_right_callback(self, msg):
        if len(self.puppet_arm_right_deque) >= 2000:
            self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)

    def end_pose_left_callback(self, msg):
        if len(self.end_pose_left_deque) >= 2000:
            self.end_pose_left_deque.popleft()
        self.end_pose_left_deque.append(msg)

    def end_pose_right_callback(self, msg):
        if len(self.end_pose_right_deque) >= 2000:
            self.end_pose_right_deque.popleft()
        self.end_pose_right_deque.append(msg)

    def cam_info_left_callback(self, msg):
        self.cam_info_left_deque.append(msg)
        self.cam_infos["camera_left"] = msg   

    def cam_info_middle_callback(self, msg):
        self.cam_info_middle_deque.append(msg)
        self.cam_infos["camera_middle"] = msg   

    def cam_info_right_callback(self, msg):
        self.cam_info_right_deque.append(msg)
        self.cam_infos["camera_right"] = msg  
    

    def init_ros(self):
        self.create_subscription(Image, self.args.img_left_topic, self.img_left_callback, 1000)
        self.create_subscription(Image, self.args.img_right_topic, self.img_right_callback, 1000)
        self.create_subscription(Image, self.args.img_front_topic, self.img_front_callback, 1000)

        if self.args.use_depth_image:
            self.create_subscription(Image, self.args.img_left_depth_topic, self.img_left_depth_callback, 1000)
            self.create_subscription(Image, self.args.img_right_depth_topic, self.img_right_depth_callback, 1000)
            self.create_subscription(Image, self.args.img_front_depth_topic, self.img_front_depth_callback, 1000)

        self.create_subscription(JointState, self.args.puppet_arm_left_topic, self.puppet_arm_left_callback, 1000)
        self.create_subscription(JointState, self.args.puppet_arm_right_topic, self.puppet_arm_right_callback, 1000)
        self.create_subscription(CameraInfo, self.args.cam_info_left_topic, self.cam_info_left_callback, 1000)
        self.create_subscription(CameraInfo, self.args.cam_info_right_topic, self.cam_info_right_callback, 1000)
        self.create_subscription(CameraInfo, self.args.cam_info_front_topic, self.cam_info_middle_callback, 1000)
        self.create_subscription(PoseStamped, self.args.end_pose_left_topic, self.end_pose_left_callback, 1000)
        self.create_subscription(PoseStamped, self.args.end_pose_right_topic, self.end_pose_right_callback, 1000)


    def process(self):
        timesteps = []
        actions = []
        timestamps = []

        # 如果启用正运动学，创建末端变换矩阵存储列表
        ee_transforms_left = []
        ee_transforms_right = []
        camera_transforms_left = []
        camera_transforms_right = []

        # 图像数据
        image = np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8)
        image_dict = dict()
        for cam_name in self.args.camera_names:
            image_dict[cam_name] = image
        count = 0

        print_flag = True

        while (count < self.args.max_timesteps + 1) and rclpy.ok():
            # Spin once to process callbacks
            rclpy.spin_once(self, timeout_sec=0.01)

            # 2 收集数据
            result = self.get_frame()
            if not result:
                if print_flag:
                    print("syn fail")
                    print_flag = False
                time.sleep(1.0 / self.args.frame_rate)
                continue
            print_flag = True
            count += 1
            (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
             puppet_arm_left, puppet_arm_right,end_pose_left,end_pose_right) = result
            
            stamp = puppet_arm_left.header.stamp
            stamp_sec = stamp.sec + stamp.nanosec * 1e-9
            timestamps.append(stamp_sec)

            # 2.1 图像信息
            image_dict = dict()
            image_dict[self.args.camera_names[0]] = img_front
            image_dict[self.args.camera_names[1]] = img_left
            image_dict[self.args.camera_names[2]] = img_right

            # 2.2 从臂的信息从臂的状态 机械臂示教模式时 会自动订阅
            obs = collections.OrderedDict()  # 有序的字典
            obs['rgb'] = image_dict
            if self.args.use_depth_image:
                image_dict_depth = dict()
                image_dict_depth[self.args.camera_names[0]] = img_front_depth
                image_dict_depth[self.args.camera_names[1]] = img_left_depth
                image_dict_depth[self.args.camera_names[2]] = img_right_depth
                obs['depth'] = image_dict_depth
            obs['qpos'] = np.concatenate((np.array(puppet_arm_left.position), np.array(puppet_arm_right.position)), axis=0)
            obs['qvel'] = np.concatenate((np.array(puppet_arm_left.velocity), np.array(puppet_arm_right.velocity)), axis=0)
            obs['effort'] = np.concatenate((np.array(puppet_arm_left.effort), np.array(puppet_arm_right.effort)), axis=0)
            obs['end_pose_left'] = end_pose_left
            obs['end_pose_right'] = end_pose_right

            if self.args.use_forward_kinematics:
                puppet_left_pos = np.array(puppet_arm_left.position)
                puppet_right_pos = np.array(puppet_arm_right.position)
                
                # 计算双臂的末端正运动学（从世界坐标系到末端，只使用前6个旋转关节）
                # 世界坐标系 = 左臂基座坐标系
                T_world_to_left_ee, T_world_to_right_ee = self.kinematics.compute_dual_arm_fk(
                    puppet_left_pos, puppet_right_pos)
                
                # 保存末端变换矩阵
                ee_transforms_left.append(T_world_to_left_ee)
                ee_transforms_right.append(T_world_to_right_ee)
                
                # 计算双臂的相机正运动学（从世界坐标系到相机光学帧）
                T_world_to_left_camera, T_world_to_right_camera = self.kinematics.compute_dual_camera_fk(
                    puppet_left_pos, puppet_right_pos)
                
                camera_transforms_left.append(T_world_to_left_camera)
                camera_transforms_right.append(T_world_to_right_camera)

                obs['extrinsic'] = {
                            'camera_left': T_world_to_left_camera,
                            'camera_right': T_world_to_right_camera
                }


            # 第一帧 只包含first， fisrt只保存StepType.FIRST
            if count == 1:
                ts = dm_env.TimeStep(
                    step_type=dm_env.StepType.FIRST,
                    reward=None,
                    discount=None,
                    observation=obs)
                timesteps.append(ts)
                continue

            # 时间步
            ts = dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=None,
                discount=None,
                observation=obs)

            # 主臂保存状态
            action = np.concatenate((np.array(puppet_arm_left.position), np.array(puppet_arm_right.position)), axis=0)
            actions.append(action)
            timesteps.append(ts)

            print("Frame data: ", count)

            if not rclpy.ok():
                  exit(-1)
            time.sleep(1.0 / self.args.frame_rate)

        print("len(timesteps): ", len(timesteps))
        print("len(actions)  : ", len(actions))
        return timesteps, actions,timestamps
    

TASK_INSTRUCTIONS = {
    "place_shoe": "Use one arm to grab the shoe from the table and place it on the mat.",
    "pick_cup": "Grasp the cup from the table and lift it up to the target height.",
    "open_door": "Reach out and grasp the handle, then pull the door open smoothly.",
    "close_drawer": "Grasp the drawer handle and push it until it is fully closed.",
    "wipe_table": "Use the sponge to wipe the marked area of the table in circular motion.",
    "plug_cable": "Pick up the connector and insert it into the socket securely.",
    "fold_towel": "Pick up the towel and fold it along the center line neatly.",
    "pour_water": "Grab the bottle and pour water into the cup until it reaches the marked level.",
}

def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', action='store', type=str, help='Dataset_dir.',
                        default="./data", required=False)
    parser.add_argument('--task_name', action='store', type=str, help='Task name.',
                        default="place_shoe", required=False)
    parser.add_argument('--episode_idx', action='store', type=int, help='Episode index.',
                        default=0, required=False)
    
    parser.add_argument('--max_timesteps', action='store', type=int, help='Max_timesteps.',
                        default=500, required=False)

    parser.add_argument('--camera_names', action='store', type=str, help='camera_names',
                        default=['camera_middle', 'camera_left', 'camera_right'], required=False)
    #  topic name of color image
    parser.add_argument('--img_front_topic', action='store', type=str, help='img_front_topic',
                        default='/camera_middle/realsense2_camera_node/color/image_raw', required=False)
    parser.add_argument('--img_left_topic', action='store', type=str, help='img_left_topic',
                        default='/camera_left/realsense2_camera_node/color/image_raw', required=False)
    parser.add_argument('--img_right_topic', action='store', type=str, help='img_right_topic',
                        default='/camera_right/realsense2_camera_node/color/image_raw', required=False)

    # topic name of depth image
    parser.add_argument('--img_front_depth_topic', action='store', type=str, help='img_front_depth_topic',
                        default='/camera_middle/realsense2_camera_node/depth/image_rect_raw', required=False)
    parser.add_argument('--img_left_depth_topic', action='store', type=str, help='img_left_depth_topic',
                        default='/camera_left/realsense2_camera_node/depth/image_rect_raw', required=False)
    parser.add_argument('--img_right_depth_topic', action='store', type=str, help='img_right_depth_topic',
                        default='/camera_right/realsense2_camera_node/depth/image_rect_raw', required=False)

    parser.add_argument('--cam_info_front_topic', type=str,
                    default='/camera_middle/realsense2_camera_node/color/camera_info', required=False)
    parser.add_argument('--cam_info_left_topic', type=str,
                    default='/camera_left/realsense2_camera_node/color/camera_info', required=False)
    parser.add_argument('--cam_info_right_topic', type=str,
                    default='/camera_right/realsense2_camera_node/color/camera_info', required=False)


    
    # topic name of arm
    parser.add_argument('--puppet_arm_left_topic', action='store', type=str, help='puppet_arm_left_topic',
                        default='/puppet/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_topic', action='store', type=str, help='puppet_arm_right_topic',
                        default='/puppet/joint_right', required=False)
    
    # collect depth image
    parser.add_argument('--use_depth_image', action='store', type=bool, help='use_depth_image',
                        default=True, required=False)
    
    parser.add_argument('--frame_rate', action='store', type=int, help='frame_rate',
                        default=30, required=False)
    
    parser.add_argument('--end_pose_left_topic', action='store', type=str,
                        default='/puppet/end_pose_left', required=False)
    parser.add_argument('--end_pose_right_topic', action='store', type=str,
                        default='/puppet/end_pose_right', required=False)
    
    parser.add_argument('--user_name', type=str, default="yuhang.zhou", help='User name')

    parser.add_argument('--use_forward_kinematics', action='store', type=bool, 
                        help='是否计算并保存从世界坐标系到末端执行器和相机光学帧的变换矩阵（正运动学）',
                        default=True, required=False)
    parser.add_argument('--arm_type', action='store', type=str, 
                        help='机械臂类型，用于选择对应的DH参数（piper）',
                        default='piper', required=False)

    args = parser.parse_args()

    args.instruction = TASK_INSTRUCTIONS.get(
        args.task_name,
        f"Perform the {args.task_name} task as demonstrated."
    )
    return args

import json

def main(args=None):
    rclpy.init(args=args)

    cmd_args = get_arguments()
    ros_operator = RosOperator(cmd_args)

    try:
        timesteps, actions, timestamps = ros_operator.process()

        dataset_dir = os.path.join(cmd_args.dataset_dir, cmd_args.task_name)
        if len(actions) < cmd_args.max_timesteps:
            print(f"\033[31m\nSave failure, please record {cmd_args.max_timesteps} timesteps of data.\033[0m\n")
            exit(-1)

        os.makedirs(dataset_dir, exist_ok=True)
        dataset_path = os.path.join(dataset_dir, f"episode_{cmd_args.episode_idx}")

        save_data(cmd_args, timesteps, timestamps, actions, dataset_path, ros_operator.cam_infos)

    except KeyboardInterrupt:
        pass
    finally:
        ros_operator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# python collect_data.py --dataset_dir ~/data --max_timesteps 500 --episode_idx 0