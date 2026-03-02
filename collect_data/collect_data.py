# -- coding: UTF-8
import os
import time
import numpy as np
import h5py
import argparse
import dm_env
import threading

import collections
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState, CameraInfo
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from pynput import keyboard


class RobotKinematics:
    """
    机器人运动学类，用于计算正运动学（FK）
    使用Modified DH参数（Craig convention）
    """
    def __init__(self, arm_type='piper'):
        self.arm_type = arm_type
        if arm_type == 'piper':
            self.dh_params = [
                [0.0,        0.0,         0.123,    0.0],
                [np.pi/2,    0.0,         0.0,      -np.pi + 0.1359],
                [0.0,        0.28503,     0.0,      -1.7939],
                [np.pi/2,    -0.021984,   0.25075,  0.0],
                [-np.pi/2,   0.0,         0.0,      0.0],
                [np.pi/2,    0.000088259, 0.091,    0.0],
                [0.0,        0.0,         0.1358,   0.0]
            ]
            self.left_base_offset = np.array([0.0, 0.0, 0.0])
            self.right_base_offset = np.array([0.0, -0.6, 0.0])
            print(f'\033[32m已加载Piper机械臂的DH参数（7个关节+末端）\033[0m')
            print(f'\033[32m世界坐标系: 左臂基座坐标系 (base_link)\033[0m')
            print(f'\033[32m左臂基座偏移: {self.left_base_offset}\033[0m')
            print(f'\033[32m右臂基座偏移: {self.right_base_offset}\033[0m')
        else:
            raise ValueError(f"不支持的机械臂类型: {arm_type}")

    def modified_dh_matrix(self, alpha, a, d, theta):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        T = np.array([
            [ct,    -st,     0,      a],
            [st*ca,  ct*ca, -sa,  -sa*d],
            [st*sa,  ct*sa,  ca,   ca*d],
            [0,      0,      0,      1]
        ])
        return T

    def forward_kinematics(self, joint_positions):
        T = np.eye(4)
        num_joints = min(len(joint_positions), 6)
        for i in range(num_joints):
            alpha, a, d, theta_offset = self.dh_params[i]
            theta = joint_positions[i] + theta_offset
            T_i = self.modified_dh_matrix(alpha, a, d, theta)
            T = np.dot(T, T_i)
        if len(self.dh_params) > num_joints:
            alpha, a, d, theta_offset = self.dh_params[num_joints]
            T_end = self.modified_dh_matrix(alpha, a, d, theta_offset)
            T = np.dot(T, T_end)
        return T

    def compute_dual_arm_fk(self, joint_positions_left, joint_positions_right):
        T_left_base_to_ee = self.forward_kinematics(joint_positions_left[:6])
        T_world_to_left_base = np.eye(4)
        T_world_to_left_base[:3, 3] = self.left_base_offset
        T_world_to_left_ee = np.dot(T_world_to_left_base, T_left_base_to_ee)

        T_right_base_to_ee = self.forward_kinematics(joint_positions_right[:6])
        T_world_to_right_base = np.eye(4)
        T_world_to_right_base[:3, 3] = self.right_base_offset
        T_world_to_right_ee = np.dot(T_world_to_right_base, T_right_base_to_ee)
        return T_world_to_left_ee, T_world_to_right_ee

    def extract_position_orientation(self, T):
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
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
        x, y, z, w = quat
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])

    def get_ee_to_camera_transform(self, side):
        if side == 'left':
            xyz = np.array([-0.07354390146283836, 0.007804615886680326, 0.038433103882865485])
            quat = np.array([0.12827667735969864, -0.14997843923484816, 0.7053150782153692, -0.6808687114652002])
        elif side == 'right':
            xyz = np.array([-0.07249626512541643, 0.006136740643933003, 0.038600128562769884])
            quat = np.array([0.1256876224765708, -0.1305963178064823, 0.6996428946145881, -0.6911201366961431])
        else:
            raise ValueError(f"不支持的侧别: {side}")
        R = self.quaternion_to_rotation_matrix(quat)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T

    def compute_dual_camera_fk(self, joint_positions_left, joint_positions_right):
        T_world_to_left_ee, T_world_to_right_ee = self.compute_dual_arm_fk(joint_positions_left, joint_positions_right)
        T_left_ee_to_camera = self.get_ee_to_camera_transform('left')
        T_right_ee_to_camera = self.get_ee_to_camera_transform('right')
        T_world_to_left_camera = np.dot(T_world_to_left_ee, T_left_ee_to_camera)
        T_world_to_right_camera = np.dot(T_world_to_right_ee, T_right_ee_to_camera)
        return T_world_to_left_camera, T_world_to_right_camera


# 保存数据函数（与原版相同，不做改动）
def save_data(args, timesteps, timestamps, actions, dataset_path, cam_infos=None):
    data_size = len(actions)
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/observations/end_pose_left': [],
        '/observations/end_pose_right': [],
    }

    for cam_name in args.camera_names:
        data_dict[f'/observation/{cam_name}/rgb'] = []
        data_dict[f'/observation/{cam_name}/extrinsic'] = []
        if args.use_depth_image:
            data_dict[f'/observation/{cam_name}/depth'] = []

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

            rgb_shape = np.array(data_dict[f'/observation/{cam_name}/rgb'][0]).shape
            cam_group.create_dataset(
                "rgb", (data_size, *rgb_shape), dtype='uint8', chunks=(1, *rgb_shape)
            )
            cam_group["rgb"][...] = np.array(data_dict[f'/observation/{cam_name}/rgb'])

            if args.use_depth_image and cam_name in depth_shapes:
                depth_shape = depth_shapes[cam_name]
                cam_group.create_dataset(
                    "depth", (data_size, *depth_shape), dtype='uint16', chunks=(1, *depth_shape)
                )
                cam_group["depth"][...] = np.array(data_dict[f'/observation/{cam_name}/depth'])

            if cam_infos and cam_name in cam_infos:
                cam_info = cam_infos[cam_name]
                P = np.array(cam_info.p, dtype=np.float32).reshape(3, 4)
                cam_group.create_dataset("intrinsic_cv", data=P, dtype="float32")

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
                middle_extrinsic = np.array([
                    [-0.015359485464868823, -0.9983525471687079, 0.05528361211070677, -0.28614708353400986],
                    [-0.6831232291348326,  -0.02989678191732818, -0.7296909182985538,  0.3628194342945678],
                    [ 0.7301415890241107,  -0.04897319667675693, -0.6815386166495675,  0.3402189061035694],
                    [0.0, 0.0, 0.0, 1.0]
                ], dtype=np.float32)
                cam_group.create_dataset(
                    "extrinsic",
                    data=np.repeat(middle_extrinsic[None, :, :], data_size, axis=0)
                )

        obs.create_dataset('end_pose_left', data=np.array(data_dict['/observations/end_pose_left']))
        obs.create_dataset('end_pose_right', data=np.array(data_dict['/observations/end_pose_right']))
        obs.create_dataset('qpos', data=np.array(data_dict['/observations/qpos']))
        obs.create_dataset('qvel', data=np.array(data_dict['/observations/qvel']))
        obs.create_dataset('effort', data=np.array(data_dict['/observations/effort']))
        root.create_dataset('time_stamps', data=np.array(timestamps, dtype=np.float64))

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

        # 键盘控制变量
        self.is_recording = False
        self.should_stop = False
        self.start_key_pressed = False

        if args.use_forward_kinematics:
            self.kinematics = RobotKinematics(arm_type=args.arm_type)
            args.kinematics = self.kinematics
            print(f'\033[32m已启用正运动学计算，机械臂类型: {args.arm_type}\033[0m')
            print(f'\033[32mDH约定: Modified DH (Craig convention)\033[0m')
        else:
            self.kinematics = None

        self.init()
        self.init_ros()

        # ----------------------------------------------------------------
        # 修复1：后台 spin 线程 —— 让 ROS2 回调持续处理，不依赖主循环里的
        #        spin_once，彻底避免因 spin 不及时导致 deque 里数据缺失
        # ----------------------------------------------------------------
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        print('\033[32m[SyncFix] 后台 spin 线程已启动\033[0m')

        # 启动键盘监听线程
        self._keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self._keyboard_listener.start()
        print('\033[33m[KeyControl] 键盘监听已启动\033[0m')
        print('\033[33m[KeyControl] 按 "s" 开始采集 (等待2秒后开始)\033[0m')
        print('\033[33m[KeyControl] 按 "q" 结束采集\033[0m')

    def _stop_spin(self):
        """停止后台 spin 线程"""
        self._executor.shutdown(timeout_sec=1.0)
        if self._keyboard_listener:
            self._keyboard_listener.stop()

    def on_key_press(self, key):
        """键盘按键回调"""
        try:
            if hasattr(key, 'char'):
                if key.char == 's' and not self.is_recording:
                    self.start_key_pressed = True
                    print('\033[33m[KeyControl] 检测到 "s" 键，2秒后开始采集...\033[0m')
                elif key.char == 'q':
                    if self.is_recording:
                        self.should_stop = True
                        print('\033[33m[KeyControl] 检测到 "q" 键，即将结束采集...\033[0m')
                    else:
                        print('\033[31m[KeyControl] 当前未在采集中\033[0m')
        except AttributeError:
            pass

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
        # ----------------------------------------------------------------
        # 修复2：为机械臂/末端位姿话题引入时间容差
        #
        # end_pose 是 piper 节点做完 FK 后再发布的，天然比图像话题晚几十 ms。
        # 原代码要求所有话题的最新时间戳都 >= frame_time（相机最慢帧时间），
        # 因此 end_pose 几乎每帧都会触发 sync fail。
        #
        # 修复方案：允许机械臂类话题在 frame_time 之前最多 ARM_TOLERANCE 秒
        # ----------------------------------------------------------------
        ARM_TOLERANCE = 0.08  # 80 ms，可根据实际延迟调小

        if len(self.img_left_deque) == 0 or len(self.img_right_deque) == 0 or len(self.img_front_deque) == 0 or \
                (self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or
                                                len(self.img_right_depth_deque) == 0 or
                                                len(self.img_front_depth_deque) == 0)):
            return False

        def stamp_to_sec(stamp):
            return stamp.sec + stamp.nanosec * 1e-9

        if self.args.use_depth_image:
            frame_time = min([
                stamp_to_sec(self.img_left_deque[-1].header.stamp),
                stamp_to_sec(self.img_right_deque[-1].header.stamp),
                stamp_to_sec(self.img_front_deque[-1].header.stamp),
                stamp_to_sec(self.img_left_depth_deque[-1].header.stamp),
                stamp_to_sec(self.img_right_depth_deque[-1].header.stamp),
                stamp_to_sec(self.img_front_depth_deque[-1].header.stamp),
            ])
        else:
            frame_time = min([
                stamp_to_sec(self.img_left_deque[-1].header.stamp),
                stamp_to_sec(self.img_right_deque[-1].header.stamp),
                stamp_to_sec(self.img_front_deque[-1].header.stamp),
            ])

        # 相机话题：严格要求 >= frame_time（frame_time 本身就是相机最小值，这里必然满足）
        # 机械臂/末端位姿话题：允许最多落后 ARM_TOLERANCE
        if len(self.puppet_arm_left_deque) == 0 or \
                stamp_to_sec(self.puppet_arm_left_deque[-1].header.stamp) < frame_time - ARM_TOLERANCE:
            return False
        if len(self.puppet_arm_right_deque) == 0 or \
                stamp_to_sec(self.puppet_arm_right_deque[-1].header.stamp) < frame_time - ARM_TOLERANCE:
            return False
        if self.args.use_depth_image and (
                len(self.img_left_depth_deque) == 0 or
                stamp_to_sec(self.img_left_depth_deque[-1].header.stamp) < frame_time):
            return False
        if self.args.use_depth_image and (
                len(self.img_right_depth_deque) == 0 or
                stamp_to_sec(self.img_right_depth_deque[-1].header.stamp) < frame_time):
            return False
        if self.args.use_depth_image and (
                len(self.img_front_depth_deque) == 0 or
                stamp_to_sec(self.img_front_depth_deque[-1].header.stamp) < frame_time):
            return False
        if len(self.end_pose_left_deque) == 0 or \
                stamp_to_sec(self.end_pose_left_deque[-1].header.stamp) < frame_time - ARM_TOLERANCE:
            return False
        if len(self.end_pose_right_deque) == 0 or \
                stamp_to_sec(self.end_pose_right_deque[-1].header.stamp) < frame_time - ARM_TOLERANCE:
            return False

        # ----------------------------------------------------------------
        # 修复3：popleft 时保留至少 1 条消息，防止容差范围内 deque 被清空后越界
        # ----------------------------------------------------------------
        while len(self.end_pose_left_deque) > 1 and \
                stamp_to_sec(self.end_pose_left_deque[0].header.stamp) < frame_time:
            self.end_pose_left_deque.popleft()
        end_pose_left_msg = self.end_pose_left_deque.popleft()

        while len(self.end_pose_right_deque) > 1 and \
                stamp_to_sec(self.end_pose_right_deque[0].header.stamp) < frame_time:
            self.end_pose_right_deque.popleft()
        end_pose_right_msg = self.end_pose_right_deque.popleft()

        end_pose_left = np.array([
            end_pose_left_msg.pose.position.x,
            end_pose_left_msg.pose.position.y,
            end_pose_left_msg.pose.position.z,
            end_pose_left_msg.pose.orientation.x,
            end_pose_left_msg.pose.orientation.y,
            end_pose_left_msg.pose.orientation.z,
            end_pose_left_msg.pose.orientation.w,
        ], dtype=np.float32)

        end_pose_right = np.array([
            end_pose_right_msg.pose.position.x,
            end_pose_right_msg.pose.position.y,
            end_pose_right_msg.pose.position.z,
            end_pose_right_msg.pose.orientation.x,
            end_pose_right_msg.pose.orientation.y,
            end_pose_right_msg.pose.orientation.z,
            end_pose_right_msg.pose.orientation.w,
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

        while len(self.puppet_arm_left_deque) > 1 and \
                stamp_to_sec(self.puppet_arm_left_deque[0].header.stamp) < frame_time:
            self.puppet_arm_left_deque.popleft()
        puppet_arm_left = self.puppet_arm_left_deque.popleft()

        while len(self.puppet_arm_right_deque) > 1 and \
                stamp_to_sec(self.puppet_arm_right_deque[0].header.stamp) < frame_time:
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
                puppet_arm_left, puppet_arm_right, end_pose_left, end_pose_right)

    # ---------- callbacks ----------
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

        ee_transforms_left = []
        ee_transforms_right = []
        camera_transforms_left = []
        camera_transforms_right = []

        image = np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8)
        image_dict = dict()
        for cam_name in self.args.camera_names:
            image_dict[cam_name] = image
        count = 0
        syn_fail_count = 0

        print_flag = True
        frame_interval = 1.0 / self.args.frame_rate

        # 等待开始信号
        print('\033[36m[KeyControl] 等待按键 "s" 开始采集...\033[0m')
        while not self.start_key_pressed and rclpy.ok():
            time.sleep(0.1)

        if not rclpy.ok():
            return timesteps, actions, timestamps

        # 倒计时2秒
        print('\033[36m[KeyControl] 倒计时: 2...\033[0m')
        time.sleep(1)
        print('\033[36m[KeyControl] 倒计时: 1...\033[0m')
        time.sleep(1)

        self.is_recording = True
        print('\033[32m[KeyControl] 开始采集数据!\033[0m')

        while (count < self.args.max_timesteps + 1) and rclpy.ok() and not self.should_stop:
            # ----------------------------------------------------------------
            # 修复1 效果：不再调用 spin_once，后台线程已持续处理所有回调
            # ----------------------------------------------------------------

            result = self.get_frame()
            if not result:
                syn_fail_count += 1
                if print_flag:
                    print(f"syn fail (total: {syn_fail_count})")
                    print_flag = False
                # ----------------------------------------------------------------
                # 修复4：sync fail 时只短暂等待，不浪费整帧时间
                # ----------------------------------------------------------------
                time.sleep(0.002)
                continue

            if not print_flag:
                print_flag = True

            count += 1
            (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
             puppet_arm_left, puppet_arm_right, end_pose_left, end_pose_right) = result

            stamp = puppet_arm_left.header.stamp
            stamp_sec = stamp.sec + stamp.nanosec * 1e-9
            timestamps.append(stamp_sec)

            image_dict = dict()
            image_dict[self.args.camera_names[0]] = img_front
            image_dict[self.args.camera_names[1]] = img_left
            image_dict[self.args.camera_names[2]] = img_right

            obs = collections.OrderedDict()
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

                T_world_to_left_ee, T_world_to_right_ee = self.kinematics.compute_dual_arm_fk(
                    puppet_left_pos, puppet_right_pos)
                ee_transforms_left.append(T_world_to_left_ee)
                ee_transforms_right.append(T_world_to_right_ee)

                T_world_to_left_camera, T_world_to_right_camera = self.kinematics.compute_dual_camera_fk(
                    puppet_left_pos, puppet_right_pos)
                camera_transforms_left.append(T_world_to_left_camera)
                camera_transforms_right.append(T_world_to_right_camera)

                obs['extrinsic'] = {
                    'camera_left': T_world_to_left_camera,
                    'camera_right': T_world_to_right_camera,
                }

            if count == 1:
                ts = dm_env.TimeStep(
                    step_type=dm_env.StepType.FIRST,
                    reward=None,
                    discount=None,
                    observation=obs)
                timesteps.append(ts)
                continue

            ts = dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=None,
                discount=None,
                observation=obs)

            action = np.concatenate((np.array(puppet_arm_left.position), np.array(puppet_arm_right.position)), axis=0)
            actions.append(action)
            timesteps.append(ts)

            print(f"Frame data: {count}  (syn_fail_total: {syn_fail_count})")

            if not rclpy.ok() or self.should_stop:
                break
            time.sleep(frame_interval)

        self.is_recording = False
        if self.should_stop:
            print('\033[33m[KeyControl] 用户手动停止采集\033[0m')
        else:
            print('\033[32m[KeyControl] 达到最大帧数，采集完成\033[0m')

        print("len(timesteps): ", len(timesteps))
        print("len(actions)  : ", len(actions))
        print(f"syn_fail_total : {syn_fail_count}")
        return timesteps, actions, timestamps


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
    parser.add_argument('--dataset_dir', action='store', type=str, default="./data", required=False)
    parser.add_argument('--task_name', action='store', type=str, default="place_empty_cup", required=False)
    # episode_idx 设为可选，程序会自动查找下一个可用编号
    parser.add_argument('--episode_idx', action='store', type=int, default=None, required=False)
    parser.add_argument('--max_timesteps', action='store', type=int, default=1000, required=False)
    parser.add_argument('--camera_names', action='store', type=str,
                        default=['camera_middle', 'camera_left', 'camera_right'], required=False)
    parser.add_argument('--img_front_topic', action='store', type=str, help='img_front_topic',
                        default='/camera_middle/color/image_raw', required=False)
    parser.add_argument('--img_left_topic', action='store', type=str, help='img_left_topic',
                        default='/camera_left/color/image_raw', required=False)
    parser.add_argument('--img_right_topic', action='store', type=str, help='img_right_topic',
                        default='/camera_right/color/image_raw', required=False)

    # topic name of depth image
    parser.add_argument('--img_front_depth_topic', action='store', type=str, help='img_front_depth_topic',
                        default='/camera_middle/depth/image_rect_raw', required=False)
    parser.add_argument('--img_left_depth_topic', action='store', type=str, help='img_left_depth_topic',
                        default='/camera_left/depth/image_rect_raw', required=False)
    parser.add_argument('--img_right_depth_topic', action='store', type=str, help='img_right_depth_topic',
                        default='/camera_right/depth/image_rect_raw', required=False)

    parser.add_argument('--cam_info_front_topic', type=str,
                    default='/camera_middle/color/camera_info', required=False)
    parser.add_argument('--cam_info_left_topic', type=str,
                    default='/camera_left/color/camera_info', required=False)
    parser.add_argument('--cam_info_right_topic', type=str,
                    default='/camera_right/color/camera_info', required=False)

    parser.add_argument('--puppet_arm_left_topic', action='store', type=str,
                        default='/puppet/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_topic', action='store', type=str,
                        default='/puppet/joint_right', required=False)
    parser.add_argument('--use_depth_image', action='store', type=bool, default=True, required=False)
    parser.add_argument('--frame_rate', action='store', type=int, default=30, required=False)
    parser.add_argument('--end_pose_left_topic', action='store', type=str,
                        default='/puppet/end_pose_left', required=False)
    parser.add_argument('--end_pose_right_topic', action='store', type=str,
                        default='/puppet/end_pose_right', required=False)
    parser.add_argument('--user_name', type=str, default="yuhang.zhou")
    parser.add_argument('--use_forward_kinematics', action='store', type=bool, default=True, required=False)
    parser.add_argument('--arm_type', action='store', type=str, default='piper', required=False)

    args = parser.parse_args()
    args.instruction = TASK_INSTRUCTIONS.get(
        args.task_name,
        f"Perform the {args.task_name} task as demonstrated."
    )
    return args


import json


def get_next_episode_idx(dataset_dir, task_name):
    """自动查找下一个可用的episode编号"""
    task_dir = os.path.join(dataset_dir, task_name)
    if not os.path.exists(task_dir):
        return 0

    existing_episodes = []
    for filename in os.listdir(task_dir):
        if filename.startswith('episode_') and filename.endswith('.hdf5'):
            try:
                # 提取编号，例如 episode_0.hdf5 -> 0
                idx = int(filename.replace('episode_', '').replace('.hdf5', ''))
                existing_episodes.append(idx)
            except ValueError:
                continue

    if not existing_episodes:
        return 0

    return max(existing_episodes) + 1


def main(args=None):
    rclpy.init(args=args)

    cmd_args = get_arguments()

    # 自动确定episode索引
    if cmd_args.episode_idx is None:
        cmd_args.episode_idx = get_next_episode_idx(cmd_args.dataset_dir, cmd_args.task_name)
        print(f'\033[36m[AutoEpisode] 自动设置 episode_idx = {cmd_args.episode_idx}\033[0m')
    else:
        print(f'\033[36m[Episode] 使用指定的 episode_idx = {cmd_args.episode_idx}\033[0m')

    ros_operator = RosOperator(cmd_args)

    try:
        timesteps, actions, timestamps = ros_operator.process()

        dataset_dir = os.path.join(cmd_args.dataset_dir, cmd_args.task_name)

        # 如果用户手动停止或数据不足，询问是否保存
        if len(actions) == 0:
            print(f"\033[31m\n没有采集到任何数据，不保存。\033[0m\n")
        elif len(actions) < cmd_args.max_timesteps:
            print(f"\033[33m\n采集到 {len(actions)} 帧数据 (目标: {cmd_args.max_timesteps})。\033[0m")
            print(f"\033[33m数据将保存为 episode_{cmd_args.episode_idx}。\033[0m")
            os.makedirs(dataset_dir, exist_ok=True)
            dataset_path = os.path.join(dataset_dir, f"episode_{cmd_args.episode_idx}")
            save_data(cmd_args, timesteps, timestamps, actions, dataset_path, ros_operator.cam_infos)
        else:
            os.makedirs(dataset_dir, exist_ok=True)
            dataset_path = os.path.join(dataset_dir, f"episode_{cmd_args.episode_idx}")
            save_data(cmd_args, timesteps, timestamps, actions, dataset_path, ros_operator.cam_infos)

    except KeyboardInterrupt:
        pass
    finally:
        ros_operator._stop_spin()
        ros_operator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# python collect_data.py --dataset_dir ~/data --max_timesteps 500 --episode_idx 0
