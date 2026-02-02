# 三相机+双主机械臂数据采集完整流程

## 📋 系统概述

本系统用于采集机器人示教学习数据，包含：
- **3个Intel RealSense相机**（左、中、右）
- **2个Piper主从机械臂**（左臂、右臂）
- **数据格式**：HDF5（包含RGB/深度图像、关节状态、末端位姿）

---

## 环境配置

### 安装 Intel RealSense SDK

```bash
# 1. 添加 Intel RealSense 软件源
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update

# 2. 安装 RealSense SDK
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev -y

# 3. 验证安装（插入相机后执行）
realsense-viewer  # 打开相机查看器，应该能看到相机图像

# 4. 检查已连接的 RealSense 设备
rs-enumerate-devices
```

### 安装 CAN 通信工具

```bash
# 安装 CAN 工具包
sudo apt install can-utils ethtool -y

# 加载 gs_usb 内核模块
sudo modprobe gs_usb

# 设置开机自动加载 gs_usb 模块
echo "gs_usb" | sudo tee -a /etc/modules

# 验证 CAN 设备（插入 USB-CAN 设备后执行）
lsusb | grep -i can
# 应该看到 CAN 转换器设备
```

### 配置 Python 环境

```bash
# 1. 确保使用 Python 3.10+
python3 --version
# 应该显示：Python 3.10.x

# 2. 安装 pip
sudo apt install python3-pip -y

# 3. 安装数据采集所需的 Python 库
pip3 install h5py dm_env numpy opencv-python

# 4. 安装 ROS2 Python 桥接库
sudo apt install ros-humble-cv-bridge python3-cv-bridge -y

# 5. 验证安装
python3 -c "import h5py; import numpy; import cv2; print('Python dependencies OK')"
```


### 编译 RealSense 相机工作空间

```bash
# 1. 进入相机工作空间
cd ~/code/cobot_magic_ros2/camera_ws

# 2. 安装依赖
sudo apt install ros-humble-realsense2-camera-msgs -y

# 3. 编译
colcon build

# 4. 验证编译成功
source install/setup.bash
ros2 pkg list | grep realsense
# 应该看到：
# realsense2_camera
# realsense2_camera_msgs
# realsense2_description
```

### 编译 Piper 机械臂工作空间

```bash
# 1. 进入 Piper 工作空间
cd ~/code/cobot_magic_ros2/Piper_ros2_humble

# 2. 编译
colcon build

# 3. 验证编译成功
source install/setup.bash
ros2 pkg list | grep piper
# 应该看到：
# piper
# piper_msgs
```

### 配置 CAN 设备映射

在启动机械臂前，需要配置 CAN 设备与左右臂的映射关系。

```bash
cd ~/code/cobot_magic_ros2/Piper_ros2_humble

# 1. 查看当前的 CAN 配置脚本
cat can_config.sh

# 2. 插入第一个 USB-CAN 设备，查看其 USB 地址
sudo ethtool -i can0 | grep bus
# 记录 bus-info 显示的地址，例如：1-8:1.0

# 3. 插入第二个 USB-CAN 设备到不同的 USB 口，查看地址
sudo ethtool -i can1 | grep bus
# 记录 bus-info 显示的地址，例如：1-9.1:1.0

# 4. 编辑 can_config.sh，修改 USB_PORTS 映射
# 找到这两行，替换为实际的 USB 地址：
#   USB_PORTS["1-8:1.0"]="can_left:1000000"
#   USB_PORTS["1-9.1:1.0"]="can_right:1000000"
```

**重要说明：**
- `can_left` 对应左侧机械臂
- `can_right` 对应右侧机械臂
- USB 地址需要与实际插入的 USB 口对应
- 波特率设置为 1000000（1Mbps）

### 配置相机序列号映射

```bash
cd ~/code/cobot_magic_ros2/camera_ws/src/realsense-ros/realsense2_camera/launch

# 1. 查看已连接相机的序列号
rs-enumerate-devices | grep "Serial Number"
# 记录三个相机的序列号

# 2. 编辑多相机启动文件
nano multi_camera.launch.py
# 或使用你喜欢的编辑器

# 3. 在文件中找到相机配置部分，修改序列号为实际值：
#   camera_left:   serial_no='<左相机序列号>'
#   camera_middle: serial_no='<中相机序列号>'
#   camera_right:  serial_no='<右相机序列号>'
```

### 环境配置验证清单

在开始数据采集前，请确认以下所有项目：

- [ ] ROS2 Humble 已安装并可运行 `ros2 --version`
- [ ] Intel RealSense SDK 已安装并可运行 `realsense-viewer`
- [ ] CAN 工具已安装并可运行 `candump --help`
- [ ] Python 依赖已安装（h5py, dm_env, numpy, opencv-python）
- [ ] camera_ws 已成功编译
- [ ] Piper_ros2_humble 已成功编译
- [ ] 三个 RealSense 相机已连接并可被 `rs-enumerate-devices` 识别
- [ ] 两个 USB-CAN 设备已连接并配置在 `can_config.sh` 中
- [ ] 相机序列号已配置在 `multi_camera.launch.py` 中
- [ ] 主从机械臂已通过航插线连接

完成以上所有步骤后，环境配置完成，可以继续执行数据采集流程。

---

## 一、系统准备和检查

### 1.1 硬件连接检查

```bash
# 检查三个RealSense相机是否连接
rs-enumerate-devices
# 应该看到三个相机的序列号，
# 然后修改camera_ws\src\realsense-ros\realsense_camera\launch\multi_camera.launch.py文件的序列号

# 检查CAN设备
lsusb | grep -i can
# 应该看到两个USB转CAN设备，可通过插拔来判断是哪个，然后修改can_config.sh对应的参数
```

### 1.2 安装必要依赖

```bash
# ROS2依赖
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge

# CAN通信工具
sudo apt install can-utils ethtool

# Python依赖
pip3 install h5py dm_env numpy opencv-python
```

---

## 二、配置CAN设备（双臂）

**⚠️ 重要**：需要为左右两个机械臂配置独立的CAN端口

### 2.1 配置方法

```bash
cd cobot_magic_ros2/Piper_ros2_humble


```

---

## 三、启动三个相机

### 3.1 启动相机节点

打开**终端1**：

```bash
cd cobot_magic_ros2/camera_ws
source install/setup.bash  

# 启动三个RealSense相机（左、中、右）
ros2 launch realsense2_camera multi_camera.launch.py
```

### 3.2 验证相机启动成功

打开新终端：

```bash
# 检查相机话题
ros2 topic list | grep camera

# 应该看到：
# /camera_left/color/image_raw
# /camera_left/depth/image_rect_raw
# /camera_middle/color/image_raw
# /camera_middle/depth/image_rect_raw
# /camera_right/color/image_raw
# /camera_right/depth/image_rect_raw

```

---

## 四、启动双主机械臂

### 4.1 启动机械臂节点

打开**终端2**：

```bash
cd Piper_ros2_humble
bash can_config.sh
source install/setup.bash

# 启动双主臂系统（模式0：数据采集模式）
ros2 launch piper start_ms_piper.launch.py mode:=0 auto_enable:=false
```

### 4.2 参数说明

- `mode:=0` - 数据采集模式，读取主从臂关节状态
- `auto_enable:=false` - 不自动上电（更安全）

### 4.3 硬件准备要求

- ✅ 主臂和从臂通过**航插线连接**
- ✅ 从臂会**跟随主臂运动**（遥操作模式）
- ✅ 确保机械臂周围有足够的活动空间

### 4.4 验证机械臂启动成功

打开新终端：

```bash
# 检查机械臂话题
ros2 topic list | grep -E "(master|puppet)"

# 应该看到：
# /master/joint_left          # 左主臂关节状态
# /master/joint_right         # 右主臂关节状态
# /puppet/joint_left          # 左从臂关节状态
# /puppet/joint_right         # 右从臂关节状态
# /puppet/end_pose_left       # 左从臂末端位姿
# /puppet/end_pose_right      # 右从臂末端位姿

# 检查关节数据发布频率
ros2 topic hz /puppet/joint_left
# 应该有持续的数据发布

# 实时查看关节状态
ros2 topic echo /master/joint_left --once
```

---

## 五、启动数据采集脚本

### 5.1 基础采集命令

打开**终端3**：

```bash
cd /home/yiqun/code/cobot_magic_ros2

# 基础采集命令（保存RGB图像）
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 0 \
    --max_timesteps 500 \
    --arm_type piper \
    --frame_rate 30 \
    --use_depth_image False \
    --use_forward_kinematics True
```

### 5.2 高质量采集命令

```bash
# 高质量采集（包含深度图和运动学）
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name grab_cup \
    --episode_idx 0 \
    --max_timesteps 800 \
    --arm_type piper \
    --frame_rate 30 \
    --use_depth_image True \
    --use_forward_kinematics True
```

### 5.3 参数说明

| 参数 | 说明 | 示例 |
|------|------|------|
| `--dataset_dir` | 数据保存目录（会自动创建） | `~/robot_data` |
| `--task_name` | 任务名称 | `place_shoe` |
| `--episode_idx` | 当前数据集的序号（从0开始） | `0, 1, 2...` |
| `--max_timesteps` | 采集的最大帧数 | `500`（约17秒@30fps） |
| `--arm_type` | 机械臂类型 | `piper` |
| `--frame_rate` | 采集帧率 | `30` |
| `--use_depth_image` | 是否采集深度图 | `True/False` |
| `--use_forward_kinematics` | 是否计算正运动学和相机外参 | `True/False` |

### 5.4 预设任务列表

| 任务名称 | 任务描述 |
|---------|---------|
| `place_shoe` | 用一只手臂抓取鞋子并放在垫子上 |
| `pick_cup` | 抓取杯子并提升到目标高度 |
| `open_door` | 抓住把手并拉开门 |
| `close_drawer` | 抓住把手并推到完全关闭 |
| `wipe_table` | 用海绵以圆周运动擦拭标记区域 |
| `plug_cable` | 拾起连接器并插入插座 |
| `fold_towel` | 拿起毛巾并沿中心线折叠 |
| `pour_water` | 抓住瓶子并将水倒入杯中 |

---

## 六、数据采集过程

### 6.1 采集流程

**1. 脚本启动后**，会显示：
```
[32m已加载Piper机械臂的DH参数（7个关节+末端）[0m
[32m世界坐标系: 左臂基座坐标系 (base_link)[0m
[32m左臂基座偏移: [0. 0. 0.][0m
[32m右臂基座偏移: [ 0.  -0.6  0. ][0m
等待数据同步...
```

**2. 同步完成后**，开始采集：
```
开始采集数据（按 Ctrl+C 提前停止）
进度: 1/500 帧
进度: 2/500 帧
...
```

**3. 采集过程中**：
- 手动操控**主臂**执行任务动作
- **从臂**会自动跟随主臂运动
- 三个相机同步录制RGB/深度图像

**4. 完成采集**：
```
采集完成！共采集 500 帧
正在保存数据到: /home/yiqun/robot_data/place_shoe/episode_0.hdf5
数据已成功保存！
```

### 6.2 提前停止

如需提前结束采集，按 `Ctrl+C`，数据会保存已采集的部分。

---

## 七、数据验证

### 7.1 查看采集的数据文件

```bash
# 检查数据文件
ls -lh ~/robot_data/place_shoe/
# 应该看到：episode_0.hdf5

# 查看文件大小
du -h ~/robot_data/place_shoe/episode_0.hdf5
```

### 7.2 查看HDF5文件结构

```bash
python3 << 'EOF'
import h5py

with h5py.File('~/robot_data/place_shoe/episode_0.hdf5', 'r') as f:
    print("数据集结构：")
    def print_structure(name, obj):
        print(name)
    f.visititems(print_structure)

    # 查看数据维度
    print("\n数据维度：")
    print(f"关节位置: {f['observations/qpos'].shape}")
    print(f"左相机RGB: {f['observations/camera_left/rgb'].shape}")
    print(f"中相机RGB: {f['observations/camera_middle/rgb'].shape}")
    print(f"右相机RGB: {f['observations/camera_right/rgb'].shape}")
    print(f"时间戳: {f['time_stamps'].shape}")

    # 查看元数据
    print("\n任务元数据：")
    print(f"任务名称: {f['meta_data/task_name'][()]}")
    print(f"总帧数: {f['meta_data/num_steps'][()]}")
EOF
```

### 7.3 数据结构说明

```
episode_0.hdf5
├── observations/
│   ├── qpos [N, 14]              # 双臂关节位置（每臂6关节+1夹爪）
│   ├── qvel [N, 14]              # 双臂关节速度
│   ├── effort [N, 14]            # 双臂关节力矩
│   ├── end_pose_left [N, 7]     # 左臂末端位姿（xyz + 四元数）
│   ├── end_pose_right [N, 7]    # 右臂末端位姿
│   ├── camera_left/
│   │   ├── rgb [N, H, W, 3]     # 左相机RGB图像
│   │   ├── depth [N, H, W]      # 左相机深度图（可选）
│   │   ├── extrinsic [N, 4, 4]  # 相机外参矩阵
│   │   └── intrinsic_cv [3, 3]  # 相机内参矩阵
│   ├── camera_middle/           # 中相机数据（结构同上）
│   └── camera_right/            # 右相机数据（结构同上）
├── meta_data/
│   ├── task_name                # 任务名称
│   ├── instruction              # 任务描述
│   ├── user_name                # 用户名
│   ├── uuid                     # 唯一标识符
│   └── num_steps                # 总帧数
└── time_stamps [N]              # 时间戳序列
```

---

## 八、多次采集（批量数据集）

### 8.1 采集多个episode示例

```bash
# Episode 0
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 0 \
    --max_timesteps 500

# Episode 1（同样的任务，不同的轨迹）
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 1 \
    --max_timesteps 500

# Episode 2
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 2 \
    --max_timesteps 500
```

### 8.2 最终数据结构

```
~/robot_data/
├── place_shoe/
│   ├── episode_0.hdf5
│   ├── episode_1.hdf5
│   └── episode_2.hdf5
├── pick_cup/
│   ├── episode_0.hdf5
│   ├── episode_1.hdf5
│   └── episode_2.hdf5
└── open_door/
    ├── episode_0.hdf5
    └── episode_1.hdf5
```
