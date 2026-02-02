# Camera Workspace - ROS 相机驱动工作空间

## 概述

这是一个 ROS Noetic 工作空间，包含了用于机器人视觉系统的多种深度相机驱动，支持 Intel RealSense 系列和 Orbbec Astra 系列相机。

## 工作空间结构

```
camera_ws/
├── build/                      # 编译生成目录
├── devel/                      # 开发环境目录
├── src/                        # 源代码目录
│   ├── realsense-ros/         # Intel RealSense 相机驱动
│   │   ├── realsense2_camera/        # 相机节点和驱动
│   │   └── realsense2_description/   # 相机 3D 模型和描述文件
│   └── ros_astra_camera/      # Orbbec Astra 相机驱动
└── .catkin_workspace          # Catkin 工作空间标识文件
```

## 支持的相机型号

### 1. Intel RealSense 系列

**软件包版本**: realsense2_camera v2.3.2

**支持的相机型号**:
- **D400 系列**（深度相机）
  - D415 - 适合室内短距离精确测量
  - D435 - 通用深度相机
  - D435i - 带 IMU 的深度相机
  - D455 - 更大视野的深度相机
- **SR300 系列** - 短距离结构光相机
- **L515 系列** - 激光雷达深度相机
- **T265 系列** - 跟踪模块（视觉里程计）

**依赖库**: librealsense2 v2.50.0

**主要功能**:
- RGB-D 图像采集（彩色 + 深度）
- 点云生成
- IMU 数据（D435i、T265）
- 多相机同步
- 深度对齐到彩色
- 动态参数配置

### 2. Orbbec Astra 系列

**软件包版本**: astra_camera v1.2.7

**支持的相机型号**:
- **Astra 系列**
  - Astra - 标准版深度相机
  - Astra Pro - 专业版
  - Astra Pro Plus - 专业增强版
- **Gemini 系列**
  - Gemini - 双目结构光相机
  - Gemini E / E Lite - 经济版
  - Gemini UW - 超宽视角版本
- **Dabai 系列**
  - Dabai、Dabai Pro、Dabai Max 系列
  - DC1、DCW、DCW2、DW、DW2 等多种型号
- **Deeyea 系列** - 高性能深度相机
- **Stereo S 系列** - 双目立体相机
- **Embedded 系列** - 嵌入式相机模块

**主要功能**:
- RGB-D 图像采集
- 点云生成
- 多相机支持
- 设备热插拔
- IR（红外）图像
- 相机标定信息管理

---

## 安装和配置

### 前置条件

```bash
# ROS Noetic 环境
source /opt/ros/noetic/setup.bash

# 安装依赖
sudo apt update
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport \
                 ros-noetic-camera-info-manager ros-noetic-rgbd-launch \
                 ros-noetic-tf2-ros
```

### Intel RealSense 驱动安装

#### 方法 1: 使用 APT 安装（推荐用于快速部署）

```bash
# 安装 RealSense SDK
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-realsense2-description
```

#### 方法 2: 从源码编译（本工作空间方式）

```bash
# 1. 安装 librealsense2 库
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev

# 2. 验证安装
realsense-viewer

# 3. 编译工作空间
cd ~/camera_ws
catkin_make
source devel/setup.bash
```

### Orbbec Astra 驱动安装

```bash
# 1. 设置 USB 权限规则
cd ~/camera_ws/src/ros_astra_camera
sudo cp 56-orbbec-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2. 安装依赖
sudo apt-get install libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

# 3. 编译
cd ~/camera_ws
catkin_make
source devel/setup.bash
```

---

## 使用方法

### Intel RealSense 相机

#### 1. 启动单个相机

```bash
# 基础启动（D435/D415/D455）
roslaunch realsense2_camera rs_camera.launch

# 带参数启动
roslaunch realsense2_camera rs_camera.launch \
    enable_depth:=true \
    enable_color:=true \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_pointcloud:=true
```

#### 2. 深度图对齐到彩色图

```bash
roslaunch realsense2_camera rs_aligned_depth.launch
```

#### 3. 启动 T265 跟踪相机

```bash
roslaunch realsense2_camera rs_t265.launch
```

#### 4. 多相机同步

```bash
roslaunch realsense2_camera rs_multiple_devices.launch
```

#### 5. RGBD 配置（与 rtabmap 等 SLAM 配合）

```bash
roslaunch realsense2_camera rs_rgbd.launch
```

#### 6. 查看相机模型（可视化）

```bash
# D435 模型
roslaunch realsense2_description view_d435_model.launch

# D415 模型
roslaunch realsense2_description view_d415_model.launch

# L515 模型
roslaunch realsense2_description view_l515_model.launch
```

### Orbbec Astra 相机

#### 1. 启动单个相机

```bash
# Astra 相机
roslaunch astra_camera astra.launch

# Astra Pro
roslaunch astra_camera astra_pro.launch

# Gemini 相机
roslaunch astra_camera gemini.launch

# Gemini E
roslaunch astra_camera gemini_e.launch

# Dabai 系列
roslaunch astra_camera dabai.launch
roslaunch astra_camera dabai_pro.launch
roslaunch astra_camera dabai_max.launch
```

#### 2. 多相机支持

```bash
# 多 Astra 相机
roslaunch astra_camera multi_astra.launch

# 多 Gemini 相机
roslaunch astra_camera multi_gemini.launch

# 多 Dabai 相机
roslaunch astra_camera multi_dabai_pro.launch

# 通用多相机配置
roslaunch astra_camera multi_camera.launch
```

#### 3. 列出所有连接的设备

```bash
roslaunch astra_camera list_devices.launch
```

---

## 发布的话题（Topics）

### RealSense 相机话题

```bash
# 图像话题
/camera/color/image_raw              # RGB 图像
/camera/depth/image_rect_raw         # 深度图像
/camera/infra1/image_rect_raw        # 左红外图像
/camera/infra2/image_rect_raw        # 右红外图像
/camera/aligned_depth_to_color/image_raw  # 对齐的深度图

# 相机信息
/camera/color/camera_info            # RGB 相机标定信息
/camera/depth/camera_info            # 深度相机标定信息

# 点云
/camera/depth/color/points           # 彩色点云

# IMU（D435i/T265）
/camera/imu                          # IMU 数据
/camera/accel/sample                 # 加速度计
/camera/gyro/sample                  # 陀螺仪

# 姿态（T265）
/camera/odom/sample                  # 里程计
/camera/pose                         # 位姿
```

### Astra 相机话题

```bash
# 图像话题
/camera/color/image_raw              # RGB 图像
/camera/depth/image_raw              # 深度图像
/camera/ir/image_raw                 # 红外图像

# 相机信息
/camera/color/camera_info
/camera/depth/camera_info

# 点云
/camera/depth/points                 # 点云数据
```

---

## 常用配置参数

### RealSense 参数

```yaml
# 图像流开关
enable_depth: true          # 启用深度流
enable_color: true          # 启用彩色流
enable_infra1: true         # 启用左红外流
enable_infra2: true         # 启用右红外流
enable_pointcloud: true     # 启用点云

# 图像分辨率和帧率
depth_width: 640
depth_height: 480
depth_fps: 30
color_width: 640
color_height: 480
color_fps: 30

# 深度配置
enable_emitter: true        # 启用红外发射器
align_depth: true           # 深度对齐到彩色

# 后处理滤波器
filters: "pointcloud,decimation,temporal"
decimation_filter: true     # 抽取滤波（提高性能）
spatial_filter: true        # 空间滤波（平滑）
temporal_filter: true       # 时间滤波（减少闪烁）
```

### Astra 参数

```yaml
# 图像流
color_enabled: true
depth_enabled: true
ir_enabled: true

# 分辨率
color_width: 640
color_height: 480
depth_width: 640
depth_height: 480

# 帧率
fps: 30

# 深度范围（毫米）
depth_min_distance: 300
depth_max_distance: 10000
```

---

## 典型应用场景

### 1. 机械臂视觉伺服

结合 Piper 机械臂进行视觉引导抓取：

```bash
# 终端 1: 启动相机
roslaunch realsense2_camera rs_aligned_depth.launch

# 终端 2: 启动机械臂
roslaunch piper start_ms_piper.launch mode:=1

# 终端 3: 启动视觉处理节点
rosrun your_package object_detection_node.py
```

### 2. 3D 重建和点云处理

```bash
# 启动相机点云
roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true

# 可视化点云
rosrun rviz rviz
# 添加 PointCloud2 显示，话题选择 /camera/depth/color/points
```

### 3. SLAM 建图

```bash
# 使用 rtabmap 进行 RGB-D SLAM
roslaunch realsense2_camera rs_rgbd.launch
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"
```

### 4. 双目立体视觉

```bash
# 使用双红外相机进行立体匹配
roslaunch realsense2_camera rs_camera.launch \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_depth:=false  # 使用立体匹配代替结构光
```

### 5. 多相机数据采集

```bash
# 同时启动左右相机采集数据
# 编辑 multi_camera.launch 配置相机序列号
roslaunch astra_camera multi_camera.launch
```

---

## 相机标定

### 使用 camera_calibration 包

```bash
# 安装标定工具
sudo apt-get install ros-noetic-camera-calibration

# 单目相机标定
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \
    --square 0.108 \
    image:=/camera/color/image_raw \
    camera:=/camera/color

# 双目相机标定（使用红外）
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \
    --square 0.108 \
    right:=/camera/infra2/image_rect_raw \
    left:=/camera/infra1/image_rect_raw \
    right_camera:=/camera/infra2 \
    left_camera:=/camera/infra1
```

标定结果会保存在 `~/.ros/camera_info/` 目录下。

---

## 故障排查

### RealSense 相机问题

#### 1. 相机无法识别

```bash
# 检查相机连接
rs-enumerate-devices

# 查看 USB 连接
lsusb | grep Intel

# 重新加载 udev 规则
sudo udevadm control --reload-rules && sudo udevadm trigger
```

#### 2. 权限问题

```bash
# 添加当前用户到 video 组
sudo usermod -a -G video $USER
# 注销并重新登录
```

#### 3. USB 带宽不足

- 降低分辨率或帧率
- 关闭不需要的图像流
- 使用 USB 3.0 端口
- 每个 USB 控制器只连接一个相机

#### 4. 启动失败

```bash
# 查看详细错误信息
roslaunch realsense2_camera rs_camera.launch --screen

# 检查 librealsense 版本
dpkg -l | grep librealsense
```

### Astra 相机问题

#### 1. 设备权限错误

```bash
# 重新安装 udev 规则
cd ~/camera_ws/src/ros_astra_camera
sudo cp 56-orbbec-usb.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
```

#### 2. 找不到设备

```bash
# 列出所有 Orbbec 设备
roslaunch astra_camera list_devices.launch

# 检查 USB 连接
lsusb | grep Orbbec
```

#### 3. 图像质量问题

- 检查镜头是否有污渍
- 调整曝光参数
- 确保环境光照充足
- 检查深度范围设置

---

## 性能优化建议

### 1. 降低计算负载

```yaml
# 降低分辨率
depth_width: 424
depth_height: 240
color_width: 424
color_height: 240

# 降低帧率
depth_fps: 15
color_fps: 15

# 关闭不需要的流
enable_infra1: false
enable_infra2: false
```

### 2. 启用硬件加速

```yaml
# 使用硬件后处理
pointcloud_texture_stream: RS2_STREAM_COLOR
pointcloud_texture_index: 0
```

### 3. 网络传输优化

```bash
# 使用压缩图像传输
rosrun image_transport republish raw in:=/camera/color/image_raw compressed out:=/camera/color/compressed
```

---

## 与 Piper 机械臂集成示例

### 视觉引导抓取 Launch 文件

创建 `vision_grasp.launch`:

```xml
<launch>
  <!-- 启动 RealSense 相机 -->
  <include file="$(find realsense2_camera)/launch/rs_aligned_depth.launch">
    <arg name="align_depth" value="true"/>
    <arg name="enable_pointcloud" value="true"/>
  </include>

  <!-- 启动 Piper 机械臂 -->
  <include file="$(find piper)/launch/start_ms_piper.launch">
    <arg name="mode" value="1"/>
    <arg name="auto_enable" value="true"/>
  </include>

  <!-- 启动相机到机械臂基座的 TF 变换 -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="camera_to_base"
        args="0.3 0 0.5 0 0 0 piper_base camera_link"/>

  <!-- 启动目标检测节点 -->
  <node pkg="your_package" type="object_detector.py" name="object_detector"/>

  <!-- 启动抓取规划节点 -->
  <node pkg="your_package" type="grasp_planner.py" name="grasp_planner"/>
</launch>
```

---

## 开发参考

### 订阅图像示例（Python）

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber:
    def __init__(self):
        rospy.init_node('camera_subscriber')
        self.bridge = CvBridge()

        # 订阅彩色图像
        self.color_sub = rospy.Subscriber(
            '/camera/color/image_raw',
            Image,
            self.color_callback)

        # 订阅深度图像
        self.depth_sub = rospy.Subscriber(
            '/camera/depth/image_rect_raw',
            Image,
            self.depth_callback)

    def color_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Color", cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        # 深度图可视化
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET)
        cv2.imshow("Depth", depth_colormap)
        cv2.waitKey(1)

if __name__ == '__main__':
    node = CameraSubscriber()
    rospy.spin()
```

---

## 参考资源

### RealSense
- [官方文档](https://github.com/IntelRealSense/realsense-ros)
- [librealsense SDK](https://github.com/IntelRealSense/librealsense)
- [RealSense Viewer 工具](https://www.intelrealsense.com/developers/)

### Astra
- [Orbbec 官网](https://www.orbbec.com/)
- [ROS Astra Camera GitHub](https://github.com/orbbec/ros_astra_camera)
- [Astra SDK](https://orbbec3d.com/develop/)

### ROS 图像处理
- [image_transport](http://wiki.ros.org/image_transport)
- [cv_bridge](http://wiki.ros.org/cv_bridge)
- [camera_calibration](http://wiki.ros.org/camera_calibration)

---

## 技术支持

如遇到问题，请按以下顺序排查：

1. 检查硬件连接和 USB 端口
2. 验证驱动和库版本
3. 查看 ROS 日志：`rosrun rqt_console rqt_console`
4. 检查话题是否正常发布：`rostopic list` 和 `rostopic hz`
5. 查阅官方 GitHub Issues

---

## 版本信息

- **ROS 版本**: Noetic
- **Ubuntu 版本**: 20.04
- **realsense2_camera**: v2.3.2
- **astra_camera**: v1.2.7
- **librealsense2**: v2.50.0

---

## 许可证

- RealSense ROS: Apache 2.0
- Astra Camera ROS: Apache 2.0

---

**最后更新**: 2026-01-28
