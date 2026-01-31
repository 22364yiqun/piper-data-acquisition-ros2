# Camera_WS ROS2 迁移指南

## 好消息

✅ **Intel RealSense** 和 **Orbbec Astra** 都有官方的 ROS2 驱动支持！

不需要手动移植代码，可以直接使用官方 ROS2 版本。

---

## 迁移方案

### 方案 1: 直接使用 ROS2 官方包（推荐）

最简单的方式是使用 APT 安装 ROS2 版本的相机驱动。

### 方案 2: 从源码编译 ROS2 版本

如果需要最新功能或自定义修改，可以从源码编译。

---

## 第一步：创建 ROS2 工作空间

```bash
# 创建 ROS2 工作空间
mkdir -p camera_ros2_ws/src
cd ~/camera_ros2_ws/src

# 确保 ROS2 环境已加载
source /opt/ros/humble/setup.bash  # 或 jazzy/iron
```

---

## 第二步：迁移 RealSense 相机

### 方案 A: 使用 APT 安装（最简单）

```bash
# 安装 ROS2 RealSense 包
sudo apt update
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-realsense2-camera-msgs
sudo apt install ros-humble-realsense2-description

# 验证安装
ros2 pkg list | grep realsense
```

### 方案 B: 从源码编译

```bash
cd ~/camera_ros2_ws/src

# 克隆 ROS2 分支
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd realsense-ros
git checkout `git tag | sort -V | grep -P "^4.\d+\.\d+" | tail -1`

# 安装依赖
cd ~/camera_ros2_ws
rosdep install -i --from-path src --rosdistro humble -y

# 编译
colcon build --symlink-install
source install/setup.bash
```

### RealSense 库安装（必需）

```bash
# 添加 Intel 仓库
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# 验证
realsense-viewer
```

---

## 第三步：迁移 Astra 相机

### 方案 A: 使用 APT 安装

```bash
# ROS2 Humble/Iron
sudo apt install ros-humble-astra-camera
sudo apt install ros-humble-astra-camera-msgs
```

**注意**：不是所有 ROS2 发行版都有预编译的 Astra 包，可能需要从源码编译。

### 方案 B: 从源码编译（推荐）

```bash
cd ~/camera_ros2_ws/src

# 克隆 ROS2 版本
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# 安装依赖
sudo apt install libgflags-dev libgoogle-glog-dev

# 编译
cd ~/camera_ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 设置 USB 权限

```bash
cd ~/camera_ros2_ws/src/OrbbecSDK_ROS2
sudo cp scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 第四步：ROS1 vs ROS2 命令对比

### RealSense 相机

| 功能 | ROS1 (Noetic) | ROS2 (Humble) |
|------|---------------|---------------|
| 启动相机 | `roslaunch realsense2_camera rs_camera.launch` | `ros2 launch realsense2_camera rs_launch.py` |
| 深度对齐 | `roslaunch realsense2_camera rs_aligned_depth.launch` | `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true` |
| T265 跟踪 | `roslaunch realsense2_camera rs_t265.launch` | `ros2 launch realsense2_camera rs_launch.py` (自动检测 T265) |
| 多相机 | `roslaunch realsense2_camera rs_multiple_devices.launch` | `ros2 launch realsense2_camera rs_multi_camera_launch.py` |
| 查看话题 | `rostopic list` | `ros2 topic list` |
| 查看图像 | `rosrun rqt_image_view rqt_image_view` | `ros2 run rqt_image_view rqt_image_view` |

### Astra 相机

| 功能 | ROS1 (Noetic) | ROS2 (Humble) |
|------|---------------|---------------|
| 启动 Astra | `roslaunch astra_camera astra.launch` | `ros2 launch astra_camera astra.launch.py` |
| 启动 Gemini | `roslaunch astra_camera gemini.launch` | `ros2 launch astra_camera gemini.launch.py` |
| 多相机 | `roslaunch astra_camera multi_camera.launch` | `ros2 launch astra_camera multi_camera.launch.py` |
| 列出设备 | `roslaunch astra_camera list_devices.launch` | `ros2 run astra_camera list_devices` |

---

## 第五步：ROS2 Launch 文件示例

### RealSense Launch 文件（Python）

创建 `rs_d435.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Camera name'
        ),

        DeclareLaunchArgument(
            'enable_color',
            default_value='true',
            description='Enable color stream'
        ),

        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='Enable depth stream'
        ),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=LaunchConfiguration('camera_name'),
            namespace=LaunchConfiguration('camera_name'),
            parameters=[{
                'enable_color': LaunchConfiguration('enable_color'),
                'enable_depth': LaunchConfiguration('enable_depth'),
                'align_depth.enable': True,
                'enable_pointcloud': True,
                'pointcloud.enable': True,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
            }],
            output='screen'
        )
    ])
```

启动：
```bash
ros2 launch realsense2_camera rs_d435.launch.py
```

### Astra Launch 文件

创建 `astra_simple.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            namespace='camera',
            parameters=[{
                'color_width': 640,
                'color_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'color_fps': 30,
                'depth_fps': 30,
                'enable_color': True,
                'enable_depth': True,
                'enable_pointcloud': True,
            }],
            output='screen'
        )
    ])
```

---

## 第六步：话题名称变化

### RealSense 话题对比

| ROS1 话题 | ROS2 话题 |
|-----------|-----------|
| `/camera/color/image_raw` | `/camera/camera/color/image_raw` |
| `/camera/depth/image_rect_raw` | `/camera/camera/depth/image_rect_raw` |
| `/camera/depth/color/points` | `/camera/camera/depth/color/points` |
| `/camera/color/camera_info` | `/camera/camera/color/camera_info` |

**注意**：ROS2 版本默认添加了额外的命名空间层级。

可以通过 `camera_name` 参数自定义：

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=my_camera
# 话题变为：/my_camera/color/image_raw
```

### Astra 话题

基本保持一致，但也有命名空间变化。

---

## 第七步：代码迁移示例

### 订阅图像（ROS2 Python）

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()

        # 订阅 RealSense 彩色图像
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 注意话题名称
            self.color_callback,
            10)

        # 订阅深度图像
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        self.get_logger().info('Camera subscriber initialized')

    def color_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Color", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET)
            cv2.imshow("Depth", depth_colormap)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

---

## 第八步：与 Piper 机械臂集成（ROS2）

### 集成 Launch 文件

创建 `vision_piper_ros2.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # RealSense 相机
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py'
            )
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
        }.items()
    )

    # 静态 TF：相机到机械臂基座
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base',
        arguments=['0.3', '0', '0.5', '0', '0', '0', 'piper_base', 'camera_link']
    )

    # 目标检测节点（需要您自己实现）
    object_detector = Node(
        package='your_package',
        executable='object_detector',
        name='object_detector',
        parameters=[{
            'camera_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/depth/image_rect_raw',
        }]
    )

    return LaunchDescription([
        realsense_launch,
        static_tf,
        object_detector,
    ])
```

---

## 第九步：测试和验证

### 1. 测试 RealSense

```bash
# 终端 1: 启动相机
ros2 launch realsense2_camera rs_launch.py

# 终端 2: 查看话题
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once

# 终端 3: 查看图像
ros2 run rqt_image_view rqt_image_view
```

### 2. 测试 Astra

```bash
# 终端 1: 启动相机
ros2 launch astra_camera astra.launch.py

# 终端 2: 查看话题
ros2 topic list
ros2 topic hz /camera/color/image_raw

# 终端 3: 查看图像
ros2 run rqt_image_view rqt_image_view
```

### 3. 查看点云

```bash
# 安装 rviz2
sudo apt install ros-humble-rviz2

# 启动 rviz2
rviz2

# 在 rviz2 中：
# 1. Fixed Frame 设置为 camera_link
# 2. Add -> PointCloud2
# 3. Topic 选择 /camera/camera/depth/color/points
```

---

## 第十步：性能和配置建议

### QoS 设置（ROS2 特有）

ROS2 需要配置 QoS 以匹配相机发布者：

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# 传感器数据 QoS（推荐）
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

self.image_sub = self.create_subscription(
    Image,
    '/camera/camera/color/image_raw',
    self.callback,
    sensor_qos  # 使用传感器 QoS
)
```

### 参数配置

ROS2 参数通过 YAML 文件或命令行：

```yaml
# camera_params.yaml
/camera/camera:
  ros__parameters:
    enable_color: true
    enable_depth: true
    align_depth.enable: true
    depth_module.profile: "640x480x30"
    rgb_camera.profile: "640x480x30"
```

启动时加载：

```bash
ros2 launch realsense2_camera rs_launch.py \
    config_file:=/path/to/camera_params.yaml
```

---

## 常见问题

### 1. 找不到相机

```bash
# 检查 USB 连接
lsusb | grep Intel  # RealSense
lsusb | grep Orbbec  # Astra

# RealSense: 使用工具检查
realsense-viewer

# Astra: 检查权限
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. 话题订阅不到

检查 QoS 兼容性：

```bash
# 查看发布者 QoS
ros2 topic info /camera/camera/color/image_raw --verbose

# 订阅时使用匹配的 QoS
```

### 3. 编译错误

```bash
# 更新依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 清理后重新编译
rm -rf build install log
colcon build --symlink-install
```

---

## 迁移检查清单

- [ ] ROS2 环境已安装（Humble/Iron/Jazzy）
- [ ] librealsense2 库已安装（RealSense）
- [ ] USB 权限规则已配置（Astra）
- [ ] ROS2 相机驱动已安装或编译
- [ ] 相机能正常识别和启动
- [ ] 话题正常发布（ros2 topic list）
- [ ] 图像能正常显示（rqt_image_view）
- [ ] QoS 配置正确
- [ ] Launch 文件已转换为 Python 格式
- [ ] 代码已迁移到 rclpy
- [ ] 与机械臂集成测试通过

---

## 参考资源

### RealSense ROS2
- [官方 ROS2 仓库](https://github.com/IntelRealSense/realsense-ros/tree/ros2-development)
- [ROS2 分支文档](https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/README.md)

### Astra ROS2
- [Orbbec SDK ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)
- [Astra Camera ROS2](https://github.com/orbbec/ros2_astra_camera)

### ROS2 教程
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [从 ROS1 迁移](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)

---

## 快速开始命令总结

```bash
# 1. 安装 ROS2 相机驱动
sudo apt install ros-humble-realsense2-camera ros-humble-astra-camera

# 2. 启动 RealSense
ros2 launch realsense2_camera rs_launch.py

# 3. 启动 Astra
ros2 launch astra_camera astra.launch.py

# 4. 查看图像
ros2 run rqt_image_view rqt_image_view

# 5. 查看点云
rviz2
```

---

**迁移预计时间**: 1-2 天（取决于自定义代码数量）

**推荐方案**: 使用 APT 安装 + 从源码编译（获得最新功能）

🔍 具体修改内容

  1. 硬编码了具体的相机序列号

  官方版本（rs_multiple_devices.launch）:
  <arg name="serial_no_camera1" default=""/> <!-- 空的，需要用户填写 -->
  <arg name="serial_no_camera2" default=""/>
  <arg name="serial_no_camera3" default=""/>

  自定义版本（multi_camera.launch）:
  <arg name="serial_no_camera1" default="401622071599"/>  <!-- 已填写实际序列号 -->
  <arg name="serial_no_camera2" default="401622073209"/>
  <arg name="serial_no_camera3" default="401522071118"/>

  2. 改了相机命名

  官方版本:
  <arg name="camera1" default="camera1"/>
  <arg name="camera2" default="camera2"/>
  <arg name="camera3" default="camera3"/>

  自定义版本:
  <arg name="camera1" default="camera_left"/>    <!-- 左中右命名 -->
  <arg name="camera2" default="camera_middle"/>
  <arg name="camera3" default="camera_right"/>

  3. 添加了中文注释

  <!-- 相机序列号，根据实际修改 -->
  <!-- 相机命名空间 -->
  <!-- 启动时复位相机，可以避免掉线 -->
  <!-- 通用参数：分辨率+帧率 -->
  <!-- 只开彩色+深度 -->

  4. 启用了 initial_reset

  官方版本:
  <arg name="initial_reset" default="false"/>

  自定义版本:
  <arg name="initial_reset" default="true"/>  <!-- 默认复位，避免掉线 -->

  5. 添加了详细的相机参数配置

  自定义版本增加了统一的分辨率和帧率配置，并且显式禁用了不需要的流（红外、IMU）：

  <!-- 通用参数：分辨率+帧率 -->
  <arg name="depth_width" value="640"/>
  <arg name="depth_height" value="480"/>
  <arg name="depth_fps" value="30"/>
  <arg name="color_width" value="640"/>
  <arg name="color_height" value="480"/>
  <arg name="color_fps" value="30"/>

  <!-- 只开彩色+深度 -->
  <arg name="enable_color" value="true"/>
  <arg name="enable_depth" value="true"/>
  <arg name="enable_infra1" value="false"/>
  <arg name="enable_infra2" value="false"/>
  <arg name="enable_gyro" value="false"/>
  <arg name="enable_accel" value="false"/>

  ---
  📊 总结

  ✅ 其他文件状态

  - 所有其他文件未修改（时间戳都是 2024年4月18日，原始版本）
  - 没有 .git 目录，说明是从压缩包或直接复制的源码

  🎯 修改目的

  原作者的修改是为了：
  1. 快速启动三个 RealSense 相机（左、中、右）
  2. 针对特定硬件（已知的三个相机序列号）
  3. 优化稳定性（启用 initial_reset）
  4. 降低带宽（只开彩色+深度，关闭红外和IMU）

  💡 对 ROS2 迁移的影响

  这个自定义 launch 文件在 ROS2 中需要重写为 Python 格式，但修改的逻辑（相机序列号、命名、参数）可以直接迁移过去！
