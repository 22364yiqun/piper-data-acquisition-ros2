# Piper 机械臂 ROS1 到 ROS2 迁移指南

## 概述

本文档详细说明如何将 Piper 机械臂项目从 ROS Noetic (ROS1) 迁移到 ROS2 (推荐 Humble 或 Iron)。

## 项目当前结构

- **piper** - 主功能包，包含节点和启动文件
- **piper_msgs** - 自定义消息包
- Python 节点使用 `rospy`
- Launch 文件使用 XML 格式
- 构建系统：catkin

## 迁移步骤总览

1. 环境准备
2. 包结构调整
3. 消息定义迁移
4. Python 代码迁移（rospy → rclpy）
5. Launch 文件迁移（XML → Python）
6. 构建系统迁移（catkin → ament）
7. 测试和验证

---

## 第一步：环境准备

### 1.1 安装 ROS2

```bash
# Ubuntu 22.04 推荐安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

# 或者 Ubuntu 24.04 安装 ROS2 Jazzy
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

### 1.2 安装构建工具

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2
```

### 1.3 Python 依赖保持不变

```bash
pip3 install python-can
pip3 install piper_sdk
```

---

## 第二步：包结构调整

### 2.1 创建 ROS2 工作空间

```bash
mkdir -p ~/piper_ros2_ws/src
cd ~/piper_ros2_ws/src
```

### 2.2 复制源代码

```bash
# 复制两个功能包
cp -r /path/to/Piper_ros_private-ros-noetic/src/piper .
cp -r /path/to/Piper_ros_private-ros-noetic/src/piper_msgs .
```

### 2.3 调整目录结构

ROS2 Python 包需要特定的目录结构：

```
piper/
├── package.xml          # 需要修改
├── setup.py            # 新建（ROS2 Python 包必需）
├── setup.cfg           # 新建
├── piper/              # Python 模块目录（重命名 scripts/）
│   └── __init__.py
│   └── piper_start_ms_node.py
│   └── piper_start_slave_node.py
│   └── piper_start_master_node.py
│   └── piper_read_master_node.py
├── launch/             # Launch 文件需要改为 Python 格式
│   └── start_ms_piper.launch.py
└── resource/
    └── piper

piper_msgs/
├── package.xml         # 需要修改
├── CMakeLists.txt      # 需要大幅修改
└── msg/
    ├── PiperStatusMsg.msg
    └── PosCmd.msg
```

---

## 第三步：消息定义迁移

### 3.1 修改 piper_msgs/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>piper_msgs</name>
  <version>1.0.0</version>
  <description>Custom messages for Piper robotic arm</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 3.2 修改 piper_msgs/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(piper_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PiperStatusMsg.msg"
  "msg/PosCmd.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### 3.3 消息文件无需修改

`.msg` 文件格式在 ROS1 和 ROS2 中是兼容的，无需修改。

---

## 第四步：Python 代码迁移（核心部分）

### 4.1 主要变化对照表

| ROS1 (rospy) | ROS2 (rclpy) | 说明 |
|--------------|--------------|------|
| `rospy.init_node('name')` | `rclpy.init()` + `Node('name')` | 节点初始化 |
| `rospy.Publisher()` | `self.create_publisher()` | 发布者 |
| `rospy.Subscriber()` | `self.create_subscription()` | 订阅者 |
| `rospy.Service()` | `self.create_service()` | 服务 |
| `rospy.get_param()` | `self.declare_parameter()` + `self.get_parameter()` | 参数 |
| `rospy.loginfo()` | `self.get_logger().info()` | 日志 |
| `rospy.Rate(10)` | `self.create_rate(10)` | 频率控制 |
| `rospy.spin()` | `rclpy.spin(node)` | 主循环 |
| `rospy.is_shutdown()` | `rclpy.ok()` | 运行状态 |

### 4.2 迁移示例：piper_start_ms_node.py

#### ROS1 版本关键代码：

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class C_PiperRosNode():
    def __init__(self):
        rospy.init_node('piper_start_all_node', anonymous=True)

        # 获取参数
        self.can_port = rospy.get_param("~can_port", "can0")
        self.mode = rospy.get_param("~mode", 0)

        # 创建发布者
        self.joint_pub = rospy.Publisher('/puppet/joint_states',
                                         JointState,
                                         queue_size=1)

        # 创建订阅者
        if self.mode == 1:
            rospy.Subscriber('/master/joint_states',
                           JointState,
                           self.joint_callback)

        # 创建服务
        self.service = rospy.Service('/go_zero_master',
                                     Trigger,
                                     self.handle_service)

        rospy.loginfo("Node started")
        rospy.spin()
```

#### ROS2 版本：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

class PiperRosNode(Node):
    def __init__(self):
        super().__init__('piper_start_all_node')

        # 声明并获取参数
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('mode', 0)
        self.can_port = self.get_parameter('can_port').value
        self.mode = self.get_parameter('mode').value

        # 创建发布者（QoS 设置）
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos = QoSProfile(depth=1,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.joint_pub = self.create_publisher(JointState,
                                                '/puppet/joint_states',
                                                qos)

        # 创建订阅者
        if self.mode == 1:
            self.joint_sub = self.create_subscription(
                JointState,
                '/master/joint_states',
                self.joint_callback,
                qos)

        # 创建服务
        self.service = self.create_service(Trigger,
                                           '/go_zero_master',
                                           self.handle_service)

        # 创建定时器（替代 rospy.Rate）
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

        self.get_logger().info("Node started")

    def joint_callback(self, msg):
        # 订阅回调函数
        pass

    def handle_service(self, request, response):
        # 服务回调函数
        response.success = True
        response.message = "Service completed"
        return response

    def timer_callback(self):
        # 定时器回调（主循环）
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PiperRosNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.3 关键变化详解

#### 1. 类继承 Node
```python
# ROS1
class C_PiperRosNode():
    def __init__(self):
        rospy.init_node('node_name')

# ROS2
class PiperRosNode(Node):
    def __init__(self):
        super().__init__('node_name')
```

#### 2. 参数处理
```python
# ROS1
can_port = rospy.get_param("~can_port", "can0")

# ROS2
self.declare_parameter('can_port', 'can0')
can_port = self.get_parameter('can_port').value
```

#### 3. 时间戳
```python
# ROS1
msg.header.stamp = rospy.Time.now()

# ROS2
msg.header.stamp = self.get_clock().now().to_msg()
```

#### 4. 频率控制
```python
# ROS1
rate = rospy.Rate(100)
while not rospy.is_shutdown():
    # do work
    rate.sleep()

# ROS2 - 方式1：定时器（推荐）
self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

# ROS2 - 方式2：Rate
rate = self.create_rate(100)
while rclpy.ok():
    # do work
    rate.sleep()
```

#### 5. 线程处理
```python
# ROS1
import threading
sub_thread = threading.Thread(target=self.SubPosThread)
sub_thread.start()

# ROS2 - 类似，但建议使用 MultiThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

---

## 第五步：Launch 文件迁移

### 5.1 ROS1 Launch (XML)

```xml
<launch>
  <arg name="mode" default="0" />
  <arg name="auto_enable" default="true" />

  <node name="piper_left" pkg="piper" type="piper_start_ms_node.py" output="screen">
    <param name="can_port" value="can_left" />
    <param name="mode" value="$(arg mode)" />
    <param name="auto_enable" value="$(arg auto_enable)" />
    <remap from="/puppet/joint_states" to="/puppet/joint_left" />
    <remap from="/master/joint_states" to="/master/joint_left" />
  </node>
</launch>
```

### 5.2 ROS2 Launch (Python)

创建 `launch/start_ms_piper.launch.py`：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明 launch 参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description='Operating mode: 0=read data, 1=control'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Auto enable the arm'
    )

    # 左侧机械臂节点
    piper_left_node = Node(
        package='piper',
        executable='piper_start_ms_node',
        name='piper_left',
        output='screen',
        parameters=[{
            'can_port': 'can_left',
            'mode': LaunchConfiguration('mode'),
            'auto_enable': LaunchConfiguration('auto_enable'),
        }],
        remappings=[
            ('/puppet/joint_states', '/puppet/joint_left'),
            ('/master/joint_states', '/master/joint_left'),
        ]
    )

    # 右侧机械臂节点
    piper_right_node = Node(
        package='piper',
        executable='piper_start_ms_node',
        name='piper_right',
        output='screen',
        parameters=[{
            'can_port': 'can_right',
            'mode': LaunchConfiguration('mode'),
            'auto_enable': LaunchConfiguration('auto_enable'),
        }],
        remappings=[
            ('/puppet/joint_states', '/puppet/joint_right'),
            ('/master/joint_states', '/master/joint_right'),
        ]
    )

    return LaunchDescription([
        mode_arg,
        auto_enable_arg,
        piper_left_node,
        piper_right_node,
    ])
```

### 5.3 启动命令对比

```bash
# ROS1
roslaunch piper start_ms_piper.launch mode:=0 auto_enable:=true

# ROS2
ros2 launch piper start_ms_piper.launch.py mode:=0 auto_enable:=true
```

---

## 第六步：构建系统迁移

### 6.1 创建 setup.py（piper 包）

```python
from setuptools import setup
import os
from glob import glob

package_name = 'piper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # 配置文件（如果有）
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Piper robotic arm ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_start_ms_node = piper.piper_start_ms_node:main',
            'piper_start_slave_node = piper.piper_start_slave_node:main',
            'piper_start_master_node = piper.piper_start_master_node:main',
            'piper_read_master_node = piper.piper_read_master_node:main',
        ],
    },
)
```

### 6.2 创建 setup.cfg（piper 包）

```ini
[develop]
script_dir=$base/lib/piper
[install]
install_scripts=$base/lib/piper
```

### 6.3 创建 resource/piper

```bash
mkdir -p resource
touch resource/piper
```

### 6.4 修改 piper/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>piper</name>
  <version>1.0.0</version>
  <description>Piper robotic arm control package</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_srvs</depend>
  <depend>piper_msgs</depend>

  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 第七步：构建和测试

### 7.1 构建工作空间

```bash
cd ~/piper_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 7.2 测试节点

```bash
# 终端1：启动 launch 文件
ros2 launch piper start_ms_piper.launch.py mode:=0

# 终端2：查看话题
ros2 topic list
ros2 topic echo /puppet/joint_left

# 终端3：调用服务
ros2 service call /can_left/go_zero_master std_srvs/srv/Trigger
```

---

## 常见问题和注意事项

### 1. QoS 策略

ROS2 引入了 QoS（服务质量）设置，对于实时控制，建议使用：

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 或 RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST
)

self.publisher = self.create_publisher(JointState, '/topic', qos)
```

### 2. tf 库变化

```python
# ROS1
from tf.transformations import quaternion_from_euler

# ROS2
from tf_transformations import quaternion_from_euler
# 或者
from scipy.spatial.transform import Rotation
```

### 3. 多线程

如需多线程处理：

```python
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()
    node = PiperRosNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

### 4. 命名空间

ROS2 中命名空间处理略有不同：

```python
# 使用 namespace 参数
node = Node('my_node', namespace='robot1')
```

### 5. 参数回调

ROS2 支持参数动态修改：

```python
from rcl_interfaces.msg import SetParametersResult

def __init__(self):
    super().__init__('node')
    self.declare_parameter('my_param', 1.0)
    self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'my_param':
            self.get_logger().info(f'Parameter changed to {param.value}')
    return SetParametersResult(successful=True)
```

---

## 迁移检查清单

- [ ] 消息定义迁移完成
- [ ] CMakeLists.txt 和 package.xml 更新
- [ ] Python 节点改为继承 Node 类
- [ ] 所有 rospy 调用替换为 rclpy
- [ ] 参数使用 declare_parameter/get_parameter
- [ ] 日志使用 self.get_logger()
- [ ] 时间戳使用 self.get_clock().now()
- [ ] Launch 文件改为 Python 格式
- [ ] setup.py 和 setup.cfg 创建
- [ ] entry_points 配置正确
- [ ] 构建成功（colcon build）
- [ ] 测试所有功能正常

---

## 推荐资源

- [ROS2 官方迁移指南](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)
- [rclpy API 文档](https://docs.ros2.org/latest/api/rclpy/)
- [ROS2 Launch 文档](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

---

## 总结

ROS2 迁移主要工作量在 Python 代码重构和 Launch 文件重写。关键点：

1. 所有节点改为类继承 `Node`
2. `rospy` → `rclpy` API 替换
3. XML Launch → Python Launch
4. `catkin` → `ament_python/ament_cmake`
5. 注意 QoS 设置和多线程处理

预计迁移时间：2-5 天（取决于代码复杂度和测试要求）
