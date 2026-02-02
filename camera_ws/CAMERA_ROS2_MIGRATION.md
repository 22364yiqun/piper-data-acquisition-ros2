# Camera ROS1 到 ROS2 迁移说明

## 概述

如果您的 Piper 机械臂项目中使用了相机进行视觉采集或视觉控制，本文档说明如何将相机相关的 ROS1 功能迁移到 ROS2。

## 常见 ROS1 相机驱动包及其 ROS2 对应版本

| ROS1 包名 | ROS2 包名 | 状态 | 说明 |
|----------|----------|------|------|
| `usb_cam` | `usb_cam` | ✅ 可用 | USB 相机驱动，已移植到 ROS2 |
| `cv_camera` | `cv_camera` | ✅ 可用 | OpenCV 相机驱动，已移植 |
| `image_transport` | `image_transport` | ✅ 可用 | 图像传输库 |
| `image_proc` | `image_proc` | ✅ 可用 | 图像处理节点 |
| `compressed_image_transport` | `compressed_image_transport` | ✅ 可用 | 压缩图像传输 |
| `realsense2_camera` | `realsense2_camera` | ✅ 可用 | Intel RealSense 相机 |
| `cv_bridge` | `cv_bridge` | ✅ 可用 | OpenCV-ROS 消息转换 |

---

## 第一部分：相机驱动迁移

### 1. USB 相机（usb_cam）

#### ROS1 安装和使用

```bash
# 安装
sudo apt install ros-noetic-usb-cam

# 启动
rosrun usb_cam usb_cam_node

# 或使用 launch 文件
roslaunch usb_cam usb_cam-test.launch
```

#### ROS2 迁移

```bash
# 安装（Humble）
sudo apt install ros-humble-usb-cam

# 启动
ros2 run usb_cam usb_cam_node_exe

# 或使用 launch 文件
ros2 launch usb_cam camera.launch.py
```

#### 参数配置变化

**ROS1 Launch (XML):**

```xml
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
    <param name="video_device" value="/dev/video0"/>
    <param name="image_width" value="640"/>
    <param name="image_height" value="480"/>
    <param name="pixel_format" value="yuyv"/>
    <param name="camera_frame_id" value="usb_cam"/>
    <param name="io_method" value="mmap"/>
    <param name="framerate" value="30"/>
  </node>
</launch>
```

**ROS2 Launch (Python):**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'usb_cam',
                'io_method': 'mmap',
                'framerate': 30.0,
            }]
        )
    ])
```

### 2. RealSense 相机

#### ROS1

```bash
sudo apt install ros-noetic-realsense2-camera
roslaunch realsense2_camera rs_camera.launch
```

#### ROS2

```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

---

## 第二部分：图像处理代码迁移

### 1. cv_bridge 使用变化

#### ROS1 代码

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor')
        self.bridge = CvBridge()

        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/camera/image_raw',
                                          Image,
                                          self.image_callback)

        # 发布处理后的图像
        self.image_pub = rospy.Publisher('/processed_image',
                                         Image,
                                         queue_size=1)

        rospy.spin()

    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 图像处理
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # OpenCV → ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(gray, "mono8")
            self.image_pub.publish(out_msg)

        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    ImageProcessor()
```

#### ROS2 代码

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        # 订阅图像话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # 发布处理后的图像
        self.image_pub = self.create_publisher(
            Image,
            '/processed_image',
            10)

    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 图像处理
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # OpenCV → ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(gray, "mono8")
            out_msg.header = msg.header  # 保留时间戳
            self.image_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()

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

### 2. 关键变化点

| 功能 | ROS1 | ROS2 |
|------|------|------|
| cv_bridge 导入 | `from cv_bridge import CvBridge` | 相同，无变化 |
| imgmsg_to_cv2 | `bridge.imgmsg_to_cv2(msg, "bgr8")` | 相同，无变化 |
| cv2_to_imgmsg | `bridge.cv2_to_imgmsg(img, "bgr8")` | 相同，无变化 |
| 时间戳 | 自动继承 | 建议手动设置 `out_msg.header = msg.header` |

**好消息**：`cv_bridge` 的核心 API 在 ROS2 中基本保持不变！

---

## 第三部分：图像传输优化

### 1. 压缩图像传输

ROS2 支持图像压缩以节省带宽：

#### 安装

```bash
sudo apt install ros-humble-compressed-image-transport
sudo apt install ros-humble-image-transport-plugins
```

#### 使用

```python
# 发布者端 - 自动支持压缩
self.image_pub = self.create_publisher(
    Image,
    '/camera/image_raw',
    10)

# 订阅者端 - 可以订阅压缩话题
self.image_sub = self.create_subscription(
    CompressedImage,
    '/camera/image_raw/compressed',
    self.compressed_callback,
    10)
```

### 2. QoS 设置（重要）

ROS2 引入了 QoS，对于图像话题建议使用：

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# 传感器数据 QoS（推荐用于相机）
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

self.image_sub = self.create_subscription(
    Image,
    '/camera/image_raw',
    self.image_callback,
    sensor_qos)
```

---

## 第四部分：常见相机使用场景迁移

### 场景 1：机械臂视觉伺服

#### ROS1 架构

```
相机节点 → 图像处理节点 → 目标检测节点 → 机械臂控制节点
```

#### ROS2 迁移要点

1. **每个节点都需要继承 `Node` 类**
2. **使用 `image_transport` 优化图像传输**
3. **配置正确的 QoS 策略**
4. **考虑使用组合节点（Component）提高性能**

#### 示例：组合节点

```python
# ROS2 支持多个节点在同一进程中运行，减少通信开销
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()

    camera_node = CameraNode()
    processor_node = ImageProcessorNode()
    detector_node = ObjectDetectorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)
    executor.add_node(processor_node)
    executor.add_node(detector_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()
```

### 场景 2：多相机同步

#### ROS1（使用 message_filters）

```python
import message_filters
from sensor_msgs.msg import Image

# 同步两个相机
left_sub = message_filters.Subscriber('/left/image_raw', Image)
right_sub = message_filters.Subscriber('/right/image_raw', Image)

ts = message_filters.TimeSynchronizer([left_sub, right_sub], 10)
ts.registerCallback(self.stereo_callback)
```

#### ROS2（message_filters 已移植）

```python
import message_filters
from sensor_msgs.msg import Image

class StereoNode(Node):
    def __init__(self):
        super().__init__('stereo_node')

        # 同步订阅
        left_sub = message_filters.Subscriber(self, Image, '/left/image_raw')
        right_sub = message_filters.Subscriber(self, Image, '/right/image_raw')

        ts = message_filters.TimeSynchronizer([left_sub, right_sub], 10)
        ts.registerCallback(self.stereo_callback)

    def stereo_callback(self, left_msg, right_msg):
        self.get_logger().info('Synchronized stereo images received')
```

**注意**：ROS2 中 `message_filters.Subscriber` 第一个参数是 node 对象！

---

## 第五部分：话题名称变化

### 标准相机话题

ROS2 保持了与 ROS1 相同的话题命名规范：

```
/camera/image_raw              # 原始图像
/camera/image_raw/compressed   # 压缩图像
/camera/camera_info            # 相机标定信息
/camera/depth/image_raw        # 深度图像（深度相机）
```

### 查看话题

```bash
# ROS1
rostopic list
rostopic echo /camera/image_raw
rostopic hz /camera/image_raw

# ROS2
ros2 topic list
ros2 topic echo /camera/image_raw
ros2 topic hz /camera/image_raw
```

---

## 第六部分：相机标定

### ROS1

```bash
sudo apt install ros-noetic-camera-calibration
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw
```

### ROS2

```bash
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 --ros-args --remap image:=/camera/image_raw
```

标定结果文件（`camera.yaml`）格式保持兼容。

---

## 第七部分：性能优化建议

### 1. 使用 Intra-process 通信

ROS2 支持进程内零拷贝通信：

```python
# 在 Node 构造函数中
super().__init__('node_name',
                 allow_undeclared_parameters=True,
                 automatically_declare_parameters_from_overrides=True,
                 enable_rosout=True,
                 start_parameter_services=True,
                 parameter_overrides=[])

# 发布者和订阅者使用 intra-process
self.image_pub = self.create_publisher(
    Image,
    '/camera/image_raw',
    10,
    qos_profile=sensor_qos,
    publisher_options=rclpy.publisher.PublisherOptions(use_intra_process_comms=True))
```

### 2. 图像分辨率调整

对于机械臂控制，通常不需要高分辨率：

```python
parameters=[{
    'image_width': 640,    # 降低到 640x480
    'image_height': 480,
    'framerate': 30.0,     # 30fps 通常足够
}]
```

### 3. 使用 GPU 加速

如果使用深度学习视觉处理：

```bash
# 考虑使用 ROS2 + Isaac ROS（NVIDIA）
sudo apt install ros-humble-isaac-ros-*
```

---

## 第八部分：与 Piper 机械臂集成

### 示例：视觉引导抓取

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')

        self.bridge = CvBridge()

        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # 订阅机械臂状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/puppet/joint_states',
            self.joint_callback,
            10)

        # 发布目标位姿
        self.pose_pub = self.create_publisher(
            Pose,
            '/target_pose',
            10)

    def image_callback(self, msg):
        # 图像处理 - 检测目标
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 示例：简单的颜色检测
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (100, 100, 100), (130, 255, 255))

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 找到最大轮廓
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # 转换为机械臂坐标并发布
                target_pose = self.pixel_to_pose(cx, cy)
                self.pose_pub.publish(target_pose)

    def joint_callback(self, msg):
        # 处理机械臂反馈
        pass

    def pixel_to_pose(self, x, y):
        # 相机坐标到机械臂坐标的转换
        pose = Pose()
        # ... 坐标转换逻辑
        return pose

def main():
    rclpy.init()
    node = VisualServoNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 第九部分：常见问题

### 1. 图像延迟高

**解决方案**：
- 使用 `BEST_EFFORT` QoS
- 降低图像分辨率
- 使用压缩传输
- 启用 intra-process 通信

### 2. 图像话题订阅不到

**检查 QoS 兼容性**：

```bash
# 查看发布者 QoS
ros2 topic info /camera/image_raw --verbose

# 订阅时匹配 QoS
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
```

### 3. cv_bridge 编译错误

```bash
# 确保安装了 cv_bridge
sudo apt install ros-humble-cv-bridge

# Python 环境
pip3 install opencv-python
```

---

## 迁移检查清单

- [ ] 相机驱动包已更新到 ROS2 版本
- [ ] Launch 文件改为 Python 格式
- [ ] 图像处理节点改用 `rclpy.node.Node`
- [ ] QoS 策略配置正确
- [ ] cv_bridge 正常工作
- [ ] 图像话题延迟可接受
- [ ] 多相机同步（如需要）工作正常
- [ ] 与机械臂节点通信正常
- [ ] 相机标定文件兼容

---

## 总结

相机相关的 ROS2 迁移相对简单：

✅ **好消息**：
- cv_bridge API 基本不变
- 主流相机驱动都已移植
- 图像处理库（OpenCV）无需修改

⚠️ **需要注意**：
- QoS 配置（新增）
- message_filters 使用略有变化
- 节点需要继承 Node 类

**预计迁移时间**：0.5-2 天（取决于视觉处理复杂度）

---

## 推荐资源

- [ROS2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [cv_bridge Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/cv_bridge.html)
- [ROS2 QoS](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
