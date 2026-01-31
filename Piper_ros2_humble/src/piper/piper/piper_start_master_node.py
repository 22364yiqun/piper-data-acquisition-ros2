#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为打开主臂读取
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from piper_sdk import *
from piper_sdk import C_PiperInterface


class C_PiperRosNode(Node):
    """机械臂ros节点
    """
    def __init__(self) -> None:
        super().__init__('piper_start_all_node')

        # 声明并获取参数
        self.declare_parameter('can_port', 'can0')
        self.can_port = self.get_parameter('can_port').value
        self.get_logger().info(f'can_port is {self.can_port}')

        # 默认模式为0，读取主从臂消息
        self.joint_std_pub_master = self.create_publisher(JointState, '/master/joint_states', 1)
        
        # 主臂消息
        self.joint_state_master = JointState()
        self.joint_state_master.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_master.position = [0.0] * 7
        self.joint_state_master.velocity = [0.0] * 7
        self.joint_state_master.effort = [0.0] * 7

        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

    def Pubilsh(self):
        """机械臂消息发布
        """
        # enable_flag = False
        # # 设置超时时间（秒）
        # timeout = 5
        # # 记录进入循环前的时间
        # start_time = time.time()
        # elapsed_time_flag = False
        while rclpy.ok():
            # 发布主臂消息
            self.PublishMasterArmJointAndGripper()
            time.sleep(1.0 / 200.0)  # 200 Hz
    
    def PublishMasterArmJointAndGripper(self):
        # 主臂控制消息
        self.joint_state_master.header.stamp = self.get_clock().now().to_msg()
        joint_0:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
        self.joint_state_master.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_std_pub_master.publish(self.joint_state_master)

def main(args=None):
    rclpy.init(args=args)
    piper_ms = C_PiperRosNode()

    try:
        piper_ms.Publish()
    except KeyboardInterrupt:
        pass
    finally:
        piper_ms.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

