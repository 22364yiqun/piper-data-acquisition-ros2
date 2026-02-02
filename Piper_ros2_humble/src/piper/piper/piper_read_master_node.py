#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为读取主臂发送的消息，当机械臂设置为主臂，主臂只发送关节角消息和控制指令
from piper_sdk import *

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class C_PiperRosNode(Node):
    """机械臂ros节点
    """
    def __init__(self) -> None:
        super().__init__('piper_read_master_node')

        # 声明并获取参数
        self.declare_parameter('can_port', 'can0')
        self.can_port = self.get_parameter('can_port').value
        self.get_logger().info(f'can_port is {self.can_port}')

        # 发布者
        self.joint_std_pub_master = self.create_publisher(JointState, '/master/joint_states', 1)

        self.joint_state_master = JointState()
        self.joint_state_master.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_master.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0]
        self.joint_state_master.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0]
        self.joint_state_master.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0]

        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

    def Pubilsh(self):
        """机械臂消息发布
        """
        while rclpy.ok():
            self.joint_state_master.header.stamp = rospy.Time.now()
            # Here, you can set the joint positions to any value you want
            joint_0 = self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000 * 0.017444
            joint_1 = self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000 * 0.017444
            joint_2 = self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000 * 0.017444
            joint_3 = self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000 * 0.017444
            joint_4 = self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000 * 0.017444
            joint_5 = self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000 * 0.017444
            joint_6 = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
            self.joint_state_master.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values

            self.joint_std_pub_master.publish(self.joint_state_master)
            time.sleep(1.0 / 200.0)  # 200 Hz

def main(args=None):
    rclpy.init(args=args)
    piper_node = C_PiperRosNode()

    try:
        piper_node.Publish()
    except KeyboardInterrupt:
        pass
    finally:
        piper_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
