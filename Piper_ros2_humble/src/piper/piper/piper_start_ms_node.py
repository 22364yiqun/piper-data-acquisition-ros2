#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为同时打开主从臂的节点，通过mode参数控制是读取还是控制
# 默认认为从臂有夹爪
# mode为0时为发送主从臂消息，
# mode为1时为控制从臂，不发送主臂消息，此时如果要控制从臂，需要给主臂的topic发送消息

import rclpy
from rclpy import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from piper_msgs.msg import PiperStatusMsg, PosCmd
from geometry_msgs.msg import Pose,PoseStamped
from std_srvs.srv import Trigger, TriggerResponse

import time
import threading
import argparse
import math

from piper_sdk import *
from piper_sdk import C_PiperInterface

# from tf.transformations import quaternion_from_euler  # 用于欧拉角到四元数的转换
import tf_transformations

def check_ros_master():
    try:
        rosnode.rosnode_ping('rosout', max_count=1, verbose=False)
        rospy.loginfo("ROS Master is running.")
    except rosnode.ROSNodeIOException:
        rospy.logerr("ROS Master is not running.")
        raise RuntimeError("ROS Master is not running.")

class C_PiperRosNode(Node):
    """机械臂ros节点
    """
    def __init__(self) -> None:
        super().__init__('piper_start_all_node')

        # 声明并获取参数
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('mode', 0)
        self.declare_parameter('auto_enable', False)

        self.can_port = self.get_parameter('can_port').value
        self.mode = self.get_parameter('mode').value
        self.auto_enable = self.get_parameter('auto_enable').value and self.mode == 1

        self.get_logger().info(f'can_port is {self.can_port}')
        self.get_logger().info(f'mode is {self.mode}')
        self.get_logger().info(f'auto_enable is {self.auto_enable}')

        self.gripper_exist = True
        # publish
        self.joint_std_pub_puppet = self.create_publisher(JointState, '/puppet/joint_states', 1)
          # 默认模式为0，读取主从臂消息
        if self.mode == 0:
            self.joint_std_pub_master = self.create_publisher(JointState, '/master/joint_states', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, '/puppet/arm_status', 1)
        self.end_pose_pub = self.create_publisher(PoseStamped, '/puppet/end_pose', 1)
        self.end_pose_euler_pub = self.create_publisher(PosCmd, '/puppet/end_pose_euler', 1)

        self.__enable_flag = False

        # 从臂消息
        self.joint_state_slave = JointState()
        self.joint_state_slave.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_slave.position = [0.0] * 7
        self.joint_state_slave.velocity = [0.0] * 7
        self.joint_state_slave.effort = [0.0] * 7
        # 主臂消息
        self.joint_state_master = JointState()
        self.joint_state_master.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_master.position = [0.0] * 7
        self.joint_state_master.velocity = [0.0] * 7
        self.joint_state_master.effort = [0.0] * 7

        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()
        # service
        str_can_port = str(self.can_port)
        # 主臂单独回零
        self.master_go_zero_service = self.create_service(
              Trigger, f'/{str_can_port}/go_zero_master', self.handle_master_go_zero_service)
        # 主从臂一起回零
        self.master_slave_go_zero_service = self.create_service(
              Trigger, f'/{str_can_port}/go_zero_master_slave', self.handle_master_slave_go_zero_service)
        # Restoring the master and slave
        self.restore_ms_mode_service = self.create_service(
              Trigger, f'/{str_can_port}/restore_ms_mode', self.handle_restore_ms_mode_service)

        # 模式为1的时候，订阅控制消息
        if self.mode == 1:
            self.pos_sub = self.create_subscription(PosCmd, '/pos_cmd', self.pos_callback, 1)
            self.joint_sub = self.create_subscription(JointState, '/master/joint_states', self.joint_callback, 1)
            self.enable_sub = self.create_subscription(Bool, '/enable_flag', self.enable_callback, 1)


    def GetEnableFlag(self):
        return self.__enable_flag

    def Pubilsh(self):
        """机械臂消息发布
        """
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False

        while rclpy.ok():
            if self.auto_enable and self.mode == 1:
                while not enable_flag:
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("使能状态:",enable_flag)
                    if(enable_flag):
                        self.__enable_flag = True
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0,1000,0x02, 0)
                    self.piper.GripperCtrl(0,1000,0x01, 0)
                    print("--------------------")
                    # 检查是否超过超时时间
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    
            if(elapsed_time_flag):
                print("程序自动使能超时,退出程序")
                exit(0)
            # 发布消息
            self.PublishSlaveArmJointAndGripper()
            self.PublishSlaveArmState()
            self.PublishSlaveArmEndPose()
            # 模式为0的时候，发布主臂消息
            if(self.mode == 0):
                self.PublishMasterArmJointAndGripper()

            time.sleep(1.0 / 200.0)  # 200 Hz
    
    def PublishSlaveArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)
    
    def PublishSlaveArmEndPose(self):
        # 末端位姿
        endpos = PoseStamped()
        endpos.pose.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis/1000000
        endpos.pose.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis/1000000
        endpos.pose.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis/1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis/1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis/1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis/1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        endpos.pose.orientation.x = quaternion[0]
        endpos.pose.orientation.y = quaternion[1]
        endpos.pose.orientation.z = quaternion[2]
        endpos.pose.orientation.w = quaternion[3]
        # 为末端位姿增加时间戳
        endpos.header.stamp = self.get_clock().now().to_msg()
        self.end_pose_pub.publish(endpos)
        
        end_pose_euler = PosCmd()
        end_pose_euler.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis/1000000
        end_pose_euler.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis/1000000
        end_pose_euler.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis/1000000
        end_pose_euler.roll = roll
        end_pose_euler.pitch = pitch
        end_pose_euler.yaw = yaw
        end_pose_euler.gripper = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        end_pose_euler.mode1 = 0
        end_pose_euler.mode2 = 0
        self.end_pose_euler_pub.publish(end_pose_euler)
    
    def PublishSlaveArmJointAndGripper(self):
        # 从臂反馈消息
        self.joint_state_slave.header.stamp = self.get_clock().now().to_msg()
        joint_0:float = (self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        vel_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed/1000
        vel_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed/1000
        vel_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed/1000
        vel_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed/1000
        vel_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed/1000
        vel_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_state_slave.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_state_slave.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]  # Example values
        self.joint_state_slave.effort[6] = effort_6
        self.joint_std_pub_puppet.publish(self.joint_state_slave)
    
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

    def pos_callback(self, pos_data):
        """机械臂末端位姿订阅回调函数

        Args:
            pos_data (): 
        """
        factor = 180 / 3.1415926
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor) 
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        self.get_logger().info("Received PosCmd:")
        self.get_logger().info(f"x: {x}")
        self.get_logger().info(f"y: {y}")
        self.get_logger().info(f"z: {z}")
        self.get_logger().info(f"roll: {rx}")
        self.get_logger().info(f"pitch: {ry}")
        self.get_logger().info(f"yaw: {rz}")
        self.get_logger().info(f"gripper: {pos_data.gripper}")
        self.get_logger().info(f"mode1: {pos_data.mode1}")
        self.get_logger().info(f"mode2: {pos_data.mode2}")
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)
            self.piper.EndPoseCtrl(x, y, z, 
                                    rx, ry, rz)
            gripper = round(pos_data.gripper*1000*1000)
            if(pos_data.gripper>80000): gripper = 80000
            if(pos_data.gripper<0): gripper = 0
            if(self.gripper_exist):
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)
    
    def joint_callback(self, joint_data):
        """机械臂关节角回调函数

        Args:
            joint_data (): 
        """
        factor = 57324.840764 #1000*180/3.14
        self.get_logger().info("Received Joint States:")
        self.get_logger().info(f"joint_0: {joint_data.position[0]*1}")
        self.get_logger().info(f"joint_1: {joint_data.position[1]*1}")
        self.get_logger().info(f"joint_2: {joint_data.position[2]*1}")
        self.get_logger().info(f"joint_3: {joint_data.position[3]*1}")
        self.get_logger().info(f"joint_4: {joint_data.position[4]*1}")
        self.get_logger().info(f"joint_5: {joint_data.position[5]*1}")
        self.get_logger().info(f"joint_6: {joint_data.position[6]*1}")
        joint_0 = round(joint_data.position[0]*factor)
        joint_1 = round(joint_data.position[1]*factor)
        joint_2 = round(joint_data.position[2]*factor)
        joint_3 = round(joint_data.position[3]*factor)
        joint_4 = round(joint_data.position[4]*factor)
        joint_5 = round(joint_data.position[5]*factor)
        joint_6 = round(joint_data.position[6]*1000*1000)
        if(joint_6>80000): joint_6 = 80000
        if(joint_6<0): joint_6 = 0
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_2(0x01, 0x01, 100)
            self.piper.JointCtrl(joint_0, joint_1, joint_2, 
                                    joint_3, joint_4, joint_5)
            self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x01, 100)
    
    def enable_callback(self, enable_flag:Bool):
        """机械臂使能回调函数

        Args:
            enable_flag (): 
        """
        self.get_logger().info("Received enable flag:")
        self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if(enable_flag.data):
            self.__enable_flag = True
            self.piper.EnableArm(7)
            self.piper.GripperCtrl(0,1000,0x02, 0)
            self.piper.GripperCtrl(0,1000,0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            self.piper.GripperCtrl(0,1000,0x00, 0)
    
    def handle_master_go_zero_service(self,requset,response):
        self.get_logger().info("-----------------------RESET---------------------------")
        self.get_logger().info(f"{self.can_port} send piper master go zero service")
        self.get_logger().info("-----------------------RESET---------------------------")
        self.piper.ReqMasterArmMoveToHome(1)
        response.success = True
        response.message = str({self.can_port}) + "send piper master go zero service success"
        self.get_logger().info(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    def handle_master_slave_go_zero_service(self,requset,response):
        self.get_logger().info("-----------------------RESET---------------------------")
        self.get_logger().info(f"{self.can_port} send piper master slave go zero service")
        self.get_logger().info("-----------------------RESET---------------------------")
        self.piper.ReqMasterArmMoveToHome(2)
        response.success = True
        response.message = str({self.can_port}) + "send piper master slave go zero service success"
        self.get_logger().info(f"Returning resetResponse: {response.success}, {response.message}")
        return response
    
    def handle_restore_ms_mode_service(self,requset,response):
        self.get_logger().info("-----------------------RESET---------------------------")
        self.get_logger().info(f"{self.can_port} send piper restore master slave mode service")
        self.get_logger().info("-----------------------RESET---------------------------")
        self.piper.ReqMasterArmMoveToHome(0)
        response.success = True
        response.message = str({self.can_port}) + "send piper restore master slave mode service success"
        self.get_logger().info(f"Returning resetResponse: {response.success}, {response.message}")
        return response

def main(args=None):
    rclpy.init(args=args)
    piper_node = C_PiperRosNode()

    # 使用MultiThreadedExecutor来并行处理回调和发布循环
    executor = MultiThreadedExecutor()
    executor.add_node(piper_node)

    # 在单独的线程中运行executor来处理订阅回调和服务
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # 在主线程中运行发布循环
        piper_node.Publish()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        piper_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

