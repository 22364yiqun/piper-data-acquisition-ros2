from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 声明参数，这样你可以在命令行通过 mode:=1 来修改它
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='0',
        description='0: 读取主从臂消息; 1: 控制从臂'
    )
    
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable', default_value='false',
        description='是否自动上电使能'
    )

    # 2. 获取参数值的引用
    mode = LaunchConfiguration('mode')
    auto_enable = LaunchConfiguration('auto_enable')

    # 3. 定义左臂节点配置
    piper_left_node = Node(
        package='piper',              # 你的包名
        executable='piper_start_ms_node',  # 你的 Python 脚本文件名
        name='piper_left',            # 节点运行时的名字
        output='screen',
        parameters=[{
            'can_port': 'can_left',   # 对应左臂的 CAN 接口
            'mode': mode,
            'auto_enable': auto_enable
        }],
        remappings=[
            ('/puppet/arm_status', '/puppet/arm_status_left'),
            ('/puppet/joint_states', '/puppet/joint_left'),
            ('/master/joint_states', '/master/joint_left'),
            ('/puppet/end_pose', '/puppet/end_pose_left'),
            ('/pos_cmd', '/puppet/pos_cmd_left'),
            ('/puppet/end_pose_euler', '/puppet/end_pose_euler_left'),
        ]
    )

    # 4. 定义右臂节点配置
    piper_right_node = Node(
        package='piper',
        executable='piper_start_ms_node',
        name='piper_right',
        output='screen',
        parameters=[{
            'can_port': 'can_right',  # 对应右臂的 CAN 接口
            'mode': mode,
            'auto_enable': auto_enable
        }],
        remappings=[
            ('/puppet/arm_status', '/puppet/arm_status_right'),
            ('/puppet/joint_states', '/puppet/joint_right'),
            ('/master/joint_states', '/master/joint_right'),
            ('/puppet/end_pose', '/puppet/end_pose_right'),
            ('/pos_cmd', '/puppet/pos_cmd_right'),
            ('/puppet/end_pose_euler', '/puppet/end_pose_euler_right'),
        ]
    )

    # 5. 返回启动描述符
    return LaunchDescription([
        mode_arg,
        auto_enable_arg,
        piper_left_node,
        piper_right_node
    ])