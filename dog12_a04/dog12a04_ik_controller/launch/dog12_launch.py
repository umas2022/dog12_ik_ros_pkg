# launch/robot_complete.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_dir = get_package_share_directory('dog12a04u03_description')
    
    # -------- 复用你已有的Launch配置 --------
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(package_dir, 'urdf', 'dog12a04u03_description.xacro'),
        description='URDF/Xacro file for the robot')
    
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_dir, 'rviz', 'default.rviz'),
        description='Full path to the RVIZ config file to use')
        
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ GUI')

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # -------- 添加你的自定义节点 --------
    
    # 1. 启动 control_ik.py 节点
    #    这个节点负责发布 /joint_states，替代了 joint_state_publisher
    start_ik_controller_cmd = Node(
        package='dog12a04_ik_controller',
        executable='ik_controller',  # 使用在 setup.py 中定义的可执行文件名
        # name='combined_motion_controller', # 脚本中CombinedMotionController已经定义过了name
        output='screen'
    )

    # 2. 启动 pub_foot_traj.py 节点
    start_foot_traj_publisher_cmd = Node(
        package='dog12a04_ik_controller',
        executable='foot_trajectory_publisher', # 使用在 setup.py 中定义的可执行文件名
        name='foot_trajectory_publisher_node',
        output='screen'
    )
    

    # -------- 整合到LaunchDescription中 --------
    ld = LaunchDescription()
    
    # 添加声明
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_rviz_cfg_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # 添加基础节点 (Rviz, Robot State Publisher)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    
    # 添加你的自定义节点
    ld.add_action(start_ik_controller_cmd)
    ld.add_action(start_foot_traj_publisher_cmd)

    return ld