import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    package_dir = get_package_share_directory('dog12a04u03_description')

    urdf_file = LaunchConfiguration('urdf_file')
    xacro_args = LaunchConfiguration('xacro_args')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(package_dir, 'urdf', 'dog12a04u03_description.xacro'),
        description='URDF/Xacro file for the robot')
    declare_xacro_cmd = DeclareLaunchArgument(
        'xacro_args',
        default_value="",
        description='Arguments for xacro')
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_dir, 'rviz', 'default.rviz'),
        description='Full path to the RVIZ config file to use')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ GUI')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_args, " ", urdf_file]),
        value_type=str
    )

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

    ld = LaunchDescription()
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_xacro_cmd)
    ld.add_action(declare_rviz_cfg_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld
