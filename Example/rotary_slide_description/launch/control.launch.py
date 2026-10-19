import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('rotary_slide_description')
    description = xacro.process_file(os.path.join(pkg, 'urdf', 'rotary_slide.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'viewers', 'display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[os.path.join(pkg, 'config', 'controllers.yaml')],
             remappings=[('~/robot_description', '/robot_description'),
                         ('robot_description', '/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                        '--param-file', os.path.join(pkg, 'config', 'controllers.yaml')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager',
                        '--param-file', os.path.join(pkg, 'config', 'controllers.yaml')], output='screen'),
    ])
