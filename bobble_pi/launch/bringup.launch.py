"""Brings up the MPU6050 IMU node and the balance controller node.

    ros2 launch bobble_pi bringup.launch.py

Teleop is launched separately (it needs its own interactive terminal):

    ros2 run bobble_pi keyboard_control_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('bobble_pi'), 'config', 'balance_control.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the balance_control.yaml parameter file',
    )
    config_file = LaunchConfiguration('config_file')

    imu_node = Node(
        package='bobble_pi',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[config_file],
    )

    balance_controller_node = Node(
        package='bobble_pi',
        executable='balance_controller_node',
        name='balance_controller_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        config_arg,
        imu_node,
        balance_controller_node,
    ])
