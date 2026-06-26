"""bringup.launch.py — start bridge + IMU + controller with shared params."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rc_car'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(package='rc_car', executable='esp32_bridge',
             name='esp32_bridge', parameters=[params], output='screen'),
        Node(package='rc_car', executable='imu_node',
             name='imu_node', parameters=[params], output='screen'),
        Node(package='rc_car', executable='controller_node',
             name='controller_node', parameters=[params], output='screen'),
    ])
