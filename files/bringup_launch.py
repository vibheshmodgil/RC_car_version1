"""bringup_launch.py — bridge + IMU + controller + rosbridge (for the dashboard).

Starts everything the car needs plus a rosbridge WebSocket server on :9090 so a
browser on your laptop can subscribe to topics, tune PID params, and send
maneuvers without installing ROS.

    ros2 launch rc_car bringup_launch.py

Requires: sudo apt install ros-jazzy-rosbridge-suite
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rc_car'), 'config', 'params.yaml')

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')))

    return LaunchDescription([
        Node(package='rc_car', executable='esp32_bridge',
             name='esp32_bridge', parameters=[params], output='screen'),
        Node(package='rc_car', executable='imu_node',
             name='imu_node', parameters=[params], output='screen'),
        Node(package='rc_car', executable='controller_node',
             name='controller_node', parameters=[params], output='screen'),
        rosbridge,
    ])
