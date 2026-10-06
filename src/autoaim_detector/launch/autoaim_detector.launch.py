import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('autoaim_detector'), 'config', 'detector.yaml')

    return LaunchDescription([
        Node(
            package='autoaim_detector',
            executable='detector_node',
            output='screen',
            parameters=[config],
        ),
    ])
