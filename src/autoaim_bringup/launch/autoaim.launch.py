import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    camera_config = os.path.join(
        get_package_share_directory('autoaim_camera'), 'config', 'config.yaml')
    detector_config = os.path.join(
        get_package_share_directory('autoaim_detector'), 'config', 'detector.yaml')

    return LaunchDescription([
        ComposableNodeContainer(
            name='autoaim_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='autoaim_camera',
                    plugin='CameraNode::CameraNode',
                    name='autoaim_camera',
                    parameters=[camera_config],
                ),
                ComposableNode(
                    package='autoaim_detector',
                    plugin='DetectorNode::DetectorNode',
                    name='autoaim_detector',
                    parameters=[detector_config],
                ),
            ],
        ),
    ])
