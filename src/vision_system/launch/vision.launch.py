from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_system',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),
        Node(
            package='vision_system',
            executable='coordinate_transform',
            name='coordinate_transform',
            output='screen'
        ),
        Node(
            package='vision_system',
            executable='image_processing',
            name='image_processing',
            output='screen'
        ),
        Node(
            package='vision_system',
            executable='shape_detector',
            name='shape_detector',
            output='screen'
        ),
    ])