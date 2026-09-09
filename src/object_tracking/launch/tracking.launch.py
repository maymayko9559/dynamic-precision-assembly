import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('object_tracking'),
        'config',
        'tracking_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='object_tracking',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
