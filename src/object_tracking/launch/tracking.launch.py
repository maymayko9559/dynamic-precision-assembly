from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_tracking',
            executable='tracking_node',
            name='tracking_node',
            output='screen'
        ),
        Node(
            package='object_tracking',
            executable='kalman_filter',
            name='kalman_filter',
            output='screen'
        ),
        Node(
            package='object_tracking',
            executable='motion_predictor',
            name='motion_predictor',
            output='screen'
        ),
        Node(
            package='object_tracking',
            executable='velocity_estimator',
            name='velocity_estimator',
            output='screen'
        ),
    ])