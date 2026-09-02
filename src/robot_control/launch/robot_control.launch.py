from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='robot_node',
            name='robot_node',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='assembly_controller',
            name='assembly_controller',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='insertion_controller',
            name='insertion_controller',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='motion_planner',
            name='motion_planner',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='target_manager',
            name='target_manager',
            output='screen'
        ),
    ])