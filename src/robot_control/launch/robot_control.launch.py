# ============================================================
# robot_control.launch.py
# ============================================================
# Robot Control 관련 ROS2 Node를
# 한 번에 실행하기 위한 Launch 파일.
# ============================================================


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
        Node(
            package='robot_control',
            executable='motion_utils',
            name='motion_utils',
            output='screen',
        ),
    ])