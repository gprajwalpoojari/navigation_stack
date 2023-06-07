from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_generation_ros2',
            executable='trajectory_rviz2',
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='sim'
        )
    ])