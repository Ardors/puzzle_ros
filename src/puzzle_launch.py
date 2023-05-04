from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planner',
            executable='planner',
            name='planner'
        ),
        Node(
            package='vision',
            executable='vision',
            name='vision'
        ),
        Node(
            package='gui',
            executable='gui',
            name='gui'
        )
    ])