from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='task_1', executable='minimal_publisher', output='screen'),
        Node(package='task_1', executable='minimal_subscriber', output='screen'),
    ])
