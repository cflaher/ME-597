from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_2',
            executable='talker',
            name='publisher_member_function',
            output='screen'
        ),
        Node(
            package='task_2',
            executable='service',
            name='service',
            output='screen'
        )
    ])