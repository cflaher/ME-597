from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot4_navigation',
            executable='slam.launch',
            name='robot',
            output='screen'),
        Node(
            package='turtlebot4_viz',
            executable='view_robot.launch',
            name='robot',
            output='screen'),
        
    ])
