from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            Node(package='task_5', executable='image_publisher', name='image_publisher', output='screen'),
            Node(package='task_5', executable='object_detector', name='object_detector', output='screen')      
    ])