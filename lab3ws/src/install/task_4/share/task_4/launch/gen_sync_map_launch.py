from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    turtlebot4_navigation_pkg = FindPackageShare('turtlebot4_navigation').find('turtlebot4_navigation')
    turtlebot4_viz_pkg = FindPackageShare('turtlebot4_viz').find('turtlebot4_viz')

    slam_launch_file = os.path.join(turtlebot4_navigation_pkg, 'launch', 'slam.launch.py')
    view_robot_launch_file = os.path.join(turtlebot4_viz_pkg, 'launch', 'view_robot.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_robot_launch_file)
        )
    ])