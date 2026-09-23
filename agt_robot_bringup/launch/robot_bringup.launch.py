"""Compatibility alias for the single robot hardware owner."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agt_robot_bringup'), 'launch',
                'robot_hardware.launch.py']))),
    ])
