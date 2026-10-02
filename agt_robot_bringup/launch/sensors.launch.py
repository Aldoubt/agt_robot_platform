"""Compose profile-enabled LiDAR, RTK and camera atomics; no base driver."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction

_SHARE = Path(get_package_share_directory('agt_robot_bringup'))
sys.path.insert(0, str(_SHARE / 'tools'))
from robot_hardware_components import aggregator_actions, common_arguments


def generate_launch_description():
    return LaunchDescription([
        *common_arguments(),
        OpaqueFunction(function=lambda context: aggregator_actions(context, sensors_only=True)),
    ])
