"""Standalone physical base-driver owner; Bunker is the supported baseline."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction

_SHARE = Path(get_package_share_directory('agt_robot_bringup'))
sys.path.insert(0, str(_SHARE / 'tools'))
from robot_hardware_components import common_arguments, component_actions


def generate_launch_description():
    return LaunchDescription([
        *common_arguments(),
        OpaqueFunction(function=lambda context: component_actions(context, 'base')),
    ])
