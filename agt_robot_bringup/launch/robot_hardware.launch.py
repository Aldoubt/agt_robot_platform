"""Compatibility hardware aggregator; device owners live in atomic launches."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import sys

_SHARE = Path(get_package_share_directory('agt_robot_bringup'))
sys.path.insert(0, str(_SHARE / 'tools'))
from robot_hardware_components import aggregator_actions, common_arguments


def _hardware_actions(context):
    return aggregator_actions(context)


def generate_launch_description():
    return LaunchDescription([
        *common_arguments(),
        OpaqueFunction(function=_hardware_actions),
    ])
