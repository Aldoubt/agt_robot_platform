"""The only physical driver and robot_state_publisher lifecycle owner."""

from pathlib import Path
import json

import jsonschema
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include(package, launch_file, enabled, arguments=None, launch_dir='launch'):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / launch_dir / launch_file)),
        condition=IfCondition(enabled),
        launch_arguments=(arguments or {}).items(),
    )


def _validate_profile(context):
    name = LaunchConfiguration('robot').perform(context)
    share = Path(get_package_share_directory('agt_robot_description'))
    profile_dir = share / 'config' / 'robot_profiles'
    profile = yaml.safe_load((profile_dir / f'{name}.yaml').read_text(encoding='utf-8'))
    schema = json.loads((profile_dir / 'schema.json').read_text(encoding='utf-8'))
    jsonschema.validate(profile, schema)
    if profile['robot_id'] != name:
        raise RuntimeError(f'robot profile ID mismatch: {name}')
    for key in ('xacro', 'calibration'):
        relative = Path(profile['description'][key])
        # With colcon --symlink-install, package files legitimately resolve into
        # src/, outside the install share directory. Validate the profile path
        # itself before following those installation links.
        if relative.is_absolute() or '..' in relative.parts:
            raise RuntimeError(f'robot profile {key} escapes package: {relative}')
        candidate = share / relative
        if not candidate.is_file():
            raise RuntimeError(f'robot profile {key} is missing or outside package: {candidate}')
    return []


def _mid360_actions(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('enable_mid360').lower() != 'true':
        return []
    mode = value('mid360_driver_mode')
    if mode == 'vendor_launch':
        return [_include('livox_ros_driver2', 'msg_MID360_launch.py', 'true',
                         launch_dir='launch_ROS2')]
    if mode != 'mapping_custom':
        raise RuntimeError(f'unknown MID360 driver mode: {mode}')
    config = Path(value('mapping_livox_config')).expanduser().resolve()
    if not config.is_file():
        raise RuntimeError(f'mapping_livox_config does not exist: {config}')
    return [Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen', parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': float(value('mapping_publish_freq')),
            'output_data_type': 0,
            'frame_id': ParameterValue(value('mapping_frame_id'), value_type=str),
            'lvx_file_path': '',
            'user_config_path': ParameterValue(str(config), value_type=str),
            'cmdline_input_bd_code': 'livox0000000001',
        }])]


def generate_launch_description():
    description_share = Path(get_package_share_directory('agt_robot_description'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='bunker_v1', choices=['bunker_v1']),
        OpaqueFunction(function=_validate_profile),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_robot_description', default_value='true'),
        DeclareLaunchArgument(
            'robot_description_calibration_file',
            default_value=str(description_share / 'config' / 'field_acceptance.yaml')),
        DeclareLaunchArgument('enable_mid360', default_value='true'),
        DeclareLaunchArgument('mid360_driver_mode', default_value='vendor_launch',
                              choices=['vendor_launch', 'mapping_custom']),
        DeclareLaunchArgument('mapping_livox_config', default_value=''),
        DeclareLaunchArgument('mapping_publish_freq', default_value='10.0'),
        DeclareLaunchArgument('mapping_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('enable_bunker_can', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_camera_gimbal', default_value='true'),
        DeclareLaunchArgument('bunker_can_port', default_value='can0'),
        DeclareLaunchArgument('camera_device_path', default_value='/dev/video0'),
        # Stop-and-shoot inspection does not need a continuous 30 FPS decode.
        # Keep full-HD capture while reducing CPU pressure on Batch-LIO.
        DeclareLaunchArgument('camera_image_width', default_value='1920'),
        DeclareLaunchArgument('camera_image_height', default_value='1080'),
        DeclareLaunchArgument('camera_fps', default_value='10.0'),
        DeclareLaunchArgument('camera_pixel_format', default_value='mjpeg2rgb'),
        DeclareLaunchArgument(
            'gimbal_port_name',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),

        # The robot description is started only here.
        _include(
            'agt_robot_description', 'display.launch.py',
            LaunchConfiguration('enable_robot_description'), {
                'robot': LaunchConfiguration('robot'),
                'calibration_file': LaunchConfiguration(
                    'robot_description_calibration_file'),
                'start_rviz': 'false',
            }),
        OpaqueFunction(function=_mid360_actions),
        _include(
            'bunker_base', 'bunker_base.launch.py',
            LaunchConfiguration('enable_bunker_can'), {
                'use_sim_time': use_sim_time,
                'port_name': LaunchConfiguration('bunker_can_port'),
                'odom_topic_name': '/wheel/odom',
                'publish_odom_tf': 'false',
            }),
        _include(
            'agt_asensing_driver', 'asensing.launch.py',
            LaunchConfiguration('enable_rtk')),
        _include(
            'autolabor_c1_bringup', 'autolabor_c1.launch.py',
            LaunchConfiguration('enable_camera_gimbal'), {
                'device_path': LaunchConfiguration('camera_device_path'),
                'image_width': LaunchConfiguration('camera_image_width'),
                'image_height': LaunchConfiguration('camera_image_height'),
                'fps': LaunchConfiguration('camera_fps'),
                'pixel_format': LaunchConfiguration('camera_pixel_format'),
                'port_name': LaunchConfiguration('gimbal_port_name'),
                'gui': 'false',
            }),
    ])
