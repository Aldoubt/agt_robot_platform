"""Profile-resolved actions shared by the atomic physical-device launches."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_BRINGUP_SHARE = Path(get_package_share_directory('agt_robot_bringup'))
sys.path.insert(0, str(_BRINGUP_SHARE / 'tools'))
import robot_config


def common_arguments():
    """Declarations shared by every hardware atomic and its aggregator."""
    managed = [DeclareLaunchArgument(
        name, default_value='', description=f'override; empty = robot config ({source}:{path})')
        for name, (source, path) in robot_config.MANAGED_ARGS.items()]
    return [
        DeclareLaunchArgument('robot_config', default_value=''),
        DeclareLaunchArgument('robot', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_robot_description', default_value='true'),
        DeclareLaunchArgument('robot_description_calibration_file', default_value=''),
        DeclareLaunchArgument('mapping_livox_config', default_value=''),
        DeclareLaunchArgument('mapping_publish_freq', default_value='10.0'),
        DeclareLaunchArgument('mapping_frame_id', default_value='livox_frame'),
        *managed,
    ]


def launch_arguments(context):
    """Resolve the selected whole-robot config and its managed arg values."""
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    description_share = Path(get_package_share_directory('agt_robot_description'))
    overrides = {name: value(name) for name in robot_config.MANAGED_ARGS}
    return robot_config.resolve(
        _BRINGUP_SHARE / 'config' / 'robots',
        description_share / 'config' / 'robot_profiles',
        robot_config=value('robot_config'), robot_profile=value('robot'),
        overrides=overrides, description_share=description_share)


def include(package, launch_file, arguments=None, launch_dir='launch'):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / launch_dir / launch_file)),
        launch_arguments=(arguments or {}).items())


def component_actions(context, component):
    """Return only the actions owned by one declared physical component."""
    resolved = launch_arguments(context)
    args = resolved.launch_args
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    use_sim_time = value('use_sim_time')

    if component == 'lidar':
        if args['enable_mid360'] != 'true':
            return []
        if args['mid360_driver_mode'] == 'vendor_launch':
            return [include('livox_ros_driver2', 'msg_MID360_launch.py', launch_dir='launch_ROS2')]
        if args['mid360_driver_mode'] != 'mapping_custom':
            raise RuntimeError(f"unknown MID360 driver mode: {args['mid360_driver_mode']}")
        config = Path(value('mapping_livox_config')).expanduser().resolve()
        if not config.is_file():
            raise RuntimeError(f'mapping_livox_config does not exist: {config}')
        return [Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen', parameters=[{
                'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                'publish_freq': float(value('mapping_publish_freq')),
                'output_data_type': 0,
                'frame_id': ParameterValue(value('mapping_frame_id'), value_type=str),
                'lvx_file_path': '', 'user_config_path': ParameterValue(str(config), value_type=str),
                'cmdline_input_bd_code': 'livox0000000001',
            }])]

    if component == 'base':
        if args['enable_yhs_can'] == 'true' or resolved.base_adapter == 'yhs':
            raise RuntimeError('YHS base startup is BLOCKED pending protocol and kinematics audit')
        if args['enable_bunker_can'] != 'true' or resolved.base_adapter != 'bunker':
            raise RuntimeError('base.launch requires an enabled, supported Bunker base profile')
        return [include('bunker_base', 'bunker_base.launch.py', {
            'use_sim_time': use_sim_time,
            'port_name': args['bunker_can_port'],
            'odom_topic_name': '/wheel/odom',
            'publish_odom_tf': 'false',
        })]

    if component == 'camera':
        if args['enable_camera_gimbal'] != 'true':
            return []
        return [include('autolabor_c1_bringup', 'autolabor_c1.launch.py', {
            'device_path': args['camera_device_path'],
            'image_width': args['camera_image_width'],
            'image_height': args['camera_image_height'],
            'fps': args['camera_fps'],
            'pixel_format': args['camera_pixel_format'],
            'port_name': args['gimbal_port_name'],
            'gui': 'false',
        })]

    if component == 'rtk':
        if args['enable_rtk'] != 'true':
            return []
        return [include('agt_asensing_driver', 'asensing.launch.py')]

    raise ValueError(f'unknown hardware component {component!r}')


def resolved_include_arguments(context, resolved=None):
    """Forward source selection and validated values to an included atomic."""
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    result = {
        'robot_config': value('robot_config'),
        'robot': value('robot'),
        'use_sim_time': value('use_sim_time'),
        'enable_robot_description': value('enable_robot_description'),
        'robot_description_calibration_file': value('robot_description_calibration_file'),
        'mapping_livox_config': value('mapping_livox_config'),
        'mapping_publish_freq': value('mapping_publish_freq'),
        'mapping_frame_id': value('mapping_frame_id'),
    }
    resolved = resolved or launch_arguments(context)
    result.update(resolved.launch_args)
    return result


def aggregator_actions(context, sensors_only=False):
    resolved = launch_arguments(context)
    args = resolved.launch_args
    values = resolved_include_arguments(context, resolved)
    actions = []
    if not sensors_only and LaunchConfiguration('enable_robot_description').perform(context).lower() == 'true':
        description_share = Path(get_package_share_directory('agt_robot_description'))
        calibration = LaunchConfiguration('robot_description_calibration_file').perform(context)
        calibration = calibration or str(
            description_share / 'config' / 'field_acceptance.yaml')
        actions.append(include('agt_robot_description', 'description.launch.py', {
            'robot': resolved.robot_profile,
            'calibration_file': calibration,
        }))
    if args['enable_mid360'] == 'true':
        actions.append(include('agt_robot_bringup', 'lidar.launch.py', values))
    if not sensors_only and (args['enable_bunker_can'] == 'true' or args['enable_yhs_can'] == 'true'):
        actions.append(include('agt_robot_bringup', 'base.launch.py', values))
    if args['enable_rtk'] == 'true':
        actions.append(include('agt_robot_bringup', 'rtk.launch.py', values))
    if args['enable_camera_gimbal'] == 'true':
        actions.append(include('agt_robot_bringup', 'camera.launch.py', values))
    return actions
