"""The only physical driver and robot_state_publisher lifecycle owner.

Device selection comes from a whole-robot config
(``config/robots/<robot_config>/robot.yaml`` + ``devices.yaml``) resolved by
``tools/robot_config.py``.  The Robot Profile (geometry/extrinsics) is owned by
agt_robot_description and referenced from robot.yaml.

Compatibility: ``robot:=bunker_v1`` without ``robot_config`` resolves to the
single supported config using that profile (``bunker_inspection``).  Managed
launch arguments default to "" meaning "take the value from the robot config";
an explicit value is an override that is validated (e.g. enabling a device that
is not installed, or a robot/robot_config mismatch, is a hard error).
Blocked configs (e.g. yhs_harvesting) fail before any node starts; there is no
fallback to another robot.  Hot switching is not supported: stop the stack first.
"""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_BRINGUP_SHARE = Path(get_package_share_directory('agt_robot_bringup'))
sys.path.insert(0, str(_BRINGUP_SHARE / 'tools'))
import robot_config  # noqa: E402  (installed with this package)


def _include(package, launch_file, arguments=None, launch_dir='launch'):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / launch_dir / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def _mid360_actions(value, args):
    if args['enable_mid360'] != 'true':
        return []
    mode = args['mid360_driver_mode']
    if mode == 'vendor_launch':
        return [_include('livox_ros_driver2', 'msg_MID360_launch.py', launch_dir='launch_ROS2')]
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


def _yhs_actions(args, use_sim_time):
    """Official YUHESEN TK-mid-ros2 driver + agt_yhs_adapter Twist bridge.

    The driver's relative topics are pinned under /yhs; its integrated odometry
    goes to /wheel/odom (diagnostics only, never navigation). The driver does not
    broadcast TF. High levels keep publishing Twist on /mux/cmd_vel.
    """
    gear = args.get('yhs_drive_gear', '')
    if gear in ('', 'None', 'null'):
        raise RuntimeError('yhs_drive_gear is not confirmed; refusing to start the YHS base')
    sim = ParameterValue(use_sim_time, value_type=bool)
    return [
        Node(package='yhs_can_control', executable='yhs_can_control_node',
             name='yhs_can_control_node', output='screen',
             parameters=[{'can_name': args['yhs_can_port'], 'use_sim_time': sim}],
             remappings=[('ctrl_cmd', '/yhs/ctrl_cmd'), ('io_cmd', '/yhs/io_cmd'),
                         ('free_ctrl_cmd', '/yhs/free_ctrl_cmd'),
                         ('chassis_info_fb', '/yhs/chassis_info_fb'),
                         ('odom', '/wheel/odom')]),
        Node(package='agt_yhs_adapter', executable='yhs_cmd_vel_bridge',
             name='agt_yhs_cmd_vel_bridge', output='screen',
             parameters=[{'input_topic': '/mux/cmd_vel', 'output_topic': '/yhs/ctrl_cmd',
                          'drive_gear': int(gear), 'use_sim_time': sim}]),
    ]


def _hardware_actions(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    description_share = Path(get_package_share_directory('agt_robot_description'))
    overrides = {name: value(name) for name in robot_config.MANAGED_ARGS}
    resolved = robot_config.resolve(
        _BRINGUP_SHARE / 'config' / 'robots',
        description_share / 'config' / 'robot_profiles',
        robot_config=value('robot_config'), robot_profile=value('robot'),
        overrides=overrides, description_share=description_share)
    args = resolved.launch_args
    calibration = value('robot_description_calibration_file') or str(
        description_share / resolved.profile['description']['calibration'])
    use_sim_time = value('use_sim_time')

    actions = [LogInfo(msg=(
        f'[robot_config] {resolved.robot_id} profile={resolved.robot_profile} '
        f'base={resolved.base_adapter} args={args} overrides={resolved.overrides} '
        f'reserved={resolved.reserved} payload_interlock={resolved.payload_interlock}'))]
    if value('enable_robot_description').lower() == 'true':
        # The robot description is started only here.
        actions.append(_include('agt_robot_description', 'display.launch.py', {
            'robot': resolved.robot_profile,
            'calibration_file': calibration,
            'start_rviz': 'false',
        }))
    actions += _mid360_actions(value, args)
    if args['enable_bunker_can'] == 'true':
        actions.append(_include('bunker_base', 'bunker_base.launch.py', {
            'use_sim_time': use_sim_time,
            'port_name': args['bunker_can_port'],
            'odom_topic_name': '/wheel/odom',
            'publish_odom_tf': 'false',
        }))
    if args['enable_yhs_can'] == 'true':
        actions += _yhs_actions(args, use_sim_time)
    if args['enable_rtk'] == 'true':
        actions.append(_include('agt_asensing_driver', 'asensing.launch.py'))
    if args['enable_camera_gimbal'] == 'true':
        actions.append(_include('autolabor_c1_bringup', 'autolabor_c1.launch.py', {
            'device_path': args['camera_device_path'],
            'image_width': args['camera_image_width'],
            'image_height': args['camera_image_height'],
            'fps': args['camera_fps'],
            'pixel_format': args['camera_pixel_format'],
            'port_name': args['gimbal_port_name'],
            'gui': 'false',
        }))
    return actions


def generate_launch_description():
    managed = [DeclareLaunchArgument(
        name, default_value='',
        description=f'override; empty = robot config ({source}:{path})')
        for name, (source, path) in robot_config.MANAGED_ARGS.items()]
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value='',
                              description='robot config id, directory or robot.yaml path'),
        DeclareLaunchArgument('robot', default_value='',
                              description='legacy robot profile id (e.g. bunker_v1); must '
                                          'match robot_config when both are given'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_robot_description', default_value='true'),
        DeclareLaunchArgument('robot_description_calibration_file', default_value='',
                              description='empty = calibration referenced by the robot profile'),
        DeclareLaunchArgument('mapping_livox_config', default_value=''),
        DeclareLaunchArgument('mapping_publish_freq', default_value='10.0'),
        DeclareLaunchArgument('mapping_frame_id', default_value='livox_frame'),
        *managed,
        OpaqueFunction(function=_hardware_actions),
    ])
