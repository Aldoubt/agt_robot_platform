#!/usr/bin/env python3
"""Whole-robot combination loader for agt_robot_bringup.

A *robot config* (``config/robots/<robot_id>/robot.yaml`` + ``devices.yaml``)
selects which base adapter, sensors and payloads make up one physical robot.
It deliberately does NOT contain geometry, extrinsics, footprint or motion
limits: those stay in the Robot Profile owned by ``agt_robot_description``
(``config/robot_profiles/<robot_profile>.yaml``) and are only referenced here.

Ownership of values (single source each):
  robot.yaml    -> which devices are installed/enabled, base adapter, profile id
  devices.yaml  -> deployment values (CAN port, device paths, camera mode)
  robot profile -> model, frames, calibration, footprint, motion limits
  launch args   -> explicit per-run overrides (validated, never silently ignored)

The module is pure Python (PyYAML, optional jsonschema) so it can be unit
tested without ROS, used from launch files and called from shell scripts.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

SCHEMA_VERSION = 1
DEFAULT_LEGACY_PROFILE = 'bunker_v1'

# Base adapters known to the platform.  'available' means a ROS 2 driver is
# built in this workspace and wired into robot_hardware.launch.py.
BASE_ADAPTERS = {
    'bunker': {
        'status': 'available',
        'enable_arg': 'enable_bunker_can',
        'driver_package': 'bunker_base',
    },
    'yhs': {
        # Official YUHESEN-Robot/TK-mid-ros2 @6d002bd vendored in
        # src/drivers/yhs_tk_mid_ros2 (package yhs_can_control) plus the
        # agt_yhs_adapter Twist->CtrlCmd bridge.  Needs devices.yaml
        # base.can_port and a chassis-confirmed base.drive_gear.
        'status': 'available',
        'enable_arg': 'enable_yhs_can',
        'driver_package': 'yhs_can_control',
    },
}

# Hardware-launch payloads: config key -> launch enable argument.
HARDWARE_PAYLOADS = {'camera_gimbal': 'enable_camera_gimbal'}
# Payloads started outside robot_hardware.launch.py (reserved integration points).
RESERVED_PAYLOADS = {
    'arm': 'agt_arm_capability (started by agt_mission_bringup enable_arm)',
    'picking_camera': ('kiwipickingandmove.py via pyorbbecsdk inside the agt_arm_capability '
                       'picker process (not a ROS driver; no hardware-launch node)'),
}
# Payloads whose activity must hold the chassis (fail-closed drive permission).
INTERLOCK_PAYLOADS = ('arm',)

# Launch arguments whose defaults come from robot.yaml/devices.yaml.
# value: (source file, dotted path)
MANAGED_ARGS = {
    'enable_mid360': ('robot', 'sensors.navigation_lidar.enabled'),
    'mid360_driver_mode': ('devices', 'navigation_lidar.driver_mode'),
    'enable_bunker_can': ('derived', 'base.adapter == bunker'),
    'bunker_can_port': ('devices', 'base.can_port'),
    'enable_yhs_can': ('derived', 'base.adapter == yhs'),
    'yhs_can_port': ('devices', 'base.can_port'),
    'yhs_drive_gear': ('devices', 'base.drive_gear'),
    'enable_rtk': ('robot', 'sensors.rtk.enabled'),
    'enable_camera_gimbal': ('robot', 'payloads.camera_gimbal.enabled'),
    'camera_device_path': ('devices', 'camera_gimbal.device_path'),
    'camera_image_width': ('devices', 'camera_gimbal.image_width'),
    'camera_image_height': ('devices', 'camera_gimbal.image_height'),
    'camera_fps': ('devices', 'camera_gimbal.fps'),
    'camera_pixel_format': ('devices', 'camera_gimbal.pixel_format'),
    'gimbal_port_name': ('devices', 'camera_gimbal.port_name'),
}
BOOL_ARGS = {'enable_mid360', 'enable_bunker_can', 'enable_yhs_can', 'enable_rtk',
             'enable_camera_gimbal'}


class RobotConfigError(RuntimeError):
    """Invalid or conflicting configuration. Launch must not start."""


class RobotConfigBlocked(RobotConfigError):
    """Configuration is well-formed but depends on missing drivers/data."""

    def __init__(self, robot_id, blockers):
        self.robot_id = robot_id
        self.blockers = list(blockers)
        text = '; '.join(self.blockers)
        super().__init__(f'robot config {robot_id!r} is BLOCKED and will not start '
                         f'(no fallback to another robot): {text}')


@dataclass
class ResolvedRobot:
    robot_id: str
    robot_profile: str
    config_dir: Path
    base_adapter: str
    launch_args: dict = field(default_factory=dict)
    overrides: dict = field(default_factory=dict)
    reserved: dict = field(default_factory=dict)
    profile: dict = field(default_factory=dict)
    task_handlers: list = field(default_factory=list)
    payload_interlock: bool = False

    def summary(self):
        return {
            'robot_config': self.robot_id,
            'robot_profile': self.robot_profile,
            'config_dir': str(self.config_dir),
            'base_adapter': self.base_adapter,
            'launch_args': dict(self.launch_args),
            'overrides': dict(self.overrides),
            'reserved_payloads': dict(self.reserved),
            'task_handlers': list(self.task_handlers),
            'payload_interlock': bool(self.payload_interlock),
        }


# --------------------------------------------------------------------- helpers
def _get(data, dotted):
    node = data
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node:
            raise RobotConfigError(f'missing required field {dotted!r}')
        node = node[part]
    return node


def _as_bool(value, name):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ('true', '1', 'yes'):
        return True
    if text in ('false', '0', 'no'):
        return False
    raise RobotConfigError(f'{name} must be true/false, got {value!r}')


def _launch_text(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _read_yaml(path):
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise RobotConfigError(f'file not found: {path}') from exc
    except yaml.YAMLError as exc:
        raise RobotConfigError(f'invalid YAML in {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise RobotConfigError(f'{path} must contain a mapping')
    return data


def _check_keys(data, allowed, where):
    extra = set(data) - set(allowed)
    if extra:
        raise RobotConfigError(f'unknown field(s) in {where}: {sorted(extra)}')


# ---------------------------------------------------------------- discovery
def list_robot_configs(robots_root):
    root = Path(robots_root)
    return sorted(p.parent.name for p in root.glob('*/robot.yaml'))


def config_dir_for(spec, robots_root):
    """Accept a robot id, a directory, or a path to robot.yaml."""
    candidate = Path(str(spec)).expanduser()
    if candidate.suffix in ('.yaml', '.yml'):
        if not candidate.is_file():
            raise RobotConfigError(f'robot config file not found: {candidate}')
        if candidate.name != 'robot.yaml':
            raise RobotConfigError(f'robot config file must be named robot.yaml: {candidate}')
        return candidate.resolve().parent
    if candidate.is_dir() and (candidate / 'robot.yaml').is_file():
        return candidate.resolve()
    by_id = Path(robots_root) / str(spec)
    if (by_id / 'robot.yaml').is_file():
        return by_id.resolve()
    known = ', '.join(list_robot_configs(robots_root)) or '<none>'
    raise RobotConfigError(f'unknown robot config {spec!r}; known: {known}')


def configs_for_profile(profile_id, robots_root):
    matches = []
    for robot_id in list_robot_configs(robots_root):
        data = _read_yaml(Path(robots_root) / robot_id / 'robot.yaml')
        if data.get('robot_profile') == profile_id:
            matches.append((robot_id, data.get('support_status')))
    return matches


# ------------------------------------------------------------------ loading
ROBOT_KEYS = {'schema_version', 'robot_id', 'support_status', 'blockers', 'description',
              'robot_profile', 'base', 'sensors', 'payloads', 'mission', 'devices_file'}
DEVICES_KEYS = {'schema_version', 'robot_id', 'base', 'navigation_lidar', 'camera_gimbal',
                'rtk', 'arm'}


def load_robot_files(config_dir):
    config_dir = Path(config_dir)
    robot = _read_yaml(config_dir / 'robot.yaml')
    _check_keys(robot, ROBOT_KEYS, f'{config_dir}/robot.yaml')
    if robot.get('schema_version') != SCHEMA_VERSION:
        raise RobotConfigError(f'{config_dir}/robot.yaml schema_version must be {SCHEMA_VERSION}')
    robot_id = robot.get('robot_id')
    if robot_id != config_dir.name:
        raise RobotConfigError(
            f'robot_id {robot_id!r} must equal its directory name {config_dir.name!r}')
    status = robot.get('support_status')
    if status not in ('supported', 'blocked'):
        raise RobotConfigError(f'{robot_id}: support_status must be supported|blocked')
    devices_name = robot.get('devices_file', 'devices.yaml')
    if Path(devices_name).name != devices_name:
        raise RobotConfigError(f'{robot_id}: devices_file must be a file name in the config dir')
    devices = _read_yaml(config_dir / devices_name)
    _check_keys(devices, DEVICES_KEYS, f'{config_dir}/{devices_name}')
    if devices.get('schema_version') != SCHEMA_VERSION:
        raise RobotConfigError(f'{devices_name} schema_version must be {SCHEMA_VERSION}')
    if devices.get('robot_id') != robot_id:
        raise RobotConfigError(f'{devices_name} robot_id must be {robot_id!r}')
    return robot, devices


def load_profile(profile_id, profile_dir, description_share=None):
    profile_dir = Path(profile_dir)
    path = profile_dir / f'{profile_id}.yaml'
    if not path.is_file():
        raise RobotConfigError(f'robot profile {profile_id!r} not found: {path}')
    profile = _read_yaml(path)
    schema_path = profile_dir / 'schema.json'
    if schema_path.is_file():
        try:
            import jsonschema
        except ImportError as exc:  # pragma: no cover - dependency declared in package.xml
            raise RobotConfigError('python3-jsonschema is required to validate robot profiles') from exc
        try:
            jsonschema.validate(profile, json.loads(schema_path.read_text(encoding='utf-8')))
        except jsonschema.ValidationError as exc:
            raise RobotConfigError(f'robot profile {profile_id} schema error: {exc.message}') from exc
    if profile.get('robot_id') != profile_id:
        raise RobotConfigError(f'robot profile ID mismatch: {profile_id}')
    if description_share is not None:
        share = Path(description_share)
        for key in ('xacro', 'calibration'):
            relative = Path(profile['description'][key])
            if relative.is_absolute() or '..' in relative.parts:
                raise RobotConfigError(f'robot profile {key} escapes package: {relative}')
            if not (share / relative).is_file():
                raise RobotConfigError(f'robot profile {key} is missing: {share / relative}')
    return profile


# --------------------------------------------------------------- resolution
def _defaults_from_config(robot, devices):
    """Compute managed launch-argument values from robot.yaml/devices.yaml."""
    adapter = _get(robot, 'base.adapter')
    sensors = _get(robot, 'sensors')
    payloads = _get(robot, 'payloads')
    lidar = _get(sensors, 'navigation_lidar')
    rtk = _get(sensors, 'rtk')
    cam = payloads.get('camera_gimbal', {'installed': False, 'enabled': False})
    for name, item in (('sensors.navigation_lidar', lidar), ('sensors.rtk', rtk),
                       ('payloads.camera_gimbal', cam)):
        installed = _as_bool(item.get('installed', False), f'{name}.installed')
        enabled = _as_bool(item.get('enabled', False), f'{name}.enabled')
        if enabled and not installed:
            raise RobotConfigError(f'{name}: enabled=true requires installed=true')
    if _as_bool(rtk.get('enabled', False), 'rtk.enabled') and rtk.get('role') != 'metadata_only':
        raise RobotConfigError('sensors.rtk.role must be metadata_only (RTK never drives map->odom)')

    values = {
        'enable_mid360': _as_bool(lidar.get('enabled', False), 'navigation_lidar.enabled'),
        'enable_rtk': _as_bool(rtk.get('enabled', False), 'rtk.enabled'),
        'enable_camera_gimbal': _as_bool(cam.get('enabled', False), 'camera_gimbal.enabled'),
        'enable_bunker_can': adapter == 'bunker',
        'enable_yhs_can': adapter == 'yhs',
    }
    if values['enable_mid360']:
        if lidar.get('model') != 'mid360':
            raise RobotConfigError(
                f"navigation_lidar.model {lidar.get('model')!r} has no hardware launch integration")
        values['mid360_driver_mode'] = _get(devices, 'navigation_lidar.driver_mode')
    if adapter == 'bunker':
        values['bunker_can_port'] = _get(devices, 'base.can_port')
    if adapter == 'yhs':
        values['yhs_can_port'] = _get(devices, 'base.can_port')
        values['yhs_drive_gear'] = _get(devices, 'base.drive_gear')
    if _as_bool(cam.get('installed', False), 'camera_gimbal.installed'):
        for key, arg in (('device_path', 'camera_device_path'), ('image_width', 'camera_image_width'),
                         ('image_height', 'camera_image_height'), ('fps', 'camera_fps'),
                         ('pixel_format', 'camera_pixel_format'), ('port_name', 'gimbal_port_name')):
            values[arg] = _get(devices, f'camera_gimbal.{key}')
    return {k: _launch_text(v) for k, v in values.items()}


def _installed(robot, section, name):
    item = (robot.get(section) or {}).get(name) or {}
    return bool(item) and _as_bool(item.get('installed', False), f'{section}.{name}.installed')


def resolve(robots_root, profile_dir, robot_config='', robot_profile='', overrides=None,
            description_share=None):
    """Resolve the robot combination and validated launch arguments.

    ``overrides`` holds only launch arguments the user set explicitly (non-empty).
    Raises RobotConfigError / RobotConfigBlocked; never falls back to another robot.
    """
    overrides = {k: str(v) for k, v in (overrides or {}).items() if str(v) != ''}
    unknown = set(overrides) - set(MANAGED_ARGS)
    if unknown:
        raise RobotConfigError(f'not robot-config managed launch args: {sorted(unknown)}')

    robot_config = (robot_config or '').strip()
    robot_profile = (robot_profile or '').strip()
    if not robot_config:
        legacy = robot_profile or DEFAULT_LEGACY_PROFILE
        matches = [m for m in configs_for_profile(legacy, robots_root) if m[1] == 'supported']
        if len(matches) != 1:
            raise RobotConfigError(
                f'robot profile {legacy!r} maps to {len(matches)} supported robot configs '
                f'{[m[0] for m in matches]}; pass robot_config explicitly')
        robot_config = matches[0][0]

    config_dir = config_dir_for(robot_config, robots_root)
    robot, devices = load_robot_files(config_dir)
    robot_id = robot['robot_id']

    blockers = []
    if robot['support_status'] == 'blocked':
        blockers.extend(str(b) for b in (robot.get('blockers') or ['support_status=blocked']))
    adapter = _get(robot, 'base.adapter')
    if adapter not in BASE_ADAPTERS:
        raise RobotConfigError(f'{robot_id}: unknown base.adapter {adapter!r}')
    if BASE_ADAPTERS[adapter]['status'] != 'available':
        blockers.append(f"base adapter {adapter}: {BASE_ADAPTERS[adapter]['reason']}")
    if adapter == 'yhs':
        base_dev = devices.get('base') or {}
        if not base_dev.get('can_port'):
            blockers.append('devices.yaml base.can_port not set for the YHS chassis')
        gear = base_dev.get('drive_gear')
        if gear is None:
            blockers.append('devices.yaml base.drive_gear not confirmed: YHS README says 4=D, '
                            'the V1.1.1 PDF manual says 3=kinematic control; confirm on the '
                            'real chassis/DBC before setting it')
        elif isinstance(gear, bool) or not isinstance(gear, int) or not 0 <= gear <= 15:
            raise RobotConfigError(f'{robot_id}: base.drive_gear must be an integer 0..15')
    profile_id = robot.get('robot_profile')
    if not profile_id:
        raise RobotConfigError(f'{robot_id}: robot_profile is required')
    profile_path = Path(profile_dir) / f'{profile_id}.yaml'
    if not profile_path.is_file():
        blockers.append(f'robot profile {profile_id!r} does not exist in agt_robot_description '
                        '(geometry/extrinsics must be measured, not copied)')
    if blockers:
        raise RobotConfigBlocked(robot_id, dict.fromkeys(blockers))
    if robot['support_status'] != 'supported':
        raise RobotConfigError(f'{robot_id}: not supported')

    if robot_profile and robot_profile != profile_id:
        raise RobotConfigError(
            f'conflict: robot:={robot_profile} but robot_config {robot_id} uses '
            f'robot_profile {profile_id}')

    profile = load_profile(profile_id, profile_dir, description_share)
    expected_driver = BASE_ADAPTERS[adapter]['driver_package']
    if profile['base'].get('driver') != expected_driver:
        raise RobotConfigError(
            f"{robot_id}: profile base.driver {profile['base'].get('driver')!r} does not match "
            f"adapter {adapter} ({expected_driver})")
    caps = profile.get('capabilities', {})
    payloads = robot.get('payloads') or {}
    if _installed(robot, 'payloads', 'camera_gimbal') and not caps.get('acquire_view'):
        raise RobotConfigError(f'{robot_id}: camera_gimbal installed but profile {profile_id} '
                               'capabilities.acquire_view=false')
    if _installed(robot, 'payloads', 'arm') and not caps.get('manipulation'):
        raise RobotConfigError(f'{robot_id}: arm installed but profile {profile_id} '
                               'capabilities.manipulation=false')
    unknown_payloads = set(payloads) - set(HARDWARE_PAYLOADS) - set(RESERVED_PAYLOADS)
    if unknown_payloads:
        raise RobotConfigError(f'{robot_id}: unknown payloads {sorted(unknown_payloads)}')

    launch_args = _defaults_from_config(robot, devices)
    applied = {}
    for name, value in overrides.items():
        if name in BOOL_ARGS:
            want = _as_bool(value, name)
            if want:
                if name == 'enable_bunker_can' and adapter != 'bunker':
                    raise RobotConfigError(f'conflict: enable_bunker_can:=true on {adapter} robot {robot_id}')
                if name == 'enable_yhs_can' and adapter != 'yhs':
                    raise RobotConfigError(f'conflict: enable_yhs_can:=true on {adapter} robot {robot_id}')
                if name == 'enable_camera_gimbal' and not _installed(robot, 'payloads', 'camera_gimbal'):
                    raise RobotConfigError(f'conflict: camera_gimbal not installed on {robot_id}')
                if name == 'enable_rtk' and not _installed(robot, 'sensors', 'rtk'):
                    raise RobotConfigError(f'conflict: rtk not installed on {robot_id}')
                if name == 'enable_mid360' and not _installed(robot, 'sensors', 'navigation_lidar'):
                    raise RobotConfigError(f'conflict: navigation_lidar not installed on {robot_id}')
            value = _launch_text(want)
        elif name not in launch_args:
            raise RobotConfigError(f'{name} is not applicable to robot config {robot_id}')
        if launch_args.get(name) != value:
            applied[name] = {'config': launch_args.get(name), 'override': value}
        launch_args[name] = value
    # Anything not produced by the config is disabled/irrelevant for the launch.
    for name in BOOL_ARGS:
        launch_args.setdefault(name, 'false')

    reserved = {}
    for name, owner in RESERVED_PAYLOADS.items():
        if _installed(robot, 'payloads', name):
            reserved[name] = owner
    handlers = list((robot.get('mission') or {}).get('task_handlers') or [])
    # Physical installation decides: a disabled arm (software off) is not proof
    # that the arm is stowed. There is no exemption switch; a verified
    # mechanical-lock procedure would need its own reviewed mechanism.
    interlock = any(_installed(robot, 'payloads', name) for name in INTERLOCK_PAYLOADS)
    return ResolvedRobot(robot_id=robot_id, robot_profile=profile_id, config_dir=config_dir,
                         base_adapter=adapter, launch_args=launch_args, overrides=applied,
                         reserved=reserved, profile=profile, task_handlers=handlers,
                         payload_interlock=interlock)


# ---------------------------------------------------------------------- CLI
def _default_roots():
    try:
        from ament_index_python.packages import get_package_share_directory
        bringup = Path(get_package_share_directory('agt_robot_bringup'))
        description = Path(get_package_share_directory('agt_robot_description'))
    except Exception:  # noqa: BLE001 - source-tree fallback for tests/CI
        here = Path(__file__).resolve()
        bringup = here.parents[1]
        description = here.parents[3] / 'agt_robot_description'
    return (bringup / 'config' / 'robots', description / 'config' / 'robot_profiles', description)


def payload_interlock_required(robot_config_spec, robot_profile=''):
    """Installed-share helper for launch files: does this robot need the chassis
    drive-permission interlock?  Blocked/invalid configs raise (never guess)."""
    robots_root, profile_dir, description_share = _default_roots()
    return resolve(robots_root, profile_dir, robot_config=robot_config_spec,
                   robot_profile=robot_profile,
                   description_share=description_share).payload_interlock


def main(argv=None):
    robots_root, profile_dir, description_share = _default_roots()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--robots-root', default=str(robots_root))
    parser.add_argument('--profile-dir', default=str(profile_dir))
    parser.add_argument('--description-share', default=str(description_share))
    sub = parser.add_subparsers(dest='cmd', required=True)
    res = sub.add_parser('resolve', help='print resolved key=value lines')
    res.add_argument('--robot-config', default='')
    res.add_argument('--robot', default='')
    res.add_argument('--set', action='append', default=[], metavar='ARG=VALUE')
    res.add_argument('--json', action='store_true')
    sub.add_parser('list', help='list robot configs and their status')
    args = parser.parse_args(argv)

    if args.cmd == 'list':
        for robot_id in list_robot_configs(args.robots_root):
            data = _read_yaml(Path(args.robots_root) / robot_id / 'robot.yaml')
            print(f"{robot_id}\t{data.get('support_status')}\tprofile={data.get('robot_profile')}")
        return 0
    overrides = {}
    for item in args.set:
        if '=' not in item:
            parser.error(f'--set expects ARG=VALUE: {item}')
        key, value = item.split('=', 1)
        overrides[key] = value
    try:
        resolved = resolve(args.robots_root, args.profile_dir, args.robot_config, args.robot,
                           overrides, args.description_share)
    except RobotConfigBlocked as exc:
        print(f'BLOCKED: {exc}', file=sys.stderr)
        return 3
    except RobotConfigError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(resolved.summary(), indent=2, ensure_ascii=False))
        return 0
    print(f'robot_config={resolved.robot_id}')
    print(f'robot_profile={resolved.robot_profile}')
    print(f'robot_config_dir={resolved.config_dir}')
    print(f'base_adapter={resolved.base_adapter}')
    for key in sorted(resolved.launch_args):
        print(f'{key}={resolved.launch_args[key]}')
    print(f"reserved_payloads={','.join(sorted(resolved.reserved))}")
    print(f"task_handlers={','.join(resolved.task_handlers)}")
    print(f"payload_interlock={'true' if resolved.payload_interlock else 'false'}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
