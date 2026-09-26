#!/usr/bin/env python3
"""Print what robot_hardware.launch.py WOULD start, without starting anything.

Usage: hardware_launch_plan.py [name:=value ...]   (same arguments as ros2 launch)
Output: JSON {"ok": bool, "includes": [...], "nodes": [...], "error": str, "error_type": str}
Exit: 0 resolved, 2 refused (config error), 3 blocked.

It loads the installed launch file, applies the declared defaults and the given
arguments to a LaunchContext and calls the OpaqueFunction body directly; no
LaunchService runs, so no process can be spawned.
"""

import ast
import importlib.util
import json
import re
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node


def _location(action, ctx):
    src = action.launch_description_source
    loc = getattr(src, '_LaunchDescriptionSource__location', None) or src.location
    if not isinstance(loc, str):
        items = loc if isinstance(loc, (list, tuple)) else [loc]
        loc = ''.join(item.perform(ctx) for item in items)
    return '/'.join(loc.split('/')[-3:])


def plan(arguments):
    path = get_package_share_directory('agt_robot_bringup') + '/launch/robot_hardware.launch.py'
    spec = importlib.util.spec_from_file_location('agt_robot_hardware_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ctx = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = ''.join(
                s.perform(ctx) for s in entity.default_value)
    ctx.launch_configurations.update(arguments)
    result = {'ok': False, 'includes': [], 'nodes': [], 'launch_args': {},
              'error': '', 'error_type': ''}
    try:
        actions = module._hardware_actions(ctx)
    except Exception as exc:  # noqa: BLE001 - reported to caller
        result['error'] = str(exc)
        result['error_type'] = type(exc).__name__
        return result
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            result['includes'].append(_location(action, ctx))
        elif isinstance(action, LogInfo):
            text = ''.join(part.perform(ctx) for part in action.msg)
            match = re.search(r' args=(\{.*?\}) overrides=', text)
            if match:
                result['launch_args'] = ast.literal_eval(match.group(1))
        elif isinstance(action, Node):
            result['nodes'].append(action.node_package)
    result['ok'] = True
    return result


def main(argv=None):
    arguments = {}
    for item in (sys.argv[1:] if argv is None else argv):
        if ':=' not in item:
            print(f'expected name:=value, got {item!r}', file=sys.stderr)
            return 2
        key, value = item.split(':=', 1)
        arguments[key] = value
    result = plan(arguments)
    print(json.dumps(result, ensure_ascii=False))
    if result['ok']:
        return 0
    return 3 if result['error_type'] == 'RobotConfigBlocked' else 2


if __name__ == '__main__':
    sys.exit(main())
