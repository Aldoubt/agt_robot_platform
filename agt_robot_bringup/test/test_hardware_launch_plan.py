"""Assert what robot_hardware.launch.py selects for real caller arguments.

Requires a sourced workspace (agt_robot_bringup installed); skipped otherwise.
No process is started: see tools/hardware_launch_plan.py.
"""

import shutil
import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'tools'))

try:
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory('agt_robot_bringup')
    import hardware_launch_plan as hlp
except Exception:  # noqa: BLE001
    pytest.skip('agt_robot_bringup not installed/sourced', allow_module_level=True)

DESC = 'agt_robot_description/launch/display.launch.py'
MID = 'livox_ros_driver2/launch_ROS2/msg_MID360_launch.py'
BUNKER = 'bunker_base/launch/bunker_base.launch.py'
RTK = 'agt_asensing_driver/launch/asensing.launch.py'
CAM = 'autolabor_c1_bringup/launch/autolabor_c1.launch.py'


def run(**args):
    return hlp.plan({k: str(v) for k, v in args.items()})


def test_direct_default_is_bunker_inspection_yaml():
    r = run()
    assert r['ok'], r['error']
    assert r['includes'] == [DESC, MID, BUNKER, CAM]


def test_run_field_stack_navigation_legacy_args():
    # exactly what run_field_stack.sh passes for --mode navigation without --robot-config
    r = run(robot='bunker_v1', enable_camera_gimbal='false')
    assert r['ok'], r['error']
    assert r['includes'] == [DESC, MID, BUNKER]


def test_run_field_stack_inspection_rtk_args():
    r = run(robot='bunker_v1', enable_camera_gimbal='true', enable_rtk='true')
    assert r['includes'] == [DESC, MID, BUNKER, RTK, CAM]


def test_mapping_live_launch_args():
    r = run(robot='bunker_v1', enable_robot_description='false', enable_mid360='true',
            mid360_driver_mode='mapping_custom', mapping_livox_config='/etc/hostname',
            enable_bunker_can='false', enable_rtk='false', enable_camera_gimbal='false')
    assert r['ok'], r['error']
    assert r['includes'] == [] and r['nodes'] == ['livox_ros_driver2']


def test_yhs_blocked():
    r = run(robot_config='yhs_harvesting')
    assert not r['ok'] and r['error_type'] == 'RobotConfigBlocked'
    assert r['includes'] == [] and r['nodes'] == []


def test_conflict_refused():
    r = run(robot_config='bunker_inspection', robot='yhs_v1')
    assert not r['ok'] and r['error_type'] == 'RobotConfigError' and 'conflict' in r['error']


def test_external_config_directory_values_are_used(tmp_path):
    # Regression for review P1: a config passed by path must be the one launched.
    src = Path(get_package_share_directory('agt_robot_bringup')) / 'config/robots/bunker_inspection'
    dst = tmp_path / 'bunker_inspection'
    shutil.copytree(src, dst)
    dev = yaml.safe_load((dst / 'devices.yaml').read_text())
    dev['base']['can_port'] = 'can9'
    (dst / 'devices.yaml').write_text(yaml.safe_dump(dev))
    import robot_config as rc
    desc = Path(get_package_share_directory('agt_robot_description'))
    resolved = rc.resolve(src.parent, desc / 'config/robot_profiles', robot_config=str(dst),
                          description_share=desc)
    assert resolved.launch_args['bunker_can_port'] == 'can9'
    assert resolved.config_dir == dst.resolve()
