"""Unit tests for the whole-robot config loader (no ROS runtime needed)."""

import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'tools'))
import robot_config as rc  # noqa: E402

ROBOTS = PKG / 'config' / 'robots'
DESC = PKG.parents[1] / 'agt_robot_description'
PROFILES = DESC / 'config' / 'robot_profiles'

# Values robot_hardware.launch.py hard-coded before the robot config existed,
# with enable_rtk as run_field_stack.sh passed it (false unless --rtk).
LEGACY_BUNKER = {
    'enable_mid360': 'true', 'mid360_driver_mode': 'vendor_launch',
    'enable_bunker_can': 'true', 'bunker_can_port': 'can0', 'enable_yhs_can': 'false',
    'enable_rtk': 'false', 'enable_camera_gimbal': 'true',
    'camera_device_path': '/dev/video0', 'camera_image_width': '1920',
    'camera_image_height': '1080', 'camera_fps': '10.0',
    'camera_pixel_format': 'mjpeg2rgb',
    'gimbal_port_name': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
}


def resolve(**kw):
    return rc.resolve(ROBOTS, PROFILES, description_share=DESC, **kw)


def test_all_configs_parse():
    assert rc.list_robot_configs(ROBOTS) == ['bunker_inspection', 'yhs_harvesting']
    for robot_id in rc.list_robot_configs(ROBOTS):
        rc.load_robot_files(ROBOTS / robot_id)


def test_bunker_defaults_equal_previous_hardware_launch():
    r = resolve(robot_config='bunker_inspection')
    assert r.robot_profile == 'bunker_v1'
    assert r.base_adapter == 'bunker'
    assert r.launch_args == LEGACY_BUNKER
    assert r.overrides == {}
    assert r.reserved == {}
    assert r.task_handlers == ['camera.acquire_view']
    assert r.payload_interlock is False


@pytest.mark.parametrize('kw', [{}, {'robot_profile': 'bunker_v1'},
                                {'robot_config': 'bunker_inspection', 'robot_profile': 'bunker_v1'}])
def test_legacy_robot_argument_maps_to_bunker_inspection(kw):
    assert resolve(**kw).robot_id == 'bunker_inspection'


def test_path_forms_accepted():
    assert resolve(robot_config=str(ROBOTS / 'bunker_inspection')).robot_id == 'bunker_inspection'
    assert resolve(robot_config=str(ROBOTS / 'bunker_inspection' / 'robot.yaml')).robot_id == 'bunker_inspection'


def test_run_field_stack_and_mapping_overrides_are_valid():
    stack = resolve(robot_profile='bunker_v1', overrides={
        'enable_rtk': 'false', 'enable_camera_gimbal': 'true'})
    assert stack.overrides == {}
    rtk = resolve(robot_profile='bunker_v1', overrides={'enable_rtk': 'true'})
    assert rtk.launch_args['enable_rtk'] == 'true'
    assert rtk.overrides == {'enable_rtk': {'config': 'false', 'override': 'true'}}
    mapping = resolve(robot_profile='bunker_v1', overrides={
        'enable_mid360': 'true', 'mid360_driver_mode': 'mapping_custom',
        'enable_bunker_can': 'false', 'enable_rtk': 'false', 'enable_camera_gimbal': 'false'})
    assert mapping.launch_args['enable_bunker_can'] == 'false'
    assert mapping.launch_args['mid360_driver_mode'] == 'mapping_custom'


def test_empty_override_means_config_value():
    r = resolve(robot_config='bunker_inspection', overrides={k: '' for k in rc.MANAGED_ARGS})
    assert r.launch_args == LEGACY_BUNKER


def test_robot_and_robot_config_conflict_is_rejected(tmp_path):
    root = tmp_path / 'robots'
    shutil.copytree(ROBOTS, root)
    with pytest.raises(rc.RobotConfigError, match='unknown robot config'):
        rc.resolve(root, PROFILES, robot_config='nope')
    with pytest.raises(rc.RobotConfigError, match='conflict: robot:=other_v1'):
        # profile check comes after blockers; use a fake supported profile id
        rc.resolve(root, PROFILES, robot_config='bunker_inspection', robot_profile='other_v1')


def test_yhs_is_blocked_and_never_falls_back():
    with pytest.raises(rc.RobotConfigBlocked) as info:
        resolve(robot_config='yhs_harvesting')
    text = str(info.value)
    assert 'no fallback' in text
    assert any('yhs_v1' in b for b in info.value.blockers)
    assert any('drive_gear' in b for b in info.value.blockers)
    # Even if someone flips support_status the missing driver/profile still block.
    with pytest.raises(rc.RobotConfigBlocked):
        resolve(robot_config='yhs_harvesting', robot_profile='bunker_v1')


def test_yhs_unblocked_status_still_blocked_by_missing_driver(tmp_path):
    root = tmp_path / 'robots'
    shutil.copytree(ROBOTS, root)
    p = root / 'yhs_harvesting' / 'robot.yaml'
    data = yaml.safe_load(p.read_text())
    data['support_status'] = 'supported'
    data.pop('blockers')
    p.write_text(yaml.safe_dump(data, allow_unicode=True))
    with pytest.raises(rc.RobotConfigBlocked) as info:
        rc.resolve(root, PROFILES, robot_config='yhs_harvesting')
    assert any('drive_gear' in b for b in info.value.blockers)
    assert any('yhs_v1' in b for b in info.value.blockers)


def _yhs_unblocked(tmp_path, gear=4):
    """Test-only: pretend every YHS blocker is resolved (fake profile, fake gear)."""
    root = tmp_path / 'robots'
    shutil.copytree(ROBOTS, root)
    p = root / 'yhs_harvesting' / 'robot.yaml'
    data = yaml.safe_load(p.read_text())
    data['support_status'] = 'supported'
    data.pop('blockers')
    data['sensors']['navigation_lidar'] = {'model': 'mid360', 'installed': True, 'enabled': True}
    p.write_text(yaml.safe_dump(data, allow_unicode=True))
    d = root / 'yhs_harvesting' / 'devices.yaml'
    dev = yaml.safe_load(d.read_text())
    dev['base']['drive_gear'] = gear
    dev['navigation_lidar'] = {'driver_mode': 'vendor_launch'}
    d.write_text(yaml.safe_dump(dev))
    profiles = tmp_path / 'profiles'
    shutil.copytree(PROFILES, profiles)
    prof = yaml.safe_load((profiles / 'bunker_v1.yaml').read_text())
    prof['robot_id'] = 'yhs_v1' if 'robot_id' in prof else prof.get('robot_id')
    for key in ('profile_id', 'id', 'name'):
        if key in prof:
            prof[key] = 'yhs_v1'
    prof['base']['driver'] = 'yhs_can_control'
    prof.setdefault('capabilities', {})['manipulation'] = True
    prof['capabilities']['acquire_view'] = False
    (profiles / 'yhs_v1.yaml').write_text(yaml.safe_dump(prof, allow_unicode=True))
    return root, profiles


def test_yhs_resolves_with_interlock_once_unblocked(tmp_path):
    root, profiles = _yhs_unblocked(tmp_path)
    r = rc.resolve(root, profiles, robot_config='yhs_harvesting', description_share=DESC)
    assert r.base_adapter == 'yhs'
    assert r.launch_args['enable_yhs_can'] == 'true'
    assert r.launch_args['enable_bunker_can'] == 'false'
    assert r.launch_args['yhs_drive_gear'] == '4'
    assert r.launch_args['yhs_can_port'] == 'can0'
    assert 'bunker_can_port' not in r.launch_args
    assert r.payload_interlock is True
    assert set(r.reserved) == {'arm', 'picking_camera'}
    with pytest.raises(rc.RobotConfigError, match='enable_bunker_can'):
        rc.resolve(root, profiles, robot_config='yhs_harvesting', description_share=DESC,
                   overrides={'enable_bunker_can': 'true'})


def test_yhs_invalid_gear_is_error(tmp_path):
    root, profiles = _yhs_unblocked(tmp_path, gear='D')
    with pytest.raises(rc.RobotConfigError, match='drive_gear must be an integer'):
        rc.resolve(root, profiles, robot_config='yhs_harvesting', description_share=DESC)


def test_enable_yhs_can_on_bunker_is_conflict():
    with pytest.raises(rc.RobotConfigError, match='enable_yhs_can'):
        resolve(robot_config='bunker_inspection', overrides={'enable_yhs_can': 'true'})


@pytest.mark.parametrize('override,match', [
    ({'enable_bunker_can': 'maybe'}, 'true/false'),
    ({'not_managed': 'x'}, 'not robot-config managed'),
])
def test_bad_overrides(override, match):
    with pytest.raises(rc.RobotConfigError, match=match):
        resolve(robot_config='bunker_inspection', overrides=override)


def _mutated(tmp_path, robot_id, fn, filename='robot.yaml'):
    root = tmp_path / 'robots'
    shutil.copytree(ROBOTS, root)
    p = root / robot_id / filename
    data = yaml.safe_load(p.read_text())
    fn(data)
    p.write_text(yaml.safe_dump(data, allow_unicode=True))
    return root


def test_enabling_uninstalled_device_is_rejected(tmp_path):
    def no_cam(d):
        d['payloads']['camera_gimbal'] = {'installed': False, 'enabled': False}
    root = _mutated(tmp_path, 'bunker_inspection', no_cam)
    r = rc.resolve(root, PROFILES, robot_config='bunker_inspection')
    assert r.launch_args['enable_camera_gimbal'] == 'false'
    with pytest.raises(rc.RobotConfigError, match='camera_gimbal not installed'):
        rc.resolve(root, PROFILES, robot_config='bunker_inspection',
                   overrides={'enable_camera_gimbal': 'true'})
    with pytest.raises(rc.RobotConfigError, match='not applicable'):
        rc.resolve(root, PROFILES, robot_config='bunker_inspection',
                   overrides={'camera_device_path': '/dev/video2'})


@pytest.mark.parametrize('fn,match', [
    (lambda d: d.update(robot_id='x'), 'directory name'),
    (lambda d: d.update(schema_version=2), 'schema_version'),
    (lambda d: d.update(extra=1), 'unknown field'),
    (lambda d: d['sensors']['rtk'].update(enabled=True, role='navigation'), 'metadata_only'),
    (lambda d: d['sensors']['rtk'].update(installed=False, enabled=True), 'requires installed'),
    (lambda d: d['payloads'].update(arm={'installed': True, 'enabled': True}), 'manipulation=false'),
    (lambda d: d['payloads'].update(laser={'installed': True}), 'unknown payloads'),
    (lambda d: d['base'].update(adapter='tank'), 'unknown base.adapter'),
])
def test_invalid_robot_yaml(tmp_path, fn, match):
    root = _mutated(tmp_path, 'bunker_inspection', fn)
    with pytest.raises(rc.RobotConfigError, match=match):
        rc.resolve(root, PROFILES, robot_config='bunker_inspection')


def test_missing_device_value_is_rejected(tmp_path):
    root = _mutated(tmp_path, 'bunker_inspection', lambda d: d['base'].pop('can_port'),
                    'devices.yaml')
    with pytest.raises(rc.RobotConfigError, match='base.can_port'):
        rc.resolve(root, PROFILES, robot_config='bunker_inspection')


def _keys(node):
    if isinstance(node, dict):
        for key, value in node.items():
            yield key
            yield from _keys(value)
    elif isinstance(node, list):
        for value in node:
            yield from _keys(value)


def test_no_geometry_duplicated_in_robot_configs():
    forbidden = {'footprint', 'extrinsics', 'max_linear_velocity', 'max_angular_velocity',
                 'xyz', 'rpy', 'width', 'length', 'geometry', 'frames', 'calibration'}
    for path in ROBOTS.glob('*/*.yaml'):
        keys = set(_keys(yaml.safe_load(path.read_text(encoding='utf-8'))))
        assert not keys & forbidden, f'{sorted(keys & forbidden)} duplicated in {path}'


def test_cli_exit_codes():
    tool = str(PKG / 'tools' / 'robot_config.py')
    base = [sys.executable, tool, '--robots-root', str(ROBOTS), '--profile-dir', str(PROFILES),
            '--description-share', str(DESC)]
    ok = subprocess.run(base + ['resolve', '--robot', 'bunker_v1'], capture_output=True, text=True)
    assert ok.returncode == 0, ok.stderr
    assert 'robot_config=bunker_inspection' in ok.stdout
    blocked = subprocess.run(base + ['resolve', '--robot-config', 'yhs_harvesting'],
                             capture_output=True, text=True)
    assert blocked.returncode == 3 and 'BLOCKED' in blocked.stderr
    bad = subprocess.run(base + ['resolve', '--robot-config', 'bunker_inspection',
                                 '--robot', 'yhs_v1'], capture_output=True, text=True)
    assert bad.returncode == 2 and 'conflict' in bad.stderr


def test_installed_but_disabled_arm_still_requires_interlock(tmp_path):
    root, profiles = _yhs_unblocked(tmp_path)
    p = root / 'yhs_harvesting' / 'robot.yaml'
    data = yaml.safe_load(p.read_text())
    data['payloads']['arm']['enabled'] = False
    p.write_text(yaml.safe_dump(data, allow_unicode=True))
    r = rc.resolve(root, profiles, robot_config='yhs_harvesting', description_share=DESC)
    assert r.payload_interlock is True


def test_not_installed_arm_has_no_interlock(tmp_path):
    root, profiles = _yhs_unblocked(tmp_path)
    p = root / 'yhs_harvesting' / 'robot.yaml'
    data = yaml.safe_load(p.read_text())
    data['payloads']['arm'] = {'installed': False, 'enabled': False}
    data['mission']['task_handlers'] = []
    p.write_text(yaml.safe_dump(data, allow_unicode=True))
    r = rc.resolve(root, profiles, robot_config='yhs_harvesting', description_share=DESC)
    assert r.payload_interlock is False
