"""Twist -> YHS TK-mid CtrlCmd conversion (no ROS imports).

Source of the units (official driver YUHESEN-Robot/TK-mid-ros2 @6d002bd,
yhs_can_control_node.cpp):
  * ctrl_cmd_linear is sent as ``short(linear * 1000)``  -> m/s, 0.001 m/s/bit
  * ctrl_cmd_angular is sent as ``short(angular * 100)`` and the matching
    feedback ``ctrl_fb_angular`` is documented in the V1.1.1 manual as deg/s
    and converted by the driver with ``/180*3.14`` -> deg/s, 0.01 deg/s/bit
The gear value is NOT derived here: the README (4 = D) and the PDF manual
(3 = kinematic control) disagree, so it must be confirmed on the real chassis
and passed in explicitly. No default exists.
"""
import math

INT16_MAX = 32767


class ConversionError(ValueError):
    pass


def validate_gear(value, name='drive_gear'):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 15:
        raise ConversionError(f'{name} must be a chassis-confirmed integer in [0, 15], got {value!r}')
    return value


def twist_to_ctrl(linear_mps, angular_radps, drive_gear, allow_reverse=False):
    """Return (gear, linear_mps, angular_degps). Non-finite input -> stop."""
    validate_gear(drive_gear)
    if not (math.isfinite(linear_mps) and math.isfinite(angular_radps)):
        return drive_gear, 0.0, 0.0
    linear = float(linear_mps)
    if linear < 0.0 and not allow_reverse:
        # Reverse semantics (negative speed in drive gear vs. R gear) are not
        # confirmed for this chassis: refuse reverse instead of guessing.
        linear = 0.0
    angular_deg = math.degrees(float(angular_radps))
    lim_lin = INT16_MAX / 1000.0
    lim_ang = INT16_MAX / 100.0
    linear = max(-lim_lin, min(lim_lin, linear))
    angular_deg = max(-lim_ang, min(lim_ang, angular_deg))
    return drive_gear, linear, angular_deg


def stop_command(drive_gear):
    return validate_gear(drive_gear), 0.0, 0.0
