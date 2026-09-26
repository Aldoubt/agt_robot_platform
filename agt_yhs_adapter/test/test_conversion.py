import math

import pytest

from agt_yhs_adapter.conversion import ConversionError, stop_command, twist_to_ctrl


def test_units_rad_to_deg_and_mps_passthrough():
    gear, lin, ang = twist_to_ctrl(0.3, math.radians(10.0), 4)
    assert gear == 4 and lin == pytest.approx(0.3) and ang == pytest.approx(10.0)


def test_reverse_refused_unless_allowed():
    assert twist_to_ctrl(-0.2, 0.0, 4)[1] == 0.0
    assert twist_to_ctrl(-0.2, 0.0, 4, allow_reverse=True)[1] == pytest.approx(-0.2)


def test_non_finite_is_stop():
    assert twist_to_ctrl(float('nan'), 0.1, 4)[1:] == (0.0, 0.0)
    assert twist_to_ctrl(0.1, float('inf'), 4)[1:] == (0.0, 0.0)


@pytest.mark.parametrize('gear', [-1, 16, None, True, 3.0, '4'])
def test_gear_must_be_explicit_int(gear):
    with pytest.raises(ConversionError):
        twist_to_ctrl(0.1, 0.0, gear)
    with pytest.raises(ConversionError):
        stop_command(gear)


def test_saturates_to_int16_wire_range():
    _, lin, ang = twist_to_ctrl(100.0, 100.0, 4)
    assert lin == pytest.approx(32.767) and ang == pytest.approx(327.67)
