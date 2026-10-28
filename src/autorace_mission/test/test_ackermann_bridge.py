import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.ackermann_bridge import twist_to_ackermann  # noqa: E402


def test_straight():
    assert twist_to_ackermann(0.3, 0.0) == (0.3, 0.0)


def test_left_turn_positive():
    v, d = twist_to_ackermann(0.3, 0.3 * 1.0)          # kappa = 1.0 (R = 1 m)
    assert v == 0.3
    assert math.isclose(d, math.atan(0.2 * 1.0), rel_tol=1e-6)
    assert d > 0


def test_right_turn_negative():
    assert twist_to_ackermann(0.3, -0.3 * 1.0)[1] < 0


def test_clamp_to_max_steer():
    _, d = twist_to_ackermann(0.2, 0.2 * 5.0)           # R = 0.2 m: 물리적으로 불가능
    assert math.isclose(d, 0.40)


def test_stopped_has_zero_steer():
    assert twist_to_ackermann(0.0, 1.0) == (0.0, 0.0)


def test_reverse_keeps_curve_direction():
    # 후진 중 같은 kappa(왼쪽 +)면 조향 방향도 같다: w/v 가 같은 부호
    _, d = twist_to_ackermann(-0.2, -0.2 * 1.0)
    assert d > 0
