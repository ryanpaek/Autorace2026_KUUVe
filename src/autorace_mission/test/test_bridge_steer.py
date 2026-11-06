"""작은 조향 보정 (11/06)."""

import pytest

from autorace_mission.ackermann_bridge import small_steer_comp, twist_to_ackermann


def test_off_by_default():
    for d in (-0.3, -0.05, 0.0, 0.05, 0.3):
        assert small_steer_comp(d) == d


@pytest.mark.parametrize('g', [1.2, 1.3, 1.5])
def test_continuous_and_monotonic(g):
    xs = [i * 0.001 for i in range(0, 400)]
    ys = [small_steer_comp(x, g, 0.1) for x in xs]
    assert all(b >= a for a, b in zip(ys, ys[1:]))
    assert max(abs(b - a) for a, b in zip(ys, ys[1:])) < 0.003
    assert ys[100] == pytest.approx(0.1 * g)
    assert small_steer_comp(0.25, g, 0.1) == 0.25        # 큰 조향은 그대로
    assert small_steer_comp(-0.05, g, 0.1) == pytest.approx(-0.05 * g)


def test_bridge_uses_comp():
    _, d0 = twist_to_ackermann(0.3, 0.3 * 0.2)
    _, d1 = twist_to_ackermann(0.3, 0.3 * 0.2, small_gain=1.3)
    assert d1 == pytest.approx(d0 * 1.3, rel=1e-6)
