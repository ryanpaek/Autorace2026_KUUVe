"""11/19: 속도 S자 가감속 (speed_smooth.py)"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.speed_smooth import SpeedSmoother  # noqa: E402


def _run(sm, target, n, dt=0.05):
    return [sm.step(target, dt) for _ in range(n)]


def test_s_curve_start_has_limited_jerk_and_accel():
    sm = SpeedSmoother(0.8, 1.0, 3.0, 0.5)
    vs = [0.0] + _run(sm, 0.5, 40)
    acc = [(b - a) / 0.05 for a, b in zip(vs, vs[1:])]
    jerk = [(b - a) / 0.05 for a, b in zip(acc, acc[1:])]
    assert max(acc) <= 0.8 + 1e-6
    reach = next(i for i, v in enumerate(vs) if v >= 0.5 - 1e-9)
    assert max(abs(j) for j in jerk[:reach - 2]) <= 3.0 + 1e-6     # 목표에 닿기 전까지 (닿는 칸은 가속도 0으로)
    assert acc[0] < acc[3] < acc[6]                                 # 처음엔 천천히 (S자 아래쪽)
    assert vs[-1] == pytest.approx(0.5)


def test_zero_and_reverse_and_bypass_are_immediate():
    sm = SpeedSmoother()
    _run(sm, 0.5, 60)
    assert sm.step(0.0, 0.05) == 0.0                                # 정지는 바로
    sm.reset(0.3)
    assert sm.step(-0.1, 0.05) == -0.1                              # 전진 <-> 후진은 바로
    sm.reset(0.5)
    assert sm.step(0.15, 0.05, bypass=True) == 0.15                 # M7 (bypass)


def test_small_jitter_is_filtered_but_big_change_is_quick():
    sm = SpeedSmoother(0.8, 1.0, 3.0, 0.5)
    sm.reset(0.5)
    out = [sm.step(0.5 if i % 2 else 0.45, 0.05) for i in range(200)]
    assert max(out[100:]) - min(out[100:]) < 0.03                  # 입력은 0.05씩 흔들림
    sm.reset(0.5)
    d, n = 0.0, 0
    while sm.v > 0.155:
        d += sm.step(0.15, 0.05) * 0.05
        n += 1
    assert d < 0.3 and n * 0.05 < 1.0                               # 0.5 -> 0.15: 30 cm·1초 안
