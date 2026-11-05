"""라바콘 구간 닫힌 루프 시뮬레이션 테스트 (가상 라이다 + 차량 모델, cone_sim.py).

통과 기준: 콘에 닿지 않음(차체 직사각형 ~ 콘 바닥 원 여유 > 10mm), 구간 끝에서 done,
끝선에서 차로 중심과 150mm 안 (그 뒤는 차선 노드가 잡는다).
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cone_sim as cs  # noqa: E402


@pytest.mark.parametrize('name', list(cs.SCENARIOS))
def test_scenarios_no_contact(name):
    cones = cs.SCENARIOS[name]()
    res = cs.run(cones, axis_err=math.radians(2.0), lane_err_mm=30.0, seed=1)
    assert res.done, res.follower.state_dict()
    assert res.min_clear > 10.0, (name, res.clear_by_cone)
    assert res.exit_x_err is not None and abs(res.exit_x_err) < 150.0
    assert res.done_y is not None and res.done_y < cs.END_Y + 100.0


@pytest.mark.parametrize('case', [
    dict(axis_err=math.radians(-4.0)),
    dict(drop_scan_prob=0.3),
    dict(r_scan=45.0),
    dict(steer_delay=0.45),
    dict(lidar_yaw_err=math.radians(1.0)),
])
def test_robust_straight_and_wave(case):
    for name in ('straight_500', 'wave_460'):
        res = cs.run(cs.SCENARIOS[name](), seed=2, **case)
        assert res.done and res.min_clear > 5.0, (name, case, res.clear_by_cone)


def test_no_cones_still_finishes_straight():
    res = cs.run([], seed=3)
    assert res.done and 'max progress' in res.follower.done_reason
    assert res.exit_x_err is not None and abs(res.exit_x_err) < 80.0
