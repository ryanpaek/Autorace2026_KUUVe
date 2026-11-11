"""11/11 위치 추정 강화: 후진 판단, 경로 방향 투영(조향 반영), 출발 정렬."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.route import MapPose, RouteMap, RouteTracker, motion_sign  # noqa: E402

# 동쪽으로 5 m 직선 -> 반지름 1 m 왼쪽 반원
ROUTE = {
    'start_xy': [0, 0],
    'segments': [
        {'type': 'line', 'to': [5000, 0]},
        {'type': 'arc', 'center': [5000, 1000], 'radius': 1000, 'start_deg': -90, 'end_deg': 90},
    ],
}


@pytest.fixture
def route():
    return RouteMap(ROUTE)


# --- 후진 판단 -----------------------------------------------------------------

def test_forward():
    assert motion_sign(0.01, 0.0, 0.0, 0.2, 0.2) == (1, '')


def test_reverse_from_odom_speed():
    assert motion_sign(-0.01, 0.0, 0.0, -0.2, None) == (-1, 'odom_v')


def test_reverse_from_odom_position_when_speed_unsigned():
    # 드라이버가 속도를 + 로만 줘도 위치가 차 방향 반대로 가면 후진
    assert motion_sign(-0.01, 0.0, 0.0, 0.2, None) == (-1, 'odom_xy')


def test_reverse_from_cmd_when_odom_never_reverses():
    # 드라이버가 후진에도 위치를 앞으로 쌓는 경우: 최근 /cmd_vel 부호로
    assert motion_sign(0.01, 0.0, 0.0, 0.2, -0.15) == (-1, 'cmd')


# --- 경로 방향 투영 ------------------------------------------------------------

def test_angled_step_split_into_along_and_lateral(route):
    mp = MapPose(route)
    assert mp.align(1000.0, 0.0, 0.0)               # IMU yaw 0 = 경로 방향(동쪽)
    ds = mp.advance(100.0, 1000.0, math.radians(30))  # 왼쪽으로 30도 틀어서 100 mm
    assert ds == pytest.approx(100 * math.cos(math.radians(30)), abs=0.1)
    assert mp.d == pytest.approx(50.0, abs=0.1)


def test_reverse_moves_s_back_and_d_back(route):
    mp = MapPose(route)
    mp.align(1000.0, 0.0, 0.0)
    ds = mp.advance(-100.0, 1000.0, math.radians(30))
    assert ds == pytest.approx(-86.6, abs=0.1)
    assert mp.d == pytest.approx(-50.0, abs=0.1)


def test_no_heading_yet_uses_raw_step(route):
    mp = MapPose(route)                               # align 안 함
    assert mp.advance(100.0, 1000.0, 0.5) == 100.0
    assert mp.d == 0.0


def test_huge_heading_error_falls_back_to_raw_step(route):
    mp = MapPose(route)
    mp.align(1000.0, 0.0, 0.0)
    assert mp.advance(100.0, 1000.0, math.radians(100)) == 100.0


def test_inside_of_curve_advances_s_faster(route):
    mp = MapPose(route)
    s_arc = 5000.0 + 200.0
    mp.align(s_arc, route.heading(s_arc), 0.0)
    mp.d = 200.0                                      # 왼쪽(곡선 안쪽)으로 200 mm
    ds = mp.advance(100.0, s_arc, route.heading(s_arc))
    assert ds == pytest.approx(100.0 / (1.0 - 0.2), rel=1e-3)


def test_weaving_no_longer_overcounts(route):
    """지그재그처럼 ±25도로 흔들며 직선 3 m를 가면 바퀴 거리는 ~10% 길다. 투영하면 도면 거리에 맞는다."""
    mp = MapPose(route)
    tr = RouteTracker(route)
    tr.reset(500.0, 0.0)
    mp.align(500.0, 0.0, 0.0)
    raw, along, x = 0.0, 0.0, 0.0
    step = 10.0
    while x < 3000.0:
        yaw = math.radians(25.0) * math.sin(2 * math.pi * x / 900.0)
        raw += step
        along += mp.advance(step, tr.s, yaw)
        tr.update(along)
        x += step * math.cos(yaw)
    assert raw / x > 1.05                              # 예전 방식이면 5% 넘게 과대
    assert abs(tr.s - 500.0 - x) < 0.01 * x            # 투영하면 1% 안


def test_reverse_back_and_forth_returns_to_same_s(route):
    """주차처럼 앞뒤로 꺾어 움직여도 같은 자리로 돌아오면 s도 같은 값."""
    mp = MapPose(route)
    tr = RouteTracker(route)
    tr.reset(2000.0, 0.0)
    mp.align(2000.0, 0.0, 0.0)
    along = 0.0
    path = [(30.0, 0.4)] * 20 + [(-30.0, 0.4)] * 20   # 비스듬히 600 전진 후 같은 방향으로 600 후진
    for st, yaw in path:
        along += mp.advance(st, tr.s, yaw)
        tr.update(along)
    assert tr.s == pytest.approx(2000.0, abs=1.0)
    assert mp.d == pytest.approx(0.0, abs=1.0)
