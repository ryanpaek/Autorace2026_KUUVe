"""m7_waypoint_core (11/19 실험, m7_impl:=waypoint) 오프라인 시뮬로 확인."""
import math

import numpy as np
import pytest

from limo_obstacle_avoid.m7_waypoint_core import M7Waypoint, WaypointParams, fit_box
from limo_obstacle_avoid.m7_waypoint_sim import run


def test_fit_box_rear_face_gives_heading_and_center():
    # 가로 정차 LIMO(차로 방향 0.22, 폭 0.32) 뒷면만 보임, 차가 5° 틀어짐
    th = math.radians(5.0)
    cx, cy = 1.2, 0.05
    pts = []
    for i in range(16):
        b = -0.16 + i * 0.02
        a = -0.11
        x = cx + a * math.cos(-th) - b * math.sin(-th)
        y = cy + a * math.sin(-th) + b * math.cos(-th)
        pts.append((x, y, math.atan2(y, x)))
    r = fit_box(pts, 0.0, 0.22, 0.32, 0.10)
    assert r is not None
    assert abs(r[0] - cx) < 0.03 and abs(r[1] - cy) < 0.03
    assert abs(math.degrees(r[2]) + 5.0) < 2.0
    assert r[3]                      # 뒷면(32 cm)만으로 방향을 잼


def test_m2_only_mode_finishes_immediately():
    p = WaypointParams()
    p.m2_test_start = True
    c = M7Waypoint(p)
    c.start()
    assert c.state == 'DONE'


@pytest.mark.parametrize('seed', [0, 2, 3, 5])
def test_race1_layout_straight_entry_passes_without_reverse(seed):
    r = run(seed, 'straight', {'reverse_max': 0})
    assert r['feasible']
    assert not r['contact'] and not r['depart']
    assert r['success'], r


@pytest.mark.parametrize('seed', [1, 4])
def test_infeasible_layout_stops_without_contact(seed):
    r = run(seed, 'straight', {'reverse_max': 0}, t_max=40.0)
    assert not r['feasible']
    assert not r['contact']


def test_lane_core_yellow_fit_ignores_far_points():
    from autorace_lane.lane_core import _yellow_fit_lat
    # 오른쪽 노란선이 차 오른쪽 0.6 m에 있고 차가 20° 그쪽으로 틀어짐 -> 앞으로 갈수록 옆 거리가 줄어듦
    a = math.radians(20.0)
    ys = np.arange(600.0, 2100.0, 20.0)
    xs = 600.0 / math.cos(a) - ys * math.tan(a)        # x 오른쪽 + (mm)
    pts = np.stack([xs, ys], axis=1)
    lat = _yellow_fit_lat(pts, 550.0)
    assert lat == pytest.approx(600.0 / math.cos(a) - 550.0 * math.tan(a), abs=1.0)   # 약 438 mm
    assert float(np.percentile(xs, 15)) < 150.0                                         # 예전 방식은 '붙음'으로 봄
